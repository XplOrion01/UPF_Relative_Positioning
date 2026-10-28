# ================================================================
# 2D UPF-UKF  --  BOTH BOTS MOVING, one bot (A) estimates the other (B)
# Variant 2: UWB noise 0.01 m
# ================================================================
# Shared between the bots: ONLY odometry increments (body frame) + one UWB range.
# No positions / poses are ever shared. Ground truth is used for scoring only.
#
# State per particle = pose of B expressed in A's CURRENT body frame [x, y, yaw].
# Because A moves and tracks its own heading, the relative pose is fully
# observable once the relative motion is not degenerate (no free "gauge" rotation
# like the static-A case), so NO azimuth pinning is needed here: the particles
# cover the 2 unknowns (bearing of B, heading of B) on a grid.
#
# Prediction uses BOTH agents' odometry:
#   p' = R(-dth_A) * ( p + R(theta) * dp_B - dp_A )
#   theta' = theta + dth_B - dth_A
# Noise of both odometries is added; A's heading noise swings the relative
# position by ~|p|*sigma (this plays the role of the paper's rotational-drift
# pseudo-state).
# ================================================================

# ---------------- USER SETTINGS ----------------
UWB_NOISE_STD = 0.01      # metres, 1-sigma of UWB range noise (paper: 0.1 ... 1.0)
N_BEARING = 16               # bearing hypotheses of B around A
N_HEADING = 16               # heading hypotheses of B  (particles = N_BEARING * N_HEADING)
# -----------------------------------------------

import sys
import math
import random
import csv
from fim_toolkit import FIMMonitor   # FIM / observability monitor (keep fim_toolkit.py next to this file)
from datetime import datetime
from types import SimpleNamespace
import numpy as np
from PyQt6.QtCore import Qt, QTimer, QPointF
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFont
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QLabel, QTextEdit, QGroupBox, QSplitter
)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class RelPoseUPF:
    """State per particle = pose of B expressed in A's CURRENT body frame [x, y, yaw]."""
    def __init__(self, n_bearing=24, n_heading=24, uwb_std=0.01, alpha=1.0, beta=2.0, kappa=-1.0):
        self.nb, self.nh, self.uwb_std = n_bearing, n_heading, uwb_std
        self.N = n_bearing*n_heading; self.n=3
        lam = alpha**2*(self.n+kappa)-self.n; self.c=self.n+lam
        self.Wm=np.r_[lam/self.c, np.full(2*self.n,0.5/self.c)]; self.Wc=self.Wm.copy(); self.Wc[0]+=1-alpha**2+beta
        self.initialised=False
    def _init(self, d0):
        d0=max(d0,0.05); nb,nh=self.nb,self.nh
        phi=np.repeat(2*np.pi*np.arange(nb)/nb, nh); th=np.tile(2*np.pi*np.arange(nh)/nh, nb)
        self.X=np.stack([d0*np.cos(phi), d0*np.sin(phi), wrap(th)],1)
        sr=max(0.02,self.uwb_std); st=d0*0.5*(2*np.pi/nb); sy=0.5*(2*np.pi/nh)
        P=np.zeros((self.N,3,3)); c,s=np.cos(phi),np.sin(phi)
        # radial/tangential covariance rotated to the particle's bearing
        P[:,0,0]=c*c*sr**2+s*s*st**2; P[:,1,1]=s*s*sr**2+c*c*st**2; P[:,0,1]=P[:,1,0]=c*s*(sr**2-st**2); P[:,2,2]=sy**2
        self.P=P; self.logw=np.full(self.N,-np.log(self.N)); self.initialised=True
    def _sigma(self):
        L=np.linalg.cholesky(0.5*(self.P+self.P.transpose(0,2,1))*self.c+1e-12*np.eye(3))
        S=np.empty((self.N,7,3)); S[:,0]=self.X
        for i in range(3): S[:,1+2*i]=self.X+L[:,:,i]; S[:,2+2*i]=self.X-L[:,:,i]
        return S
    def _predict(self, oi, oj, si, sj):
        (dxi,dyi,dti),(dxj,dyj,dtj)=oi,oj
        S=self._sigma(); th=S[...,2]; c,s=np.cos(th),np.sin(th)
        qx=S[...,0]+c*dxj-s*dyj-dxi; qy=S[...,1]+s*dxj+c*dyj-dyi
        ci,sn=math.cos(dti),math.sin(dti)
        Sp=np.stack([ci*qx+sn*qy, -sn*qx+ci*qy, wrap(th+dtj-dti)],-1)
        mx=(self.Wm*Sp[...,0]).sum(1); my=(self.Wm*Sp[...,1]).sum(1)
        mth=np.arctan2((self.Wm*np.sin(Sp[...,2])).sum(1),(self.Wm*np.cos(Sp[...,2])).sum(1))
        M=np.stack([mx,my,mth],1); D=Sp-M[:,None,:]; D[...,2]=wrap(D[...,2])
        P=np.einsum('k,nki,nkj->nij',self.Wc,D,D)
        # ---- odometry noise of BOTH agents ----
        psi=mth-dti; cj,sj_=np.cos(psi),np.sin(psi)
        Rj=np.zeros((self.N,2,2)); Rj[:,0,0]=cj;Rj[:,0,1]=-sj_;Rj[:,1,0]=sj_;Rj[:,1,1]=cj
        Ri=np.array([[ci,sn],[-sn,ci]])
        Qj=Rj@np.diag([sj[0]**2,sj[1]**2])@Rj.transpose(0,2,1)
        Qi=Ri@np.diag([si[0]**2,si[1]**2])@Ri.T
        P[:,:2,:2]+=Qj+Qi
        P[:,2,2]+=sj[2]**2+si[2]**2
        # A's heading noise swings the whole relative position (the "rotational drift" effect, ~ |p|*sigma)
        g=np.stack([my, -mx, -np.ones(self.N)],1)
        P+=si[2]**2*g[:,:,None]*g[:,None,:]
        self.X=M; self.P=0.5*(P+P.transpose(0,2,1))
    def _update(self,d,sd):
        S=self._sigma(); Z=np.maximum(np.hypot(S[...,0],S[...,1]),1e-5); zm=(self.Wm*Z).sum(1)
        zd=Z-zm[:,None]; xd=S-self.X[:,None,:]; xd[...,2]=wrap(xd[...,2])
        Sinn=(self.Wc*zd**2).sum(1)+sd**2; Pxz=np.einsum('k,nki,nk->ni',self.Wc,xd,zd)
        K=Pxz/Sinn[:,None]; r=d-zm
        self.X=self.X+K*r[:,None]; self.X[:,2]=wrap(self.X[:,2])
        self.P=self.P-K[:,:,None]*Sinn[:,None,None]*K[:,None,:]; self.P=0.5*(self.P+self.P.transpose(0,2,1))
        self.logw+=-0.5*r**2/Sinn-0.5*np.log(2*np.pi*Sinn)
        self.logw-=self.logw.max(); self.logw-=np.log(np.exp(self.logw).sum())
    def step(self,oi,oj,si,sj,d,sd):
        if not self.initialised: self._init(d)
        self._predict(oi,oj,si,sj); self._update(d,sd)
    def get_estimated_pose(self):
        if not self.initialised: return 0.0, 0.0, 0.0
        x, y, th = self.X[int(np.argmax(self.logw))]
        return float(x), float(y), float(th)
    def plot_particles(self, min_w=1e-4):
        # only hypotheses that still carry weight (keeps the map readable and fast)
        w = np.exp(self.logw)
        idx = np.nonzero(w > min_w)[0]
        return [SimpleNamespace(state=list(self.X[i]), weight=float(w[i])) for i in idx]
    def n_modes(self,thr=0.05): return int((np.exp(self.logw)>thr).sum())



# --- GUI MAP RENDERER (drawn in A's BODY frame: A at centre, facing right) ---

class MapCanvas(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.true_x, self.true_y = 1.0, 0.0
        self.est_x, self.est_y = 0.0, 0.0
        self.particles = []
        self.setStyleSheet("background-color: #1e1e1e; border-radius: 6px;")

    def update_state(self, bx, by, ex, ey, particles):
        self.true_x, self.true_y = bx, by
        self.est_x, self.est_y = ex, ey
        self.particles = particles
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        width, height = self.width(), self.height()
        cx, cy = width / 2.0, height / 2.0
        # auto-scale so B stays on screen (both bots wander, separation grows)
        reach = max(1.5, math.hypot(self.true_x, self.true_y),
                    math.hypot(self.est_x, self.est_y))
        scale = min(120.0, 0.42 * min(width, height) / reach)

        painter.setPen(QPen(QColor("#2d2d2d"), 1, Qt.PenStyle.DashLine))
        step = max(10, int(scale))
        for x in range(int(cx) % step, width, step):
            painter.drawLine(x, 0, x, height)
        for y in range(int(cy) % step, height, step):
            painter.drawLine(0, y, width, y)
        painter.setPen(QPen(QColor("#444444"), 2))
        painter.drawLine(0, int(cy), width, int(cy))
        painter.drawLine(int(cx), 0, int(cx), height)

        painter.setBrush(QBrush(QColor("#00bcd4")))
        painter.setPen(Qt.PenStyle.NoPen)
        for p in self.particles:
            painter.drawEllipse(QPointF(cx + p.state[0] * scale, cy - p.state[1] * scale), 3, 3)

        painter.setBrush(QBrush(QColor("#4caf50")))
        painter.setPen(QPen(QColor("#ffffff"), 1))
        painter.drawRect(int(cx) - 12, int(cy) - 12, 24, 24)
        painter.setPen(QColor("#ffffff"))
        painter.setFont(QFont("Arial", 9, QFont.Weight.Bold))
        painter.drawText(int(cx) - 45, int(cy) + 30, "UGV A (estimator)")

        tbx, tby = cx + self.true_x * scale, cy - self.true_y * scale
        painter.setBrush(QBrush(QColor("#ff9800")))
        painter.setPen(QPen(QColor("#ffffff"), 2))
        painter.drawEllipse(QPointF(tbx, tby), 14, 14)
        painter.drawText(int(tbx) - 25, int(tby) - 20, "True B")

        ebx, eby = cx + self.est_x * scale, cy - self.est_y * scale
        painter.setBrush(QBrush(QColor("transparent")))
        painter.setPen(QPen(QColor("#f44336"), 3, Qt.PenStyle.SolidLine))
        painter.drawRect(int(ebx) - 14, int(eby) - 14, 28, 28)
        painter.setPen(QColor("#f44336"))
        painter.drawText(int(ebx) - 30, int(eby) + 32, "Estimated B")


class ProUGVDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        random.seed(42)
        self.setWindowTitle("2D UPF-UKF: both bots moving (50 Hz)  UWB noise %.2f m" % UWB_NOISE_STD)
        self.resize(1200, 800)

        # --- TRUE world poses (never given to the filter) ---
        d0 = 1.0
        bearing = random.uniform(-math.pi, math.pi)
        self.a = [0.0, 0.0, random.uniform(-math.pi, math.pi)]
        self.b = [d0 * math.cos(bearing), d0 * math.sin(bearing), random.uniform(-math.pi, math.pi)]
        self.va, self.wa = 0.05, 0.01
        self.vb, self.wb = 0.05, -0.01

        self.fim = FIMMonitor(sigma_r=UWB_NOISE_STD)
        self.upf = RelPoseUPF(N_BEARING, N_HEADING, uwb_std=UWB_NOISE_STD)

        self.csv_filename = "results.csv"
        self.init_csv_file()
        self.step_count = 0

        self.setStyleSheet("""
            QMainWindow { background-color: #121212; }
            QGroupBox { color: #b0bec5; font-weight: bold; font-size: 11pt; border: 1px solid #333333; border-radius: 6px; margin-top: 10px; padding-top: 15px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QTextEdit { background-color: #181818; color: #00e676; border: 1px solid #333333; font-family: 'Consolas'; font-size: 10pt; border-radius: 4px; }
            QLabel { color: #e0e0e0; font-size: 10pt; }
        """)

        main_widget = QWidget(self)
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(splitter)

        left_container = QWidget()
        left_layout = QVBoxLayout(left_container)
        map_group = QGroupBox("Relative map in A's body frame (A estimates B)")
        map_layout = QVBoxLayout(map_group)
        self.map_canvas = MapCanvas()
        map_layout.addWidget(self.map_canvas)
        left_layout.addWidget(map_group)
        splitter.addWidget(left_container)

        right_container = QWidget()
        right_layout = QVBoxLayout(right_container)
        status_group = QGroupBox("Real-Time Telemetry Metrics")
        status_layout = QVBoxLayout(status_group)
        self.lbl_uwb = QLabel("UWB Measured Distance: -")
        self.lbl_true = QLabel("True B in A frame: -")
        self.lbl_est = QLabel("Estimated B in A frame: -")
        self.lbl_err = QLabel("Position Error: -")
        for lbl in [self.lbl_uwb, self.lbl_true, self.lbl_est, self.lbl_err]:
            status_layout.addWidget(lbl)
        right_layout.addWidget(status_group)

        log_group = QGroupBox("System Packet Logs (saved to results.csv)")
        log_layout = QVBoxLayout(log_group)
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        log_layout.addWidget(self.console)
        right_layout.addWidget(log_group)
        splitter.addWidget(right_container)
        splitter.setSizes([700, 500])

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.simulation_step)
        self.timer.start(20)

    def init_csv_file(self):
        with open(self.csv_filename, mode='w', newline='') as f:
            csv.writer(f).writerow([
                "Timestamp", "Step", "True_Rel_X", "True_Rel_Y", "Est_Rel_X", "Est_Rel_Y",
                "UWB_Distance", "Position_Error", "Heading_Error", "Active_Modes",
                "FIM_Cond", "FIM_Sigma_t_m", "FIM_Sigma_Theta_deg", "FIM_Singular", "Moving_Enough"])

    @staticmethod
    def _move(p, v, w):
        # body-frame increment in the PREVIOUS frame: translate with old yaw, then rotate
        p[0] += math.cos(p[2]) * v
        p[1] += math.sin(p[2]) * v
        p[2] = math.atan2(math.sin(p[2] + w), math.cos(p[2] + w))

    def simulation_step(self):
        self.step_count += 1
        if self.step_count % 100 == 0:
            self.va, self.wa = random.uniform(0.01, 0.04), random.uniform(-0.02, 0.02)
            self.vb, self.wb = random.uniform(0.01, 0.04), random.uniform(-0.02, 0.02)

        # 1. true motion of BOTH bots
        self._move(self.a, self.va, self.wa)
        self._move(self.b, self.vb, self.wb)

        # 2. noisy odometry of each bot. This (and only this) is shared.
        sig = (0.003, 0.002, 0.002)
        odo_a = (self.va + random.gauss(0, sig[0]), random.gauss(0, sig[1]), self.wa + random.gauss(0, sig[2]))
        odo_b = (self.vb + random.gauss(0, sig[0]), random.gauss(0, sig[1]), self.wb + random.gauss(0, sig[2]))

        # 3. UWB range
        true_dist = math.hypot(self.b[0] - self.a[0], self.b[1] - self.a[1])
        uwb = true_dist + random.gauss(0, UWB_NOISE_STD)

        # 4. filter
        self.upf.step(odo_a, odo_b, sig, sig, uwb, UWB_NOISE_STD)
        ex, ey, eyaw = self.upf.get_estimated_pose()
        # FIM monitor: same shared data as the filter (both odometries)
        self.fim.push(odo_a, odo_b)
        fr = self.fim.report(rel_est=(ex, ey, eyaw))
        fim_cols = [round(fr['cond'], 2) if fr['cond'] != float('inf') else 'inf',
                    round(fr['sigma_t'], 4) if fr['sigma_t'] != float('inf') else 'inf',
                    round(fr['sigma_theta_deg'], 3) if fr['sigma_theta_deg'] != float('inf') else 'inf',
                    int(fr['singular']), int(fr['moving_enough'])]

        # truth, for scoring only: B's pose in A's body frame
        c, s = math.cos(self.a[2]), math.sin(self.a[2])
        dx, dy = self.b[0] - self.a[0], self.b[1] - self.a[1]
        tx, ty = c * dx + s * dy, -s * dx + c * dy
        tyaw = math.atan2(math.sin(self.b[2] - self.a[2]), math.cos(self.b[2] - self.a[2]))

        pos_error = math.hypot(tx - ex, ty - ey)
        yaw_error = abs(math.atan2(math.sin(tyaw - eyaw), math.cos(tyaw - eyaw)))
        modes = self.upf.n_modes()

        with open(self.csv_filename, mode='a', newline='') as f:
            csv.writer(f).writerow([
                datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3], self.step_count,
                round(tx, 4), round(ty, 4), round(ex, 4), round(ey, 4),
                round(uwb, 4), round(pos_error, 4), round(yaw_error, 4), modes,
                *fim_cols])

        self.map_canvas.update_state(tx, ty, ex, ey, self.upf.plot_particles())
        self.lbl_uwb.setText(f"UWB Measured Distance: {uwb:.3f} m")
        self.lbl_true.setText(f"True B in A frame: X: {tx:.2f}m, Y: {ty:.2f}m")
        self.lbl_est.setText(f"Estimated B in A frame: X: {ex:.2f}m, Y: {ey:.2f}m")
        self.lbl_err.setText(f"Position Error: {pos_error:.3f} m")

        if self.step_count % 10 == 0:
            self.console.append(
                f"<span style='color:#00bcd4;'>[UPF 50Hz]</span> Step:{self.step_count} | "
                f"Modes:{modes} | Dist: <b>{uwb:.3f}m</b><br>"
                f"<span style='color:#4caf50;'>[EST]</span> X:{ex:.2f}, Y:{ey:.2f} | "
                f"<span style='color:#f44336;'>[ERR]</span> {pos_error:.3f}m<br>"
                f"--------------------------------------------------<br>")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    dashboard = ProUGVDashboard()
    dashboard.show()
    sys.exit(app.exec())

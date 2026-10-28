# ================================================================
# 2D UPF + UKF REFERENCE IMPLEMENTATION
# ================================================================
# This is a rigorous 2D particle-filter/UKF implementation.
#
# It intentionally models:
#   - state: [x, y, yaw]
#   - planar UWB range: sqrt(x^2 + y^2)
#   - noisy odometry increments
#   - 32 particles = 8 position hypotheses x 4 heading hypotheses
#   - true UKF sigma points
#   - multiplicative particle weights
#   - systematic resampling
#
# It is NOT claimed to be Yuri's exact UPF because Yuri uses a
# different branch-kill/merge resampling mechanism and a richer state.
# ================================================================

import sys
import math
import random
import csv
from fim_toolkit import FIMMonitor   # FIM / observability monitor (keep fim_toolkit.py next to this file)
import os
from datetime import datetime
from PyQt6.QtCore import Qt, QTimer, QPointF
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFont
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QLabel, QTextEdit, QGroupBox, QSplitter
)


# ================================================================
# GAUGE-FIXED UPF  (replaces RigorousParticle / RigorousUPFSystem)
# ================================================================
# Key idea: UGV A is static and shares NO pose, only B's odometry.
# Rotating B's whole trajectory about A leaves every UWB range unchanged,
# so B's start AZIMUTH around A is unobservable (a free "gauge").
# We pin it (B starts on A's +x axis) and spend ALL particles on the one
# quantity range+odometry can actually reveal: B's initial HEADING.
# 32 hypotheses -> 11.25 deg spacing (the old 8x4 grid had only 8 distinct
# observable hypotheses at 45 deg spacing).
# Output frame = "gauge frame". To compare with a ground truth you must
# rotate by ONE angle psi (simulation: known; hardware: fit it once, e.g. vs Vicon).
# ================================================================

import numpy as np
from types import SimpleNamespace


def _wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


class GaugeFixedUPF:
    def __init__(self, num_particles=32, sig_pos=0.02,
                 alpha=1.0, beta=2.0, kappa=-1.0):
        self.N = num_particles
        self.n = 3
        self.sig_pos = sig_pos
        self.sig_th = 0.5 * (2.0 * np.pi / num_particles)   # overlap neighbouring hypotheses
        lam = alpha ** 2 * (self.n + kappa) - self.n
        self.c = self.n + lam
        self.Wm = np.r_[lam / self.c, np.full(2 * self.n, 0.5 / self.c)]
        self.Wc = self.Wm.copy()
        self.Wc[0] += 1.0 - alpha ** 2 + beta
        self.initialised = False
        self.particles = []        # SimpleNamespace(state=[x,y,yaw], weight=w) for plotting
        self.best_idx = 0

    # -- initialisation uses the FIRST measured range ----------------
    def _init(self, d0):
        N = self.N
        th = _wrap(2.0 * np.pi * np.arange(N) / N)
        self.X = np.stack([np.full(N, d0), np.zeros(N), th], 1)
        self.P = np.tile(np.diag([self.sig_pos ** 2, self.sig_pos ** 2,
                                  self.sig_th ** 2]), (N, 1, 1))
        self.logw = np.full(N, -np.log(N))
        self.initialised = True

    def _sigma(self):
        A = 0.5 * (self.P + self.P.transpose(0, 2, 1)) * self.c + 1e-12 * np.eye(3)
        L = np.linalg.cholesky(A)
        S = np.empty((self.N, 7, 3))
        S[:, 0] = self.X
        for i in range(3):
            S[:, 1 + 2 * i] = self.X + L[:, :, i]
            S[:, 2 + 2 * i] = self.X - L[:, :, i]
        return S

    def _predict(self, dx, dy, dth, sx, sy, sth):
        S = self._sigma()
        th = S[..., 2]
        c, s = np.cos(th), np.sin(th)
        # body-frame increment expressed in the PREVIOUS body frame, then rotate
        Sp = np.stack([S[..., 0] + c * dx - s * dy,
                       S[..., 1] + s * dx + c * dy,
                       _wrap(th + dth)], -1)
        mx = (self.Wm * Sp[..., 0]).sum(1)
        my = (self.Wm * Sp[..., 1]).sum(1)
        mth = np.arctan2((self.Wm * np.sin(Sp[..., 2])).sum(1),
                         (self.Wm * np.cos(Sp[..., 2])).sum(1))
        M = np.stack([mx, my, mth], 1)
        D = Sp - M[:, None, :]
        D[..., 2] = _wrap(D[..., 2])
        P = np.einsum('k,nki,nkj->nij', self.Wc, D, D)
        # odometry noise lives in the BODY frame -> rotate it into the global frame
        c0, s0 = np.cos(mth), np.sin(mth)
        Qb = np.zeros((self.N, 3, 3))
        Qb[:, 0, 0], Qb[:, 1, 1], Qb[:, 2, 2] = sx ** 2, sy ** 2, sth ** 2
        R = np.zeros((self.N, 3, 3))
        R[:, 0, 0], R[:, 0, 1], R[:, 1, 0], R[:, 1, 1], R[:, 2, 2] = c0, -s0, s0, c0, 1.0
        P += R @ Qb @ R.transpose(0, 2, 1)
        self.X, self.P = M, 0.5 * (P + P.transpose(0, 2, 1))

    def _update(self, d, sd):
        S = self._sigma()
        Z = np.maximum(np.hypot(S[..., 0], S[..., 1]), 1e-5)
        zm = (self.Wm * Z).sum(1)
        zd = Z - zm[:, None]
        xd = S - self.X[:, None, :]
        xd[..., 2] = _wrap(xd[..., 2])
        Sinn = (self.Wc * zd ** 2).sum(1) + sd ** 2
        Pxz = np.einsum('k,nki,nk->ni', self.Wc, xd, zd)
        K = Pxz / Sinn[:, None]
        r = d - zm
        self.X = self.X + K * r[:, None]
        self.X[:, 2] = _wrap(self.X[:, 2])
        self.P = self.P - K[:, :, None] * Sinn[:, None, None] * K[:, None, :]
        self.P = 0.5 * (self.P + self.P.transpose(0, 2, 1))
        # CUMULATIVE evidence. NO weight floor and NO forgetting: a floor lets
        # dead hypotheses "resurrect" and take over (measured: 25 m error).
        self.logw += -0.5 * r ** 2 / Sinn - 0.5 * np.log(2.0 * np.pi * Sinn)
        self.logw -= self.logw.max()
        self.logw -= np.log(np.exp(self.logw).sum())

    def step(self, measured_dx, measured_dy, measured_d_yaw, Q_cov,
             measured_distance, R_var):
        if not self.initialised:
            self._init(measured_distance)
        self._predict(measured_dx, measured_dy, measured_d_yaw,
                      math.sqrt(Q_cov[0][0]), math.sqrt(Q_cov[1][1]), math.sqrt(Q_cov[2][2]))
        self._update(measured_distance, math.sqrt(R_var))
        self.best_idx = int(np.argmax(self.logw))
        w = np.exp(self.logw)
        self.particles = [SimpleNamespace(state=list(self.X[i]), weight=float(w[i]))
                          for i in range(self.N)]

    def get_estimated_pose(self):
        if not self.initialised:
            return 0.0, 0.0, 0.0
        x, y, th = self.X[self.best_idx]
        return float(x), float(y), float(th)

    def n_active_modes(self, thr=0.05):
        return int((np.exp(self.logw) > thr).sum())


# --- GUI MAP RENDERER & DASHBOARD WINDOW ---

class MapCanvas(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.true_b_x = 0.0
        self.true_b_y = 1.0
        self.est_x = 0.0
        self.est_y = 0.0
        self.particles = []
        self.setStyleSheet("background-color: #1e1e1e; border-radius: 6px;")

    def update_state(self, bx, by, ex, ey, particles):
        self.true_b_x = bx
        self.true_b_y = by
        self.est_x = ex
        self.est_y = ey
        self.particles = particles
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        width = self.width()
        height = self.height()
        cx, cy = width / 2.0, height / 2.0
        scale = 120.0

        painter.setPen(QPen(QColor("#2d2d2d"), 1, Qt.PenStyle.DashLine))
        step = int(scale)
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
            px = cx + (p.state[0] * scale)
            py = cy - (p.state[1] * scale)
            painter.drawEllipse(QPointF(px, py), 3, 3)

        painter.setBrush(QBrush(QColor("#4caf50")))
        painter.setPen(QPen(QColor("#ffffff"), 1))
        painter.drawRect(int(cx) - 12, int(cy) - 12, 24, 24)
        painter.setPen(QColor("#ffffff"))
        painter.setFont(QFont("Arial", 9, QFont.Weight.Bold))
        painter.drawText(int(cx) - 35, int(cy) + 30, "UGV A (Base)")

        tbx = cx + (self.true_b_x * scale)
        tby = cy - (self.true_b_y * scale)
        painter.setBrush(QBrush(QColor("#ff9800")))
        painter.setPen(QPen(QColor("#ffffff"), 2))
        painter.drawEllipse(QPointF(tbx, tby), 14, 14)
        painter.drawText(int(tbx) - 25, int(tby) - 20, "True B")

        ebx = cx + (self.est_x * scale)
        eby = cy - (self.est_y * scale)
        painter.setBrush(QBrush(QColor("transparent")))
        painter.setPen(QPen(QColor("#f44336"), 3, Qt.PenStyle.SolidLine))
        painter.drawRect(int(ebx) - 14, int(eby) - 14, 28, 28)
        painter.setPen(QColor("#f44336"))
        painter.drawText(int(ebx) - 30, int(eby) + 32, "Estimated B")


class ProUGVDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        # Reproducible benchmark run. Change/remove this for random trials.
        random.seed(42)
        self.setWindowTitle("2D Gauge-Fixed UPF-UKF Estimator (50 Hz)")
        self.resize(1200, 800)
        
        # Random start pose (NOT on any particle-grid point). B starts 1 m from A.
        # start_phi = B's true start azimuth around A. Range+odometry can never
        # reveal it (A is static), so the filter works in a "gauge frame" where B
        # starts on +x; we rotate the estimate by start_phi ONLY for scoring.
        self.start_phi = random.uniform(-math.pi, math.pi)
        self.true_b_x = math.cos(self.start_phi)
        self.true_b_y = math.sin(self.start_phi)
        self.true_b_yaw = random.uniform(-math.pi, math.pi)
        
        self.current_v = 0.05
        self.current_w = 0.01
        
        self.fim = FIMMonitor(sigma_r=0.01)
        self.upf = GaugeFixedUPF(num_particles=32)
        
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
        map_group = QGroupBox("Live 2D UGV Map (gauge-fixed UPF-UKF)")
        map_layout = QVBoxLayout(map_group)
        self.map_canvas = MapCanvas()
        map_layout.addWidget(self.map_canvas)
        left_layout.addWidget(map_group)
        splitter.addWidget(left_container)

        right_container = QWidget()
        right_layout = QVBoxLayout(right_container)
        
        status_group = QGroupBox("Real-Time Telemetry Metrics")
        status_layout = QVBoxLayout(status_group)
        self.lbl_uwb = QLabel("UWB Measured Distance: 1.000 m")
        self.lbl_true = QLabel("True Pos (B): X: 0.00m, Y: 1.00m")
        self.lbl_est = QLabel("Estimated Pos (B): X: 0.00m, Y: 0.00m")
        self.lbl_err = QLabel("Position Error: 0.000 m")
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
            writer = csv.writer(f)
            writer.writerow([
                "Timestamp", "Step", "True_X", "True_Y", 
                "Est_X", "Est_Y", "UWB_Distance", "Position_Error",
                "Heading_Error", "A_in_B_Error", "Active_Modes",
                "FIM_Cond", "FIM_Sigma_t_m", "FIM_Sigma_Theta_deg", "FIM_Singular", "Moving_Enough"
            ])

    def simulation_step(self):
        self.step_count += 1
        
        if self.step_count % 100 == 0:
            self.current_v = random.uniform(0.01, 0.04)
            self.current_w = random.uniform(-0.02, 0.02)
            
        # 1. True robot movement (Fixes Mistake #10: separating true motion from sensor noise)
        true_dx = self.current_v
        true_dy = 0.0
        true_d_yaw = self.current_w
        
        # Odometry increments are expressed in the PREVIOUS body frame (same
        # convention as the filter): translate with the old yaw, THEN rotate.
        # (Original rotated first -> systematic sim/filter mismatch.)
        cos_y = math.cos(self.true_b_yaw)
        sin_y = math.sin(self.true_b_yaw)
        self.true_b_x += cos_y * true_dx - sin_y * true_dy
        self.true_b_y += sin_y * true_dx + cos_y * true_dy
        self.true_b_yaw = math.atan2(math.sin(self.true_b_yaw + true_d_yaw),
                                     math.cos(self.true_b_yaw + true_d_yaw))
        
        # 2. Noisy odometry sensor packet sent to filter
        measured_dx = true_dx + random.gauss(0, 0.003)
        measured_dy = true_dy + random.gauss(0, 0.002)
        measured_d_yaw = true_d_yaw + random.gauss(0, 0.002)
        
        # Odometry covariance must match the simulated sensor noise.
        # measured_dx std = 0.003 m
        # measured_dy std = 0.002 m
        # measured_d_yaw std = 0.002 rad
        Q_cov = [
            [0.003 ** 2, 0.0, 0.0],
            [0.0, 0.002 ** 2, 0.0],
            [0.0, 0.0, 0.002 ** 2]
        ]
        
        # 3. UWB distance sensor with consistent noise variance R = 0.03**2 (Fixes Mistake #2)
        true_dist = math.sqrt(self.true_b_x**2 + self.true_b_y**2)
        uwb_noise_std = 0.01
        measured_uwb_dist = true_dist + random.gauss(0, uwb_noise_std)
        R_var = uwb_noise_std ** 2
        
        # 4. Execute Rigorous UPF Step
        self.upf.step(measured_dx, measured_dy, measured_d_yaw, Q_cov, measured_uwb_dist, R_var)
        gx, gy, gyaw = self.upf.get_estimated_pose()      # gauge-frame estimate
        # FIM monitor: A is static (zero odometry); B's noisy odometry is what is shared
        self.fim.push((0.0, 0.0, 0.0), (measured_dx, measured_dy, measured_d_yaw))
        fr = self.fim.report(rel_est=(gx, gy, gyaw))
        fim_cols = [round(fr['cond'], 2) if fr['cond'] != float('inf') else 'inf',
                    round(fr['sigma_t'], 4) if fr['sigma_t'] != float('inf') else 'inf',
                    round(fr['sigma_theta_deg'], 3) if fr['sigma_theta_deg'] != float('inf') else 'inf',
                    int(fr['singular']), int(fr['moving_enough'])]
        cp, sp = math.cos(self.start_phi), math.sin(self.start_phi)
        est_x = cp * gx - sp * gy                         # rotate into A's true frame (scoring only)
        est_y = sp * gx + cp * gy
        est_yaw = math.atan2(math.sin(gyaw + self.start_phi), math.cos(gyaw + self.start_phi))
        # rotate particle cloud the same way so the map shows hypotheses in A's frame
        for p in self.upf.particles:
            px, py, pth = p.state
            p.state = [cp * px - sp * py, sp * px + cp * py, pth + self.start_phi]
        
        pos_error = math.sqrt((self.true_b_x - est_x)**2 + (self.true_b_y - est_y)**2)
        yaw_error = abs(math.atan2(math.sin(self.true_b_yaw - est_yaw), math.cos(self.true_b_yaw - est_yaw)))
        # Gauge-invariant error: where is A, seen from B's body frame? (observable once B turns)
        def a_in_b(px, py, yw):
            c, s = math.cos(yw), math.sin(yw)
            return c * (-px) + s * (-py), -s * (-px) + c * (-py)
        ta = a_in_b(self.true_b_x, self.true_b_y, self.true_b_yaw)
        ea = a_in_b(est_x, est_y, est_yaw)
        inv_error = math.hypot(ta[0] - ea[0], ta[1] - ea[1])
        
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        with open(self.csv_filename, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp, self.step_count, round(self.true_b_x, 4), round(self.true_b_y, 4),
                round(est_x, 4), round(est_y, 4), round(measured_uwb_dist, 4), round(pos_error, 4),
                round(yaw_error, 4), round(inv_error, 4), self.upf.n_active_modes(),
                *fim_cols
            ])

        self.map_canvas.update_state(self.true_b_x, self.true_b_y, est_x, est_y, self.upf.particles)
        
        self.lbl_uwb.setText(f"UWB Measured Distance: {measured_uwb_dist:.3f} m")
        self.lbl_true.setText(f"True Pos (B): X: {self.true_b_x:.2f}m, Y: {self.true_b_y:.2f}m")
        self.lbl_est.setText(f"Estimated Pos (B): X: {est_x:.2f}m, Y: {est_y:.2f}m")
        self.lbl_err.setText(f"Position Error: {pos_error:.3f} m")
        
        if self.step_count % 10 == 0:
            log_html = (
                f"<span style='color:#00bcd4;'>[2D-UPF 50Hz]</span> Step:{self.step_count} | "
                f"Modes:{self.upf.n_active_modes()} | Dist: <b>{measured_uwb_dist:.3f}m</b><br>"
                f"<span style='color:#4caf50;'>[EST]</span> X:{est_x:.2f}, Y:{est_y:.2f} | "
                f"<span style='color:#f44336;'>[ERR]</span> {pos_error:.3f}m<br>"
                f"--------------------------------------------------<br>"
            )
            self.console.append(log_html)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    dashboard = ProUGVDashboard()
    dashboard.show()
    sys.exit(app.exec())
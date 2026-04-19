# Vispy, pyqtgraph

"""
Script to visualize 3D data points.

Receives data from a 3D coordinate reconstruction algorithm.
The data is in the format: (X, Y, Z, ID)

The system shows these points in a 3D space as red spheres, along with a label for each point
to shows their coordinates and ID. The data points are updated real-time to show
the markers movement.

The data points are updated at an ideal rate of 30 fps.

Among the main visualization space, the app should have other control options:
 - Data stream management:
    - Play/Pause/Stop
    - Data source: real time / csv
    - Real-time metrics: FPS, Network latency (ms), Dropped frames
 - Visualization options:
    - Show / Hide labels
    - Trajectory trails: a trail connects N closest points to track movement
    - View presets: Reset viewpoint to standard viewpoints:
        - Top (XY)
        - Front (XZ)
        - Side (YZ)
        - Free Roam
    - Show / Hide reference grid and axis
 - Record to CSV.
"""
import os
import sys

"""
xcb or X C Binding is the standard API core for X11 graphic system.
Sets QT_QPA_PLATFORMS to xcb forces PyQt5 to use X11 instead of the default Wayland, which
works poorly with PyQT5 and caused a "Attempt to retrieve context when no valid context" error.

xcb_egl, EGL is one of the two ways to connect OpenGL to the graphic window (the other is GLX).
Use EGL because it works, and GLX doesn't.
"""
os.environ["QT_QPA_PLATFORM"] = "xcb"
os.environ["QT_XCB_GL_INTEGRATION"] = "xcb_egl"

import time
import csv
from collections import deque
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QComboBox, QLabel, 
                             QCheckBox, QSlider, QGroupBox, QFormLayout) # GUI Components
from PyQt6.QtCore import QTimer, Qt 
from PyQt6.QtGui import QVector3D
import pyqtgraph as pg
import pyqtgraph.opengl as gl

# Demo
import math

class MoCapApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("3D MoCap Visualizer")
        self.setGeometry(100, 100, 1200, 800)

        # Cấu trúc dữ liệu nội bộ
        self.is_playing = False
        self.is_recording = False
        self.csv_file = None
        self.csv_writer = None
        self.last_time = time.time()
        
        self.marker_ids = [1, 2, 3]
        self.trail_length = 20
        self.history = {m_id: deque(maxlen=self.trail_length) for m_id in self.marker_ids}
        self.sim_t = 0.0 # Biến mô phỏng thời gian

        self.init_ui()

        # self.show()
        # self.view.makeCurrent()

        self.init_3d_environment()
        
        # Timer chạy ở lý tưởng 30 FPS (~33ms)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(33)

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        # --- BẢNG ĐIỀU KHIỂN (LEFT PANEL) ---
        control_panel = QWidget()
        control_panel.setFixedWidth(300)
        control_layout = QVBoxLayout(control_panel)

        # 1. Data Stream Management
        stream_group = QGroupBox("Data Stream Management")
        stream_layout = QVBoxLayout()
        
        btn_layout = QHBoxLayout()
        self.btn_play = QPushButton("Play")
        self.btn_pause = QPushButton("Pause")
        self.btn_stop = QPushButton("Stop")
        self.btn_play.clicked.connect(self.play_stream)
        self.btn_pause.clicked.connect(self.pause_stream)
        self.btn_stop.clicked.connect(self.stop_stream)
        btn_layout.addWidget(self.btn_play)
        btn_layout.addWidget(self.btn_pause)
        btn_layout.addWidget(self.btn_stop)
        
        self.combo_source = QComboBox()
        self.combo_source.addItems(["Real-time (Simulated)", "CSV Playback"])
        
        form_metrics = QFormLayout()
        self.lbl_fps = QLabel("0")
        self.lbl_latency = QLabel("0 ms (Placeholder)")
        self.lbl_drops = QLabel("0 (Placeholder)")
        form_metrics.addRow("FPS:", self.lbl_fps)
        form_metrics.addRow("Network Latency:", self.lbl_latency)
        form_metrics.addRow("Dropped Frames:", self.lbl_drops)

        stream_layout.addLayout(btn_layout)
        stream_layout.addWidget(QLabel("Data Source:"))
        stream_layout.addWidget(self.combo_source)
        stream_layout.addLayout(form_metrics)
        stream_group.setLayout(stream_layout)

        # 2. Visualization Options
        vis_group = QGroupBox("Visualization Options")
        vis_layout = QVBoxLayout()

        self.chk_labels = QCheckBox("Show Labels (Panel)")
        self.chk_labels.setChecked(True)
        self.chk_grid_axis = QCheckBox("Show Grid & Axis")
        self.chk_grid_axis.setChecked(True)
        self.chk_grid_axis.stateChanged.connect(self.toggle_grid_axis)

        self.slider_trail = QSlider(Qt.Orientation.Horizontal)
        self.slider_trail.setRange(0, 100)
        self.slider_trail.setValue(self.trail_length)
        self.lbl_trail_val = QLabel(f"Trail Length: {self.trail_length}")
        self.slider_trail.valueChanged.connect(self.update_trail_length)

        self.combo_view = QComboBox()
        self.combo_view.addItems(["Free Roam", "Top (XY)", "Front (XZ)", "Side (YZ)"])
        self.combo_view.currentIndexChanged.connect(self.change_view_preset)

        # Khung hiển thị text tọa độ do OpenGL khó render text 3D trực tiếp
        self.lbl_coordinates = QLabel("Marker Coordinates:\nWaiting for data...")
        self.lbl_coordinates.setStyleSheet("font-family: monospace; background: #eee; padding: 5px;")

        vis_layout.addWidget(self.chk_labels)
        vis_layout.addWidget(self.chk_grid_axis)
        vis_layout.addWidget(self.lbl_trail_val)
        vis_layout.addWidget(self.slider_trail)
        vis_layout.addWidget(QLabel("View Presets:"))
        vis_layout.addWidget(self.combo_view)
        vis_layout.addWidget(self.lbl_coordinates)
        vis_group.setLayout(vis_layout)

        # 3. Record Management
        rec_group = QGroupBox("Record Management")
        rec_layout = QVBoxLayout()
        self.btn_record = QPushButton("Start Record to CSV")
        self.btn_record.setCheckable(True)
        self.btn_record.clicked.connect(self.toggle_record)
        rec_layout.addWidget(self.btn_record)
        rec_group.setLayout(rec_layout)

        # Đưa vào control panel
        control_layout.addWidget(stream_group)
        control_layout.addWidget(vis_group)
        control_layout.addWidget(rec_group)
        control_layout.addStretch()

        # --- KHUNG HIỂN THỊ 3D (RIGHT PANEL) ---
        self.view = gl.GLViewWidget()
        self.view.opts['distance'] = 40

        main_layout.addWidget(control_panel)
        main_layout.addWidget(self.view, 1) # view chiếm tỷ lệ không gian lớn hơn

    def init_3d_environment(self):
        # Grid và Trục tọa độ
        self.grid = gl.GLGridItem(size=QVector3D(50,50,0) if hasattr(pg, 'QVector3D') else None)
        self.grid.scale(2, 2, 2)
        self.axis = gl.GLAxisItem()
        self.axis.setSize(10, 10, 10)
        self.view.addItem(self.grid)
        self.view.addItem(self.axis)

        # Vẽ các quả cầu đỏ (Markers) sử dụng GLMeshItem
        md = gl.MeshData.sphere(rows=10, cols=20)
        self.markers = {}
        for m_id in self.marker_ids:
            mesh = gl.GLMeshItem(meshdata=md, smooth=True, color=(1, 0, 0, 1), 
                                 shader='shaded', glOptions='opaque')
            mesh.scale(0.5, 0.5, 0.5) # Bán kính quả cầu
            self.markers[m_id] = mesh
            self.view.addItem(mesh)

        # Quỹ đạo di chuyển (Trails)
        self.trail_lines = {}
        for m_id in self.marker_ids:
            line = gl.GLLinePlotItem(color=pg.mkColor('r'), width=2, antialias=True)
            self.trail_lines[m_id] = line
            self.view.addItem(line)

    # --- CÁC HÀM XỬ LÝ SỰ KIỆN UI ---
    def play_stream(self):
        self.is_playing = True
        self.last_time = time.time()

    def pause_stream(self):
        self.is_playing = False

    def stop_stream(self):
        self.is_playing = False
        self.sim_t = 0.0
        for m_id in self.marker_ids:
            self.history[m_id].clear()
            self.trail_lines[m_id].setData(pos=np.empty((0,3)))
        self.lbl_coordinates.setText("Marker Coordinates:\nStopped.")

    def toggle_grid_axis(self):
        state = self.chk_grid_axis.isChecked()
        self.grid.setVisible(state)
        self.axis.setVisible(state)

    def update_trail_length(self):
        self.trail_length = self.slider_trail.value()
        self.lbl_trail_val.setText(f"Trail Length: {self.trail_length}")
        for m_id in self.marker_ids:
            self.history[m_id] = deque(self.history[m_id], maxlen=self.trail_length)

    def change_view_preset(self):
        preset = self.combo_view.currentText()
        if preset == "Top (XY)":
            self.view.setCameraPosition(elevation=90, azimuth=0)
        elif preset == "Front (XZ)":
            self.view.setCameraPosition(elevation=0, azimuth=90)
        elif preset == "Side (YZ)":
            self.view.setCameraPosition(elevation=0, azimuth=0)
        elif preset == "Free Roam":
            self.view.setCameraPosition(elevation=30, azimuth=45)

    def toggle_record(self):
        if self.btn_record.isChecked():
            self.btn_record.setText("Stop Recording")
            self.btn_record.setStyleSheet("background-color: red; color: white;")
            filename = f"mocap_record_{int(time.time())}.csv"
            self.csv_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["Timestamp", "ID", "X", "Y", "Z"])
            self.is_recording = True
        else:
            self.btn_record.setText("Start Record to CSV")
            self.btn_record.setStyleSheet("")
            self.is_recording = False
            if self.csv_file:
                self.csv_file.close()

    # --- HÀM GIẢ LẬP DỮ LIỆU ĐẦU VÀO ---
    def fetch_data(self):
        # Format trả về: [(X, Y, Z, ID), ...]
        self.sim_t += 0.05
        data = []
        # Chuyển động giả lập của 3 marker
        data.append((5 * math.cos(self.sim_t), 5 * math.sin(self.sim_t), 5, 1))
        data.append((10 * math.cos(self.sim_t + 2), 5, 10 * math.sin(self.sim_t + 2), 2))
        data.append((0, 8 * math.cos(self.sim_t * 1.5), 8 * math.sin(self.sim_t * 1.5), 3))
        return data

    # --- VÒNG LẶP CẬP NHẬT CHÍNH ---
    def update_loop(self):
        if not self.is_playing:
            return

        # Tính FPS
        current_time = time.time()
        dt = current_time - self.last_time
        if dt > 0:
            fps = 1.0 / dt
            self.lbl_fps.setText(f"{fps:.1f}")
        self.last_time = current_time

        # Lấy dữ liệu (Thay hàm này bằng lệnh đọc UDP socket trong thực tế)
        incoming_data = self.fetch_data()
        
        coord_text = "Marker Coordinates:\n"

        for x, y, z, m_id in incoming_data:
            # 1. Cập nhật vị trí quả cầu
            self.markers[m_id].resetTransform()
            self.markers[m_id].translate(x, y, z)

            # 2. Cập nhật quỹ đạo (Trails)
            self.history[m_id].append((x, y, z))
            if self.trail_length > 1 and len(self.history[m_id]) > 1:
                pts = np.array(self.history[m_id])
                self.trail_lines[m_id].setData(pos=pts)
            else:
                self.trail_lines[m_id].setData(pos=np.empty((0,3)))

            # 3. Gom chữ tạo Label
            coord_text += f"ID {m_id}: ({x:5.2f}, {y:5.2f}, {z:5.2f})\n"

            # 4. Lưu CSV
            if self.is_recording and self.csv_writer:
                self.csv_writer.writerow([current_time, m_id, x, y, z])

        # Cập nhật Label hiển thị (nếu checkbox đang bật)
        if self.chk_labels.isChecked():
            self.lbl_coordinates.setText(coord_text)
        else:
            self.lbl_coordinates.setText("Marker Coordinates:\n[Hidden]")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MoCapApp()
    window.show()
    sys.exit(app.exec())


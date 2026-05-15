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
import sys

"""
xcb or X C Binding is the standard API core for X11 graphic system.
Sets QT_QPA_PLATFORMS to xcb forces PyQt5 to use X11 instead of the default Wayland, which
works poorly with PyQT5 and caused a "Attempt to retrieve context when no valid context" error.

xcb_egl, EGL is one of the two ways to connect OpenGL to the graphic window (the other is GLX).
Use EGL because it works, and GLX doesn't.

PyQt6 doesn't have this issue, so ignore this part.
"""
# os.environ["QT_QPA_PLATFORM"] = "xcb"
# os.environ["QT_XCB_GL_INTEGRATION"] = "xcb_egl"

# tcp://127.0.0.1:5555

import time
import csv
from collections import deque
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QComboBox, QLabel, 
                             QCheckBox, QSlider, QGroupBox, QFormLayout, QListWidget,
                             QDialog, QFormLayout, QLineEdit, QComboBox, QAbstractItemView,
                             QDialogButtonBox, QListWidgetItem, QMessageBox) # GUI Components

from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QVector3D, QFont, QIcon, QPixmap, QPainter, QColor, QCloseEvent
import pyqtgraph as pg
import pyqtgraph.opengl as gl

import zmq
import struct

import math

MOCAP_CHANNEL = b'\x01'

class Marker():
    base_mesh_data = gl.MeshData.sphere(rows=10, cols=20)

    def __init__(self, id, color=(255, 0, 0, 255), scale=(0.5, 0.5, 0.5), trailLength=20, trailWidth=2, initCoord=(0, 0, 0)):
        self.color = color # RGBA format, default is red. Color for both marker and trail
        self.scale = scale # Default X, Y, Z scale of the sphere. Should be the same value
        self.id = id
        self.trailLength = trailLength
        self.trailHistory = deque(maxlen=trailLength)
        self.trailDrawer = gl.GLLinePlotItem(color=self._convertRGBA(color), width=trailWidth, antialias=True)

        self.coord = initCoord # X, Y, Z

        self.mesh = gl.GLMeshItem(
            meshdata=self.base_mesh_data, smooth=True, color=self._convertRGBA(color),
            shader='shaded', glOptions='opaque'
        )
        self.mesh.scale(scale[0], scale[1], scale[2])
    
    @staticmethod
    def _convertRGBA(rgba):
        # Convert 0 -> 255 value range to 0 -> 1 value range
        return (rgba[0] / 255, rgba[1] / 255, rgba[2] / 255, rgba[3] / 255)
    
    def changeColor(self, color):
        if color != self.color:
            self.mesh.setColor(*self._convertRGBA(color))

    def changeScale(self, scale):
        if self.scale != scale:
            new_scale = (new / old for new in scale for old in self.scale)
            self.mesh.scale(*new_scale)

    def changeTrailLength(self, length):
        if self.trailLength != length and length >= 0:
            self.trailLength = length
            self.trailHistory = deque(self.trailHistory, maxlen=length)

    def getMesh(self):
        return self.mesh
    
    def getTrailDrawer(self):
        return self.trailDrawer
    
    def getCoord(self):
        return self.coord
    
    def updateCoord(self, coord):
        self.coord = coord

class ConfigMarkerDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Add / Edit Marker")
        self.setMinimumWidth(300)

        layout = QFormLayout(self)

        self.id_field = QLineEdit()
        layout.addRow("Enter ID:", self.id_field)

        self.scale_field = QLineEdit()
        layout.addRow("Enter marker scale:", self.scale_field)

        RGBA_layout = QHBoxLayout()
        self.clr_R = QLineEdit()
        self.clr_R.setPlaceholderText("R value")
        self.clr_G = QLineEdit()
        self.clr_G.setPlaceholderText("G value")
        self.clr_B = QLineEdit()
        self.clr_B.setPlaceholderText("B value")
        self.clr_A = QLineEdit()
        self.clr_A.setPlaceholderText("A value")

        RGBA_layout.addWidget(self.clr_R)
        RGBA_layout.addWidget(self.clr_G)
        RGBA_layout.addWidget(self.clr_B)
        RGBA_layout.addWidget(self.clr_A)

        layout.addRow("Enter color value:", RGBA_layout)

        self.btn_box = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        
        self.btn_box.accepted.connect(self.accept) 
        self.btn_box.rejected.connect(self.reject)
        layout.addWidget(self.btn_box)

class SubCoordDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Enter subscribe data")
        self.setMinimumWidth(300)

        layout = QFormLayout(self)

        self.ip_field = QLineEdit()
        layout.addRow("Enter IP:", self.ip_field)

        self.port_field = QLineEdit()
        layout.addRow("Enter port number:", self.port_field)

        self.btn_box = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        
        self.btn_box.accepted.connect(self.accept) 
        self.btn_box.rejected.connect(self.reject)
        layout.addWidget(self.btn_box)

class MoCapApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("3D MoCap Visualizer")
        self.setGeometry(100, 100, 1200, 1000) # Set window size

        self.is_playing = False
        self.is_recording = False
        self.csv_file = None
        self.csv_writer = None
        self.last_time = time.time() # Save the time of the previous frame. Used in fps calculation
        
        self.markers = {} # A dict of markers, marker id - marker as the key - value pair
        self.common_trailLength = 20

        self.sim_t = 0.0 # To create sim data

        # Store previous fps value to calculate average
        self.fpsQueueLen = 30
        self.fpsHistory = deque([0] * self.fpsQueueLen, maxlen=self.fpsQueueLen)

        self.init_ui() # Build the 2D UI
        self.init_3d_environment()
        
        # Start a timer that calls update_loop() every 33 ms (30 fps refresh rate)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(33)

    def init_ui(self):
        """
        QMainWindow <- QWidget (main widget) <- Main layout <- Sub widget <- Sub widget layout <- Sub layout of sub widget layout <- Controller widgets (buttons, ...)
        """
        main_widget = QWidget() # A platform to build the UI
        self.setCentralWidget(main_widget) # Align to the center of QMainWindow
        main_layout = QHBoxLayout(main_widget) # Create a horizontal layout

        # Build the left-side control panel
        control_panel = QWidget()
        control_panel.setFixedWidth(300)
        control_layout = QVBoxLayout(control_panel) # The control panel has a vertical layout

        # Data stream management section
        stream_group = QGroupBox("Data Stream Management") # Create a labeled frame
        stream_layout = QVBoxLayout()
        
        btn_layout = QHBoxLayout()
        self.btn_play = QPushButton("Play")
        self.btn_pause = QPushButton("Pause")
        self.btn_stop = QPushButton("Stop")
        self.btn_play.clicked.connect(self.play_stream) # Connect the button's press signal to a function (slot)
        self.btn_pause.clicked.connect(self.pause_stream)
        self.btn_stop.clicked.connect(self.stop_stream)
        btn_layout.addWidget(self.btn_play)
        btn_layout.addWidget(self.btn_pause)
        btn_layout.addWidget(self.btn_stop)
        
        self.combo_source = QComboBox() # Create a drop down menu
        self.sub_context = None
        self.sub = None
        self.combo_source.addItems(["Simulation", "Real-time", "CSV Playback"])
        self.combo_source.currentIndexChanged.connect(self.call_sub_coord_prompt)
        
        form_metrics = QFormLayout() # A layout for key - value type display
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

        # Visualization options
        vis_group = QGroupBox("Visualization Options")
        vis_layout = QVBoxLayout()

        self.chk_labels = QCheckBox("Show Labels (Panel)")
        self.chk_labels.setChecked(True)
        self.chk_grid_axis = QCheckBox("Show Grid and Axis")
        self.chk_grid_axis.setChecked(True)
        self.chk_grid_axis.stateChanged.connect(self.toggle_grid_axis)

        self.slider_trail = QSlider(Qt.Orientation.Horizontal)
        self.slider_trail.setRange(0, 100)
        self.slider_trail.setValue(self.common_trailLength)
        self.lbl_trail_val = QLabel(f"Trail Length: {self.common_trailLength}")
        self.slider_trail.valueChanged.connect(self.update_trail_length)

        self.combo_view = QComboBox()
        self.combo_view.addItems(["Free Roam", "Top (XY)", "Front (XZ)", "Side (YZ)"])
        self.combo_view.currentIndexChanged.connect(self.change_view_preset)

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

        # Marker management
        marker_grp = QGroupBox("Maker Management")
        marker_layout = QVBoxLayout()
        self.lbl_list = QLabel(f"Marker list: {len(self.markers)} running marker(s)")
        self.marker_list = QListWidget()
        self.marker_list.setSelectionMode(QAbstractItemView.SelectionMode.MultiSelection)

        marker_btn_layout = QHBoxLayout()
        self.btn_add_marker = QPushButton("Add / Config")
        self.btn_delete_marker = QPushButton("Delete")
        self.btn_add_marker.clicked.connect(self.call_add_config_prompt)
        self.btn_delete_marker.clicked.connect(self.del_marker)
        marker_btn_layout.addWidget(self.btn_add_marker)
        marker_btn_layout.addWidget(self.btn_delete_marker)

        marker_layout.addWidget(self.lbl_list)
        marker_layout.addWidget(self.marker_list)
        marker_layout.addLayout(marker_btn_layout)

        marker_grp.setLayout(marker_layout)

        # Record management
        rec_group = QGroupBox("Record Management")
        rec_layout = QVBoxLayout()
        self.btn_record = QPushButton("Start Record to CSV")
        self.btn_record.setCheckable(True) # Button becomes a switch (toggle)
        self.btn_record.clicked.connect(self.toggle_record)
        rec_layout.addWidget(self.btn_record)
        rec_group.setLayout(rec_layout)

        # Assemble
        control_layout.addWidget(stream_group)
        control_layout.addWidget(marker_grp)
        control_layout.addWidget(vis_group)
        control_layout.addWidget(rec_group)
        control_layout.addStretch() # Push everything to the top, avoid sparse layout when widening the window 

        # 3D Visual space
        self.view = gl.GLViewWidget() # Create an OpenGL view widget
        self.view.opts['distance'] = 100

        main_layout.addWidget(control_panel)
        main_layout.addWidget(self.view, 1)

    def init_3d_environment(self):
        # Init grid
        self.grid = gl.GLGridItem(size=QVector3D(50,50,0) if hasattr(pg, 'QVector3D') else None)
        self.grid.scale(2, 2, 2)

        # Init axis
        axis_length = 40
        line_width = 3
        
        # Khởi tạo một đối tượng Font với kích thước to rõ (ví dụ size 20)
        label_font = QFont('Arial', 20) 
        label_font.setBold(True)

        # Trục X (Đỏ)
        x_line = gl.GLLinePlotItem(pos=np.array([[0,0,0], [axis_length,0,0]]), color=(1,0,0,1), width=line_width)
        self.view.addItem(x_line)
        x_label = gl.GLTextItem(pos=[axis_length + 0.5, 0, 0], text='X', color=pg.mkColor('r'), font=label_font)
        self.view.addItem(x_label)

        # Trục Y (Xanh lá)
        y_line = gl.GLLinePlotItem(pos=np.array([[0,0,0], [0,axis_length,0]]), color=(0,1,0,1), width=line_width)
        self.view.addItem(y_line)
        y_label = gl.GLTextItem(pos=[0, axis_length + 0.5, 0], text='Y', color=pg.mkColor('g'), font=label_font)
        self.view.addItem(y_label)

        # Trục Z (Xanh dương)
        z_line = gl.GLLinePlotItem(pos=np.array([[0,0,0], [0,0,axis_length]]), color=(0,0,1,1), width=line_width)
        self.view.addItem(z_line)
        z_label = gl.GLTextItem(pos=[0, 0, axis_length + 0.5], text='Z', color=pg.mkColor('b'), font=label_font)
        self.view.addItem(z_label)

        self.view.addItem(self.grid)

    # UI event processing functions
    def sub_coord(self):
        ip = self.sub_dialog.ip_field.text()
        port = self.sub_dialog.port_field.text()

        self.sub_context = zmq.Context()
        self.sub = self.sub_context.socket(zmq.SUB)
        try:
            print(f"[ZMQ] Estabilishing connection to tcp://{ip}:{port}")
            self.sub.connect(f"tcp://{ip}:{port}")
        except zmq.error.ZMQError:
            print(f"[ZMQ] Failed to establish connection")
            QMessageBox.warning(self, "Unable to connect", "Check your IP and port")
            self.sub_context = None
            self.sub = None
        else:
            print(f"[ZMQ] Set header")
            self.sub.setsockopt(zmq.SUBSCRIBE, MOCAP_CHANNEL)
            print(f"[ZMQ] Done setting header")

    def unsub_coord(self):
        if self.sub != None and self.sub_context != None:
            ip = self.sub_dialog.ip_field.text()
            port = self.sub_dialog.port_field.text()
            try:
                self.sub.disconnect(f"tcp://{ip}:{port}")
            except zmq.error.ZMQError:
                pass

            QMessageBox.warning(self, "Unsub", f"Disconnected from {ip}:{port}")

            self.sub_context = None
            self.sub = None

    def call_sub_coord_prompt(self):
        option = self.combo_source.currentText()

        if option == "Real-time":
            self.sub_dialog = SubCoordDialog()
            self.sub_dialog.accepted.connect(self.sub_coord)
            self.sub_dialog.show()
        else:
            self.unsub_coord()

    def play_stream(self):
        self.is_playing = True
        self.last_time = time.time()

    def pause_stream(self):
        self.is_playing = False

    def stop_stream(self):
        self.is_playing = False
        self.sim_t = 0.0

        # Clear trails and trail history
        for marker in self.markers.values():
            marker.trailHistory.clear()
            marker.trailDrawer.setData(pos=np.empty(0, 3))
        self.lbl_coordinates.setText("Marker Coordinates:\nStopped.")

    def _create_color_icon(self, rgba):
        pixmap = QPixmap(24, 24)
        pixmap.fill(Qt.GlobalColor.transparent) 
        
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        qt_color = QColor(*rgba)
        
        painter.setBrush(qt_color)
        painter.setPen(Qt.PenStyle.NoPen)
        
        painter.drawEllipse(0, 0, 24, 24)
        
        painter.end()
        
        return QIcon(pixmap)

    def _apply_marker_add_config(self):
        # Hàm này chỉ được kích hoạt khi người dùng bấm "OK" trên cái Dialog kia
        
        # Trích xuất dữ liệu từ các Widget bên trong Dialog
        new_id = int(self.config_dialog.id_field.text())
        new_scale = float(self.config_dialog.scale_field.text())
        clr_R = int(self.config_dialog.clr_R.text())
        clr_G = int(self.config_dialog.clr_G.text())
        clr_B = int(self.config_dialog.clr_B.text())
        clr_A = int(self.config_dialog.clr_A.text())

        if new_id not in self.markers.keys(): # Add
            new_marker = Marker(
                id=new_id, color=(clr_R, clr_G, clr_B, clr_A),
                scale=(new_scale, new_scale, new_scale), trailLength=self.common_trailLength
            )
            self.markers[new_id] = new_marker
            self.view.addItem(new_marker.getMesh())
            self.view.addItem(new_marker.trailDrawer)

            # Create list item
            new_item = QListWidgetItem(f"Marker ID {new_id}")
            color_icon = self._create_color_icon((clr_R, clr_G, clr_B, clr_A))
            new_item.setIcon(color_icon)
            new_item.setData(Qt.ItemDataRole.UserRole, new_id)
            self.marker_list.addItem(new_item)

        elif new_id in self.markers.keys(): # Config
            self.markers[new_id].getMesh().changeColor((clr_R, clr_G, clr_B, clr_A))
            self.markers[new_id].getMesh().changeScale((new_scale, new_scale, new_scale))

            color_icon = self._create_color_icon((clr_R, clr_G, clr_B, clr_A))
            for i in range(self.marker_list.count()):
                item = self.marker_list.item(i)

                if item.data(Qt.ItemDataRole.UserRole) == new_id:
                    item.setIcon(color_icon)
                    break
            
    def call_add_config_prompt(self):
        self.config_dialog = ConfigMarkerDialog()
        self.config_dialog.accepted.connect(self._apply_marker_add_config)
        self.config_dialog.show()

    def del_marker(self):
        selected_items = self.marker_list.selectedItems()

        if not selected_items:
            return
        
        for item in selected_items:
            id = item.data(Qt.ItemDataRole.UserRole)

            if id in self.markers.keys():
                # Remove the sphere
                self.view.removeItem(self.markers[id].getMesh())

                # Remove the trail drawer
                self.view.removeItem(self.markers[id].getTrailDrawer())

                # Remove the drawer object
                del self.markers[id]
            
            # Remove item from list
            row = self.marker_list.row(item)
            self.marker_list.takeItem(row)

    def toggle_grid_axis(self):
        state = self.chk_grid_axis.isChecked()
        self.grid.setVisible(state)

    def update_trail_length(self):
        self.common_trailLength = self.slider_trail.value()
        self.lbl_trail_val.setText(f"Trail Length: {self.common_trailLength}")

        for marker in self.markers.values():
            marker.changeTrailLength(self.common_trailLength)

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

    # Data simulation
    def fetch_data_sim(self):
        self.sim_t += 0.05
        data = []

        data.append((5 * math.cos(self.sim_t), 5 * math.sin(self.sim_t), 5, 1))
        data.append((10 * math.cos(self.sim_t + 2), 5, 10 * math.sin(self.sim_t + 2), 2))
        data.append((0, 8 * math.cos(self.sim_t * 1.5), 8 * math.sin(self.sim_t * 1.5), 3))

        return data

    def fetch_data_sub(self):
        data = []
        
        if self.sub is None:
            return data

        try:
            while True: # recv_multipart will repeatedly take out data, and invoke an exception when all data is received.
                parts = self.sub.recv_multipart(flags=zmq.NOBLOCK) # NOBLOCK => recv doesn't block the program
                # Check if there's data, if not, evoke zmq.error.Again
                
                topic = parts[0]
                raw_bytes = parts[1]

                if topic == MOCAP_CHANNEL:
                    unpacked_data = struct.unpack("<i3f", raw_bytes)
                    
                    m_id = unpacked_data[0]
                    x = unpacked_data[1]    
                    y = unpacked_data[2]    
                    z = unpacked_data[3]
                    
                    data.append((x, y, z, m_id))
                    print(f"Received data: {(x, y, z, m_id)}")
                    
        except zmq.error.Again:
            pass
        except Exception as e:
            print(f"Unable to read byte stream: {e}")

        return data

    def fetch_data_csv(self):
        return (0, 0, 0, 0)

    def update_loop(self):
        if not self.is_playing:
            return

        # Calc fps
        current_time = time.time()
        dt = current_time - self.last_time
        if dt > 0:
            fps = 1.0 / dt
            self.fpsHistory.append(fps)
            avg_fps = sum(self.fpsHistory) / self.fpsQueueLen
            self.lbl_fps.setText(f"{avg_fps:.1f}")
        self.last_time = current_time

        # Fetch data
        data_src = self.combo_source.currentText()
        if data_src == "Real-time":
            incoming_data = self.fetch_data_sub()
        elif data_src == "Simulation":
            incoming_data = self.fetch_data_sim()
        elif data_src == "CSV Playback":
            incoming_data = self.fetch_data_csv()
        
        coord_text = "Marker Coordinates:\n"

        available_id = self.markers.keys()

        for x, y, z, m_id in incoming_data:
            if m_id in available_id:
                # Update sphere position
                oldX, oldY, oldZ = self.markers[m_id].getCoord()
                dx, dy, dz = (x - oldX, y - oldY, z - oldZ)
                self.markers[m_id].getMesh().translate(dx, dy, dz)

                self.markers[m_id].updateCoord((x, y, z))

                # Update trail
                self.markers[m_id].trailHistory.append((x, y, z))
                if self.common_trailLength > 1 and len(self.markers[m_id].trailHistory) > 1:
                    pts = np.array(self.markers[m_id].trailHistory)
                    self.markers[m_id].trailDrawer.setData(pos=pts)
                else:
                    self.markers[m_id].trailDrawer.setData(pos=np.empty((0,3)))

                # Create coord label
                coord_text += f"ID {m_id}: ({x:.2f}, {y:.2f}, {z:.2f})\n"

                # Save into csv
                if self.is_recording and self.csv_writer:
                    self.csv_writer.writerow([current_time, m_id, x, y, z])

        # Update coordinate label
        if self.chk_labels.isChecked():
            self.lbl_coordinates.setText(coord_text)
        else:
            self.lbl_coordinates.setText("Marker Coordinates:\n[Hidden]")

    # Clean up
    def closeEvent(self, event: QCloseEvent):
        if self.sub is not None:
            self.sub.close()
        
        if self.sub_context is not None:
            self.sub_context.term()

        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv) # QApplication manages the event loop, initialize the application, and other management tasks
    window = MoCapApp()
    window.show()
    sys.exit(app.exec()) # app.exec() the event loop

"""
#include <zmq.hpp>
#include <iostream>
#include <cstdint> // Bắt buộc để dùng int32_t (đảm bảo đúng 4 byte)

// 1. ĐỊNH NGHĨA CẤU TRÚC DỮ LIỆU (Ép không cho C++ chèn byte rác)
#pragma pack(push, 1)
struct MoCapPoint {
    int32_t id;      // 4 bytes
    float x;         // 4 bytes
    float y;         // 4 bytes
    float z;         // 4 bytes
};                   // Tổng cộng: ĐÚNG 16 bytes
#pragma pack(pop)

int main() {
    zmq::context_t context(1);
    zmq::socket_t publisher(context, ZMQ_PUB);
    publisher.bind("tcp://127.0.0.1:5555");

    while (true) {
        // Tạo dữ liệu giả lập
        MoCapPoint point = {1, 1.5f, 2.0f, 3.5f};

        // 2. GỬI PHẦN 1: Tên kênh (Label)
        std::string topic = "MOCAP_DATA";
        zmq::message_t msg_topic(topic.size());
        memcpy(msg_topic.data(), topic.data(), topic.size());
        
        // Cờ ZMQ_SNDMORE báo cho ZMQ biết: "Khoan gửi, tôi còn đính kèm file"
        publisher.send(msg_topic, zmq::send_flags::sndmore); 

        // 3. GỬI PHẦN 2: Dữ liệu nhị phân (Binary Payload)
        zmq::message_t msg_payload(sizeof(MoCapPoint));
        memcpy(msg_payload.data(), &point, sizeof(MoCapPoint)); // Lấy thẳng địa chỉ RAM của struct ném vào
        
        publisher.send(msg_payload, zmq::send_flags::none); // Gửi toàn bộ đi
    }
    return 0;
}
"""

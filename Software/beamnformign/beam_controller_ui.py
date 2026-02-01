import sys
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QComboBox, QCheckBox, QGridLayout
)
from PyQt5.QtCore import Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from hardware_control import HardwareControl

class BeamformingGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Beamforming Controller")
        self.hw_control = HardwareControl()
        self.initUI()

    def initUI(self):
        # Main widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Steering Angle Slider
        angle_layout = QHBoxLayout()
        self.angle_label = QLabel("Steering Angle: 0°")
        self.angle_slider = QSlider(Qt.Horizontal)
        self.angle_slider.setRange(-90, 90)
        self.angle_slider.setValue(0)
        self.angle_slider.valueChanged.connect(self.update_beam)
        angle_layout.addWidget(self.angle_label)
        angle_layout.addWidget(self.angle_slider)
        main_layout.addLayout(angle_layout)

        # Array Configuration Dropdown
        config_layout = QHBoxLayout()
        self.config_label = QLabel("Array Configuration:")
        self.config_combo = QComboBox()
        self.config_combo.addItems(["4-element", "8-element"])
        self.config_combo.setCurrentText("8-element")
        self.config_combo.currentIndexChanged.connect(self.update_beam)
        config_layout.addWidget(self.config_label)
        config_layout.addWidget(self.config_combo)
        main_layout.addLayout(config_layout)

        # Tapering Dropdown
        taper_layout = QHBoxLayout()
        self.taper_label = QLabel("Tapering:")
        self.taper_combo = QComboBox()
        self.taper_combo.addItems(["Uniform", "Hamming", "Hann"])
        self.taper_combo.currentIndexChanged.connect(self.update_beam)
        taper_layout.addWidget(self.taper_label)
        taper_layout.addWidget(self.taper_combo)
        main_layout.addLayout(taper_layout)

        # Nulling
        nulling_layout = QHBoxLayout()
        self.nulling_checkbox = QCheckBox("Enable Nulling")
        self.nulling_checkbox.stateChanged.connect(self.update_beam)
        self.nulling_angle_label = QLabel("Nulling Angle: 0°")
        self.nulling_angle_slider = QSlider(Qt.Horizontal)
        self.nulling_angle_slider.setRange(-90, 90)
        self.nulling_angle_slider.setValue(0)
        self.nulling_angle_slider.valueChanged.connect(self.update_beam)
        nulling_layout.addWidget(self.nulling_checkbox)
        nulling_layout.addWidget(self.nulling_angle_label)
        nulling_layout.addWidget(self.nulling_angle_slider)
        main_layout.addLayout(nulling_layout)

        # Live value tracking
        self.live_values_layout = QGridLayout()
        self.live_value_labels = []
        for i in range(8):
            element_label = QLabel(f"Element {i+1}:")
            phase_label = QLabel("Phase: 0.0°")
            atten_label = QLabel("Atten: 0.0 dB")
            self.live_values_layout.addWidget(element_label, i, 0)
            self.live_values_layout.addWidget(phase_label, i, 1)
            self.live_values_layout.addWidget(atten_label, i, 2)
            self.live_value_labels.append((element_label, phase_label, atten_label))
        main_layout.addLayout(self.live_values_layout)

        # Matplotlib Polar Plot
        self.plot_canvas = MplCanvas(self, width=5, height=4, dpi=100)
        main_layout.addWidget(self.plot_canvas)

        self.update_beam()

    def update_beam(self):
        steering_angle = self.angle_slider.value()
        self.angle_label.setText(f"Steering Angle: {steering_angle}°")
        
        tapering = self.taper_combo.currentText()
        config = self.config_combo.currentText()
        N = 8 if config == "8-element" else 4
        d_lambda_ratio = 0.5

        nulling_enabled = self.nulling_checkbox.isChecked()
        nulling_angle = self.nulling_angle_slider.value()
        self.nulling_angle_label.setText(f"Nulling Angle: {nulling_angle}°")

        # Calculate steering vector for main direction
        theta_rad = np.deg2rad(steering_angle)
        a_theta0 = np.exp(1j * 2 * np.pi * d_lambda_ratio * np.sin(theta_rad) * np.arange(N))

        if nulling_enabled:
            # Calculate steering vector for null direction
            null_rad = np.deg2rad(nulling_angle)
            a_theta1 = np.exp(1j * 2 * np.pi * d_lambda_ratio * np.sin(null_rad) * np.arange(N))
            
            # Zero-forcing
            inner_prod_01 = np.dot(np.conj(a_theta1), a_theta0)
            inner_prod_11 = np.dot(np.conj(a_theta1), a_theta1)
            weights = a_theta0 - (inner_prod_01 / inner_prod_11) * a_theta1
            
            phase_shifts_rad = np.angle(weights)
            # Use the magnitude of the complex weights for attenuation
            amplitude_weights = np.abs(weights)
        else:
            # Standard beamforming (no nulling)
            phase_shifts_rad = 2 * np.pi * d_lambda_ratio * np.sin(theta_rad) * np.arange(N)
            if tapering == "Hamming":
                amplitude_weights = np.hamming(N)
            elif tapering == "Hann":
                amplitude_weights = np.hanning(N)
            else:  # Uniform
                amplitude_weights = np.ones(N)

        phase_shifts_deg = np.rad2deg(phase_shifts_rad)
        attenuations = -20 * np.log10(amplitude_weights / np.max(amplitude_weights))

        # Update live value labels
        for i in range(8):
            if i < N:
                self.live_value_labels[i][0].setVisible(True)
                self.live_value_labels[i][1].setVisible(True)
                self.live_value_labels[i][2].setVisible(True)
                self.live_value_labels[i][1].setText(f"Phase: {phase_shifts_deg[i]:.1f}°")
                self.live_value_labels[i][2].setText(f"Atten: {attenuations[i]:.1f} dB")
            else:
                self.live_value_labels[i][0].setVisible(False)
                self.live_value_labels[i][1].setVisible(False)
                self.live_value_labels[i][2].setVisible(False)

        # Update hardware and plot
        if self.hw_control.spi:
            self.update_hardware(phase_shifts_deg, attenuations)
        self.plot_canvas.update_plot(N, d_lambda_ratio, phase_shifts_rad, amplitude_weights)

    def update_hardware(self, phase_shifts_deg, attenuations):
        N = 8 if self.config_combo.currentText() == "8-element" else 4
        
        for i in range(N):
            if i < 4:
                phase_value = int(round((phase_shifts_deg[i] % 360) / 5.6))
                self.hw_control.set_phase(i + 1, phase_value)

                attenuation_db = attenuations[i]
                gain_value = int(round(attenuation_db / 0.25))
                gain_value = max(0, min(127, gain_value))
                self.hw_control.set_gain(i + 1, gain_value)

    def closeEvent(self, event):
        if self.hw_control.spi:
            self.hw_control.deinitialize_hardware()
        super().closeEvent(event)

class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111, polar=True)
        super().__init__(fig)
        self.setParent(parent)

    def update_plot(self, N, d_lambda_ratio, phase_shifts_rad, weights):
        self.axes.clear()
        theta = np.linspace(-np.pi/2, np.pi/2, 360)
        af = np.zeros_like(theta, dtype=np.complex64)

        for i in range(N):
            af += weights[i] * np.exp(1j * (2 * np.pi * i * d_lambda_ratio * np.sin(theta) - phase_shifts_rad[i]))

        af_mag_db = 20 * np.log10(np.abs(af) / np.max(np.abs(af)))
        af_mag_db = np.clip(af_mag_db, -50, 0)

        self.axes.plot(theta, af_mag_db)
        self.axes.set_theta_zero_location("N")
        self.axes.set_thetamin(-90)
        self.axes.set_thetamax(90)
        self.axes.set_rlim([-50, 0])
        self.axes.set_title("Array Factor")
        self.draw()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_win = BeamformingGUI()
    main_win.show()
    sys.exit(app.exec_())
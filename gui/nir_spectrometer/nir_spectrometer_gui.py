# nir_spectrometer_gui.py
import os
import pyqtgraph as pg
from qtpy import QtWidgets, uic
from core.connector import Connector
from gui.guibase import GUIBase


class NIRSpectrometerGui(GUIBase):
    """
    GUI for the NIR Spectrometer.
    """
    spectrometer_logic = Connector(interface='NIRSpectrometerLogic')

    def on_activate(self):
        self._logic = self.spectrometer_logic()

        # Get the path to the *.ui file
        this_dir = os.path.dirname(__file__)
        ui_file = os.path.join(this_dir, 'nir_spectrometer.ui')

        # Load it
        self._mw = uic.loadUi(ui_file)

        # --- Plot setup ---
        # The plot widget is named 'plot_widget' in the .ui file
        self.spectrum_plot = self._mw.plot_widget.plot(pen='g')
        self._mw.plot_widget.setLabel('bottom', 'Position', units='m')
        self._mw.plot_widget.setLabel('left', 'Intensity', units='counts/s')
        self._mw.plot_widget.showGrid(x=True, y=True)

        # --- Set initial values from logic ---
        self.update_gui_parameters()

        # --- Connect GUI widget signals to logic slots/methods ---
        self._mw.start_button.clicked.connect(self._logic.start_scan)
        self._mw.stop_button.clicked.connect(self._logic.stop_scan)
        self._mw.save_button.clicked.connect(self.save_data)
        self._mw.apply_calib_button.clicked.connect(self.apply_calibration)
        self._mw.calib_group.toggled.connect(self.apply_calibration)

        spinboxes = [
            self._mw.start_pos_spinbox, self._mw.stop_pos_spinbox,
            self._mw.num_points_spinbox, self._mw.time_per_point_spinbox
        ]
        for spinbox in spinboxes:
            # Using editingFinished is safer to avoid sending too many signals while typing
            if hasattr(spinbox, 'editingFinished'):
                spinbox.editingFinished.connect(self.set_scan_parameters)
            else:  # for QSpinBox
                spinbox.valueChanged.connect(self.set_scan_parameters)

        # --- Connect logic signals to GUI slots ---
        self._logic.sigSpectrumUpdated.connect(self.update_plot)
        self._logic.sigStatusUpdated.connect(self.update_status)

        # Note: The logic needs a 'sigScanStarted' signal for this to work perfectly
        # It's good practice to add `sigScanStarted = QtCore.Signal()` to the logic class
        # and emit it at the beginning of the `start_scan` method.
        if hasattr(self._logic, 'sigScanStarted'):
            self._logic.sigScanStarted.connect(self.on_scan_started)

        self._logic.sigScanFinished.connect(self.on_scan_finished)

        # Set initial button state
        self._mw.stop_button.setEnabled(False)

        self.show()

    def on_deactivate(self):
        self._mw.close()

    def show(self):
        self._mw.show()
        self._mw.activateWindow()
        self._mw.raise_()

    def update_plot(self, x_data, y_data):
        self.spectrum_plot.setData(x_data, y_data)
        if self._mw.calib_group.isChecked():
            self._mw.plot_widget.setLabel('bottom', 'Wavelength', units='nm')
        else:
            self._mw.plot_widget.setLabel('bottom', 'Position', units='m')

    def update_status(self, message):
        self._mw.statusbar.showMessage(message, 5000)  # Show for 5 seconds

    def on_scan_started(self):
        self._mw.start_button.setEnabled(False)
        self._mw.stop_button.setEnabled(True)
        self.update_status("Scan started...")

    def on_scan_finished(self):
        self._mw.start_button.setEnabled(True)
        self._mw.stop_button.setEnabled(False)

    def set_scan_parameters(self):
        self._logic.set_scan_parameters(
            start=self._mw.start_pos_spinbox.value(),
            stop=self._mw.stop_pos_spinbox.value(),
            points=self._mw.num_points_spinbox.value(),
            time_pp=self._mw.time_per_point_spinbox.value()
        )

    def apply_calibration(self):
        params = {
            'pos1': self._mw.calib_pos1_spinbox.value(),
            'wl1': self._mw.calib_wl1_spinbox.value(),
            'pos2': self._mw.calib_pos2_spinbox.value(),
            'wl2': self._mw.calib_wl2_spinbox.value(),
            'use_calib': self._mw.calib_group.isChecked()
        }
        self._logic.set_calibration(params)

    def save_data(self):
        filetag = self._mw.filetag_lineedit.text()
        self._logic.save_data(filetag)
        self.update_status("Data saved.")

    def update_gui_parameters(self):
        """Sets the GUI widgets to match the current logic parameters."""
        # Block signals to prevent feedback loops on initialization
        widgets_to_block = [
            self._mw.start_pos_spinbox, self._mw.stop_pos_spinbox, self._mw.num_points_spinbox,
            self._mw.time_per_point_spinbox, self._mw.calib_pos1_spinbox, self._mw.calib_wl1_spinbox,
            self._mw.calib_pos2_spinbox, self._mw.calib_wl2_spinbox, self._mw.calib_group
        ]
        for widget in widgets_to_block:
            widget.blockSignals(True)

        self._mw.start_pos_spinbox.setValue(self._logic.start_position)
        self._mw.stop_pos_spinbox.setValue(self._logic.stop_position)
        self._mw.num_points_spinbox.setValue(self._logic.num_points)
        self._mw.time_per_point_spinbox.setValue(self._logic.time_per_point)
        self._mw.calib_pos1_spinbox.setValue(self._logic.calib_pos1)
        self._mw.calib_wl1_spinbox.setValue(self._logic.calib_wl1)
        self._mw.calib_pos2_spinbox.setValue(self._logic.calib_pos2)
        self._mw.calib_wl2_spinbox.setValue(self._logic.calib_wl2)
        self._mw.calib_group.setChecked(self._logic.use_calibration)

        # Unblock signals
        for widget in widgets_to_block:
            widget.blockSignals(False)
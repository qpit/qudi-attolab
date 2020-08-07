# -*- coding: utf-8 -*-
"""
This file contains the Qudi GUI module to operate the voltage (laser) scanner.

Qudi is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Qudi is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Qudi. If not, see <http://www.gnu.org/licenses/>.

Copyright (c) the Qudi Developers. See the COPYRIGHT.txt file at the
top-level directory of this distribution and at <https://github.com/Ulm-IQO/qudi/>
"""

import os
import time
from core.module import Connector
from gui.guibase import GUIBase
from gui.colordefs import QudiPalettePale as Palette
from qtpy import QtWidgets
from qtpy import uic
import pyqtgraph as pg


class MainWindow(QtWidgets.QMainWindow):
    """ Create the Main Window based on the corresponding *.ui file. """

    def __init__(self):
        # Get the path to the *.ui file
        this_dir = os.path.dirname(__file__)
        ui_file = os.path.join(this_dir, 'ui_laserpowercontrolgui.ui')
        self._doubleclicked = False

        # Load it
        super(MainWindow, self).__init__()
        uic.loadUi(ui_file, self)
        self.show()
        self.move(280, 0)


class PowerStabilizationGui(GUIBase):
    """ Main Gui Class for fiber shooting experiment."""
    _modclass = 'fiber_shooting_GUI'
    _modtype = 'gui'

    # declare connectors
    power_stabilization_logic = Connector(interface='PowerStabilizationLogic')

    def show(self):
        """Make main window visible and put it above all other windows. """
        # Show the Main Confocal GUI:
        self._mw.show()
        self._mw.activateWindow()
        self._mw.raise_()

    def on_activate(self):
        """ Initializes all needed UI files and establishes the connectors.
        This method executes the all the inits for the differnt GUIs and passes
        the event argument from fysom to the methods.
        """
        # Getting an access to all connectors:
        self._power_stabilization_logic = self.power_stabilization_logic()
        self.init_main_ui()  # initialize the main GUI

    def init_main_ui(self):
        """ Definition, configuration and initialisation of the confocal GUI.
        This init connects all the graphic modules, which were created in the
        *.ui file and configures the event handling between the modules.
        Moreover it sets default values.
        """
        self._mw = MainWindow()

        self.pid_status = False  # Laser OFF
        self.setpoint = 0

        self.power_data = []
        self.time_data = []
        self._pw = None
        self.curve = []

        self.acquisition_time = 60
        self.beam_splitter_coef = 1

        self.time_start = 0


        self._mw.Kp_doubleSpinBox.setValue(self.get_kp())
        self._mw.Ki_doubleSpinBox.setValue(self.get_ki())
        self._mw.Kd_doubleSpinBox.setValue(self.get_kd())


        self._mw.acquisition_time_spinBox.setValue(10)

        # graph

        self._pw = self._mw.power_PlotWidget
        self._pw.setLabel('left', 'Power', units='W')
        self._pw.setLabel('bottom', 'Time', units='s')
        self._pw.showGrid(x=True, y=True)

        self.curve.append(pg.PlotDataItem(pen=pg.mkPen(Palette.c1), symbol=None))
        self._pw.addItem(self.curve[-1])
        self.curve.append(pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen(Palette.c2)))
        self._pw.addItem(self.curve[-1])

        # Laser's connectors
        self._mw.PID_ON_checkBox.clicked.connect(self.switch_pid)
        self._mw.PID_ON_checkBox.clicked.connect(self.set_pid_status)
        self._mw.setpoint_spinBox.editingFinished.connect(self.set_setpoint)
        self._mw.Kp_doubleSpinBox.editingFinished.connect(self.set_kp)
        self._mw.Ki_doubleSpinBox.editingFinished.connect(self.set_ki)
        self._mw.Kd_doubleSpinBox.editingFinished.connect(self.set_kd)
        # Graph's connectors
        self._mw.acquisition_time_spinBox.editingFinished.connect(self.set_acquisition_time)
        # Handling signals from the logic
        self._power_stabilization_logic.sigPowerUpdated.connect(self.update_data)

    def on_deactivate(self):
        """ Reverse steps of activation """
        self._mw.close()
        return

    # Laser's methods

    def switch_pid(self):
        """ Switch on the laser output and start time of registering data from the power meter """
        if self._mw.PID_ON_checkBox.isChecked():
            self.time_start = time.time()
            self._power_stabilization_logic.set_pid_status(True) # Start reading data from PowerMeter
            self.power_data = []
            self.time_data = []
            self._power_stabilization_logic.set_power()
            self.pid_status = True
        else:
            self._power_stabilization_logic.set_pid_status(False) # Stop reading data from PowerMeter
            self.pid_status = False
        return

    def set_setpoint(self):
        """ Set the laser output power set-point """
        self.setpoint = self._mw.setpoint_spinBox.value() / self.beam_splitter_coef * 1e-9
        print(self.setpoint)
        self._power_stabilization_logic.set_setpoint(self.setpoint)
        self.curve[1].setValue(self.setpoint * self.beam_splitter_coef)
        if self._power_stabilization_logic.is_pid_status():
            self._power_stabilization_logic.set_ramp_status(True)
            self._power_stabilization_logic.set_pid_status(False)
        else:
            pass
        return

    def set_pid_status(self):
        """ Set the PID power lock ON/OFF """
        if self._mw.PID_ON_checkBox.isChecked():
            self._power_stabilization_logic.set_ramp_status(True)
            self._power_stabilization_logic.set_pid_status(False)
        else:
            self._power_stabilization_logic.set_ramp_status(False)
            self._power_stabilization_logic.set_pid_status(False)
        return

    def set_kp(self):
        """ Set the proportional factor of the PID """
        self._power_stabilization_logic.set_kp(self._mw.Kp_doubleSpinBox.value())
        return

    def get_kp(self):
        """ Get the proportional factor of the PID """
        return self._power_stabilization_logic.kp

    def set_ki(self):
        """ Set the integral factor of the PID """
        self._power_stabilization_logic.set_ki(self._mw.Ki_doubleSpinBox.value())
        return

    def get_ki(self):
        """ Get the integral factor of the PID """
        return self._power_stabilization_logic.ki

    def set_kd(self):
        """ Set the derivative factor of the PID """
        self._power_stabilization_logic.set_kd(self._mw.Kd_doubleSpinBox.value())
        return

    def get_kd(self):
        """ Get the derivative factor of the PID """
        return self._power_stabilization_logic.kd

    # Graph's methods

    def update_data(self):
        """ Get the data from the logic and update the graph on the gui """
        self.time_data.append(self._power_stabilization_logic.time_loop[-1] - self.time_start)
        self.power_data.append(self._power_stabilization_logic.power * self.beam_splitter_coef)
  #      self._mw.duty_cycle_doubleSpinBox.setValue(self._power_stabilization_logic.get_duty_cycle())
        if self.time_data[-1] > int(self._mw.acquisition_time_spinBox.value()):
            # If the len of the data is over a define value
            del self.time_data[0]
            del self.power_data[0]
        self.curve[0].setData(y=self.power_data, x=self.time_data)
        return

    def set_acquisition_time(self):
        """ Set the acquisition time length of the plot on the GUI """
        self.acquisition_time = self._mw.acquisition_time_spinBox.value()
        return

    def get_acquisition_time(self):
        """ Get the acquisition time length of the plot on the GUI """
        return self.acquisition_time
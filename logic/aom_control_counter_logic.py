# -*- coding: utf-8 -*-
"""
This file contains a Qudi logic module for controlling a laser powered by an AOM.
The AOM is controlled with the analog output of the NI card.
It can be power stabilized with the readout of a PM100D feedback loop.
Finally, the module offers a the possibility to measure
the saturaction curve by varying the AOM control between 2 values
and reading the count-rate of an emitter on an APD, connected
to the analog input of the NI card.

First version written by Maxime Bergamin - 2020/07/23
At the DTU - QPIT

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
import numpy as np
import os
import pyqtgraph as pg

from collections import OrderedDict
from core.connector import Connector
from gui.colordefs import ColorScaleInferno
from gui.guibase import GUIBase
from gui.guiutils import ColorBar
from qtpy import QtCore
from qtpy import QtGui
from qtpy import QtWidgets
from qtpy import uic

class AOMControlCounterLogic(GenericLogic):

    """This logic module controls scans of DC voltage on the third analog
    output channel of the NI Card.  It collects count-rate as a function of voltage.
    """

    sig_data_updated = QtCore.Signal()

    # declare connectors
    confocalscanner1 = Connector(interface='ConfocalScannerInterface')
    savelogic = Connector(interface='SaveLogic')
    powermeter1 = Connector(interface='pm100d')
    fitlogic = Connector(interface='FitLogic')

    scan_range = StatusVar('scan_range', [0, 5])
    number_of_repeats = StatusVar(default=10)
    resolution = StatusVar('resolution', 500)
    _scan_speed = StatusVar('scan_speed', 5)
    _static_v = StatusVar('goto_voltage', 0)

    sigChangeVoltage = QtCore.Signal(float)
    sigVoltageChanged = QtCore.Signal(float)
    sigScanNextLine = QtCore.Signal()
    signal_change_position = QtCore.Signal(str)
    sigUpdatePlots = QtCore.Signal()
    sigScanFinished = QtCore.Signal()
    sigScanStarted = QtCore.Signal()

    def __init__(self, **kwargs):
        """ Create VoltageScanningLogic object with connectors.

          @param dict kwargs: optional parameters
        """
        super().__init__(**kwargs)

        # locking for thread safety
        self.threadlock = Mutex()
        self.stopRequested = False

        self.fit_x = []
        self.fit_y = []
        self.plot_x = []
        self.plot_y = []
        self.plot_y2 = []



    def on_activate(self):
        """ Initialisation performed during activation of the module.
        """
        self._scanning_device = self.confocalscanner1()
        self._save_logic = self.savelogic()

        # Reads in the maximal scanning range. The unit of that scan range is
        # micrometer!
        self.a_range = self._scanning_device.get_position_range()[3]

        # Initialise the current position of all four scanner channels.
        self.current_position = self._scanning_device.get_scanner_position()

        # initialise the range for scanning
        self.scan_range = [0.0, 4.0]
        self.set_scan_range(self.scan_range)

        self._static_v = 2

        # Keep track of the current static voltage even while a scan may cause the real-time
        # voltage to change.
        self.goto_voltage(self._static_v)

        # Sets connections between signals and functions
        self.sigChangeVoltage.connect(self._change_voltage, QtCore.Qt.QueuedConnection)
        self.sigScanNextLine.connect(self._do_next_line, QtCore.Qt.QueuedConnection)

        # Initialization of internal counter for scanning
        self._scan_counter_up = 0
        self._scan_counter_down = 0
        # Keep track of scan direction
        self.upwards_scan = True

        # calculated number of points in a scan, depends on speed and max step size
        self._num_of_steps = 50  # initialising.  This is calculated for a given ramp.



    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
        """
        self.set_voltage(0)
        self.stopRequested = True

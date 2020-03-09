# -*- coding: utf-8 -*-
"""
This file contains a Qudi logic module for controlling scans of the
CTL Toptica via the DLCpro.
This logic file is a combination of the laser_scanning_logic.py and
the laser_logic.py, as the laser can be both controlled and scanned
with the following interface.

It was originally written for scanning laser frequency, but
it can be used to control any parameter in the experiment that
is voltage controlled.  The hardware range is typically -10 to +10 V.

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

import time
import numpy as np
from qtpy import QtCore

from core.connector import Connector
from core.configoption import ConfigOption
from logic.generic_logic import GenericLogic
from interface.simple_laser_interface import LaserState


class TopticaDLCproLogic(GenericLogic):
    """ Logic module agreggating multiple hardware switches.
    """

    # waiting time between queries im milliseconds
    laser = Connector(interface='SimpleLaserInterface')
    queryInterval = ConfigOption('query_interval', 100)
    savelogic = Connector(interface='SaveLogic')

    sigUpdate = QtCore.Signal()

    def on_activate(self):
        """ Prepare logic module for work.
        """
        self._laser = self.laser()
        self.stopRequest = False
        self.bufferLength = 100
        self.data = {}

        # delay timer for querying laser
        self.queryTimer = QtCore.QTimer()
        self.queryTimer.setInterval(self.queryInterval)
        self.queryTimer.setSingleShot(True)
        self.queryTimer.timeout.connect(self.check_laser_loop, QtCore.Qt.QueuedConnection)

        # get laser capabilities
        self.laser_state = self._laser.get_laser_state()
        self.laser_can_turn_on = self.laser_state.value <= LaserState.ON.value
        #self.laser_current_range = self._laser.get_current_range()
        #self.laser_current_setpoint = self._laser.get_current_setpoint()

        self.init_data_logging()
        self.start_query_loop()

    def on_deactivate(self):
        """ Deactivate modeule.
        """
        self.stop_query_loop()
        for i in range(5):
            time.sleep(self.queryInterval / 1000)
            QtCore.QCoreApplication.processEvents()

    @QtCore.Slot()
    def check_laser_loop(self):
        """ Get power, current, shutter state and temperatures from laser. """
        if self.stopRequest:
            if self.module_state.can('stop'):
                self.module_state.stop()
            self.stopRequest = False
            return
        qi = self.queryInterval
        try:
            #print('laserloop', QtCore.QThread.currentThreadId())
            self.laser_state = self._laser.get_laser_state()
            self.laser_current = self._laser.get_current()
            self.laser_wavelength = self._laser.get_wavelength()
            #self.laser_current_setpoint = self._laser.get_current_setpoint()
            self.laser_temps = self._laser.get_temperatures()

            for k in self.data:
                self.data[k] = np.roll(self.data[k], -1)

            self.data['current'][-1] = self.laser_current
            # self.data['wavelength'][-1] = self.laser_wavelength
            self.data['time'][-1] = time.time()

            for k, v in self.laser_temps.items():
                self.data[k][-1] = v
        except:
            qi = 3000
            self.log.exception("Exception in laser status loop, throttling refresh rate.")

        self.queryTimer.start(qi)
        self.sigUpdate.emit()

    @QtCore.Slot()
    def start_query_loop(self):
        """ Start the readout loop. """
        self.module_state.run()
        self.queryTimer.start(self.queryInterval)

    @QtCore.Slot()
    def stop_query_loop(self):
        """ Stop the readout loop. """
        self.stopRequest = True
        for i in range(10):
            if not self.stopRequest:
                return
            QtCore.QCoreApplication.processEvents()
            time.sleep(self.queryInterval/1000)

    def init_data_logging(self):
        """ Zero all log buffers. """
        self.data['current'] = np.zeros(self.bufferLength)
        self.data['time'] = np.ones(self.bufferLength) * time.time()
        temps = self._laser.get_temperatures()
        for name in temps:
             self.data[name] = np.zeros(self.bufferLength)

    @QtCore.Slot(bool)
    def set_laser_state(self, state):
        """ Turn laser on or off. """
        if state and self.laser_state == LaserState.OFF:
            self._laser.on()
        if not state and self.laser_state == LaserState.ON:
            self._laser.off()
        self.sigUpdate.emit()
    #
    # @QtCore.Slot(float)
    # def set_power(self, power):
    #     """ Set laser output power. """
    #     self._laser.set_power(power)

    @QtCore.Slot(float)
    def set_current(self, current):
        """ Set laser diode current. """
        max_current = 167
        if current <= max_current:
            self._laser.set_current(current)
        else:
            self._laser.set_current(max_current)

    @QtCore.Slot(float)
    def set_wavelength(self, wavelength):
        """ Set laser diode current. """
        min_wavelength = 955
        max_wavelength = 965
        if wavelength < min_wavelength:
            self._laser.set_wavelength(min_wavelength)
        elif wavelength > max_wavelength:
            self._laser.set_wavelengtH(max_wavelength)
        else:
            self._laser.set_wavelength(wavelength)
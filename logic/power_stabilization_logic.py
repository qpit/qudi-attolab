"""
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

from core.module import Base
from interface.empty_interface import EmptyInterface
from core.module import Connector
from core.util.mutex import Mutex
from logic.generic_logic import GenericLogic


class PowerStabilizationLogic(GenericLogic):

    """This is the Interface class to define the controls for the simple
    microwave hardware.
    """
    _modclass = 'power_stabilization_logic'
    _modtype = 'logic'

    # connectors
    # TiS_camera_hardware = Connector(interface='EmptyInterface')
    # arduino_hardware = Connector(interface='EmptyInterface')
    # power_meter_hardware = Connector(interface='EmptyInterface')

    powermeter1 = Connector(interface='SimpleDataInterface')
    confocalscanner1 = Connector(interface='ConfocalScannerInterface')

    sigPowerUpdated = QtCore.Signal()
    sigPowerDataNext = QtCore.Signal()

    def on_activate(self):
        """ Initialisation performed during activation of the module. """

        self._powermeter = self.powermeter1()
        self._daq_card = self.confocalscanner1()


        #### activate the NI card AO channel
        self.z_range = self._daq_card.get_position_range()[2]
        # Initialise the current position of all four scanner channels.
        self.current_position = self._daq_card.get_scanner_position()
        # initialise the range for scanning
        # self.scan_range = [0.0, 1.0]
        # self.set_scan_range(self.scan_range)
        self._static_v = 0.
        # Keep track of the current static voltage even while a scan may cause the real-time
        # voltage to change.
        self.goto_voltage(self._static_v)

        # set power meter
        self._powermeter.set_wavelength(600.0)
        self._powermeter.set_autorange(1)

        # PID
        self.pid_status = False
        self.ramp_status = False
        self.setpoint = 0.
        self.polarity = 1
        self.kp, self.ki, self.kd = 500, 1, 0
        self.min_pid_out, self.max_pid_out = 0., 0.4
        self.min_volt, self.max_volt = 0., 0.8
        self.ramping_factor = 0.01
        self.offset = 0
        self.error_p_prev = 0.
        self.power_prev = 0.
        self.error_p = 0
        self.error_i = 0
        self.error_d = 0
        self.output = 0
        self.time_loop = []
        self.power = None
        self.error = None

        # Thread
        self.threadlock = Mutex()

        self.sigPowerDataNext.connect(self.set_power, QtCore.Qt.QueuedConnection)
        return

    def on_deactivate(self):
        """  Performed during deactivation of the module. """
        # self._TiS_camera_hardware.on_deactivate()
        # self.set_power(0)
        # self._arduino_hardware.on_deactivate()
        self.sigPowerDataNext.disconnect()
        self.goto_voltage(1)
        return

    #### NI card scanner

    @QtCore.Slot(float)
    def goto_voltage(self, volts=None):
        """Forwarding the desired output voltage to the scanning ♣.

        @param float volts: desired voltage (volts)

        @return int: error code (0:OK, -1:error)
        """
        # print(tag, x, y, z)
        # Changes the respective value
        if volts is not None:
            self._static_v = volts
        ch_array = ['z']
        pos_array = [self._static_v]
        pos_dict = {}
        pos_dict[ch_array[0]] = pos_array[0]

        self._daq_card.scanner_set_position(**pos_dict)

    def set_voltage(self, volts):
        """ Set the channel idle voltage """
        self._static_v = np.clip(volts, self.z_range[0], self.z_range[1])
        self.goto_voltage(self._static_v)

    def get_current_voltage(self):
        """returns current voltage of hardware device(atm NIDAQ 3rd output)"""
        return self._daq_card.get_scanner_position()[2]

    #### PID controler

    def set_setpoint(self, setpoint):
        """ Set the power setpoint of the laser. """
        self.setpoint = setpoint
        return

    def get_setpoint(self):
        """ Get the power setpoint of the laser. """
        return self.setpoint

    def set_pid_status(self, boolean):
        """ Set the PID to True or False. """
        self.pid_status = boolean
        return

    def is_pid_status(self):
        """ Get the status of the PID (boolean). """
        return self.pid_status

    def set_kp(self, value):
        """ Set the proportional term of the PID. """
        self.kp = value
        return

    def set_ki(self, value):
        """ Set the integral term of the PID. """
        self.ki = value
        return

    def set_kd(self, value):
        """ Set the derivative term of the PID. """
        self.kd = value
        return

    def clear_integral(self):
        """ Clear the integral term of the PID. """
        self.error_i = 0
        return

    def set_ramp_status(self, boolean):
        """ Set the ramp status (True ot False).
        Used in order to increase the changes of laser power"""
        self.ramp_status = boolean
        return

    def set_power(self):
        """Set the voltage with or without PID"""
        # if not self.laser_on:
        #     # Switch console from remote to local mode and exit
        #     self._power_meter_hardware.power_meter._inst.control_ren(6)
        #     return

        # measure power and the time
        self.power = self.get_power()
        self.time_loop.append(time.time())
        # We delete the useless data in order to not saturate the memory
        if len(self.time_loop) > 2:
            del self.time_loop[0]
            if self.time_loop[-1] == self.time_loop[-2]:
                # If the time is the same for two loops then we call the function again
                pass
            else:
                # We update the power on the GUI
                self.sigPowerUpdated.emit()
                # get error
                self.error = self.get_setpoint() - self.power
                #### The ramp is there to speed up the process to fo the the setpoint value
                #### At the moment, the use of the ramp is disable by setting a high value
                #### TODO, delete the ramp which is more a source of trouble than anything
                if self.ramp_status:
                    if abs(self.error) > 10e-7:
                        self.offset = self.output
                        self.output += np.sign(self.error)*1e-4
                        #### set the output value
                        self.goto_voltage(self.output)
                    else:
                        self.ramp_status = False
                        self.clear_integral()
                        self.pid_status = True
                elif self.pid_status:
                    delta_t = self.time_loop[-1] - self.time_loop[-2]
                    self.error_p = self.error
                    self.error_i += self.error * delta_t
                    self.error_d = (self.error - self.error_p_prev) / delta_t
                    p = self.kp * self.error_p
                    i = self.ki * self.error_i
                    d = self.kd * self.error_d
                    pid_out = self.polarity * (p + i + d / 100)

                    #### ramp up/down until setpoint is reached?
                    correction = self.offset + pid_out
                    k_factor = 1e3

                    if correction >= self.max_pid_out:
                        self.output = self.max_pid_out
                    elif self.get_current_voltage() + correction*k_factor <= self.min_pid_out:
                        self.output = self.min_pid_out
                    else:
                        self.output = correction

                    err = self.output - self.power_prev

                    #### What to do with the correction, is it the right way the correction signal? Medidate on that
                    if abs(err) > 1e-10:
                        if self.get_current_voltage() + correction*k_factor >= self.max_volt:
                            self.goto_voltage(self.get_current_voltage())
                            print("maximum voltage reached")
                        elif self.get_current_voltage() + correction*k_factor <= self.min_volt:
                            self.goto_voltage(self.get_current_voltage())
                            print("minimum voltage reached")
                        else:
                            self.goto_voltage(self.get_current_voltage() + correction*k_factor)
                    self.power_prev = self.output
                    self.error_p_prev = self.error_p
                else:
                    self.goto_voltage(self._static_v)
        self.sigPowerDataNext.emit()
        return

    #### Power Meter

    def get_power(self):
        """ Get the optical power measured by the power meter. """
        return self._powermeter.get_power()

    def get_wavelength(self):
        """ Get the wavelength value of the PM100 powermeter """
        return self._powermeter.get_wavelength()

    def set_wavelength(self, wavelength):
        """ Set the wavelength value to the PM100 powermeter """
        self._powermeter.set_wavelength(wavelength)


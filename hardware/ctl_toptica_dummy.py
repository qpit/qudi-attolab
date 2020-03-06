# -*- coding: utf-8 -*-
"""
This module contains the hardware file for the CTL Toptica laser.
CTL: Continuous Tubable Laser
Toptica SDK https://toptica.github.io/python-lasersdk/index.html

The communication is actually done via the DLC pro

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

from core.module import Base

from interface.simple_laser_interface import SimpleLaserInterface
from interface.confocal_scanner_interface import ConfocalScannerInterface

import socket
import time
import random

class Toptica_DLC_proDummy(Base, SimpleLaserInterface, ConfocalScannerInterface):
    """ Implements the CTL Toptica laser.

    Took the base code from:
        https://github.com/iontrapimperial2/XCon_Imperial_laserlock/blob/5c7102189276e312bea38dcbc52f0db8aaad383d/library/instr_Toptica_DLC_pro.py
    other inspiration:
        https://github.com/cphenicie/laserdaq_chris/blob/de724a7ecd759244be5fcf4118bf7f8fb137c114/topticaShortcuts.py

    Example config for copy-paste:

    laser_dummy:
       module.Class: 'ctl_toptica.Toptica_DLC_pro'

    Description of the commands sent to the DLCpro can be found on the "DLCpro-Command-Reference.pdf"
        at : \\serv309.fysik.dtu.dk\QUIN\Diamondlab\06-Hardware & Manuals\06-CTL Toptica\1_TOPTICA DLC pro SOFTWARE_2.0.3\1_DOCUMENTATION
        (checked on 05/03/2020)
    """

    _ip_address = '10.54.10.77'
    _port = 1998
    _timeout = None

    def __init__(self, **kwargs):
        """ """
        super().__init__(**kwargs)
        self.current_setpoint = 0

    def on_activate(self):
        """ Activate Module.
        """
        print('Toptica DLC pro at ip address FAKE ONE, suck it buddy, is online')

    def on_deactivate(self):
        """ Deactivate module
        """

    #######################################################################
    # =========== Set and Read parameters to DLC Pro functions ============
    #######################################################################
    """ The two following functions are used to send the right string command to the DLC pro
    
        Examples for using the functions, either to read or set signals/values: 
            self.set_parameter(command = 'laser1:dl:cc:enabled', param = '#f')
            self.read_parameter(command = 'laser1:dl:cc:current-set')
    """

    #######################################################################
    # ================== SimpleLaserInterface Commands ====================
    #######################################################################
    def get_current(self):
        """ Get current laser current
            (REAL parameter, read-write)
            Parameter to set the desired laser diode current in mA.
            If laser1:dl:cc:feedforward-enabled is #t, laser1:dl:cc:current-set is determined
            as follows:
            laser1:dl:cc:current-set = laser1:dl:cc:current-offset + laser1:dl:cc:feedforward-
            factor * (laser1:dl:pc:voltage-set - 69.5 V)
            This parameter setting affects the laser1:dl:cc:current-offset value.
        """
        return self.current_setpoint * random.gauss(1, 0.05)

    def set_current(self, current):
        """ Set laser current setpoint
            (REAL parameter, read-write)
            Parameter to set the desired laser diode current in mA.
            If laser1:dl:cc:feedforward-enabled is #t, laser1:dl:cc:current-set is determined
            as follows:
            laser1:dl:cc:current-set = laser1:dl:cc:current-offset + laser1:dl:cc:feedforward-
            factor * (laser1:dl:pc:voltage-set - 69.5 V)
            This parameter setting affects the laser1:dl:cc:current-offset value.
        """
        self.current_setpoint = current
        return self.current_setpoint

    def on(self):
        """ Turn on laser.
            (BOOLEAN parameter, read-write)
            Parameter to switch the laser emission on/off:
            #t - laser emission on
            #f - laser emission off
        """
        return

    def off(self):
        """ Turn off laser.
            (BOOLEAN parameter, read-write)
            Parameter to switch the laser emission on/off:
            #t - laser emission on
            #f - laser emission off
        """
        return

    def get_laser_state(self):
        """ Get laser state
            (BOOLEAN parameter, read-only)
            Parameter indicating the laser head emission status.
            #t - laser is switched on
            #f - laser is switched off
        """
        return "ON/OFF"

    def get_temperatures(self):
        """ Get all available temperatures.
            (REAL parameter, read-only)
            Parameter indicating the temperature inside the CTL laser head in degree Celsius.
        """
        return {'head': 42.0 * random.gauss(1, 0.2)}

    def get_current_setpoint(self):
        """ Get laser curent setpoint
            @return float: laser current setpoint
        """
        return self.current_setpoint
    ######################################################################################
    # The following methods are unused and are free to be picked up for other applications
    ######################################################################################
    def get_power_range(self):
        """ Return optical power range
            @return (float, float): power range
        """
        return

    def get_power(self):
        """ Return laser power
            @return float: Laser power in watts
        """
        return

    def get_power_setpoint(self):
        """ Return optical power setpoint.
            @return float: power setpoint in watts
        """
        return

    def set_power(self):
        """ Set power setpoint.
            @param float power: power setpoint
            @return float: actual new power setpoint
        """
        return

    def get_current_unit(self):
        """ Get unit for laser current.
            @return str: unit
        """
        return

    def get_current_range(self):
        """ Get laser current range.
            @return (float, float): laser current range
        """
        return

    def set_temperatures(self):
        """ Set temperatures for lasers with tunable temperatures.
            @return {}: empty dict, dummy not a tunable laser
        """
        return

    def get_temperature_setpoints(self):
        """ Get temperature setpoints.
            @return dict: temperature setpoints for temperature tunable lasers
        """
        return

    def get_extra_info(self):
        """ Multiple lines of dignostic information
            @return str: much laser, very useful
        """
        return

    def allowed_control_modes(self):
        """ Get supported control modes
            @return list(): list of supported ControlMode
        """
        return

    def get_control_mode(self):
        """ Get the currently active control mode
            @return ControlMode: active control mode
        """
        return

    def set_control_mode(self):
        """ Set the active control mode
            @param ControlMode control_mode: desired control mode
            @return ControlMode: actual active ControlMode
        """
        return

    def set_laser_state(self):
        """ Set laser state.
            @param LaserState state: desired laser state
            @return LaserState: actual laser state
        """
        return

    def get_shutter_state(self):
        """ Get laser shutter state
            @return ShutterState: actual laser shutter state
        """
        return

    def set_shutter_state(self):
        """ Set laser shutter state.
            @param ShutterState state: desired laser shutter state
            @return ShutterState: actual laser shutter state
        """
        return
    ###########################################################################
    # ================== ConfocalScannerInterface Commands ====================
    ###########################################################################
    def scan_line(self, line_path=None, pixel_clock=False):
        """ Scans a line and returns the counts on that line.
        @param float[][4] line_path: array of 4-part tuples defining the voltage points
        @param bool pixel_clock: whether we need to output a pixel clock for this line
        @return float[]: the photon counts per second
        """
        return

    def set_up_scanner_clock(self, clock_frequency=None, clock_channel=None):
        """ Configures the hardware clock of the NiDAQ card to give the timing.
        @param float clock_frequency: if defined, this sets the frequency of the
                                      clock
        @param str clock_channel: if defined, this is the physical channel of
                                  the clock
        @return int: error code (0:OK, -1:error)
        """
        return

    def set_up_scanner(self, counter_channels=None, sources=None,
                       clock_channel=None, scanner_ao_channels=None):
        """ Configures the actual scanner with a given clock.
        @param str counter_channel: if defined, this is the physical channel of
                                    the counter
        @param str photon_source: if defined, this is the physical channel where
                                  the photons are to count from
        @param str clock_channel: if defined, this specifies the clock for the
                                  counter
        @param str scanner_ao_channels: if defined, this specifies the analoque
                                        output channels
        @return int: error code (0:OK, -1:error)
        """
        return

    def get_scanner_position(self):
        """ Get the current position of the scanner hardware.
        @return float[]: current position in (x, y, z, a).
        """
        return

    def set_voltage_range(self, myrange=None):
        """ Sets the voltage range of the NI Card.
        @param float [2] myrange: array containing lower and upper limit
        @return int: error code (0:OK, -1:error)
        """
        return

    def scanner_set_position(self, x=None, y=None, z=None, a=None):
        """Move stage to x, y, z, a (where a is the fourth voltage channel).
        @param float x: postion in x-direction (volts)
        @param float y: postion in y-direction (volts)
        @param float z: postion in z-direction (volts)
        @param float a: postion in a-direction (volts)
        @return int: error code (0:OK, -1:error)
        """
        return


    ######################################################################################
    # The following methods are unused and are free to be picked up for other applications
    ######################################################################################
    def reset_hardware(self):
        """ Resets the hardware, so the connection is lost and other programs
            can access it.
        @return int: error code (0:OK, -1:error)
        """
        return

    def get_position_range(self):
        """ Returns the physical range of the scanner.
        @return float [4][2]: array of 4 ranges with an array containing lower
                              and upper limit
        """
        return

    def set_position_range(self, myrange=None):
        """ Sets the physical range of the scanner.
        @param float [4][2] myrange: array of 4 ranges with an array containing
                                     lower and upper limit
        @return int: error code (0:OK, -1:error)
        """
        return

    def get_scanner_axes(self):
        """ Dummy scanner is always 3D cartesian.
        """
        return

    def get_scanner_count_channels(self):
        """ 3 counting channels in dummy confocal: normal, negative and a ramp."""
        return

    def _set_up_line(self, length=100):
        """ Sets up the analoque output for scanning a line.
        @param int length: length of the line in pixel
        @return int: error code (0:OK, -1:error)
        """
        return

    def close_scanner(self):
        """ Closes the scanner and cleans up afterwards.
        @return int: error code (0:OK, -1:error)
        """
        return

    def close_scanner_clock(self, power=0):
        """ Closes the clock and cleans up afterwards.
        @return int: error code (0:OK, -1:error)
        """
        return

    # def get_voltage_act(self):
    #     return(self.dlc.laser1.dl.pc.voltage_act.get())

    # def get_max_current(self):
    #     return(self.dlc.laser1.dl.cc.current_clip.get())

    # def set_voltage(self, value):
    #     self.dlc.laser1.dl.pc.voltage_set.set(float(value))
    # def userLevel(self):
    #     return(self.dlc.ul.get())

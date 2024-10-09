# -*- coding: utf-8 -*-
"""
This file contains the Qudi counter logic class.

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

from qtpy import QtCore
from collections import OrderedDict
import numpy as np
import time
import matplotlib.pyplot as plt
import datetime

from core.connector import Connector
from core.statusvariable import StatusVar
from logic.generic_logic import GenericLogic
from interface.slow_counter_interface import CountingMode
from core.util.mutex import Mutex

STAGE_PRECISION = 0.01 #deg

class MotorizedStageLogic(GenericLogic):
    """ This logic module gathers data from a hardware counting device.

    @signal sigCounterUpdate: there is new counting data available
    @signal sigCountContinuousNext: used to simulate a loop in which the data
                                    acquisition runs.
    @sigmal sigCountGatedNext: ???

    @return error: 0 is OK, -1 is error
    """
    sigCounterUpdated = QtCore.Signal()

    sigCountDataNext = QtCore.Signal()

    sigGatedCounterFinished = QtCore.Signal()
    sigGatedCounterContinue = QtCore.Signal(bool)
    sigCountingSamplesChanged = QtCore.Signal(int)
    sigCountLengthChanged = QtCore.Signal(int)
    sigCountFrequencyChanged = QtCore.Signal(float)
    sigSavingStatusChanged = QtCore.Signal(bool)
    sigCountStatusChanged = QtCore.Signal(bool)
    sigCountingModeChanged = QtCore.Signal(CountingMode)

    # declare connectors
    counter1 = Connector(interface='SlowCounterInterface')
    savelogic = Connector(interface='SaveLogic')
    quarterwaveplate = Connector(interface='MotorInterface')
    halfwaveplate = Connector(interface='MotorInterface')


    # status vars
    _count_length = StatusVar('count_length', 300)
    _smooth_window_length = StatusVar('smooth_window_length', 10)
    _counting_samples = StatusVar('counting_samples', 1)
    _count_frequency = StatusVar('count_frequency', 50)
    _saving = StatusVar('saving', False)


    def __init__(self, config, **kwargs):
        """ Create CounterLogic object with connectors.

        @param dict config: module configuration
        @param dict kwargs: optional parameters
        """
        super().__init__(config=config, **kwargs)

        #locking for thread safety
        self.threadlock = Mutex()

        self.log.debug('The following configuration was found.')

        # checking for the right configuration
        for key in config.keys():
            self.log.debug('{0}: {1}'.format(key, config[key]))

        # in bins
        self._count_length = 300
        self._smooth_window_length = 10
        self._counting_samples = 1      # oversampling
        # in hertz
        self._count_frequency = 50

        # self._binned_counting = True  # UNUSED?
        self._counting_mode = CountingMode['CONTINUOUS']

        self._saving = False

        # rotation
        self.qwp_initial_angle = 0
        self.qwp_final_angle = 90
        self.qwp_step_size = 1

        self.hwp_initial_angle = 0
        self.hwp_final_angle = 90
        self.hwp_step_size = 1

        self.qwp_angles = []
        self.hwp_angles = []

        self.N_qwp_angles = 0
        self.N_hwp_angles = 0

        self.measured_qwp_angles = []
        self.measured_hwp_angles = []
        self.angle_index = 0

        return

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        """
        # Connect to hardware and save logic
        self._counting_device = self.counter1()
        self._quarterwaveplate= self.quarterwaveplate()
        self._halfwaveplate = self.halfwaveplate()

        self._save_logic = self.savelogic()

        # Recall saved app-parameters
        if 'counting_mode' in self._statusVariables:
            self._counting_mode = CountingMode[self._statusVariables['counting_mode']]

        constraints = self.get_hardware_constraints()
        number_of_detectors = constraints.max_detectors

        # initialize data arrays
        self.countdata = np.zeros([len(self.get_channels()), self._count_length])
        self.countdata_smoothed = np.zeros([len(self.get_channels()), self._count_length])
        self.rawdata = np.zeros([len(self.get_channels()), self._counting_samples])
        self._already_counted_samples = 0  # For gated counting
        self._data_to_save = []

        # Flag to stop the loop
        self.stopRequested = False

        self._saving_start_time = time.time()

        # connect signals
        self.sigCountDataNext.connect(self.count_loop_body, QtCore.Qt.QueuedConnection)
        return

    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
        """
        # Save parameters to disk
        self._statusVariables['counting_mode'] = self._counting_mode.name

        # Stop measurement
        if self.module_state() == 'locked':
            self._stopCount_wait()

        self.sigCountDataNext.disconnect()
        return

    def get_hardware_constraints(self):
        """
        Retrieve the hardware constrains from the counter device.

        @return SlowCounterConstraints: object with constraints for the counter
        """
        return self._counting_device.get_constraints()

    def set_counting_samples(self, samples=1):
        """
        Sets the length of the counted bins.
        The counter is stopped first and restarted afterwards.

        @param int samples: oversampling in units of bins (positive int ).

        @return int: oversampling in units of bins.
        """
        # Determine if the counter has to be restarted after setting the parameter
        if self.module_state() == 'locked':
            restart = True
        else:
            restart = False

        if samples > 0:
            self._stopCount_wait()
            self._counting_samples = int(samples)
            # if the counter was running, restart it
            if restart:
                self.startCount()
        else:
            self.log.warning('counting_samples has to be larger than 0! Command ignored!')
        self.sigCountingSamplesChanged.emit(self._counting_samples)
        return self._counting_samples

    def set_count_length(self, length=300):
        """ Sets the time trace in units of bins.

        @param int length: time trace in units of bins (positive int).

        @return int: length of time trace in units of bins

        This makes sure, the counter is stopped first and restarted afterwards.
        """
        if self.module_state() == 'locked':
            restart = True
        else:
            restart = False

        if length > 0:
            self._stopCount_wait()
            self._count_length = int(length)
            # if the counter was running, restart it
            if restart:
                self.startCount()
        else:
            self.log.warning('count_length has to be larger than 0! Command ignored!')
        self.sigCountLengthChanged.emit(self._count_length)
        return self._count_length

    def set_count_frequency(self, frequency=50):
        """ Sets the frequency with which the data is acquired.

        @param float frequency: the desired frequency of counting in Hz

        @return float: the actual frequency of counting in Hz

        This makes sure, the counter is stopped first and restarted afterwards.
        """
        constraints = self.get_hardware_constraints()

        if self.module_state() == 'locked':
            restart = True
        else:
            restart = False

        if constraints.min_count_frequency <= frequency <= constraints.max_count_frequency:
            self._stopCount_wait()
            self._count_frequency = frequency
            # if the counter was running, restart it
            if restart:
                self.startCount()
        else:
            self.log.warning('count_frequency not in range! Command ignored!')
        self.sigCountFrequencyChanged.emit(self._count_frequency)
        return self._count_frequency

    def get_count_length(self):
        """ Returns the currently set length of the counting array.

        @return int: count_length
        """
        return self._count_length

    #FIXME: get from hardware
    def get_count_frequency(self):
        """ Returns the currently set frequency of counting (resolution).

        @return float: count_frequency
        """
        return self._count_frequency

    def get_counting_samples(self):
        """ Returns the currently set number of samples counted per readout.

        @return int: counting_samples
        """
        return self._counting_samples



    def set_counting_mode(self, mode='CONTINUOUS'):
        """Set the counting mode, to change between continuous and gated counting.
        Possible options are:
            'CONTINUOUS'    = counts continuously
            'GATED'         = bins the counts according to a gate signal
            'FINITE_GATED'  = finite measurement with predefined number of samples

        @return str: counting mode
        """
        constraints = self.get_hardware_constraints()
        if self.module_state() != 'locked':
            if CountingMode[mode] in constraints.counting_mode:
                self._counting_mode = CountingMode[mode]
                self.log.debug('New counting mode: {}'.format(self._counting_mode))
            else:
                self.log.warning('Counting mode not supported from hardware. Command ignored!')
            self.sigCountingModeChanged.emit(self._counting_mode)
        else:
            self.log.error('Cannot change counting mode while counter is still running.')
        return self._counting_mode

    def get_counting_mode(self):
        """ Retrieve the current counting mode.

        @return str: one of the possible counting options:
                'CONTINUOUS'    = counts continuously
                'GATED'         = bins the counts according to a gate signal
                'FINITE_GATED'  = finite measurement with predefined number of samples
        """
        return self._counting_mode

    def qwp_get_pos(self):
        return self._quarterwaveplate.get_pos()['phi']

    def qwp_move_abs(self, pos):
        return self._quarterwaveplate.move_abs({'phi': pos})

    def qwp_moving(self):
        return self._quarterwaveplate.get_status()['phi'][0] != 0

    def hwp_get_pos(self):
        return self._halfwaveplate.get_pos()['phi']

    def hwp_move_abs(self, pos):
        return self._halfwaveplate.move_abs({'phi': pos})

    def hwp_moving(self):
        return self._halfwaveplate.get_status()['phi'][0] != 0

    # FIXME: Not implemented for self._counting_mode == 'gated'
    def startCount(self):
        """ This is called externally, and is basically a wrapper that
            redirects to the chosen counting mode start function.

            @return error: 0 is OK, -1 is error
        """
        # Sanity checks
        constraints = self.get_hardware_constraints()
        if self._counting_mode not in constraints.counting_mode:
            self.log.error('Unknown counting mode "{0}". Cannot start the counter.'
                           ''.format(self._counting_mode))
            self.sigCountStatusChanged.emit(False)
            return -1

        with self.threadlock:
            # Lock module
            if self.module_state() != 'locked':
                self.module_state.lock()
            else:
                self.log.warning('Counter already running. Method call ignored.')
                return 0

            # Set up clock
            clock_status = self._counting_device.set_up_clock(clock_frequency=self._count_frequency)
            if clock_status < 0:
                self.module_state.unlock()
                self.sigCountStatusChanged.emit(False)
                return -1

            # Set up counter
            counter_status = self._counting_device.set_up_counter()

            if counter_status < 0:
                self._counting_device.close_clock()
                self.module_state.unlock()
                self.sigCountStatusChanged.emit(False)
                return -1

            # the sample index for gated counting
            self._already_counted_samples = 0


            # Starts rotating the stage
            _qwp_angles = np.arange(self.qwp_initial_angle, self.qwp_final_angle, self.qwp_step_size)
            _hwp_angles = np.arange(self.hwp_initial_angle, self.hwp_final_angle, self.hwp_step_size)

            self.qwp_angles, self.hwp_angles = np.meshgrid(_qwp_angles, _hwp_angles)

            self.qwp_angles = self.qwp_angles.flatten()
            self.hwp_angles = self.hwp_angles.flatten()

            self.measured_qwp_angles = np.zeros(self.qwp_angles.shape)
            self.measured_hwp_angles = np.zeros(self.hwp_angles.shape)

            self.countdata = np.zeros(len(self.qwp_angles))

            self.angle_index = 0
            self.qwp_move_abs(self.qwp_initial_angle)
            self.hwp_move_abs(self.hwp_initial_angle)


            # Start data reader loop
            self.sigCountStatusChanged.emit(True)
            self.sigCountDataNext.emit()
            return

    def stopCount(self):
        """ Set a flag to request stopping counting.
        """
        if self.module_state() == 'locked':
            with self.threadlock:
                self.stopRequested = True
        return

    def count_loop_body(self):
        """ This method gets the count data from the hardware for the continuous counting mode (default).

        It runs repeatedly in the logic module event loop by being connected
        to sigCountContinuousNext and emitting sigCountContinuousNext through a queued connection.
        """
        if self.module_state() == 'locked':
            with self.threadlock:
                # check for aborts of the thread in break if necessary
                if self.stopRequested:
                    # close off the actual counter
                    cnt_err = self._counting_device.close_counter()
                    clk_err = self._counting_device.close_clock()
                    if cnt_err < 0 or clk_err < 0:
                        self.log.error('Could not even close the hardware, giving up.')
                    # switch the state variable off again
                    self.stopRequested = False
                    self.module_state.unlock()
                    self.sigCounterUpdated.emit()
                    return

                # read the current counter value
                rawdata = self._counting_device.get_counter(samples=self._counting_samples)

                # If the stage has reached destination
                if not self.qwp_moving() and not self.hwp_moving():
                    # time.sleep(.5)
                    if self.angle_index % 10 == 0:
                        self.log.info(f"Angle combination {self.angle_index} of {len(self.qwp_angles)}: {self.qwp_get_pos()}, {self.hwp_get_pos()} Count rate: {rawdata[0, 0]}")
                    # Save measured angle at the position
                    self.measured_qwp_angles[self.angle_index] = self.qwp_get_pos()
                    self.measured_hwp_angles[self.angle_index] = self.hwp_get_pos()

                    # Record measurement
                    self.countdata[self.angle_index] = rawdata[0, 0]
                    # move to the next one
                    self.angle_index += 1
                    if self.angle_index == len(self.qwp_angles):
                        self.stopRequested = True
                        self.save_data()
                    else:
                        self.qwp_move_abs(self.qwp_angles[self.angle_index])
                        self.hwp_move_abs(self.hwp_angles[self.angle_index])

            # call this again from event loop
            self.sigCounterUpdated.emit()
            self.sigCountDataNext.emit()
        return


    def save_data(self):
        print("saving data")
        filepath = self._save_logic.get_path_for_module(module_name='MotorizedStage')
        timestamp = datetime.datetime.now()

        filelabel = 'angle_data'

        data = OrderedDict()
        data['Target QWP Angle (deg)'] = self.qwp_angles
        data['Target HWP Angle (deg)'] = self.hwp_angles
        data['Measured QWP Angle (deg)'] = self.measured_qwp_angles
        data['Measured HWP Angle (deg)'] = self.measured_hwp_angles

        data['Counts (counts/s)'] = self.countdata

        parameters = OrderedDict()
        parameters['QWP Initial angle (deg)'] = self.qwp_initial_angle
        parameters['QWP Final angle (deg)'] = self.qwp_final_angle
        parameters['QWP Step (deg)'] = self.qwp_step_size

        parameters['HWP Initial angle (deg)'] = self.hwp_initial_angle
        parameters['HWP Final angle (deg)'] = self.hwp_final_angle
        parameters['HWP Step (deg)'] = self.hwp_step_size

        # Create plot
        plt.style.use(self._save_logic.mpl_qd_style)

        fig, ax = plt.subplots()

        _qwp_angles = np.arange(self.qwp_initial_angle, self.qwp_final_angle, self.qwp_step_size)
        _hwp_angles = np.arange(self.hwp_initial_angle, self.hwp_final_angle, self.hwp_step_size)

        _count_data = self.countdata.reshape((len(_qwp_angles), len(_hwp_angles)))
        im = ax.imshow(_count_data, extent=[self.qwp_initial_angle, self.qwp_final_angle,self.hwp_initial_angle, self.hwp_final_angle])
        fig.colorbar(im)
        ax.set_xlabel("QWP Rotation angle (deg)")
        ax.set_ylabel("HWP Rotation angle (deg)")

        self._save_logic.save_data(
            data,
            filepath=filepath,
            parameters=parameters,
            filelabel=filelabel,
            fmt='%.6e',
            delimiter='\t',
            timestamp=timestamp,
            plotfig=fig
        )

        return

    def get_channels(self):
        """ Shortcut for hardware get_counter_channels.

            @return list(str): return list of active counter channel names
        """
        return self._counting_device.get_counter_channels()


    def _stopCount_wait(self, timeout=5.0):
        """
        Stops the counter and waits until it actually has stopped.

        @param timeout: float, the max. time in seconds how long the method should wait for the
                        process to stop.

        @return: error code
        """
        self.stopCount()
        start_time = time.time()
        while self.module_state() == 'locked':
            time.sleep(0.1)
            if time.time() - start_time >= timeout:
                self.log.error('Stopping the counter timed out after {0}s'.format(timeout))
                return -1
        return 0

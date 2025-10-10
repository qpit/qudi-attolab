# nir_spectrometer_logic.py
import numpy as np
import time
from core.module import Base
from core.connector import Connector
from qtpy import QtCore


class NIRSpectrometerLogic(Base):
    """
    Logic for a NIR spectrometer using a motorized LVF and a counter.
    """
    # Define connectors to hardware and other logic modules
    nir_stage = Connector(interface='MotorInterface')
    nir_counter = Connector(interface='CounterInterface')
    savelogic = Connector(interface='SaveLogic')

    # Signals to communicate with the GUI
    sigSpectrumUpdated = QtCore.Signal(object, object)  # x_data, y_data
    sigStatusUpdated = QtCore.Signal(str)
    sigScanFinished = QtCore.Signal()

    def on_activate(self):
        """
        Initializes the logic module.
        """
        self._nir_stage = self.nir_stage()
        self._nir_counter = self.nir_counter()
        self._savelogic = self.savelogic()

        # Default scan parameters
        self.start_position = 0.001  # in meters
        self.stop_position = 0.006  # 5 mm scan range
        self.num_points = 100
        self.time_per_point = 0.1  # in seconds

        # Data storage
        self.x_data = np.linspace(self.start_position, self.stop_position, self.num_points)
        self.y_data = np.zeros_like(self.x_data)
        self.spectrum_data = np.vstack((self.x_data, self.y_data)).T

        # Calibration parameters (Position -> Wavelength)
        self.calib_pos1 = 0.001
        self.calib_wl1 = 1300  # nm
        self.calib_pos2 = 0.006
        self.calib_wl2 = 1650  # nm
        self.use_calibration = False

        self._stop_requested = False
        self._is_scanning = False

    def start_scan(self):
        """
        Starts a new spectrometer scan in a separate thread.
        """
        if self._is_scanning:
            self.log.warning('A scan is already in progress.')
            return

        self._is_scanning = True
        self._stop_requested = False
        self.run_module_function(self._scan_thread)

    def stop_scan(self):
        """
        Requests to stop the current scan.
        """
        if self._is_scanning:
            self._stop_requested = True
            self.sigStatusUpdated.emit('Stopping scan...')

    def _scan_thread(self):
        """
        The main scanning routine.
        """
        try:
            self.sigStatusUpdated.emit('Starting scan...')
            positions = np.linspace(self.start_position, self.stop_position, self.num_points)
            counts = np.zeros_like(positions)

            # Move to start position before scan
            self.sigStatusUpdated.emit(f'Moving to start position: {positions[0]:.4f} m')
            self._nir_stage.move_abs({'lvf_position': positions[0]})
            self._wait_for_move()

            for i, pos in enumerate(positions):
                if self._stop_requested:
                    self.log.info('Scan stopped by user.')
                    break

                self.sigStatusUpdated.emit(f'Scanning point {i + 1}/{self.num_points} at {pos:.4f} m')
                self._nir_stage.move_abs({'lvf_position': pos})
                self._wait_for_move()

                # Acquire counts
                # Assuming the first counter channel is the NIR detector
                # The get_counter method returns counts/sec, so we multiply by time
                # This part might need adjustment based on your exact counter implementation
                samples_to_average = int(self.time_per_point * self._nir_counter.get_clock_frequency())
                if samples_to_average < 1:
                    samples_to_average = 1

                # We are calling get_counter from the provided snippet.
                # Assuming it returns an array of shape (channels, samples)
                # We take the mean of the samples for the first channel.
                raw_counts = self._nir_counter.get_counter(samples=samples_to_average)[0]
                counts[i] = np.mean(raw_counts)

                self.y_data = counts
                self.update_data_and_emit()

        except Exception as e:
            self.log.error(f'Error during scan: {e}', exc_info=True)
            self.sigStatusUpdated.emit(f'Error: {e}')
        finally:
            self._is_scanning = False
            self._stop_requested = False
            self.sigStatusUpdated.emit('Scan finished.')
            self.sigScanFinished.emit()

    def _wait_for_move(self, timeout=10):
        """
        Waits for the motorized stage to stop moving.
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            status_dict = self._nir_stage.get_status(['lvf_position'])
            # Status 0 means stopped
            if status_dict['lvf_position'][0] == 0:
                return
            time.sleep(0.05)
        self.log.warning('Move timed out.')

    def update_data_and_emit(self):
        """
        Updates the internal data representation and emits the sigSpectrumUpdated signal.
        """
        if self.use_calibration:
            # Linear interpolation/extrapolation for wavelength
            slope = (self.calib_wl2 - self.calib_wl1) / (self.calib_pos2 - self.calib_pos1)
            self.x_data = self.calib_wl1 + slope * (
                        np.linspace(self.start_position, self.stop_position, self.num_points) - self.calib_pos1)
        else:
            self.x_data = np.linspace(self.start_position, self.stop_position, self.num_points)

        self.spectrum_data = np.vstack((self.x_data, self.y_data)).T
        self.sigSpectrumUpdated.emit(self.x_data, self.y_data)

    def set_scan_parameters(self, start, stop, points, time_pp):
        self.start_position = start
        self.stop_position = stop
        self.num_points = int(points)
        self.time_per_point = time_pp
        self.update_data_and_emit()

    def set_calibration(self, params):
        self.calib_pos1 = params['pos1']
        self.calib_wl1 = params['wl1']
        self.calib_pos2 = params['pos2']
        self.calib_wl2 = params['wl2']
        self.use_calibration = params['use_calib']
        self.update_data_and_emit()

    def save_data(self, filetag=''):
        """
        Saves the current spectrum data.
        """
        if self.use_calibration:
            header = 'Wavelength (nm), Counts (counts/s)'
            default_name = 'NIR_Spectrum_WL'
        else:
            header = 'Position (m), Counts (counts/s)'
            default_name = 'NIR_Spectrum_POS'

        self._savelogic.save_data(data=self.spectrum_data,
                                  header=header,
                                  default_name=default_name,
                                  file_tag=filetag)
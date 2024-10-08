from core.connector import Connector
from logic.generic_logic import GenericLogic
from qtpy import QtCore
import numpy as np
from core.statusvariable import StatusVar
import time
import datetime
from core.util.mutex import Mutex
from collections import OrderedDict
from interface.simple_laser_interface import LaserState
class Pollogic(GenericLogic):
    """
    Logic for measureing countrate as a function of
    config example:
        pol_logic:
            module.Class: polarization_logic.Pollogic
                connector:
                    motor: 'rotationstage'
                    counterlogic: 'counter_logic'
    """
    motor = Connector(interface='MotorInterface')
    counterlogic = Connector(interface='CounterLogic')
    def on_activate(self):
        self.rotation_stage = self.motor()
        self.counter_logic = self.counterlogic()
        return

    def on_deactivate(self):
        pass

    ### Rotation stage
    def get_position(self):
        return self.rotation_stage.get_pos()['phi']

    def move_rel(self, step_size):
        self.rotation_stage.move_rel({'phi': step_size})

    def move_abs(self, position):
        self.rotation_stage.move_abs({'phi': position})

    ### Counter logic
    def start_counting(self):
        self.counter_logic.startCount()

    def stop_counting(self):
        self.counter_logic.stopCoun()

    def get_count_rate(self):
        data = self.counter_logic.countdata
        return data

    ### Measurement logic
    def measure_count_rate_for_each_position(self, start_pos, stop_pos, steps):
        positions = np.linsapce(start_pos, stop_pos, steps)
        data_array = np.zeros_like(positions)
        for i, pos in enumerate(positions):
            self.move_abs(pos)
            # wait for momenent to finish
            time.sleep(3)
            #count data
            self.start_counting()
            # wait for desired counting time (maybe set count frequency and count lenght in the logic to avoid having zeroes in the data)
            time.sleep(4)
            self.stop_counting()
            data_array[i] = np.mean(self.get_count_rate())
        # add save data??
        return positions, data_array
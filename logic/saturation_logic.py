# -*- coding: utf-8 -*-
"""
This module operates a confocal microsope.

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
from copy import copy
import time
import datetime
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

from logic.generic_logic import GenericLogic
from core.util.mutex import Mutex
from core.connector import Connector
from core.statusvariable import StatusVar


class OldConfigFileError(Exception):
    """ Exception that is thrown when an old config file is loaded.
    """
    def __init__(self):
        super().__init__('Old configuration file detected. Ignoring saturation history.')


class SaturationHistoryEntry(QtCore.QObject):
    """ This class contains all relevant parameters of a Saturation scan.
        It provides methods to extract, restore and serialize this data.
    """

    def __init__(self, saturation):
        """ Make a saturation data setting with default values. """
        super().__init__()

        # Reads in the maximal scanning range. The unit of that scan range is volts
        self.v_range = saturation._scanning_device._scanner_voltage_ranges[0]
        # Sets the current position to the center of the maximal scanning range
        self.current_v = (self.v_range[0] + self.v_range[1]) / 2
        # Sets the size of the scan to the maximal scanning range
        self.scan_v_range = self.v_range

        # Default value for the resolution of the scan
        self.resolution = 100

    def restore(self, saturation):
        """ Write data back into saturation logic and pull all the necessary strings """
        saturation.resolution = self.resolution
        saturation._current_v = self.current_v
        saturation.scan_v_range = np.copy(self.scan_v_range)


        saturation.initialize_image()
        try:
            if saturation.sat_image.shape == self.sat_image.shape:
                saturation.sat_image = np.copy(self.sat_image)
        except AttributeError:
            self.sat_image = np.copy(saturation.sat_image)


    def snapshot(self, saturation):
        """ Extract all necessary data from a saturation logic and keep it for later use """
        self.current_v = saturation._current_v
        self.scan_v_range = np.copy(saturation.scan_v_range)
        self.resolution = saturation.resolution

        self.sat_image = np.copy(saturation.sat_image)

    def serialize(self):
        """ Give out a dictionary that can be saved via the usual means """
        serialized = dict()
        serialized['current_v'] = self.current_v
        serialized['v_range'] = list(self.scan_v_range)
        serialized['resolution'] = self.resolution
        serialized['sat_image'] = self.sat_image
        return serialized

    def deserialize(self, serialized):
        """ Restore Saturation history object from a dict """
        if 'current_v' in serialized:
            self.current_v = serialized['current_v']
        if 'v_range' in serialized and len(serialized['v_range']) == 2:
            self.scan_v_range = serialized['v_range']
        if 'resolution' in serialized:
            self.resolution = serialized['resolution']
        if 'sat_image' in serialized:
            if isinstance(serialized['sat_image'], np.ndarray):
                self.sat_image = serialized['sat_image']
            else:
                raise OldConfigFileError()


class SaturationLogic(GenericLogic):
    """
    This is the Logic class for saturation scanning.
    """

    # declare connectors
    scanner1 = Connector(interface='SlowCounterInterface')
    #powermeter = Connector(interface='ThorlabsPM')
    savelogic = Connector(interface='SaveLogic')

    # status vars
    _clock_frequency = StatusVar('clock_frequency', 10)
    return_slowness = StatusVar(default=50)
    max_history_length = StatusVar(default=10)

    # signals
    signal_start_scanning = QtCore.Signal(str)
    signal_scan_complete = QtCore.Signal()
    signal_save_started = QtCore.Signal()
    signal_data_saved = QtCore.Signal()

    _signal_save_data = QtCore.Signal(object, object)

    sigImageSatInitialized = QtCore.Signal()

    signal_history_event = QtCore.Signal()

    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)

        #locking for thread safety
        self.threadlock = Mutex()

        # counter for scan_image
        self.stopRequested = False

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        """
        self._scanning_device = self.scanner1()
        self._save_logic = self.savelogic()

        # Reads in the maximal voltage range.
        self.v_range = self._scanning_device._scanner_voltage_ranges[0]
        # Reads in the equivalent coordinate range used by NationalInstrumentsXSeries
        self._x_range = self._scanning_device._scanner_position_ranges[0]

        # restore here ...
        self.history = []
        for i in reversed(range(1, self.max_history_length)):
            try:
                new_history_item = SaturationHistoryEntry(self)
                new_history_item.deserialize(
                    self._statusVariables['history_{0}'.format(i)])
                self.history.append(new_history_item)
            except KeyError:
                pass
            except OldConfigFileError:
                self.log.warning(
                    'Old style config file detected. History {0} ignored.'.format(i))
            except:
                self.log.warning(
                        'Restoring history {0} failed.'.format(i))
        try:
            new_state = SaturationHistoryEntry(self)
            new_state.deserialize(self._statusVariables['history_0'])
            new_state.restore(self)
        except:
            new_state = SaturationHistoryEntry(self)
            new_state.restore(self)
        finally:
            self.history.append(new_state)

        self.history_index = len(self.history) - 1

        # Sets connections between signals and functions
        self.signal_start_scanning.connect(self.start_scanner, QtCore.Qt.QueuedConnection)
        self._signal_save_data.connect(self._save_data, QtCore.Qt.QueuedConnection)


    def on_deactivate(self):
        """ Reverse steps of activation

        @return int: error code (0:OK, -1:error)
        """
        closing_state = SaturationHistoryEntry(self)
        closing_state.snapshot(self)
        self.history.append(closing_state)
        histindex = 0
        for state in reversed(self.history):
            self._statusVariables['history_{0}'.format(histindex)] = state.serialize()
            histindex += 1
        return 0

    def set_clock_frequency(self, clock_frequency):
        """Sets the frequency of the clock
        @param int clock_frequency: desired frequency of the clock
        @return int: error code (0:OK, -1:error)
        """
        self._clock_frequency = int(clock_frequency)
        #checks if scanner is still running
        if self.module_state() == 'locked':
            return -1
        else:
            return 0

    def start_scanning(self, calibration = False, tag='logic'):
        """Starts scanning
        @param bool calibration: calibration if true, emitter brightness scan if false
        @return int: error code (0:OK, -1:error)
        """
        self._calibration = calibration
        self.signal_start_scanning.emit(tag)
        return 0

    def stop_scanning(self):
        """Stops the scan
        @return int: error code (0:OK, -1:error)
        """
        with self.threadlock:
            if self.module_state() == 'locked':
                self.stopRequested = True
        return 0

    def initialize_image(self):
        """Initalization of the image.

        @return int: error code (0:OK, -1:error)
        """
        # x1: x-start-value, x2: x-end-value
        x1, x2 = self.image_x_range[0], self.image_x_range[1]
        # y1: x-start-value, y2: x-end-value
        y1, y2 = self.image_y_range[0], self.image_y_range[1]
        # z1: x-start-value, z2: x-end-value
        z1, z2 = self.image_z_range[0], self.image_z_range[1]

        # Checks if the x-start and x-end value are ok
        if x2 < x1:
            self.log.error(
                'x1 must be smaller than x2, but they are '
                '({0:.3f},{1:.3f}).'.format(x1, x2))
            return -1

        if self._zscan:
            if self.depth_img_is_xz:
                # creates an array of evenly spaced numbers over the interval
                # x1, x2 and the spacing is equal to xy_resolution
                self._X = np.linspace(x1, x2, self.xy_resolution)
            else:
                self._Y = np.linspace(y1, y2, self.xy_resolution)

            # Checks if the z-start and z-end value are ok
            if z2 < z1:
                self.log.error(
                    'z1 must be smaller than z2, but they are '
                    '({0:.3f},{1:.3f}).'.format(z1, z2))
                return -1
            # creates an array of evenly spaced numbers over the interval
            # z1, z2 and the spacing is equal to z_resolution
            self._Z = np.linspace(z1, z2, max(self.z_resolution, 2))
        else:
            # Checks if the y-start and y-end value are ok
            if y2 < y1:
                self.log.error(
                    'y1 must be smaller than y2, but they are '
                    '({0:.3f},{1:.3f}).'.format(y1, y2))
                return -1

            # prevents distorion of the image
            if (x2 - x1) >= (y2 - y1):
                self._X = np.linspace(x1, x2, max(self.xy_resolution, 2))
                self._Y = np.linspace(y1, y2, max(int(self.xy_resolution*(y2-y1)/(x2-x1)), 2))
            else:
                self._Y = np.linspace(y1, y2, max(self.xy_resolution, 2))
                self._X = np.linspace(x1, x2, max(int(self.xy_resolution*(x2-x1)/(y2-y1)), 2))

            # In case of 3D xy scan
            if self._3Dscan:
                # Checks if the z-start and z-end value are ok
                if z2 < z1:
                    self.log.error(
                        'z1 must be smaller than z2, but they are '
                        '({0:.3f},{1:.3f}).'.format(z1, z2))
                    return -1
                # creates an array of evenly spaced numbers over the interval
                # z1, z2 and the spacing is equal to z_resolution
                self._Z = np.linspace(z1, z2, max(self.z_resolution, 2))
                self._ZL = self._Z
                self._return_ZL = np.linspace(self._ZL[-1], self._ZL[0], self.return_slowness)

        self._XL = self._X
        self._YL = self._Y
        self._AL = np.zeros(self._XL.shape)

        # Arrays for retrace line
        self._return_XL = np.linspace(self._XL[-1], self._XL[0], self.return_slowness)
        self._return_AL = np.zeros(self._return_XL.shape)

        if self._zscan:
            self._image_vert_axis = self._Z
            # update image scan direction from setting
            self.depth_img_is_xz = self.depth_scan_dir_is_xz
            # depth scan is in xz plane
            if self.depth_img_is_xz:
                #self._image_horz_axis = self._X
                # creates an image where each pixel will be [x,y,z,counts]
                self.depth_image = np.zeros((
                        len(self._image_vert_axis),
                        len(self._X),
                        3 + len(self.get_scanner_count_channels())
                    ))

                self.depth_image[:, :, 0] = np.full(
                    (len(self._image_vert_axis), len(self._X)), self._XL)

                self.depth_image[:, :, 1] = self._current_y * np.ones(
                    (len(self._image_vert_axis), len(self._X)))

                z_value_matrix = np.full((len(self._X), len(self._image_vert_axis)), self._Z)
                self.depth_image[:, :, 2] = z_value_matrix.transpose()

            # depth scan is yz plane instead of xz plane
            else:
                #self._image_horz_axis = self._Y
                # creats an image where each pixel will be [x,y,z,counts]
                self.depth_image = np.zeros((
                        len(self._image_vert_axis),
                        len(self._Y),
                        3 + len(self.get_scanner_count_channels())
                    ))

                self.depth_image[:, :, 0] = self._current_x * np.ones(
                    (len(self._image_vert_axis), len(self._Y)))

                self.depth_image[:, :, 1] = np.full(
                    (len(self._image_vert_axis), len(self._Y)), self._YL)

                z_value_matrix = np.full((len(self._Y), len(self._image_vert_axis)), self._Z)
                self.depth_image[:, :, 2] = z_value_matrix.transpose()

                # now we are scanning along the y-axis, so we need a new return line along Y:
                self._return_YL = np.linspace(self._YL[-1], self._YL[0], self.return_slowness)
                self._return_AL = np.zeros(self._return_YL.shape)

            self.sigImageDepthInitialized.emit()

        # xy scan is in xy plane
        else:
            #self._image_horz_axis = self._X
            self._image_vert_axis = self._Y
            # creates an image where each pixel will be xy_image[y,x,axis/ch] = coord/count
            self.xy_image = np.zeros((
                    len(self._image_vert_axis),
                    len(self._X),
                    3 + len(self.get_scanner_count_channels())
                ))

            self.xy_image[:, :, 0] = np.full(
                (len(self._image_vert_axis), len(self._X)), self._XL)

            y_value_matrix = np.full((len(self._X), len(self._image_vert_axis)), self._Y)
            self.xy_image[:, :, 1] = y_value_matrix.transpose()

            self.xy_image[:, :, 2] = self._current_z * np.ones(
                (len(self._image_vert_axis), len(self._X)))

            # in case of 3D xy scan, use the z-start as the z coordinate
            if self._3Dscan:
                self.xy_image[:, :, 2] = z1 * np.ones((len(self._image_vert_axis), len(self._X)))

            self.sigImageXYInitialized.emit()
        return 0

    def start_scanner(self):
        """Setting up the scanner device and starts the scanning procedure

        @return int: error code (0:OK, -1:error)
        """
        self.module_state.lock()

        self._scanning_device.module_state.lock()
        if self.initialize_image() < 0:
            self._scanning_device.module_state.unlock()
            self.module_state.unlock()
            return -1

        clock_status = self._scanning_device.set_up_scanner_clock(
            clock_frequency=self._clock_frequency)

        if clock_status < 0:
            self._scanning_device.module_state.unlock()
            self.module_state.unlock()
            self.set_voltage('scanner')
            return -1

        scanner_status = self._scanning_device.set_up_scanner()

        if scanner_status < 0:
            self._scanning_device.close_scanner_clock()
            self._scanning_device.module_state.unlock()
            self.module_state.unlock()
            self.set_voltage('scanner')
            return -1

        self.signal_scan_lines_next.emit()
        return 0

    def kill_scanner(self):
        """Closing the scanner device.

        @return int: error code (0:OK, -1:error)
        """
        try:
            self._scanning_device.close_scanner()
        except Exception as e:
            self.log.exception('Could not close the scanner.')
        try:
            self._scanning_device.close_scanner_clock()
        except Exception as e:
            self.log.exception('Could not close the scanner clock.')
        try:
            self._scanning_device.module_state.unlock()
        except Exception as e:
            self.log.exception('Could not unlock scanning device.')

        return 0

    def set_voltage(self, tag, v=None):
        """Forwarding the desired new position from the GUI to the scanning device.

        @param string tag: TODO
        @param float v: if defined, changes laser intensity by setting voltage to v
        @return int: error code (0:OK, -1:error)
        """
        if v is not None:
            self._current_v = v
        # Calculate effective coordinate
        pos = (self._x_range[1] - self._x_range[0]) / (self.v_range[1] - self.v_range[0])
              * (self._current_v - self.v_range[0]) + self._x_range[0]

        # Checks if the scanner is still running
        if self.module_state() == 'locked' or self._scanning_device.module_state() == 'locked':
            return -1
        else:
            self._change_voltage(tag)
            self._scanning_device.scanner_set_position(x=pos)
            return 0

    def get_position(self):
        """ Get position from scanning device.

        @return list: with three entries x, y and z denoting the current
                      position in meters
        """
        return self._scanning_device.get_scanner_position()

    def get_scanner_axes(self):
        """ Get axes from scanning device.
          @return list(str): names of scanner axes
        """
        return self._scanning_device.get_scanner_axes()

    def get_scanner_count_channels(self):
        """ Get lis of counting channels from scanning device.
          @return list(str): names of counter channels
        """
        return self._scanning_device.get_scanner_count_channels()

    def _scan_line(self):
        """scanning an image in either depth or xy

        """
        image = self.depth_image if self._zscan else self.xy_image
        n_ch = len(self.get_scanner_axes())
        s_ch = len(self.get_scanner_count_channels())

        # stops scanning
        if self.stopRequested:
            with self.threadlock:
                # ------ Make a line from the end-scan position to the cursor position
                if not self._zscan:
                    # Line at which scan was terminated
                    stop_line = self._scan_counter - 1
                    rs = self.return_slowness
                    lsx = np.linspace(image[stop_line, 0, 0], self._current_x, rs)
                    lsy = np.linspace(image[stop_line, 0, 1], self._current_y, rs)
                    lsz = np.linspace(image[stop_line, 0, 2], self._current_z, rs)
                    if n_ch <= 3:
                        start_line = np.vstack([lsx, lsy, lsz][0:n_ch])
                    else:
                        start_line = np.vstack(
                            [lsx, lsy, lsz, np.ones(lsx.shape) * self._current_a])
                    # move to the start position of the scan, counts are thrown away
                    self._scanning_device.scan_line(start_line)
                else:
                    # ToDo: make a proper line for zscan instead of a jump
                    self.set_voltage('scanner')
                # ---------------------------------------------------------------------
                self.kill_scanner()
                self.stopRequested = False
                self.module_state.unlock()
                self.signal_xy_image_updated.emit()
                self.signal_depth_image_updated.emit()
                if self._zscan:
                    self._depth_line_pos = self._scan_counter
                else:
                    self._xy_line_pos = self._scan_counter
                # add new history entry
                new_history = SaturationHistoryEntry(self)
                new_history.snapshot(self)
                self.history.append(new_history)
                if len(self.history) > self.max_history_length:
                    self.history.pop(0)
                self.history_index = len(self.history) - 1
                return

        try:
            if self._scan_counter == 0 or self._continuation:
                self._continuation = False  # Do this only once in resumed scan
                # make a line from the current cursor position to
                # the starting position of the first scan line of the scan
                rs = self.return_slowness
                lsx = np.linspace(self._current_x, image[self._scan_counter, 0, 0], rs)
                lsy = np.linspace(self._current_y, image[self._scan_counter, 0, 1], rs)
                lsz = np.linspace(self._current_z, image[self._scan_counter, 0, 2], rs)
                if n_ch <= 3:
                    start_line = np.vstack([lsx, lsy, lsz][0:n_ch])
                else:
                    start_line = np.vstack(
                        [lsx, lsy, lsz, np.ones(lsx.shape) * self._current_a])
                # move to the start position of the scan, counts are thrown away
                start_line_counts = self._scanning_device.scan_line(start_line)
                if np.any(start_line_counts == -1):
                    self.stopRequested = True
                    self.signal_scan_lines_next.emit()
                    return

            # adjust z of line in image to current z before building the line
            # don't do that for 3D xy scan
            if not self._zscan and not self._3Dscan:
                z_shape = image[self._scan_counter, :, 2].shape
                image[self._scan_counter, :, 2] = self._current_z * np.ones(z_shape)

            # make a line in the scan, _scan_counter says which one it is
            lsx = image[self._scan_counter, :, 0]
            lsy = image[self._scan_counter, :, 1]
            lsz = image[self._scan_counter, :, 2]
            if n_ch <= 3:
                line = np.vstack([lsx, lsy, lsz][0:n_ch])
            else:
                line = np.vstack(
                    [lsx, lsy, lsz, np.ones(lsx.shape) * self._current_a])

            # scan the line in the scan
            if self._3Dscan:
                line_counts = self._scan_3d_line(line)
            else:
                line_counts = self._scanning_device.scan_line(line, pixel_clock=True)
            if np.any(line_counts == -1):
                self.stopRequested = True
                self.signal_scan_lines_next.emit()
                return

            # make a line to go to the starting position of the next scan line
            if self.depth_img_is_xz or not self._zscan:
                if n_ch <= 3:
                    return_line = np.vstack([
                        self._return_XL,
                        image[self._scan_counter, 0, 1] * np.ones(self._return_XL.shape),
                        image[self._scan_counter, 0, 2] * np.ones(self._return_XL.shape)
                    ][0:n_ch])
                else:
                    return_line = np.vstack([
                            self._return_XL,
                            image[self._scan_counter, 0, 1] * np.ones(self._return_XL.shape),
                            image[self._scan_counter, 0, 2] * np.ones(self._return_XL.shape),
                            np.ones(self._return_XL.shape) * self._current_a
                        ])
            else:
                if n_ch <= 3:
                    return_line = np.vstack([
                            image[self._scan_counter, 0, 1] * np.ones(self._return_YL.shape),
                            self._return_YL,
                            image[self._scan_counter, 0, 2] * np.ones(self._return_YL.shape)
                        ][0:n_ch])
                else:
                    return_line = np.vstack([
                            image[self._scan_counter, 0, 1] * np.ones(self._return_YL.shape),
                            self._return_YL,
                            image[self._scan_counter, 0, 2] * np.ones(self._return_YL.shape),
                            np.ones(self._return_YL.shape) * self._current_a
                        ])

            # return the scanner to the start of next line, counts are thrown away
            return_line_counts = self._scanning_device.scan_line(return_line)
            if np.any(return_line_counts == -1):
                self.stopRequested = True
                self.signal_scan_lines_next.emit()
                return

            # update image with counts from the line we just scanned
            if self._zscan:
                if self.depth_img_is_xz:
                    self.depth_image[self._scan_counter, :, 3:3 + s_ch] = line_counts
                else:
                    self.depth_image[self._scan_counter, :, 3:3 + s_ch] = line_counts
                self.signal_depth_image_updated.emit()
            else:
                self.xy_image[self._scan_counter, :, 3:3 + s_ch] = line_counts
                self.signal_xy_image_updated.emit()

            # next line in scan
            self._scan_counter += 1

            # stop scanning when last line scan was performed and makes scan not continuable
            if self._scan_counter >= np.size(self._image_vert_axis):
                if not self.permanent_scan:
                    self.stop_scanning()
                    if self._zscan:
                        self._zscan_continuable = False
                    else:
                        self._xyscan_continuable = False
                else:
                    self._scan_counter = 0

            self.signal_scan_lines_next.emit()
        except:
            self.log.exception('The scan went wrong, killing the scanner.')
            self.stop_scanning()
            self.signal_scan_lines_next.emit()


    def _scan_3d_line(self, line_path=None):
        """ Scan a line for 3D xy scan and return both the maximum counts at each XY coordinate
        and the corresponding Z coordinate that gave the maximum count.
        It is assumed that the current position is already the start position, so no gradual
        scan line is made to the start position.

        @param float[c, m] line_path: array of coordinate tuples defining the coordinate points:
            m = samples per line, c enumerates xyza axis

        @return float[m, n]: transposed array, where each column except last gives counts on a scan channel,
        and the last column is overwritten with Z coordinate that gives the maximum count for the first scanner channel.
        m is samples per X-line, n is number of scanner channels.
        Returned counts are for the middle position of the Z scan range.

        The input array looks for a 3D xy scan of 5x5 points at the position y=1, z=-2
        like the following:
            [ [1, 2, 3, 4, 5], [1, 1, 1, 1, 1], [-2, -2, -2, -2] ]
        n is the number of scanner axes, which can vary. Typical values are 2 for galvo scanners,
        3 for xyz scanners and 4 for xyz scanners with a special function on the a axis.
        """
        if not isinstance(line_path, (frozenset, list, set, tuple, np.ndarray, ) ):
            self.log.error('Given line_path list is not array type.')
            return np.array([[-1.]])
        x_len = line_path.shape[1]  # Number of points in x-scan

        # Prepare a template for z-line scan path: number of samples per line is determined by Z resolution
        zline = np.zeros((len(line_path), len(self._ZL)), dtype=np.float64)
        zline[1, :] = np.ones(self._ZL.shape) * line_path[1, 0]  # Y coordinate is constant
        zline[2, :] = self._ZL
        if len(line_path) > 3:
            zline[3, :] = np.ones(self._ZL.shape) * line_path[3, 0]

        # Return line is similar to forward line, but differs in resolution given by self._return_ZL
        return_zline = np.zeros((len(line_path), len(self._return_ZL)), dtype=np.float64)
        return_zline[1, :] = np.ones(self._return_ZL.shape) * line_path[1, 0]  # Y coordinate is constant
        return_zline[2, :] = self._return_ZL
        if len(line_path) > 3:
            return_zline[3, :] = np.ones(self._return_ZL.shape) * line_path[3, 0]

        # Iterate through X coordinate in the x-line
        for x_count in range(x_len):
            # Make a Z scan line
            # Fix X coordinate
            zline[0, :] = line_path[0, x_count] * np.ones(self._ZL.shape)
            # scan along Z
            line_counts = self._scanning_device.scan_line(zline, pixel_clock=True)
            if np.any(line_counts == -1):
                return np.array([[-1.]])

            # Initialize the output array on the first scan
            if x_count == 0:
                # Each column keeps counts for each scan channel, last column is for Z coordinate
                all_data = np.zeros((x_len, len(line_counts[0, :])), dtype=np.float64)

            # Save counts from middle of the Z range
            all_data[x_count, :] = line_counts[len(self._ZL)//2, :]
            # Overwrite the last column with Z coordinate from a fit of the first scan channel
            all_data[x_count, -1] = self._maximizingZ(self._ZL, line_counts[:, 0])
            if all_data[x_count, -1] == -1:
                self.log.error('Fit of z-scan profile failed. Try to increase resolution along z direction.')
                return np.array([[-1.]])

            # Make a line to go to the starting position of the next Z scan line
            # Fix X coordinate
            return_zline[0, :] = line_path[0, x_count] * np.ones(self._return_ZL.shape)
            # return the scanner to the start of next Z line, counts are thrown away
            return_line_counts = self._scanning_device.scan_line(return_zline)
            if np.any(return_line_counts == -1):
                return np.array([[-1.]])

        return all_data

    def _maximizingZ(self, x=None, y=None):
        """ Fit input data y=y(x) with a Gaussian function and return the argument X that maximizes Y.

        @param float[n] x: array of argument values
        @param float[n] y: array of data values to fit

        @return float: Fitted x value that maximizes y=y(x) dependence
        """
        # Define the fit function
        def gauss(x, *params):
            A, mu, sigma, c = params
            return A*np.exp(-(x-mu)**2/(2*sigma**2)) + c

        # Initial guess values
        ymax = np.amax(y)
        ymin = np.amin(y)
        maxind = np.argmax(y)
        p0 = [ymax - ymin, x[maxind], 0.4e-6, ymin]  # A0, mu0, sigma0, c0
        try:
            pfit, _ = curve_fit(gauss, x, y, p0)
            # Return mu from the fit
            return pfit[1] * 1e6  # Convert coordinate to um
        except:
            return -1

    def save_xy_data(self, colorscale_range=None, percentile_range=None, block=True):
        """ Save the current confocal xy data to file.

        Two files are created.  The first is the imagedata, which has a text-matrix of count values
        corresponding to the pixel matrix of the image.  Only count-values are saved here.

        The second file saves the full raw data with x, y, z, and counts at every pixel.

        A figure is also saved.

        @param: list colorscale_range (optional) The range [min, max] of the display colour scale (for the figure)

        @param: list percentile_range (optional) The percentile range [min, max] of the color scale 
        
        @param: bool block (optional) If False, return immediately; if True, block until save completes."""

        if block:
            self._save_xy_data(colorscale_range, percentile_range)
        else:
            self._signal_save_xy.emit(colorscale_range, percentile_range)

    @QtCore.Slot(object, object)
    def _save_xy_data(self, colorscale_range=None, percentile_range=None):
        """ Execute save operation. Slot for _signal_save_xy.
        """
        self.signal_save_started.emit()
        filepath = self._save_logic.get_path_for_module('Confocal')
        timestamp = datetime.datetime.now()
        # Prepare the metadata parameters (common to both saved files):
        parameters = OrderedDict()

        parameters['X image min (m)'] = self.image_x_range[0]
        parameters['X image max (m)'] = self.image_x_range[1]
        parameters['X image range (m)'] = self.image_x_range[1] - self.image_x_range[0]

        parameters['Y image min'] = self.image_y_range[0]
        parameters['Y image max'] = self.image_y_range[1]
        parameters['Y image range'] = self.image_y_range[1] - self.image_y_range[0]

        parameters['XY resolution (samples per range)'] = self.xy_resolution
        parameters['XY Image at z position (m)'] = self._current_z

        parameters['Clock frequency of scanner (Hz)'] = self._clock_frequency
        parameters['Return Slowness (Steps during retrace line)'] = self.return_slowness

        # Prepare a figure to be saved
        figure_data = self.xy_image[:, :, 3]
        image_extent = [self.image_x_range[0],
                        self.image_x_range[1],
                        self.image_y_range[0],
                        self.image_y_range[1]]
        axes = ['X', 'Y']
        crosshair_pos = [self.get_position()[0], self.get_position()[1]]

        figs = {ch: self.draw_figure(data=self.xy_image[:, :, 3 + n],
                                     image_extent=image_extent,
                                     scan_axis=axes,
                                     cbar_range=colorscale_range,
                                     percentile_range=percentile_range,
                                     crosshair_pos=crosshair_pos)
                for n, ch in enumerate(self.get_scanner_count_channels())}

        # Save the image data and figure
        for n, ch in enumerate(self.get_scanner_count_channels()):
            # data for the text-array "image":
            image_data = OrderedDict()
            image_data['Confocal pure XY scan image data without axis.\n'
                'The upper left entry represents the signal at the upper left pixel position.\n'
                'A pixel-line in the image corresponds to a row '
                'of entries where the Signal is in counts/s:'] = self.xy_image[:, :, 3 + n]

            filelabel = 'confocal_xy_image_{0}'.format(ch.replace('/', ''))
            self._save_logic.save_data(image_data,
                                       filepath=filepath,
                                       timestamp=timestamp,
                                       parameters=parameters,
                                       filelabel=filelabel,
                                       fmt='%.6e',
                                       delimiter='\t',
                                       plotfig=figs[ch])

        # prepare the full raw data in an OrderedDict:
        data = OrderedDict()
        data['x position (m)'] = self.xy_image[:, :, 0].flatten()
        data['y position (m)'] = self.xy_image[:, :, 1].flatten()
        data['z position (m)'] = self.xy_image[:, :, 2].flatten()

        for n, ch in enumerate(self.get_scanner_count_channels()):
            data['count rate {0} (Hz)'.format(ch)] = self.xy_image[:, :, 3 + n].flatten()

        # Save the raw data to file
        filelabel = 'confocal_xy_data'
        self._save_logic.save_data(data,
                                   filepath=filepath,
                                   timestamp=timestamp,
                                   parameters=parameters,
                                   filelabel=filelabel,
                                   fmt='%.6e',
                                   delimiter='\t')

        self.log.debug('Confocal Image saved.')
        self.signal_xy_data_saved.emit()
        return

    def draw_figure(self, data, image_extent, scan_axis=None, cbar_range=None, percentile_range=None,  crosshair_pos=None):
        """ Create a 2-D color map figure of the scan image.

        @param: array data: The NxM array of count values from a scan with NxM pixels.

        @param: list image_extent: The scan range in the form [hor_min, hor_max, ver_min, ver_max]

        @param: list axes: Names of the horizontal and vertical axes in the image

        @param: list cbar_range: (optional) [color_scale_min, color_scale_max].  If not supplied then a default of
                                 data_min to data_max will be used.

        @param: list percentile_range: (optional) Percentile range of the chosen cbar_range.

        @param: list crosshair_pos: (optional) crosshair position as [hor, vert] in the chosen image axes.

        @return: fig fig: a matplotlib figure object to be saved to file.
        """
        if scan_axis is None:
            scan_axis = ['X', 'Y']

        # If no colorbar range was given, take full range of data
        if cbar_range is None:
            cbar_range = [np.min(data), np.max(data)]

        # Scale color values using SI prefix
        prefix = ['', 'k', 'M', 'G']
        prefix_count = 0
        image_data = data
        draw_cb_range = np.array(cbar_range)
        image_dimension = image_extent.copy()

        while draw_cb_range[1] > 1000:
            image_data = image_data/1000
            draw_cb_range = draw_cb_range/1000
            prefix_count = prefix_count + 1

        c_prefix = prefix[prefix_count]


        # Scale axes values using SI prefix
        axes_prefix = ['', 'm', r'$\mathrm{\mu}$', 'n']
        x_prefix_count = 0
        y_prefix_count = 0

        while np.abs(image_dimension[1]-image_dimension[0]) < 1:
            image_dimension[0] = image_dimension[0] * 1000.
            image_dimension[1] = image_dimension[1] * 1000.
            x_prefix_count = x_prefix_count + 1

        while np.abs(image_dimension[3] - image_dimension[2]) < 1:
            image_dimension[2] = image_dimension[2] * 1000.
            image_dimension[3] = image_dimension[3] * 1000.
            y_prefix_count = y_prefix_count + 1

        x_prefix = axes_prefix[x_prefix_count]
        y_prefix = axes_prefix[y_prefix_count]

        # Use qudi style
        plt.style.use(self._save_logic.mpl_qd_style)

        # Create figure
        fig, ax = plt.subplots()

        # Create image plot
        cfimage = ax.imshow(image_data,
                            cmap=plt.get_cmap('inferno'), # reference the right place in qd
                            origin="lower",
                            vmin=draw_cb_range[0],
                            vmax=draw_cb_range[1],
                            interpolation='none',
                            extent=image_dimension
                            )

        ax.set_aspect(1)
        ax.set_xlabel(scan_axis[0] + ' position (' + x_prefix + 'm)')
        ax.set_ylabel(scan_axis[1] + ' position (' + y_prefix + 'm)')
        ax.spines['bottom'].set_position(('outward', 10))
        ax.spines['left'].set_position(('outward', 10))
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.get_xaxis().tick_bottom()
        ax.get_yaxis().tick_left()

        # draw the crosshair position if defined
        if crosshair_pos is not None:
            trans_xmark = mpl.transforms.blended_transform_factory(
                ax.transData,
                ax.transAxes)

            trans_ymark = mpl.transforms.blended_transform_factory(
                ax.transAxes,
                ax.transData)

            ax.annotate('', xy=(crosshair_pos[0]*np.power(1000,x_prefix_count), 0),
                        xytext=(crosshair_pos[0]*np.power(1000,x_prefix_count), -0.01), xycoords=trans_xmark,
                        arrowprops=dict(facecolor='#17becf', shrink=0.05),
                        )

            ax.annotate('', xy=(0, crosshair_pos[1]*np.power(1000,y_prefix_count)),
                        xytext=(-0.01, crosshair_pos[1]*np.power(1000,y_prefix_count)), xycoords=trans_ymark,
                        arrowprops=dict(facecolor='#17becf', shrink=0.05),
                        )

        # Draw the colorbar
        cbar = plt.colorbar(cfimage, shrink=0.8)#, fraction=0.046, pad=0.08, shrink=0.75)
        cbar.set_label('Fluorescence (' + c_prefix + 'c/s)')

        # remove ticks from colorbar for cleaner image
        cbar.ax.tick_params(which=u'both', length=0)

        # If we have percentile information, draw that to the figure
        if percentile_range is not None:
            cbar.ax.annotate(str(percentile_range[0]),
                             xy=(-0.3, 0.0),
                             xycoords='axes fraction',
                             horizontalalignment='right',
                             verticalalignment='center',
                             rotation=90
                             )
            cbar.ax.annotate(str(percentile_range[1]),
                             xy=(-0.3, 1.0),
                             xycoords='axes fraction',
                             horizontalalignment='right',
                             verticalalignment='center',
                             rotation=90
                             )
            cbar.ax.annotate('(percentile)',
                             xy=(-0.3, 0.5),
                             xycoords='axes fraction',
                             horizontalalignment='right',
                             verticalalignment='center',
                             rotation=90
                             )
        self.signal_draw_figure_completed.emit()
        return fig

    ##################################### History actions ########################################

    def history_forward(self):
        """ Move forward in confocal image history.
        """
        if self.history_index < len(self.history) - 1:
            self.history_index += 1
            self.history[self.history_index].restore(self)
            self.signal_xy_image_updated.emit()
            self.signal_depth_image_updated.emit()
            self.signal_tilt_correction_update.emit()
            self.signal_tilt_correction_active.emit(self._scanning_device.tiltcorrection)
            self._change_position('history')
            self.signal_history_event.emit()

    def history_back(self):
        """ Move backwards in confocal image history.
        """
        if self.history_index > 0:
            self.history_index -= 1
            self.history[self.history_index].restore(self)
            self.signal_xy_image_updated.emit()
            self.signal_depth_image_updated.emit()
            self.signal_tilt_correction_update.emit()
            self.signal_tilt_correction_active.emit(self._scanning_device.tiltcorrection)
            self._change_position('history')
            self.signal_history_event.emit()

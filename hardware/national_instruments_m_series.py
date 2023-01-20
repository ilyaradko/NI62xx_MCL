# -*- coding: utf-8 -*-

"""
This file contains the Qudi Hardware module NICard class.

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
import re

import PyDAQmx as daq

from core.module import Base
from core.configoption import ConfigOption
from interface.slow_counter_interface import SlowCounterInterface
from interface.slow_counter_interface import SlowCounterConstraints
from interface.slow_counter_interface import CountingMode
from interface.odmr_counter_interface import ODMRCounterInterface
from interface.confocal_scanner_interface import ConfocalScannerInterface
from .national_instruments_x_series import NationalInstrumentsXSeries
from core.connector import Connector


class NationalInstrumentsMSeries(NationalInstrumentsXSeries):
    """ A National Instruments device that can count and control microvave generators.

    !!!!!! NI USB 63XX, 62XX, NI PCIe 63XX, 62XX, and NI PXIe 63XX, 62XX DEVICES ONLY !!!!!!

    See [National Instruments X Series Documentation](@ref nidaq-x-series) for details.
    National Instrument M series (62xx) supports less counters and has to be provided with an external scanner clock.

    stable: Kay Jahnke, Alexander Stark, Ilya Radko

    Example config for copy-paste:

    ni6221:
        module.Class: 'national_instruments_m_series.NationalInstrumentsMSeries'
        photon_sources:
            - '/Dev1/PFI8'
        #    - '/Dev1/PFI9'
        clock_channel: '/Dev1/Ctr0'
        default_clock_frequency: 100 # optional, in Hz
        counter_channels:
            - '/Dev1/Ctr1'
        counter_ai_channels:
            - '/Dev1/AI0'
        counter_voltage_range: [-10, 10]
        default_scanner_clock_frequency: 100 # optional, in Hz
        scanner_clock_channel: '/Dev1/Ctr2'
        pixel_clock_channel: '/Dev1/PFI6'
        scanner_ao_channels:
            - '/Dev1/AO0'
            - '/Dev1/AO1'
            - '/Dev1/AO2'
            - '/Dev1/AO3'
        scanner_ai_channels:
            - '/Dev1/AI1'
        scanner_counter_channels:
            - '/Dev1/Ctr3'
        scanner_voltage_ranges:
            - [-10, 10]
            - [-10, 10]
            - [-10, 10]
            - [-10, 10]
        scanner_position_ranges:
            - [0e-6, 200e-6]
            - [0e-6, 200e-6]
            - [-100e-6, 100e-6]
            - [-10, 10]

        odmr_trigger_channel: '/Dev1/PFI7'

        gate_in_channel: '/Dev1/PFI9'
        default_samples_number: 50
        max_counts: 3e7
        read_write_timeout: 10
        counting_edge_rising: True

    """

    # declare connectors
    scannerclock = Connector(interface='EmptyInterface')

    # config options
    _photon_sources = ConfigOption('photon_sources', list(), missing='warn')

    # slow counter
    _clock_channel = ConfigOption('clock_channel', missing='error')
    _default_clock_frequency = ConfigOption('default_clock_frequency', 100, missing='info')
    _counter_channels = ConfigOption('counter_channels', missing='error')
    _counter_ai_channels = ConfigOption('counter_ai_channels', list(), missing='info')
    _counter_voltage_range = ConfigOption('counter_voltage_range', [-10, 10], missing='info')

    # confocal scanner
    _default_scanner_clock_frequency = ConfigOption('default_scanner_clock_frequency', 100, missing='info')
    _scanner_clock_channel = ConfigOption('scanner_clock_channel', missing='error')
    _pixel_clock_channel = ConfigOption('pixel_clock_channel', None)
    _scanner_ai_channels = ConfigOption('scanner_ai_channels', list(), missing='info')
    _scanner_counter_channels = ConfigOption('scanner_counter_channels', list(), missing='warn')
    _scanner_position_ranges = ConfigOption('scanner_position_ranges', missing='error')

    # inherited from NationalInstrumentsXSeries, but not needed and hence changed to 'info'
    _scanner_ao_channels = ConfigOption('scanner_ao_channels', missing='info')
    _scanner_voltage_ranges = ConfigOption('scanner_voltage_ranges', missing='info')

    # odmr
    _odmr_trigger_channel = ConfigOption('odmr_trigger_channel', missing='error')
    _odmr_trigger_line = ConfigOption('odmr_trigger_line', 'Dev1/port0/line0', missing='warn')
    _odmr_switch_line = ConfigOption('odmr_switch_line', 'Dev1/port0/line1', missing='warn')

    _gate_in_channel = ConfigOption('gate_in_channel', missing='error')
    # number of readout samples, mainly used for gated counter
    _default_samples_number = ConfigOption('default_samples_number', 50, missing='info')
    # used as a default for expected maximum counts
    _max_counts = ConfigOption('max_counts', default=3e7)
    # timeout for the Read or/and write process in s
    _RWTimeout = ConfigOption('read_write_timeout', default=10)
    _counting_edge_rising = ConfigOption('counting_edge_rising', default=True)

    def on_activate(self):
        """ Starts up the NI Card at activation.
        """
        self._external_scanner_device = self.scannerclock()

        # the tasks used on that hardware device:
        self._counter_daq_tasks = list()
        self._counter_analog_daq_task = None
        self._clock_daq_task = None
        self._scanner_clock_daq_task = None     # Fictive value indicating whether external clock is set up
        self._scanner_counter_daq_tasks = list()
        self._line_length = None
        self._odmr_length = None
        self._gated_counter_daq_task = None
        self._scanner_analog_daq_task = None
        self._odmr_pulser_daq_task = None
        self._oversampling = 0
        self._lock_in_active = False

        self._photon_sources = self._photon_sources if self._photon_sources is not None else list()
        self._scanner_counter_channels = self._scanner_counter_channels if self._scanner_counter_channels is not None else list()
        self._scanner_ai_channels = self._scanner_ai_channels if self._scanner_ai_channels is not None else list()

        # handle all the parameters given by the config
        self._current_position = np.zeros(len(self._scanner_position_ranges))

        if len(self._scanner_counter_channels) + len(self._scanner_ai_channels) < 1:
            self.log.error(
                'Specify at least one counter or analog input channel for the scanner!')

    def on_deactivate(self):
        """ Shut down the NI card.
        """

        self.reset_hardware()

    # =================== SlowCounterInterface Commands ========================

    def set_up_clock(self, clock_frequency=None, clock_channel=None, scanner=False, idle=False):
        """ Configures the hardware clock of the NiDAQ card to give the timing.

        @param float clock_frequency: if defined, this sets the frequency of
                                      the clock in Hz
        @param string clock_channel: if defined, this is the physical channel
                                     of the clock within the NI card.
        @param bool scanner: if set to True method will set up a clock function
                             for the scanner, otherwise a clock function for a
                             counter will be set.
        @param bool idle: set whether idle situation of the counter (where
                          counter is doing nothing) is defined as
                                True  = 'Voltage High/Rising Edge'
                                False = 'Voltage Low/Falling Edge'

        @return int: error code (0:OK, -1:error)
        """

        # In M-series NI cards, there are only two counter,
        # which prohibits running simultaneously a counter and a scanner
        if self._clock_daq_task is not None or self._scanner_clock_daq_task is not None:
            self.log.error('Another counting task (counter or scanner) is already running. Close that one first.')
            return -1

        # assign the clock frequency, if given
        if clock_frequency is not None:
            if not scanner:
                self._clock_frequency = float(clock_frequency)
            else:
                self._scanner_clock_frequency = float(clock_frequency)
        else:
            if not scanner:
                self._clock_frequency = self._default_clock_frequency
            else:
                self._scanner_clock_frequency = self._default_scanner_clock_frequency

        # use the correct clock in this method
        if scanner:
            my_clock_frequency = self._scanner_clock_frequency
        else:
            my_clock_frequency = self._clock_frequency

        # assign the clock channel, if given
        if clock_channel is not None:
            if not scanner:
                self._clock_channel = clock_channel
            else:
                self._scanner_clock_channel = clock_channel

        # use the correct clock channel in this method
        if scanner:
            my_clock_channel = self._scanner_clock_channel
        else:
            my_clock_channel = self._clock_channel

        # check whether only one clock pair is available, since some NI cards
        # only one clock channel pair.
        if self._scanner_clock_channel == self._clock_channel:
            if not ((self._clock_daq_task is None) and (self._scanner_clock_daq_task is None)):
                self.log.error(
                    'Only one clock channel is available!\n'
                    'Another clock is already running, close this one first '
                    'in order to use it for your purpose!')
                return -1

        if scanner:
            # retval will receive number of clocks per pixel or -1 for error
            retval = self._external_scanner_device.set_up_clock(
                clock_frequency=my_clock_frequency,
                clock_channel=my_clock_channel)
            # Check for error
            if retval == -1:
                return retval
            # In case of success, set self._scanner_clock_daq_task to not None (any fictive value)
            else:
                self._scanner_clock_daq_task = 1
                self._clocks_per_pixel = retval
                return 0

        # Create handle for task, this task will generate pulse signal for
        # photon counting
        my_clock_daq_task = daq.TaskHandle()

        # Adjust the idle state if necessary
        my_idle = daq.DAQmx_Val_High if idle else daq.DAQmx_Val_Low
        try:
            # create task for clock
            task_name = 'CounterClock'
            daq.DAQmxCreateTask(task_name, daq.byref(my_clock_daq_task))

            # create a digital clock channel with specific clock frequency:
            daq.DAQmxCreateCOPulseChanFreq(
                # The task to which to add the channels
                my_clock_daq_task,
                # which channel is used?
                my_clock_channel,
                # Name to assign to task (NIDAQ uses by # default the physical channel name as
                # the virtual channel name. If name is specified, then you must use the name
                # when you refer to that channel in other NIDAQ functions)
                'Clock Producer',
                # units, Hertz in our case
                daq.DAQmx_Val_Hz,
                # idle state
                my_idle,
                # initial delay
                0,
                # pulse frequency, divide by 2 such that length of semi period = count_interval
                my_clock_frequency,
                # duty cycle of pulses, 0.5 such that high and low duration are both
                # equal to count_interval
                0.5)

            # Configure Implicit Timing.
            # Set timing to continuous, i.e. set only the number of samples to
            # acquire or generate without specifying timing:
            daq.DAQmxCfgImplicitTiming(
                # Define task
                my_clock_daq_task,
                # Sample Mode: set the task to generate a continuous amount of running samples
                daq.DAQmx_Val_ContSamps,
                # buffer length which stores temporarily the number of generated samples
                1000)

            # actually start the preconfigured clock task
            daq.DAQmxStartTask(my_clock_daq_task)
            self._clock_daq_task = my_clock_daq_task
        except:
            self.log.exception('Error while setting up clock.')
            return -1
        return 0


    def set_up_counter(self,
                       counter_channels=None,
                       sources=None,
                       clock_channel=None,
                       counter_buffer=None, odmr=False):
        """ This is a modified version of set_up_counter() from national_instruments_x_series to
        allow seting up a counter for ODMR if hardware trigger is not available or if the counter
        has only two channels. In this case, a software trigger is used instead, which requires
        reading the most recent counts arrived to NI card instead of those since previous read.

        Configures the actual counter with a given clock.

        @param list(str) counter_channels: optional, physical channel of the counter
        @param list(str) sources: optional, physical channel where the photons
                                  are to count from
        @param str clock_channel: optional, specifies the clock channel for the
                                  counter
        @param int counter_buffer: optional, a buffer of specified integer
                                   length, where in each bin the count numbers
                                   are saved.

        @return int: error code (0:OK, -1:error)
        """

        if self._clock_daq_task is None and clock_channel is None:
            self.log.error('No clock running, call set_up_clock before starting the counter.')
            return -1

        if len(self._counter_daq_tasks) > 0:
            self.log.error('Another counter is already running, close this one first.')
            return -1

        my_counter_channels = counter_channels if counter_channels else self._counter_channels
        my_photon_sources = sources if sources else self._photon_sources
        my_clock_channel = clock_channel if clock_channel else self._clock_channel

        if len(my_photon_sources) < len(my_counter_channels):
            self.log.error('You have given {0} sources but {1} counting channels.'
                           'Please give an equal or greater number of sources.'
                           ''.format(len(my_photon_sources), len(my_counter_channels)))
            return -1

        try:
            for i, ch in enumerate(my_counter_channels):
                # This task will count photons with binning defined by the clock_channel
                task = daq.TaskHandle()  # Initialize a Task
                # Create task for the counter
                daq.DAQmxCreateTask('Counter{0}'.format(i), daq.byref(task))
                # Create a Counter Input which samples with Semi-Periodes the Channel.
                # set up semi period width measurement in photon ticks, i.e. the width
                # of each pulse (high and low) generated by pulse_out_task is measured
                # in photon ticks.
                #   (this task creates a channel to measure the time between state
                #    transitions of a digital signal and adds the channel to the task
                #    you choose)
                daq.DAQmxCreateCISemiPeriodChan(
                    # define to which task to connect this function
                    task,
                    # use this counter channel
                    ch,
                    # name to assign to it
                    'Counter Channel {0}'.format(i),
                    # expected minimum count value
                    0,
                    # Expected maximum count value
                    self._max_counts / 2 / self._clock_frequency,
                    # units of width measurement, here photon ticks
                    daq.DAQmx_Val_Ticks,
                    # empty extra argument
                    '')

                # Set the Counter Input to a Semi Period input Terminal.
                # Connect the pulses from the counter clock to the counter channel
                daq.DAQmxSetCISemiPeriodTerm(
                        # The task to which to add the counter channel.
                        task,
                        # use this counter channel
                        ch,
                        # assign a named Terminal
                        my_clock_channel + 'InternalOutput')

                # Set a Counter Input Control Timebase Source.
                # Specify the terminal of the timebase which is used for the counter:
                # Define the source of ticks for the counter as self._photon_source for
                # the Scanner Task.
                daq.DAQmxSetCICtrTimebaseSrc(
                    # define to which task to connect this function
                    task,
                    # counter channel
                    ch,
                    # counter channel to output the counting results
                    my_photon_sources[i])

                # Configure Implicit Timing.
                # Set timing to continuous, i.e. set only the number of samples to
                # acquire or generate without specifying timing:
                daq.DAQmxCfgImplicitTiming(
                    # define to which task to connect this function
                    task,
                    # Sample Mode: Acquire or generate samples until you stop the task.
                    daq.DAQmx_Val_ContSamps,
                    # buffer length which stores  temporarily the number of generated samples
                    1000)

                # Set the Read point Relative To an operation.
                # Specifies the point in the buffer at which to begin a read operation.
                # Here we read most recent recorded samples:
                if not odmr:
                    daq.DAQmxSetReadRelativeTo(
                        # define to which task to connect this function
                        task,
                        # Start reading samples relative to the last sample returned by the previously.
                        daq.DAQmx_Val_CurrReadPos)
                else:
                    daq.DAQmxSetReadRelativeTo(
                        # define to which task to connect this function
                        task,
                        # Start reading samples relative to the last sample returned by the previously.
                        daq.DAQmx_Val_MostRecentSamp)

                # Set the Read Offset.
                # Specifies an offset in samples per channel at which to begin a read
                # operation. This offset is relative to the location you specify with
                # RelativeTo. Here we set the Offset to 0 for multiple samples:
                daq.DAQmxSetReadOffset(task, 0)

                # Set Read OverWrite Mode.
                # Specifies whether to overwrite samples in the buffer that you have
                # not yet read. Unread data in buffer will be overwritten:
                daq.DAQmxSetReadOverWrite(
                    task,
                    daq.DAQmx_Val_DoNotOverwriteUnreadSamps)
                # add task to counter task list
                self._counter_daq_tasks.append(task)

                # Counter analog input task
                if len(self._counter_ai_channels) > 0:
                    atask = daq.TaskHandle()

                    daq.DAQmxCreateTask('CounterAnalogIn', daq.byref(atask))

                    daq.DAQmxCreateAIVoltageChan(
                        atask,
                        ', '.join(self._counter_ai_channels),
                        'Counter Analog In',
                        daq.DAQmx_Val_RSE,
                        self._counter_voltage_range[0],
                        self._counter_voltage_range[1],
                        daq.DAQmx_Val_Volts,
                        ''
                    )
                    # Analog in channel timebase
                    daq.DAQmxCfgSampClkTiming(
                        atask,
                        my_clock_channel + 'InternalOutput',
                        self._clock_frequency,
                        daq.DAQmx_Val_Rising,
                        daq.DAQmx_Val_ContSamps,
                        int(self._clock_frequency * 5)
                    )
                    self._counter_analog_daq_task = atask
        except:
            self.log.exception('Error while setting up counting task.')
            return -1

        try:
            for i, task in enumerate(self._counter_daq_tasks):
                # Actually start the preconfigured counter task
                daq.DAQmxStartTask(task)
            if len(self._counter_ai_channels) > 0:
                daq.DAQmxStartTask(self._counter_analog_daq_task)
        except:
            self.log.exception('Error while starting Counter')
            try:
                self.close_counter()
            except:
                self.log.exception('Could not close counter after error')
            return -1
        return 0


    def close_clock(self, scanner=False):
        """ Closes the clock and cleans up afterwards.

        @param bool scanner: specifies if the counter- or scanner- function
                             should be used to close the device.
                                True = scanner
                                False = counter

        @return int: error code (0:OK, -1:error)
        """
        if scanner:
            self._external_scanner_device.close_clock()
            self._scanner_clock_daq_task = None
        else:
            my_task = self._clock_daq_task
            try:
                # Stop the clock task:
                daq.DAQmxStopTask(my_task)

                # After stopping delete all the configuration of the clock:
                daq.DAQmxClearTask(my_task)

                # Set the task handle to None as a safety
                self._clock_daq_task = None
            except:
                self.log.exception('Could not close clock.')
                return -1
        return 0

    # ================ End SlowCounterInterface Commands =======================

    # ================ ConfocalScannerInterface Commands =======================
    def reset_hardware(self):
        """ Resets the NI hardware, so the connection is lost and other
            programs can access it.

        @return int: error code (0:OK, -1:error)
        """
        retval = 0
        chanlist = [
            self._odmr_trigger_channel,
            self._clock_channel,
            self._scanner_clock_channel,
            self._gate_in_channel
            ]
        chanlist.extend(self._photon_sources)
        chanlist.extend(self._counter_channels)
        chanlist.extend(self._scanner_counter_channels)

        devicelist = []
        for channel in chanlist:
            if channel is None:
                continue
            match = re.match(
                '^/(?P<dev>[0-9A-Za-z\- ]+[0-9A-Za-z\-_ ]*)/(?P<chan>[0-9A-Za-z]+)',
                channel)
            if match:
                devicelist.append(match.group('dev'))
            else:
                self.log.error('Did not find device name in {0}.'.format(channel))
        for device in set(devicelist):
            self.log.info('Reset device {0}.'.format(device))
            try:
                daq.DAQmxResetDevice(device)
            except:
                self.log.exception('Could not reset NI device {0}'.format(device))
                retval = -1
        self._external_scanner_device.reset_hardware()
        return retval

    def get_scanner_axes(self):
        """ Scanner axes depends on how many channels tha analog output task has.
        """
        n_channels = len(self._scanner_position_ranges)
        possible_channels = ['x', 'y', 'z', 'a']

        return possible_channels[0:int(n_channels)]

    def _start_analog_output(self):
        """ Starts or restarts the analog output.

        @return int: error code (0:OK, -1:error)
        """
        return 0

    def _stop_analog_output(self):
        """ Stops the analog output.

        @return int: error code (0:OK, -1:error)
        """
        return 0

    def set_up_scanner(self,
                       counter_channels=None,
                       sources=None,
                       clock_channel=None,
                       scanner_ao_channels=None):
        """ Configures the actual scanner with a given clock.

        The scanner works pretty much like the counter. Here you connect a
        created clock with a counting task. That can be seen as a gated
        counting, where the counts where sampled by the underlying clock.

        @param list(str) counter_channels: this is the physical channel of the counter
        @param list(str) sources:  this is the physical channel where the photons are to count from
        @param string clock_channel: optional, if defined, this specifies the clock for the counter
        @param list(str) scanner_ao_channels: not used - kept for compatibility

        @return int: error code (0:OK, -1:error)
        """
        retval = 0
        if self._scanner_clock_daq_task is None and clock_channel is None:
            self.log.error('No clock running, call set_up_clock before starting the counter.')
            return -1

        my_counter_channels = counter_channels if counter_channels else self._scanner_counter_channels
        my_photon_sources = sources if sources else self._photon_sources
        self._my_scanner_clock_channel = clock_channel if clock_channel else self._scanner_clock_channel

        if len(my_photon_sources) < len(my_counter_channels):
            self.log.error('You have given {0} sources but {1} counting channels.'
                           'Please give an equal or greater number of sources.'
                           ''.format(len(my_photon_sources), len(my_counter_channels)))
            return -1

        try:
            for i, ch in enumerate(my_counter_channels):
                # create handle for task, this task will do the photon counting for the
                # scanner.
                task = daq.TaskHandle()

                # actually create the scanner counting task
                daq.DAQmxCreateTask('ScannerCounter{0}'.format(i), daq.byref(task))

                # Create a Counter Input which samples with Semi Periods the Channel.
                # set up semi period width measurement in photon ticks, i.e. the width
                # of each pulse (high and low) generated by pulse_out_task is measured
                # in photon ticks.
                #   (this task creates a channel to measure the time between state
                #    transitions of a digital signal and adds the channel to the task
                #    you choose)
                daq.DAQmxCreateCISemiPeriodChan(
                    # The task to which to add the channels
                    task,
                    # use this counter channel
                    ch,
                    # name to assign to it
                    'Scanner Counter Channel {0}'.format(i),
                    # expected minimum value
                    0,
                    # Expected maximum count value
                    self._max_counts / self._scanner_clock_frequency,
                    # units of width measurement, here Timebase photon ticks
                    daq.DAQmx_Val_Ticks,
                    '')

                # Set the Counter Input to a Semi Period input Terminal.
                # Connect the pulses from the scanner clock to the scanner counter
                daq.DAQmxSetCISemiPeriodTerm(
                    # The task to which to add the counter channel.
                    task,
                    # use this counter channel
                    ch,
                    # assign a Terminal Name
                    self._my_scanner_clock_channel)

                # Set a CounterInput Control Timebase Source.
                # Specify the terminal of the timebase which is used for the counter:
                # Define the source of ticks for the counter as self._photon_source for
                # the Scanner Task.
                daq.DAQmxSetCICtrTimebaseSrc(
                    # define to which task to# connect this function
                    task,
                    # counter channel to output the# counting results
                    ch,
                    # which channel to count
                    my_photon_sources[i])
                self._scanner_counter_daq_tasks.append(task)

            # Scanner analog input task
            if self._scanner_ai_channels:
                atask = daq.TaskHandle()

                daq.DAQmxCreateTask('ScanAnalogIn', daq.byref(atask))

                daq.DAQmxCreateAIVoltageChan(
                    atask,
                    ', '.join(self._scanner_ai_channels),
                    'Scan Analog In',
                    daq.DAQmx_Val_RSE,
                    self._counter_voltage_range[0],
                    self._counter_voltage_range[1],
                    daq.DAQmx_Val_Volts,
                    ''
                )
                self._scanner_analog_daq_task = atask
        except:
            self.log.exception('Error while setting up scanner.')
            retval = -1

        return retval

    def scanner_set_position(self, x=None, y=None, z=None, a=None):
        """ Move stage to x, y, z, a (where a is the fourth channel).

        @param float x: position in x-direction (in axis unit)
        @param float y: position in y-direction (in axis unit)
        @param float z: position in z-direction (in axis unit)
        @param float a: position in a-direction (in axis unit)

        @return int: error code (0:OK, -1:error)
        """

        if self.module_state() == 'locked':
            self.log.error('Another scan_line is already running, close this one first.')
            return -1

        if x is not None:
            if not(self._scanner_position_ranges[0][0] <= x <= self._scanner_position_ranges[0][1]):
                self.log.error('You want to set x out of range: {0:f}.'.format(x))
                return -1
            self._current_position[0] = np.float(x)

        if y is not None:
            if not(self._scanner_position_ranges[1][0] <= y <= self._scanner_position_ranges[1][1]):
                self.log.error('You want to set y out of range: {0:f}.'.format(y))
                return -1
            self._current_position[1] = np.float(y)

        if z is not None:
            if not(self._scanner_position_ranges[2][0] <= z <= self._scanner_position_ranges[2][1]):
                self.log.error('You want to set z out of range: {0:f}.'.format(z))
                return -1
            self._current_position[2] = np.float(z)

        if a is not None:
            if not(self._scanner_position_ranges[3][0] <= a <= self._scanner_position_ranges[3][1]):
                self.log.error('You want to set a out of range: {0:f}.'.format(a))
                return -1
            self._current_position[3] = np.float(a)

        # the position has to be a vstack
        my_position = np.vstack(self._current_position)

        # then directly write the position to the hardware
        try:
            self._write_scanner_ao(voltages=my_position, start=True)
        except:
            return -1
        return 0

    def _write_scanner_ao(self, voltages, length=1, start=False):
        """Writes a set of voltages to the analog outputs.

        @param float[][n] voltages: array of n-part tuples defining the coordinate
                        points (it is up to external module how to interpret the coordinate)
                        n depends on how many channels are configured for analog output
        @param int length: number of tuples to write
        @param bool start: write immediately (True)
                           or wait for start of task (False)

        The positions is typically a matrix like
            [[x_values], [y_values], [z_values], [a_values]]
            but x, xy, xyz and xyza are allowed formats.
        """
        return self._external_scanner_device.write_scanner_coords(voltages, length, start)

    def _set_up_line(self, length=100):
        """ Sets up the analog output for scanning a line.

        Connect the timing of the Analog scanning task with the timing of the
        counting task.

        @param int length: length of the line in pixel

        @return int: error code (0:OK, -1:error)
        """
        if self._scanner_counter_channels and len(self._scanner_counter_daq_tasks) < 1:
            self.log.error('Configured counter is not running, cannot scan a line.')
            return np.array([[-1.]])

        if self._scanner_ai_channels and self._scanner_analog_daq_task is None:
            self.log.error('Configured analog input is not running, cannot scan a line.')
            return -1

        self._line_length = length

        try:
            for i, task in enumerate(self._scanner_counter_daq_tasks):
                # Configure Implicit Timing for the scanner counting task.
                # Set timing for scanner count task to the number of pixel.
                daq.DAQmxCfgImplicitTiming(
                    # define task
                    task,
                    # only a limited number of counts
                    daq.DAQmx_Val_FiniteSamps,
                    # count twice for each voltage +1 for safety
                    2 * self._line_length * self._clocks_per_pixel + 1)

                # Set the Read point Relative To an operation.
                # Specifies the point in the buffer at which to begin a read operation,
                # here we read samples from beginning of acquisition and do not overwrite
                daq.DAQmxSetReadRelativeTo(
                    # define to which task to connect this function
                    task,
                    # Start reading samples relative to the last sample returned
                    # by the previous read
                    daq.DAQmx_Val_CurrReadPos)

                # Set the Read Offset.
                # Specifies an offset in samples per channel at which to begin a read
                # operation. This offset is relative to the location you specify with
                # RelativeTo. Here we do not read the first sample.
                daq.DAQmxSetReadOffset(
                    # connect to this task
                    task,
                    # Offset after which to read
                    1)

                # Set Read OverWrite Mode.
                # Specifies whether to overwrite samples in the buffer that you have
                # not yet read. Unread data in buffer will be overwritten:
                daq.DAQmxSetReadOverWrite(
                    task,
                    daq.DAQmx_Val_DoNotOverwriteUnreadSamps)

            # Analog channels
            if self._scanner_ai_channels:
                # Analog in channel timebase
                daq.DAQmxCfgSampClkTiming(
                    self._scanner_analog_daq_task,
                    self._scanner_clock_channel + 'InternalOutput',
                    self._scanner_clock_frequency,
                    daq.DAQmx_Val_Rising,
                    daq.DAQmx_Val_ContSamps,
                    self._line_length * self._clocks_per_pixel + 1
                )
        except:
            self.log.exception('Error while setting up scanner to scan a line.')
            return -1
        return 0

    def scan_line(self, line_path=None, pixel_clock=False):
        """ Scans a line and return the counts on that line.

        @param float[c][m] line_path: array of c-tuples defining the voltage points
            (m = samples per line)
        @param bool pixel_clock: whether we need to output a pixel clock for this line

        @return float[m][n]: m (samples per line) n-channel photon counts per second

        The input array looks for a xy scan of 5x5 points at the position z=-2
        like the following:
            [ [1, 2, 3, 4, 5], [1, 1, 1, 1, 1], [-2, -2, -2, -2] ]
        n is the number of scanner axes, which can vary. Typical values are 2 for galvo scanners,
        3 for xyz scanners and 4 for xyz scanners with a special function on the a axis.
        """
        if self._scanner_counter_channels and len(self._scanner_counter_daq_tasks) < 1:
            self.log.error('Configured counter is not running, cannot scan a line.')
            return np.array([[-1.]])

        if self._scanner_ai_channels and self._scanner_analog_daq_task is None:
            self.log.error('Configured analog input is not running, cannot scan a line.')
            return -1

        if not isinstance(line_path, (frozenset, list, set, tuple, np.ndarray, ) ):
            self.log.error('Given line_path list is not array type.')
            return np.array([[-1.]])
        try:
            self._set_up_line(np.shape(line_path)[1])
            # write the positions to the scanner
            written_voltages = self._write_scanner_ao(
                voltages=line_path,
                length=self._line_length,
                start=False)

            for i, task in enumerate(self._scanner_counter_daq_tasks):
                daq.DAQmxStopTask(task)

            if pixel_clock and self._pixel_clock_channel is not None:
                daq.DAQmxConnectTerms(
                    self._scanner_clock_channel + 'InternalOutput',
                    self._pixel_clock_channel,
                    daq.DAQmx_Val_DoNotInvertPolarity)

            # start the scanner counting task that acquires counts synchronously 
            for i, task in enumerate(self._scanner_counter_daq_tasks):
                daq.DAQmxStartTask(task)

            if self._scanner_ai_channels:
                daq.DAQmxStartTask(self._scanner_analog_daq_task)

            self._external_scanner_device.scan_line()

            for i, task in enumerate(self._scanner_counter_daq_tasks):
                # wait for the scanner counter to finish
                daq.DAQmxWaitUntilTaskDone(
                    # define task
                    task,
                    # Maximum timeout for the counter times the positions. Unit is seconds.
                    self._RWTimeout * 2 * self._line_length)

            # count data will be written here
            self._scan_data = np.empty(
                (len(self.get_scanner_count_channels()), 2 * self._line_length * self._clocks_per_pixel),
                dtype=np.uint32)

            # number of samples which were read will be stored here
            n_read_samples = daq.int32()
            for i, task in enumerate(self._scanner_counter_daq_tasks):
                # actually read the counted photons
                daq.DAQmxReadCounterU32(
                    # read from this task
                    task,
                    # read number of double the # number of samples
                    # times the number of clock signals per sample
                    2 * self._line_length * self._clocks_per_pixel,
                    # maximal timeout for the read# process
                    self._RWTimeout,
                    # write into this array
                    self._scan_data[i],
                    # length of array to write into
                    2 * self._line_length * self._clocks_per_pixel,
                    # number of samples which were actually read
                    daq.byref(n_read_samples),
                    # Reserved for future use. Pass NULL(here None) to this parameter.
                    None)

                # stop the counter task
                daq.DAQmxStopTask(task)

            # Analog channels
            if self._scanner_ai_channels:
                self._analog_data = np.full(
                    (len(self._scanner_ai_channels), self._line_length * self._clocks_per_pixel + 1),
                    222,
                    dtype=np.float64)

                analog_read_samples = daq.int32()

                daq.DAQmxReadAnalogF64(
                    self._scanner_analog_daq_task,
                    self._line_length * self._clocks_per_pixel + 1,
                    self._RWTimeout,
                    daq.DAQmx_Val_GroupByChannel,
                    self._analog_data,
                    len(self._scanner_ai_channels) * (self._line_length * self._clocks_per_pixel + 1),
                    daq.byref(analog_read_samples),
                    None
                )

                daq.DAQmxStopTask(self._scanner_analog_daq_task)

            if pixel_clock and self._pixel_clock_channel is not None:
                daq.DAQmxDisconnectTerms(
                    self._scanner_clock_channel + 'InternalOutput',
                    self._pixel_clock_channel)

            # create a new array for the final data (this time of the length
            # number of samples):
            self._real_data = np.empty(
                (len(self._scanner_counter_channels), self._line_length),
                dtype=np.uint32)

            # Add up adjacent pixels to account for several clock signals per pixel
            # and to get the counts from both the low and high time of the clock.
            # Each pixel data consists of 2*self._clocks_per_pixel values from self._scan_data:
            for i in range(self._line_length):
                self._real_data[:, i] = np.sum(self._scan_data[:, i*2*self._clocks_per_pixel:(i+1)*2*self._clocks_per_pixel], axis=1)
            # Shorter version of above in case self._clocks_per_pixel == 1:
            # self._real_data = self._scan_data[:, ::2]
            # self._real_data += self._scan_data[:, 1::2]


            all_data = np.full(
                (len(self.get_scanner_count_channels()), self._line_length), 2, dtype=np.float64)
            all_data[0:len(self._real_data)] = np.array(
                self._real_data * self._scanner_clock_frequency, np.float64)

            if self._scanner_ai_channels:
                all_data[len(self._scanner_counter_channels):] = self._analog_data[:, :-1:self._clocks_per_pixel]

            # update the scanner position instance variable
            self._current_position = np.array(line_path[:, -1])
        except:
            self.log.exception('Error while scanning line.')
            return np.array([[-1.]])
        # return values is a rate of counts/s
        return all_data.transpose()

    # ================ End ConfocalScannerInterface Commands ===================


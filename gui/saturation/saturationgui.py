# -*- coding: utf-8 -*-

"""
This file contains the Qudi GUI for general Confocal control.

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
import time

from core.connector import Connector
from core.configoption import ConfigOption
from core.statusvariable import StatusVar
from qtwidgets.scan_plotwidget import ScanImageItem
from gui.guibase import GUIBase
from gui.fitsettings import FitParametersWidget
from qtpy import QtCore
from qtpy import QtGui
from qtpy import QtWidgets
from qtpy import uic


class SaturationMainWindow(QtWidgets.QMainWindow):
    """ Create the Mainwindow based on the corresponding *.ui file. """

    sigPressKeyBoard = QtCore.Signal(QtCore.QEvent)
    sigDoubleClick = QtCore.Signal()

    def __init__(self):
        # Get the path to the *.ui file
        this_dir = os.path.dirname(__file__)
        ui_file = os.path.join(this_dir, 'ui_saturation.ui')
        self._doubleclicked = False

        # Load it
        super(SaturationMainWindow, self).__init__()
        uic.loadUi(ui_file, self)
        self.show()

    def keyPressEvent(self, event):
        """Pass the keyboard press event from the main window further. """
        self.sigPressKeyBoard.emit(event)

    def mouseDoubleClickEvent(self, event):
        self._doubleclicked = True
        self.sigDoubleClick.emit()

class SaveDialog(QtWidgets.QDialog):
    """ Dialog to provide feedback and block GUI while saving """
    def __init__(self, parent, title="Please wait", text="Saving..."):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.setWindowModality(QtCore.Qt.WindowModal)
        self.setAttribute(QtCore.Qt.WA_ShowWithoutActivating)

        # Dialog layout
        self.text = QtWidgets.QLabel("<font size='16'>" + text + "</font>")
        self.hbox = QtWidgets.QHBoxLayout()
        self.hbox.addSpacerItem(QtWidgets.QSpacerItem(50, 0))
        self.hbox.addWidget(self.text)
        self.hbox.addSpacerItem(QtWidgets.QSpacerItem(50, 0))
        self.setLayout(self.hbox)

class SaturationGui(GUIBase):
    """ Main Class for saturation scan.
    """

    # declare connectors
    saturationlogic1 = Connector(interface='SaturationLogic')
    savelogic = Connector(interface='SaveLogic')

    # status var
    # adjust_cursor_roi = StatusVar(default=True)
    # slider_small_step = StatusVar(default=10e-9)    # initial value in meter
    # slider_big_step = StatusVar(default=100e-9)     # initial value in meter

    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)

    def on_activate(self):
        """ Initializes all needed UI files and establishes the connectors.

        This method executes all the inits for the differnt GUIs and passes
        the event argument from fysom to the methods.
        """

        # Getting an access to all connectors:
        self._scanning_logic = self.saturationlogic1()
        self._save_logic = self.savelogic()

        self._hardware_state = True

        self.initMainUI()      # initialize the main GUI
        self._save_dialog = SaveDialog(self._mw)

    def initMainUI(self):
        """ Definition, configuration and initialisation of the saturation GUI.

        This init connects all the graphic modules, which were created in the
        *.ui file and configures the event handling between the modules.
        Moreover it sets default values.
        """
        self._mw = SaturationMainWindow()

        ###################################################################
        #               Configuring the dock widgets                      #
        ###################################################################
        # All our gui elements are dockable, and so there should be no "central" widget.
        self._mw.centralwidget.hide()
        self._mw.setDockNestingEnabled(True)

        ###################################################################
        #               Configuration of the plot tab                     #
        ###################################################################
        # Load an image from the previous scan
        # self.sat_image = pg.PlotDataItem(
        #     x=self._scanning_logic.sat_x_values,
        #     y=self._scanning_logic.sat_y_values,
        #     pen=pg.mkPen(palette.c1, style=QtCore.Qt.DotLine),
        #     symbol='o',
        #     symbolPen=palette.c1,
        #     symbolBrush=palette.c1,
        #     symbolSize=7
        # )
        # self.sat_fit_image = pg.PlotDataItem(
        #     x=self._scanning_logic.fit_x_values,
        #     y=self._scanning_logic.fit_y_values,
        #     pen=pg.mkPen(palette.c2)
        # )

        # Add the display item to ViewWidget, which was defined in the UI file.
        # self._mw.sat_ViewWidget.addItem(self.sat_image)
        # self._mw.sat_ViewWidget.addItem(self.sat_fit_image)

        # Labelling axes
        self._mw.sat_ViewWidget.setLabel('bottom', 'Excitation power', units='W')
        self._mw.sat_ViewWidget.setLabel('left', 'Fluorescence', units='c/s')

        # Set the stop button to disabled state.
        self._mw.action_stop_scan.setEnabled(False)

        # Setup the Slider:
        # Calculate the needed Range for the slider.
        self.slider_res = 0.01  # Voltage resolution
        # How many points are needed for that kind of resolution:
        num_of_points_v = (self._scanning_logic.v_range[1] - self._scanning_logic.v_range[0]) / self.slider_res
        # Set a Range for the slider:
        self._mw.v_SliderWidget.setRange(0, num_of_points_v)
        # Just to be sure, set also the possible man/min values for the spin box of the current value:
        self._mw.v_current_InputWidget.setRange(self._scanning_logic.v_range[0], self._scanning_logic.v_range[1])
        # Set the maximal range for the voltage from the logic:
        self._mw.v_min_InputWidget.setRange(self._scanning_logic.v_range[0], self._scanning_logic.v_range[1])
        self._mw.v_max_InputWidget.setRange(self._scanning_logic.v_range[0], self._scanning_logic.v_range[1])
        # Predefine the maximal and minimal voltage range:
        self._mw.v_min_InputWidget.setValue(self._scanning_logic.scan_v_range[0])
        self._mw.v_max_InputWidget.setValue(self._scanning_logic.scan_v_range[1])

        # Take the default values from logic:
        self._mw.pts_InputWidget.setValue(self._scanning_logic.resolution)


        #################################################################
        #                           Actions                             #
        #################################################################

        # Update the inputed/displayed numbers if the cursor has left the field:
        self._mw.freq_InputWidget.editingFinished.connect(self.update_freq)
        self._mw.pts_InputWidget.editingFinished.connect(self.update_resolution)
        self._mw.v_min_InputWidget.editingFinished.connect(self.update_v_range)
        self._mw.v_max_InputWidget.editingFinished.connect(self.update_v_range)
        self._mw.v_current_InputWidget.editingFinished.connect(self.update_from_input_v)
        self._mw.v_SliderWidget.sliderMoved.connect(self.update_from_slider_v)

        # Connect buttons on the toolbar to the events if they are clicked:
        self._mw.action_calibrate.triggered.connect(self.calibrate_clicked)
        self._mw.action_start_scan.triggered.connect(self.start_clicked)
        self._mw.action_stop_scan.triggered.connect(self.stop_clicked)
        self._mw.action_save_data.triggered.connect(self.save_clicked)
        # self._scan_start_proxy = pg.SignalProxy(
        #     self._mw.action_start_scan.triggered,
        #     delay=0.1,
        #     slot=self.xy_scan_clicked
        #     )


        # history actions
        # self._mw.actionForward.triggered.connect(self._scanning_logic.history_forward)
        # self._mw.actionBack.triggered.connect(self._scanning_logic.history_back)
        # self._scanning_logic.signal_history_event.connect(lambda: self.set_history_actions(True))
        # self._scanning_logic.signal_history_event.connect(self._mw.xy_ViewWidget.autoRange)
        # self._scanning_logic.signal_history_event.connect(self.update_scan_range_inputs)

        # Connect other signals from the logic with an update of the gui
        self._scanning_logic.signal_start_scanning.connect(self.logic_started_scanning)
        self._scanning_logic.signal_scan_complete.connect(self.refresh_plot)
        self._scanning_logic.signal_save_started.connect(self.logic_started_save)
        self._scanning_logic.signal_data_saved.connect(self.logic_finished_save)

        self._mw.sigPressKeyBoard.connect(self.keyPressEvent)
        self.show()

    def on_deactivate(self):
        """ Reverse steps of activation

        @return int: error code (0:OK, -1:error)
        """
        self._mw.close()
        return 0

    def show(self):
        """Make main window visible and put it above all other windows. """
        # Show the Main Saturation GUI:
        self._mw.show()
        self._mw.activateWindow()
        self._mw.raise_()

    def keyPressEvent(self, event):
        """ Handles the passed keyboard events from the main window.

        @param object event: qtpy.QtCore.QEvent object.
        """
        modifiers = QtWidgets.QApplication.keyboardModifiers()

        if modifiers == QtCore.Qt.ControlModifier:
            if event.key() == QtCore.Qt.Key_Right:
                pass
            elif event.key() == QtCore.Qt.Key_Left:
                pass
            else:
                event.ignore()
        else:
            if event.key() == QtCore.Qt.Key_Right:
                pass
            elif event.key() == QtCore.Qt.Key_Left:
                pass
            else:
                event.ignore()

    def enable_scan_actions(self, state=True):
        """ Reset the scan action buttons to the default active
        state when the system is idle.
        """
        # Disable the stop scanning button
        self._mw.action_stop_scan.setEnabled(not state)
        # Enable scan and calibrate buttons
        self._mw.action_start_scan.setEnabled(state)
        self._mw.action_calibrate.setEnabled(state)

        # Enable parameters change
        self._mw.v_min_InputWidget.setEnabled(state)
        self._mw.v_max_InputWidget.setEnabled(state)
        self._mw.pts_InputWidget.setEnabled(state)
        self._mw.freq_InputWidget.setEnabled(state)
        self._mw.v_current_InputWidget.setEnabled(state)
        self._mw.v_SliderWidget.setEnabled(state)

        #self.set_history_actions(state)

    def set_history_actions(self, enable):
        """ Enable or disable history arrows taking history state into account. """
        if enable and self._scanning_logic.history_index < len(self._scanning_logic.history) - 1:
            self._mw.actionForward.setEnabled(True)
        else:
            self._mw.actionForward.setEnabled(False)
        if enable and self._scanning_logic.history_index > 0:
            self._mw.actionBack.setEnabled(True)
        else:
            self._mw.actionBack.setEnabled(False)

    def update_freq(self):
        """ Update scan frequency from the GUI """
        self._scanning_logic.set_clock_frequency(self._mw.freq_InputWidget.value())

    def update_resolution(self):
        """ Update number of scan points from the GUI """
        self._scanning_logic.resolution = self._mw.pts_InputWidget.value()

    def update_v_range(self):
        """ Update the voltage scan range from the GUI """
        self._scanning_logic.scan_v_range = [
            self._mw.v_min_InputWidget.value(),
            self._mw.v_max_InputWidget.value()]

    def stop_clicked(self):
        """ Stop the scan if the state has switched to ready. """
        if self._scanning_logic.module_state() == 'locked':
            self._scanning_logic.stop_scanning()
        self.enable_scan_actions()

    def calibrate_clicked(self):
        """ Start calibration of laser intensity. """
        self.enable_scan_actions(state=False)
        self._scanning_logic.start_calibrating()

    def start_clicked(self):
        """ Manages what happens if the start scan is clicked. """
        self.enable_scan_actions(state=False)
        self._scanning_logic.start_scanning()

    def save_clicked(self):
        """ Run the save routine from the logic to save the scan data."""
        self._save_dialog.show()

        self._scanning_logic.save_data(block=False)

        # TODO: find a way to produce raw image in savelogic.  For now it is saved here.
        filepath = self._save_logic.get_path_for_module(module_name='Confocal')
        filename = os.path.join(
            filepath,
            time.strftime('%Y%m%d-%H%M-%S_saturation'))
        self.sat_image.save(filename + '_raw.png')

    def update_from_input_v(self):
        """ The user changed the number in the voltage spin box, adjust all
            other GUI elements."""
        v_val = self._mw.v_current_InputWidget.value()
        # Update the slider accordingly
        self._mw.v_SliderWidget.setValue((v_val - self._scanning_logic.v_range[0]) / self.slider_res)
        self._scanning_logic.set_voltage('vinput', v=v_val)

    def update_from_slider_v(self, sliderValue):
        """The user moved the v slider, adjust the other GUI elements.

        @params int sliderValue: slider postion, a quantized whole number
        """
        # Calculate voltage from slider position
        v_val = self._scanning_logic.v_range[0] + sliderValue * self.slider_res
        self._mw.v_current_InputWidget.setValue(v_val)
        self._scanning_logic.set_voltage('vslider', v=v_val)

    def refresh_plot(self):
        self.depth_refocus_image.setData(
            self._scanning_logic.sat_x_values,
            self._scanning_logic.sat_y_values)
        # fit made from the data
        self.depth_refocus_fit_image.setData(
            self._optimizer_logic.fit_x_values,
            self._optimizer_logic.fit_y_values)

    def logic_started_scanning(self, tag):
        """ Disable icons if a scan was started.

            @param tag str: tag indicating command source
        """
        if tag == 'logic':
            self.enable_scan_actions(state=False)

    def logic_started_save(self):
        """ Displays modal dialog when save process starts """
        self._save_dialog.show()

    def logic_finished_save(self):
        """ Hides modal dialog when save process done """
        self._save_dialog.hide()
## ToDo:

Stopped at saturation_logic.py, set_voltage(), call of self._change_voltage(tag):
- Merge self._change_voltage() with self.set_voltage()
- Modify self.self._change_voltage() so that it changes voltage instead of coordinates.

Think how to implement direct voltage scan instead of coordinate scan. Might require modified hardware file for NI card.

saturationgui:
	self.save_clicked() # Check if saving raw image is needed (whether it is saved already in logic module)

saturation_logic:
	# Properties
	self.v_range = [v_min, v_max]   # modified x_range. Taken from hardware module saturation._scanning_device._scanner_voltage_ranges
	self._x_range = [x_min, x_max]  # effective coordinate range taken from hardware.
	self.current_v  # Current voltage used in history functionality
	self._current_v  # Current voltage value
	self.resolution = number_of_scan_points
?	self.scan_v_range = [v_min, v_max]  # scan range - modified self.image_x_range Is it needed?
	self.sat_x_values  # modified _zimage_Z_values. Must be loaded from History for initialization
	self.sat_y_values  # modified z_refocus_line
	self.fit_x_values  # modified _fit_zimage_Z_values
	self.fit_y_values  # modified z_fit_data
	
	# Methods
	self.set_clock_frequency()
	self.module_state() # returns 'locked' during scan - is it needed?
	self.start_scanner() # Is call to set_voltage() needed? Only needed if self.initialize_image() changes that.
	self.start_scanning() # called on "start" button
	self.start_calibrating()  # Similar to start_scannint() but instead of photocounts takes data from Thorlabs detector
	self.save_data() # modified self.save_xy_data() - wrapper for _save_data()
	self._save_data()  # actual save of data - modified _save_xy_data()
	self.stop_scanning() # called on "stop" button. Should also hande calibration stop.


	# Signals:
    self._scanning_logic.signal_start_scanning.connect(self.logic_started_scanning)
    self._scanning_logic.signal_scan_complete.connect(self.refresh_plot) # emitted when plot data are available
    self._scanning_logic.signal_save_started.connect(self.logic_started_save)
    self._scanning_logic.signal_data_saved.connect(self.logic_finished_save)
    self._signal_save_data  # issued internally to save data as soon as possible
    self.signal_history_event
    self.sigImageXYInitialized # Is it needed?

	# Disabled functionality
	#powermeter = Connector(interface='ThorlabsPM')
	
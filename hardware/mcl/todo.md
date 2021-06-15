- Test whether aux axis works - check the device type. What happens if aux axis is commanded? Probably slice the coords array according to the functionality of the MCL controller.
	A: use piezo.info['axis_bitmap'] - it will return 7 for xyz, and 15 for xyza
- Is there multi-axis waveform? Firmware profile bit 7.
	A: No
- Test loading several axis into waveform and then moving them in sequence
	A: Does not work. Each load must follow by a trigger on the same axis, as there is seemingly only one memory cell. So each write for an axis will overwrite all previous writes. When calling `write_scanner_coords()`, consider just saving those coordinates in the class attribute and actually load them to the MCL in the `scan_line()` function just before the trigger.
- Test bindtoaxis: bind to x, bind to loadwaveform. If bound to x, will pixel clock be triggered by waveform on y and z?
	A:
	-	Binding a clock to X/Y/Z/A will not have any effect on waveforms
	-	Waveform write will generate two (and only two) events
	-	A clock can be bound to many events (read/write).

- length = 400 ns
- amp@5Ohm = 2.6V - no ring-downs
- amp@1MOhm = 3.3V - ring-downs

- In file `confocalgui.py` there are three calls to `setImage()`, which accepts an argument `lut` among `**kwargs` (see source code of `ImageItem.py` in `pyqtgraph` library). Add custom lut to confocal GUI, ideally through `qdark.qss`.

- Make arbitrary scan frequency (or at least 200 Hz).
"""
Python module to work with MadCityLabs USB controllers.
Currently, only one device is supported. No proper treatment is done if several devices are attached.

@author: Ilya Radko
"""

import os
from ctypes import cdll, c_int, c_double, c_short, c_uint, c_ubyte, byref, Structure
from time import sleep

class ProductInformation(Structure):
    _pack_ = 1
    _fields_ = [("axis_bitmap", c_ubyte), ("ADC_resolution", c_short), ("DAC_resolution", c_short),
                ("Product_id", c_short), ("FirmwareVersion", c_short), ("FirmwareProfile", c_short)]

class MCL:

    # Define the most used error code
    _MCL_Dev_Not_Ready = -5

    def __init__(self):
        # Load dll from the same folder as this module
        path = os.path.dirname(os.path.realpath(__file__))
        self.dll = cdll.LoadLibrary(path + "\Madlib.dll")
        # Device variables
        self.handle = 0     # MCL handle
        self.info = {}      # Product info dictionary
        self.is20bit = None # Bool: 16 or 20 bit USB interface


#-------------------------------------------------------------------------------------
#------------------------------------ General MCL functionality 
#-------------------------------------------------------------------------------------

    def __testForError(self, code):
        """ Print an error message if error is detected
        
        @param int code: error code returned by MCL functions
        """
        errCodes = {
             0: "MCL_Success",
            -1: "MCL_General_Error",
            -2: "MCL_Dev_Error",
            -3: "MCL_Dev_Not_Attached",
            -4: "MCL_Usage_Error",
            -5: "MCL_Dev_Not_Ready",
            -6: "MCL_Argument_Error",
            -7: "MCL_Invalid_Axis",
            -8: "MCL_Invalid_Handle"
        }
        if (type(code) is int) and (code < 0):
            print(errCodes[code])
        return


    def open(self):
        """ Get a handle to the MCL device. Fill out class attributes
        with device information.
        
        @return: MCL handle or 0 if error occured
        """
        self.handle = self.dll.MCL_InitHandleOrGetExisting()
        if self.handle == 0:
            print("Cannot initialize MCL device (zero handle returned)")
        else:
            self.info = self.getProductInfo()
            if self.info['Product_id'] & 0x2200 == 0x2200:
                self.is20bit = True
            else:
                self.is20bit = False
        return self.handle


    def close(self):
        self.dll.MCL_ReleaseHandle(self.handle)


    def getProductInfo(self):
        """ Get some information about the MCL device.
        
        @return dictionary: Dictionary with various product info
        """
        pi = ProductInformation()
        code = self.dll.MCL_GetProductInfo(byref(pi), self.handle)
        self.__testForError(code)
        return({"axis_bitmap": pi.axis_bitmap, "ADC_resolution": pi.ADC_resolution,
                "DAC_resolution": pi.DAC_resolution, "Product_id": pi.Product_id,
                "FirmwareVersion": pi.FirmwareVersion, "FirmwareProfile": pi.FirmwareProfile})


    def getProductName(self, productId=-1):
        """ Convert product ID code to a string with the product name
        
        @param int productId: product ID code returned by MCL_GetProductInfo()
                If not specified, return the name of the openned MCL device.
        @return string: product name
        """
        prodDict = {
            0x2001: "Nano-Drive Single Axis",
            0x2003: "Nano-Drive Three Axis",
            0x2053: "Nano-Drive 16 bit Tip/Tilt Z",
            0x2004: "Nano-Drive Four Axis",
            0x2201: "Nano-Drive 20 bit Single Axis",
            0x2203: "Nano-Drive 20 bit Three Axis",
            0x2253: "Nano-Drive 20 bit Tip/Tilt Z",
            0x2100: "Nano-Gauge",
            0x2401: "C-Focus"
        }
        # If function called w/o argument, get ProductId of the openned device:
        if productId == -1:
            productId = self.info['Product_id']
        if productId in prodDict:
            return prodDict[productId]
        else:
            return "Unknown MCL device"


#-------------------------------------------------------------------------------------
#------------------------------------ Standard piezo move
#-------------------------------------------------------------------------------------

    def singleWrite(self, position, axis=1):
        """ Move MCL to the specified position.

        @param float position: Position in um
        @param int axis: Which axis to move (1=X, 2=Y, 3=Z, 4=Aux).
                Default is X axis.
        @return int errorCode: 0 (on success) or a negative error code
        """
        code = self._MCL_Dev_Not_Ready
        while code == self._MCL_Dev_Not_Ready:
            code = self.dll.MCL_SingleWriteN(c_double(position), c_uint(axis), self.handle)
        self.__testForError(code)
        return code


    def singleRead(self, axis=1):
        """ Read the current position.

        @param int axis: Which axis to read (1=X, 2=Y, 3=Z, 4=Aux).
                Default is X axis.
        @return float position: Position in um or a negative (integer) error code
        """
        # If return type is not int, restype attribute must be set:
        func = self.dll.MCL_SingleReadN
        func.restype = c_double
        code = self._MCL_Dev_Not_Ready
        while code == self._MCL_Dev_Not_Ready:
            # Calling MCL_SingleReadN with c_double return type:
            code = func(c_uint(axis), self.handle)
        self.__testForError(code)
        return code

#-------------------------------------------------------------------------------------
#------------------------------------ ISS functionality
#-------------------------------------------------------------------------------------
    
    def pixelClock(self):
        """ Generate a 250-ns-wide pixel clock.

        @return int errorCode: 0 (on success) or a negative error code
        """
        code = self._MCL_Dev_Not_Ready
        while code == self._MCL_Dev_Not_Ready:
            code = self.dll.MCL_PixelClock(self.handle)
        self.__testForError(code)
        return code


    def issSetClock(self, clock=1, mode=1):
        """ Set a clock high or low.
        
        @param int clock: Which clock to set (1=Pixel, 2=Line, 3=Frame, 4=Aux)
                Default is Pixel clock.
        @param int mode: High or low (0=low, 1=high)
                Default is high.
        @return int errorCode: 0 (on success) or a negative error code
        """
        code = self._MCL_Dev_Not_Ready
        while code == self._MCL_Dev_Not_Ready:
            code = self.dll.MCL_IssSetClock(c_int(clock), c_int(mode), self.handle)
        self.__testForError(code)
        return code


    def issReset(self):
        """ Reset the ISS option to default values (no asix is bound, polarity low to high).

        @return int errorCode: 0 (on success) or a negative error code
        """
        code = self._MCL_Dev_Not_Ready
        while code == self._MCL_Dev_Not_Ready:
            code = self.dll.MCL_IssResetDefaults(self.handle)
        self.__testForError(code)
        return code


    def issBindClockToAxis(self, clock=1, mode=2, axis=6):
        """ Bind an external clock to an axis event, such that a TTL pulse is generated when piezo moves.
            Can be used e.g. for gated counting.

        @param int clock: Which external clock to bind (1=Pixel, 2=Line, 3=Frame, 4=Aux).
                Default is Pixel clock.
        @param int mode: Polarity of pulses on the external clock.
                2 = low to high pulse,
                3 = high to low pulse,
                4 = unbind the axis.
                Default is low to high.
        @param int axis: Axis or event to bind a clock to:
                1 = X axis
                2 = Y axis
                3 = Z axis
                4 = Aux axis
                5 = Waveform Read (axis is determined by setupReadWaveform() function)
                6 = Waveform Write (axis is determined by setupWriteWaveform() function)
        @return int errorCode: 0 (on success) or a negative error code
        """
        code = self._MCL_Dev_Not_Ready
        while code == self._MCL_Dev_Not_Ready:
            code = self.dll.MCL_IssBindClockToAxis(c_int(clock), c_int(mode), c_int(axis), self.handle)
        self.__testForError(code)
        return code

#-------------------------------------------------------------------------------------
#------------------------------------ Waveform piezo move
#-------------------------------------------------------------------------------------

    def ms2index(self, delay):
        """ Convert delay (in ms) into table index. Is used for settuing up ADC in 20 bit devices.

        @param float delay: Delay in ms.
                Valid values: 0.267, 0.5, 1, 2, 10, 17, 20
        @return int index: Table index or -1 for error
        """
        # Valid delay values for 20 bit ADC and corresponding table indices:
        ms20bit = {3:0.267, 4:0.5, 5:1, 6:2, 7:10, 8:17, 9:20}
        if delay not in ms20bit.values():
            return -1 # Not a valid delay
        # Get dictionary key by value:
        for index, ms in ms20bit.items():
            if ms == delay:
                delayIndex = index
        return delayIndex


    def setupWriteWaveform(self, waveform, axis=1, delay=2):
        """ Load a waveform into DAC of MCL. Do not trigger the move yet.

        @param list waveform: List filled with float coordinates (in um) to move to.
        @param int axis: Which axis to move (1=X, 2=Y, 3=Z, 4=Aux).
                Default is X axis.
        @param float delay: Delay between DAC writes in ms (rate at which to move the piezo).
                Default is 2 ms.
                Valid range for 16 bit MCL models: 1/30 ms .. 5 ms,
                            for 20 bit MCL models:  1/6 ms .. 5 ms.
        @return int errorCode: 0 (on success) or a negative error code
        """
        dataPoints = len(waveform) # number of data points
        # Make C array type - double coords[dataPoints]:
        arrayType = c_double * dataPoints
        # Instantiate the array and initialize it with the list (unpack it)
        coords = arrayType(*waveform)
        code = self._MCL_Dev_Not_Ready
        while code == self._MCL_Dev_Not_Ready:
            code = self.dll.MCL_Setup_LoadWaveFormN(c_uint(axis), c_uint(dataPoints), c_double(delay), byref(coords), self.handle)
        self.__testForError(code)
        return code


    def triggerWriteWaveform(self, axis=1):
        """ Trigger a waveform write (move the piezo). Must be configured with setupWriteWaveform().

        @param int axis: Which axis to move (1=X, 2=Y, 3=Z, 4=Aux).
                Default is X axis.
        @return int errorCode: 0 (on success) or a negative error code
        """
        code = self._MCL_Dev_Not_Ready
        while code == self._MCL_Dev_Not_Ready:
            code = self.dll.MCL_Trigger_LoadWaveFormN(c_uint(axis), self.handle)
        self.__testForError(code)
        return code


    def writeWaveform(self, waveform, axis=1, delay=5):
        """ Write a waveform into DAC of MCL. It will trigger the move immediately.

        @param list waveform: List filled with float coordinates (in um) to move to.
        @param int axis: Which axis to move (1=X, 2=Y, 3=Z, 4=Aux).
                Default is X axis.
        @param float delay: Delay between DAC writes in ms (rate at which to move the piezo).
                Default is 2 ms.
                Valid range for 16 bit MCL models: 1/30 ms .. 5 ms,
                            for 20 bit MCL models:  1/6 ms .. 5 ms.
        @return int errorCode: 0 (on success) or a negative error code
        """
        dataPoints = len(waveform) # number of data points
        # Make C array type - double coords[dataPoints]:
        arrayType = c_double * dataPoints
        # Instantiate the array and initialize it with the list (unpack it)
        coords = arrayType(*waveform)
        code = self._MCL_Dev_Not_Ready
        while code == self._MCL_Dev_Not_Ready:
            code = self.dll.MCL_LoadWaveFormN(c_uint(axis), c_uint(dataPoints), c_double(delay), byref(coords), self.handle)
        self.__testForError(code)
        return code


    def setupReadWaveform(self, length, axis=1, delay=2):
        """ Prepare ADC for reading piezo positions. Do not trigger reading yet.

        @param int length: Number of coordinates to read.
                Valid range for 16 bit models: 1..10000
                            for 20 bit models: 1.. 6666
        @param int axis: Which axis to move (1=X, 2=Y, 3=Z, 4=Aux).
                Default is X axis.
        @param float delay: Delay between ADC reads in ms.
                Default is 2 ms.
                Valid range for 16 bit models: 1/30 ms .. 5 ms.
                Valid values for 20 bit models: 0.267, 0.5, 1, 2, 10, 17, 20.
        @return int errorCode: 0 (on success) or a negative error code
        """
        # Convert delay into delay index for 20 bit devices:
        if self.is20bit:
            delay = self.ms2index(delay)
            if delay == -1:
                print('Invalid delay value')
                return -6 # MCL_Argument_Error
        code = self._MCL_Dev_Not_Ready
        while code == self._MCL_Dev_Not_Ready:
            code = self.dll.MCL_Setup_ReadWaveFormN(c_uint(axis), c_uint(length), c_double(delay), self.handle)
        self.__testForError(code)
        return code


    def triggerReadWaveform(self, length, axis=1):
        """ Trigger a waveform read (position of piezo). Must be configured with setupReadWaveform().

        @param int length: Number of coordinates to read.
                Valid range for 16 bit models: 1..10000
                            for 20 bit models: 1.. 6666
        @param int axis: Which axis to move (1=X, 2=Y, 3=Z, 4=Aux).
                Default is X axis.
        @return list coords: List of recorded coordinates
        """
        # Make C array type - double coords[length]:
        arrayType = c_double * length
        # Instantiate the array
        coords = arrayType()
        code = self._MCL_Dev_Not_Ready
        while code == self._MCL_Dev_Not_Ready:
            code = self.dll.MCL_Trigger_ReadWaveFormN(c_uint(axis), c_uint(length), byref(coords), self.handle)
        self.__testForError(code)
        waveform = [x for x in coords] # convert C array to Python list
        return waveform


    def triggerReadAndWrite(self, length, axis=1):
        """ Trigger synchronous waveform read and write.
            Must be configured with setupReadWaveform() and setupWriteWaveform()
        
        @param int length: Number of coordinates to read (might be different from write length).
                Valid range for 16 bit models: 1..10000
                            for 20 bit models: 1.. 6666
        @param int axis: Which axis to move (1=X, 2=Y, 3=Z, 4=Aux).
                Default is X axis.
        @return list coords: List of recorded coordinates
        """
        # Make C array type - double coords[length]:
        arrayType = c_double * length
        # Instantiate the array
        coords = arrayType()
        code = self._MCL_Dev_Not_Ready
        while code == self._MCL_Dev_Not_Ready:
            code = self.dll.MCL_TriggerWaveformAcquisition(c_uint(axis), c_uint(length), byref(coords), self.handle)
        self.__testForError(code)
        waveform = [x for x in coords] # convert C array to Python list
        return waveform

#-------------------------------------------------------------------------------------
#------------------------------------ ADC and DAC speed
#-------------------------------------------------------------------------------------

    def setClock(self, clock, delay=2):
        """ Set ADC or DAC delay. Larger ADC delay decreases noise in position reads.

        @param int clock: 0=ADC (read coordinate), 1=DAC (write coordinate)
        @param float delay: Clock delay in ms.
                Default is 2 ms.
                Valid range for 16 bit models (ADC and DAC): 1/30 ms .. 5 ms,
                            for 20 bit models (DAC): 1/6 ms .. 5 ms,
                            for 20 bit models (ADC): 0.267, 0.5, 1, 2, 10, 17, 20
        @return int errorCode: 0 (on success) or a negative error code
        """
        # Convert delay into delay index for 20 bit ADC:
        if self.is20bit and (clock == 0):
            delay = self.ms2index(delay)
            if delay == -1:
                print('Invalid delay value')
                return -6 # MCL_Argument_Error
        code = self._MCL_Dev_Not_Ready
        while code == self._MCL_Dev_Not_Ready:
            code = self.dll.MCL_ChangeClock(c_double(delay), c_short(clock), self.handle)
        self.__testForError(code)
        return code


    def getClock(self):
        """ Get ADC and DAC delays in ms.

        @return tuple: (ADC delay, DAC delay)
        """
        adc = c_double()
        dac = c_double()
        code = self._MCL_Dev_Not_Ready
        while code == self._MCL_Dev_Not_Ready:
            code = self.dll.MCL_GetClockFrequency(byref(adc), byref(dac), self.handle)
        self.__testForError(code)
        return (adc.value, dac.value)



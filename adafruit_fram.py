# The MIT License (MIT)
#
# Copyright (c) 2018 Michael Schroeder
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
`adafruit_fram`
====================================================

CircuitPython/Python library to support the I2C and SPI FRAM Breakouts.

* Author(s): Michael Schroeder

Implementation Notes
--------------------

**Hardware:**

 * Adafruit `I2C Non-Volatile FRAM Breakout
   <https://www.adafruit.com/product/1895>`_ (Product ID: 1895)
 * Adafruit `SPI Non-Volatile FRAM Breakout
   <https://www.adafruit.com/product/1897>`_ (Product ID: 1897)

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases

 * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

# imports
from micropython import const

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_FRAM.git"

_MAX_SIZE_I2C = const(32767)
_MAX_SIZE_SPI = const(8191)

_I2C_MANF_ID = const(0x0A)
_I2C_PROD_ID = const(0x510)

class FRAM:
    """
    Driver base for the FRAM Breakout.
    """

    def __init__(self, max_size, write_protect=False, wp_pin=None):
        self._max_size = max_size
        self._wp = write_protect
        self._wraparound = False
        if not wp_pin is None:
            #import digitalio
            self._wp_pin = wp_pin
            # Make sure write_prot is set to output
            self._wp_pin.switch_to_output()
            self._wp_pin.value = self._wp
        else:
            self._wp_pin = wp_pin

    @property
    def max_size(self):
        """ The maximum size of the current FRAM chip. This is the highest
            register location that can be read or written to.
        """
        return self._max_size

    @property
    def write_wraparound(self):
        """ Determines if sequential writes will wrapaound the ``FRAM.max_size``
            address. If ``False``, and a requested write will extend beyond the
            maximum size, an exception is raised.
        """
        return self._wraparound

    @write_wraparound.setter
    def write_wraparound(self, value):
        if not value in (True, False):
            raise ValueError("Write wraparound must be 'True' or 'False'.")
        self._wraparound = value

    @property
    def write_protected(self):
        """ The status of write protection. Default value on initialization is
            ``False``.

            When a ``WP`` pin is supplied during initialization, or using
            ``write_protect_pin``, the status is tied to that pin and enables
            hardware-level protection.

            When no ``WP`` pin is supplied, protection is only at the software
            level in this library.
        """
        return self._wp if self._wp_pin is None else self._wp_pin.value

    @write_protected.setter
    def write_protected(self, value):
        self._wp = value
        if not self._wp_pin is None:
            self._wp_pin.value = value

    def __getitem__(self, key):
        """ Read the value at the given index, or values in a slice.

            .. code-block:: python

                # read single index
                fram[0]

                # read values 0 thru 9 with a slice
                fram[0:9]

                # read every other value from 0 thru 10 using a step
                fram[0:10:2]
        """
        if isinstance(key, int):
            if key > self._max_size:
                raise ValueError("Register '{0}' greater than maximum FRAM size."
                                 " ({1})".format(key, self._max_size))
            read_buffer = self._read_byte(key)
        elif isinstance(key, slice):
            registers = list(range(key.start if not key.start is None else 0,
                                   key.stop if not key.stop is None else self._max_size,
                                   key.step if not key.step is None else 1))
            if (registers[0] + len(registers)) > self._max_size:
                raise ValueError("Register + Length greater than maximum FRAM size."
                                 " ({0})".format(self._max_size))

            read_buffer = bytearray(len(registers))
            for i, register in enumerate(registers):
                read_buffer[i] = self._read_byte(register)[0]

        return read_buffer

    def __setitem__(self, key, value):
        """ Write the value at the given index, or values in a slice.

            .. code-block:: python

                # write single index
                fram[0] = 1

                # write values 0 thru 4 with a slice
                fram[0:4] = [0,1,2,3]

            .. note:: Slice stepping is not available when writing
        """
        if self.write_protected:
            raise RuntimeError("FRAM currently write protected.")

        if isinstance(key, int):
            if not isinstance(value, int):
                raise ValueError("Data must be an integer.")
            if key > self._max_size:
                raise ValueError("Requested register '{0}' greater than maximum"
                                 " FRAM size. ({1})".format(key,
                                                            self._max_size))

            self._write_register(key.start, value)

        elif isinstance(key, slice):
            if not isinstance(value, (bytearray, list, tuple)):
                raise ValueError("Data must be either a bytearray, list, or tuple.")
            if key.start > self._max_size:
                raise ValueError("Requested register '{0}' greater than maximum"
                                 " FRAM size. ({1})".format(key.start,
                                                            self._max_size))
            if not key.step is None:
                raise ValueError("Slice steps are not allowed during write operations.")

            self._write_page(key.start, value, self._wraparound)

    def _read_byte(self, register):
        return self._read_register(register)

    def _read_register(self, register):
        # Implemented by subclass
        raise NotImplementedError

    def _write_register(self, register, data):
        # Implemented by subclass
        raise NotImplementedError

    def _write_page(self, start_register, data, wraparound):
        # Implemened by subclass
        raise NotImplementedError

class FRAM_I2C(FRAM):
    """ I2C class for FRAM.

    :param: ~busio.I2C i2c_bus: The I2C bus the FRAM is connected to.
    :param: int address: I2C address of FRAM. Default address is ``0x50``.
    :param: bool write_protect: Turns on/off initial write protection.
                                Default is ``False``.
    :param: wp_pin: (Optional) Physical pin connected to the ``WP`` breakout pin.
                    Must be a ``digitalio.DigitalInOut`` object.
    """
    #pylint: disable=too-many-arguments
    def __init__(self, i2c_bus, address=0x50, write_protect=False,
                 wp_pin=None):
        from adafruit_bus_device.i2c_device import I2CDevice as i2cdev
        dev_id_addr = 0xF8 >> 1
        read_buf = bytearray(3)
        with i2cdev(i2c_bus, dev_id_addr) as dev_id:
            dev_id.write(bytearray([(address << 1)]), stop=False)
            dev_id.readinto(read_buf)
        manf_id = (((read_buf[0] << 4) +(read_buf[1] >> 4)))
        prod_id = (((read_buf[1] & 0x0F) << 8) + read_buf[2])
        if (manf_id != _I2C_MANF_ID) and (prod_id != _I2C_PROD_ID):
            raise OSError("FRAM I2C device not found.")

        self._i2c = i2cdev(i2c_bus, address)
        super().__init__(_MAX_SIZE_I2C, write_protect, wp_pin)

    def _read_register(self, register):
        write_buffer = bytearray(2)
        write_buffer[0] = register >> 8
        write_buffer[1] = register & 0xFF
        read_buffer = bytearray(1)
        with self._i2c as i2c:
            i2c.write_then_readinto(write_buffer, read_buffer)
        return read_buffer

    def _write_register(self, register, data):
        buffer = bytearray(3)
        buffer[0] = register >> 8
        buffer[1] = register & 0xFF
        buffer[2] = data
        with self._i2c as i2c:
            i2c.write(buffer)

    def _write_page(self, start_register, data, wraparound=False):
        # Decided against using the chip's "Page Write", since that would require
        # doubling the memory usage by creating a buffer that includes the passed
        # in data so that it can be sent all in one `i2c.write`. The single-write
        # method is slower, and forces us to handle wraparound, but I feel this
        # is a better tradeoff for limiting the memory required for large writes.
        buffer = bytearray(3)
        data_length = len(data)
        if (start_register + data_length) > self._max_size:
            if wraparound:
                pass
            else:
                raise ValueError("Starting register + data length extends beyond"
                                 " FRAM maximum size. Use ``write_wraparound`` to"
                                 " override this warning.")
        with self._i2c as i2c:
            for i in range(0, data_length):
                if not (start_register + i) > self._max_size:
                    buffer[0] = (start_register + i) >> 8
                    buffer[1] = (start_register + i) & 0xFF
                else:
                    buffer[0] = ((start_register + i) - self._max_size) >> 8
                    buffer[1] = ((start_register + i) - self._max_size) & 0xFF
                buffer[2] = data[i]
                i2c.write(buffer)

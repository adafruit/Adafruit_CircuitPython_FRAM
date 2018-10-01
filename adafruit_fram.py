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

 * `Adafruit I2C Non-Volatile FRAM Breakout <https://www.adafruit.com/product/1895>`
 * `Adafruit SPI Non-Volatile FRAM Breakout <https://www.adafruit.com/product/1897>`

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases

 * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

# imports
from micropython import const

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_FRAM.git"

_MAX_SIZE_I2C = const(262143) # 32768 words x 8 bits
_MAX_SIZE_SPI = const(65535) # 8192 words x 8 bits

_I2C_MANF_ID = const(0x0A)
_I2C_PROD_ID = const(0x510)

class FRAM:
    """
    Driver base for the FRAM Breakout.
    """

    def __init__(self, max_size, write_protect=False, wp_pin=None):
        self._max_size = max_size
        self._wp = write_protect
        if not wp_pin is None:
            import digitalio
            self._wp_pin = digitalio.DigitalInOut(wp_pin)
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
    def write_protected(self):
        """ The status of write protection. Default value on initialization is
            ``False``.

            When a ``WP`` pin is supplied during initialization, or using
            ``write_protect_pin``, the status is tied to that pin and enables
            hardware-level protection.

            When no ``WP`` pin is supplied, protection is only at the software
            level in this library.
        """
        if not self._wp_pin is None:
            status = self._wp_pin.value
        else:
            status = self._wp
        return status

    @write_protected.setter
    def write_protected(self, value):
        self._wp = value
        if not self._wp_pin is None:
            self._wp_pin.value = value

    def write_protect_pin(self, wp_pin, write_protect=False):
        """ Assigns the write protection (``WP``) pin.

        :param: wp_pin: The ``board.PIN`` object connected to the ``WP`` pin
                          on the breakout board/chip. To remove a previously
                          set ``WP`` pin, set this value to ``None``.
        :param: bool write_protect: Turn on/off write protection immediately
                                 when setting the pin. Default is ``False``

        """
        if not wp_pin is None:
            import digitalio
            self._wp_pin = digitalio.DigitalInOut(wp_pin)
            # Make sure wp_pin is set to switch_to_output
            self._wp_pin.switch_to_output()
            self._wp_pin.value = write_protect
        else:
            if not self._wp_pin is None:
                # Deinit the pin to release it
                self._wp_pin.deinit()
            self._wp_pin = None

    def read(self, register, length=1):
        """ Reads the data stored on the FRAM.

        :param: int register: Register location to start reading. Range is:
                                ``0`` to ``max_size``.
        :param: int length: Length of registers to read from starting register.
                              This function will create a buffer the size of
                              ``length``; larger buffers can cause memory
                              allocation problems on some platforms.
                              Range is ``1`` (default) to ``max_size``.
                              However, ``register`` + ``length`` cannot be
                              greater than ``max_size``.
        """
        if length < 1:
            raise ValueError("Length must be '1' or greater.")
        if length > self._max_size:
            raise ValueError("Length '{0}' greater than maximum FRAM size."
                             " ({1})".format(length, self._max_size))
        if (register + length) > self._max_size:
            raise ValueError("Register + Length greater than maximum FRAM size."
                             " ({0})".format(self._max_size))
        read_buffer = bytearray(length)
        for i in range(length):
            read_buffer[i] = self._read_byte(register + i)[0]
        return read_buffer

    def write_single(self, register, data):
        """ Writes a single byte to the FRAM.

        :param: int register: Register location to write the byte data.
        :param: int data: The data to write.
        """
        if not isinstance(data, int):
            raise ValueError("Data must be an integer.")
        if self.write_protected:
            raise RuntimeError("FRAM currently write protected.")
        if register > self._max_size:
            raise ValueError("Requested register '{0}' greater than maximum"
                             " FRAM size. ({1})".format(register,
                                                        self._max_size))
        self._write_register(register, data)

    def write_sequence(self, start_register, data, wraparound=False):
        """ Writes sequential data to the FRAM.

        :param: int start_register: Register location to start writing the
                                      data.
        :param: data: The data to write. Must be an iterable type of either
                      ``bytearray``, ``list``, or ``tuple``.
        :param: bool wraparound: Controls if sequential writes can wraparound
                                 beyond the ``max_size`` to zero, when
                                 ``start_register`` + ``data length`` is
                                 greater than ``max_size``.
        """
        if not isinstance(data, (bytearray, list, tuple)):
            raise ValueError("Data must be either a bytearray, list, or tuple.")
        if self.write_protected:
            raise RuntimeError("FRAM currently write protected.")
        if start_register > self._max_size:
            raise ValueError("Requested register '{0}' greater than maximum"
                             " FRAM size. ({1})".format(start_register,
                                                        self._max_size))
        self._write_page(start_register, data, wraparound)

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

    :param: i2c_SCL: The I2C SCL pin. Must be a ``board.PIN`` object.
    :param: i2c_SDA: The I2C SDA print. Must be a ``board.PIN`` object.
    :param: int address: I2C address of FRAM. Default address is ``0x50``.
    :param: bool write_protect: Turns on/off initial write protection.
                                Default is ``False``.
    :param: wp_pin: Physical ``WP`` breakout pin. Must be a ``board.PIN``
                    object.
    """
    #pylint: disable=too-many-arguments
    def __init__(self, i2c_SCL, i2c_SDA, address=0x50, write_protect=False,
                 wp_pin=None):
        from busio import I2C as i2c
        i2c_bus = i2c(i2c_SCL, i2c_SDA)
        i2c_bus.try_lock()
        i2c_bus.writeto((0xF8 >> 1), bytearray([(address << 1)]), stop=False)
        read_buf = bytearray(3)
        i2c_bus.readfrom_into((0xF9 >> 1), read_buf)
        manf_id = (((read_buf[0] << 4) +(read_buf[1] >> 4)))
        prod_id = (((read_buf[1] & 0x0F) << 8) + read_buf[2])
        if (manf_id != _I2C_MANF_ID) and (prod_id != _I2C_PROD_ID):
            raise OSError("FRAM I2C device not found.")
        i2c_bus.unlock()

        from adafruit_bus_device.i2c_device import I2CDevice as i2cdev
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
                                 " FRAM maximum size. Use 'wraparound=True' to"
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

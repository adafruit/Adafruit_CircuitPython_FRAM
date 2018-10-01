## Simple Example For CircuitPython/Python I2C FRAM Library

import adafruit_fram
import board

## Create a FRAM object (default address used).

fram = adafruit_fram.FRAM_I2C(board.SCL, board.SDA)

## Optional FRAM object with a different I2C address, as well
## as a pin to control the hardware write protection ('WP'
## pin on breakout). 'write_protected()' can be used
## independent of the hardware pin.

#fram = adafruit_fram.FRAM_I2C(board.SCL,
#                              board.SDA,
#                              address=0x53,
#                              wp_pin=board.D4)

## Write a single-byte value to register address '0'

fram.write_single(0, 1)

## Read that byte to ensure a proper write.
## Note: 'read()' returns a bytearray

print(fram.read(0)[1])

## Or write a sequential value, then read the values back.
## Note: 'read()' returns a bytearray. It also allocates
##       a buffer the size of 'length', which may cause
##       problems on memory-constrained platforms.

#values = list(range(100)) # or bytearray or tuple
#fram.write_sequence(0, values)
#fram.read(0, length=100)

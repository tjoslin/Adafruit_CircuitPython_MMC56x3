# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2022 ladyada for Adafruit Industries
# SPDX-FileCopyrightText: 2024 Todd Joslin
#
# SPDX-License-Identifier: MIT


import time
from micropython import const
from adafruit_bus_device import i2c_device
from adafruit_register.i2c_struct import ROUnaryStruct, UnaryStruct
from adafruit_register.i2c_bit import RWBit

try:
    from typing import Tuple
    from busio import I2C
except ImportError:
    pass


_MMC5983_I2CADDR_DEFAULT: int = const(0x30)  # Default I2C address
_MMC5983_CHIP_ID = const(0x30)

_MMC5983_OUT_X_L = const(0x00)  # Register that starts the mag data out
_MMC5983_OUT_TEMP = const(0x07)  # Register that contains temp reading
_MMC5983_PRODUCT_ID = const(0x2F)  # Register that contains the part ID
_MMC5983_STATUS_REG = const(0x08)  # Register address for device status
_MMC5983_CTRL_REG0 = const(0x09)  # Register address for control 0
_MMC5983_CTRL_REG1 = const(0x0A)  # Register address for control 1
_MMC5983_CTRL_REG2 = const(0x0B)  # Register address for control 2
_MMC5983_CTRL_REG3 = const(0x0C)  # Register address for control 3


class MMC5983:
    """Driver for the MMC5983 3-axis magnetometer.

    **Quickstart: Importing and using the device**

    Here is an example of using the :py:class:`MMC5983` class.
    First you will need to import the libraries to use the sensor

    .. code-block:: python

        import board
        import adafruit_mmc5983

    Once this is done you can define your `board.I2C` object and define your sensor object

    .. code-block:: python

        i2c = board.I2C()
        sensor = adafruit_mmc5983.MMC5983(i2c)

    Now you have access to the :attr:`magnetic` attribute

    .. code-block:: python

        mag_x, mag_y, mag_z = sensor.magnetic

    :param ~busio.I2C i2c_bus: The I2C bus the MMC5983 is connected to.
    :param int address: The I2C device address. Defaults to :const:`0x30`
    """

    _chip_id = ROUnaryStruct(_MMC5983_PRODUCT_ID, "<B")
    _ctrl0_reg = UnaryStruct(_MMC5983_CTRL_REG0, "<B")
    _ctrl1_reg = UnaryStruct(_MMC5983_CTRL_REG1, "<B")
    _ctrl2_reg = UnaryStruct(_MMC5983_CTRL_REG2, "<B")
    _ctrl3_reg = UnaryStruct(_MMC5983_CTRL_REG3, "<B")
    _status_reg = ROUnaryStruct(_MMC5983_STATUS_REG, "<B")
    _raw_temp_data = ROUnaryStruct(_MMC5983_OUT_TEMP, "<B")

    _reset = RWBit(_MMC5983_CTRL_REG1, 8)
    _meas_m_done = RWBit(_MMC5983_STATUS_REG, 0)
    _meas_t_done = RWBit(_MMC5983_STATUS_REG, 1)

    def __init__(self, i2c_bus: I2C, address: int = _MMC5983_I2CADDR_DEFAULT) -> None:
        # pylint: disable=no-member
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)
        if self._chip_id != _MMC5983_CHIP_ID:
            raise RuntimeError("Failed to find MMC5983 - check your wiring!")
        self.reset()
        self._buffer = bytearray(7)

    def reset(self) -> None:
        """Reset the sensor to the default state set by the library"""
        self._ctrl1_reg = 0x80  # write only
        time.sleep(0.020)
        self._ctrl2_cache = 0
        self._odr_cache = 0
        self.set_reset()

    @property
    def temperature(self) -> float:
        """The processed temperature sensor value, returned in floating point C"""
        if self.continuous_mode:
            raise RuntimeError("Can only read temperature when not in continuous mode")
        self._ctrl0_reg = 0x02  # TM_T
        while not self._meas_t_done:
            time.sleep(0.005)
        temp = self._raw_temp_data
        temp *= 0.8  # 0.8*C / LSB
        temp -= 75  # 0 value is -75
        return temp

    @property
    def magnetic(self) -> Tuple[float, float, float]:
        """The processed magnetometer sensor values.
        A 3-tuple of X, Y, Z axis values in microteslas that are signed floats.
        """
        if not self.continuous_mode:
            self._ctrl0_reg = 0x01  # TM_M
            while not self._meas_m_done:
                time.sleep(0.005)

        self._buffer[0] = _MMC5983_OUT_X_L
        with self.i2c_device as i2c:
            i2c.write_then_readinto(self._buffer, self._buffer, out_end=1)

        x_01 = int.from_bytes(self._buffer[0:2], 'big')
        y_01 = int.from_bytes(self._buffer[2:4], 'big')
        z_01 = int.from_bytes(self._buffer[4:6], 'big')
        x_2 = ((self._buffer[6] & 0xC0) >> 6) & 0x3
        y_2 = ((self._buffer[6] & 0x30) >> 4) & 0x3
        z_2 = ((self._buffer[6] & 0x0C) >> 2) & 0x3
        x = (x_01 << 2) | x_2
        y = (y_01 << 2) | y_2
        z = (z_01 << 2) | z_2

        # fix center offsets
        x -= 1 << 19
        y -= 1 << 19
        z -= 1 << 19
        # scale to uT by LSB in datasheet
        x *= 0.00625
        y *= 0.00625
        z *= 0.00625
        return (x, y, z)

    @property
    def data_rate(self) -> int:
        """Output data rate, 0 for on-request data.
        1, 10, 20, 50, 200, or 1000 for freq of continuous-mode readings"""
        return self._odr_cache

    @data_rate.setter
    def data_rate(self, value: int) -> None:
        if value == 1000:
            self._ctrl1_reg = 0x03  # enable 800Hz Bandwidth
            self._ctrl2_cache |= 0x07  # enable 1000Hz rate
        elif value == 200:
            self._ctrl1_reg = 0x01  # enable 200Hz Bandwidth
            self._ctrl2_cache &= ~0x07  # clear the rate
            self._ctrl2_cache |= 0x06  # enable 200Hz rate
        elif value == 100:
            self._ctrl1_reg = 0x00  # enable 100Hz Bandwidth
            self._ctrl2_cache &= ~0x07  # clear the rate
            self._ctrl2_cache |= 0x05  # enable 100Hz rate
        elif value == 50:
            self._ctrl1_reg = 0x00  # enable 100Hz Bandwidth
            self._ctrl2_cache &= ~0x07  # clear the rate
            self._ctrl2_cache |= 0x04  # enable 50Hz rate
        elif value == 25:
            self._ctrl1_reg = 0x00  # enable 100Hz Bandwidth
            self._ctrl2_cache &= ~0x07  # clear the rate
            self._ctrl2_cache |= 0x03  # enable 25Hz rate
        elif value == 10:
            self._ctrl1_reg = 0x00  # enable 100Hz Bandwidth
            self._ctrl2_cache &= ~0x07  # clear the rate
            self._ctrl2_cache |= 0x02  # enable 10Hz rate
        elif value == 1:
            self._ctrl1_reg = 0x00  # enable 100Hz Bandwidth
            self._ctrl2_cache &= ~0x07  # clear the rate
            self._ctrl2_cache |= 0x01  # enable 1Hz rate
        elif value == 0:
            self._ctrl1_reg = 0x00  # enable 100Hz Bandwidth
            self._ctrl2_cache &= ~0x07  # clear the rate
        else:
            raise ValueError("Data rate invalid")
        self._ctrl2_reg = self._ctrl2_cache  # set the rate control
        self._odr_cache = value

    @property
    def continuous_mode(self) -> bool:
        """Whether or not to put the chip in continous mode - be sure
        to set the data_rate as well!
        """
        return self._ctrl2_cache & 0x08

    @continuous_mode.setter
    def continuous_mode(self, value: bool) -> None:
        if value:
            self._ctrl2_cache |= 0x08  # turn on cmm_en bit
        else:
            self._ctrl2_cache &= ~0x08  # turn off cmm_en bit
        self._ctrl2_reg = self._ctrl2_cache

    def set_reset(self) -> None:
        """Pulse large currents through the sense coils to clear any offset"""
        self._ctrl0_reg = 0x08  # turn on set bit
        time.sleep(0.001)  # 1 ms
        self._ctrl0_reg = 0x10  # turn on reset bit
        time.sleep(0.001)  # 1 ms

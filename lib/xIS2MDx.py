########################################
# Function: Driver for LIS2MDL and IIS2MDC
# Author: Louis Barbier
# Licence: CC-BY-NC-SA
# See https://creativecommons.org/licenses/by-nc-sa/4.0
########################################

import struct
import time

xIS2MDx_OFFSET_X_REG_L = 0x45 # RW (00000000)
xIS2MDx_OFFSET_X_REG_H = 0x46 # RW (00000000)
xIS2MDx_OFFSET_Y_REG_L = 0x47 # RW (00000000)
xIS2MDx_OFFSET_Y_REG_H = 0x48 # RW (00000000)
xIS2MDx_OFFSET_Z_REG_L = 0x49 # RW (00000000)
xIS2MDx_OFFSET_Z_REG_H = 0x4A # RW (00000000)
# RESERVED 0x4B => 0x4C
xIS2MDx_WHO_AM_I = 0x4F # R (01000000)
# RESERVED 0x50 => 0x5F
xIS2MDx_CFG_REG_A = 0x60 # RW (00000011)
xIS2MDx_CFG_REG_B = 0x61 # RW (00000000)
xIS2MDx_CFG_REG_C = 0x62 # RW (00000000)
xIS2MDx_INT_CTRL_REG = 0x63 # RW (11100000)
xIS2MDx_INT_SOURCE_REG = 0x64 # R (output)
xIS2MDx_INT_THS_L_REG = 0x65 # RW (00000000)
xIS2MDx_INT_THS_H_REG = 0x66 # RW (00000000)
xIS2MDx_STATUS_REG = 0x67 # R (output)
xIS2MDx_OUTX_L_REG = 0x68 # R (output)
xIS2MDx_OUTX_H_REG = 0x69 # R (output)
xIS2MDx_OUTY_L_REG = 0x6A # R (output)
xIS2MDx_OUTY_H_REG = 0x6B # R (output)
xIS2MDx_OUTZ_L_REG = 0x6C # R (output)
xIS2MDx_OUTZ_H_REG = 0x6D # R (output)
xIS2MDx_TEMP_OUT_L_REG = 0x6E # R (output)
xIS2MDx_TEMP_OUT_H_REG = 0x6F # R (output)

xIS2MDx_ODR = ('10', '20', '50', '100') # Hz
xIS2MDx_MODE = ('continuous', 'single', 'idle', 'idle')

class xIS2MDx:
    def __init__(self, i2c_bus, addr = 0x1E):
        self._bus = i2c_bus
        self._addr = int(addr)
        self._chip = ""
        self._odr = 2
        self._mode = 0
        # Get chip name from chip reference ID
        if self.read(xIS2MDx_WHO_AM_I) == 0x40:
            self._chip = "LIS2MDL or IIS2MDC"
        else:
            raise Exception('[xIS2MDx] Sensor ID error (expected 0x40, got '+hex(self.read(xIS2MDx_WHO_AM_I))+')')
        # SOFT RESET
        self.read_modify_write(xIS2MDx_CFG_REG_A, 0x20, 0x20)
        time.sleep(0.2)
        if (self.read(xIS2MDx_CFG_REG_A) & 0x20 == 0x20):
            raise Exception('[xIS2MDx] Soft reset not done in time')
        # REBOOT (reload internal trimming)
        self.read_modify_write(xIS2MDx_CFG_REG_A, 0x40, 0x40)
        time.sleep(0.05)
        # Enable temperature compensation, ODR 50Hz, continuous mode
        self.write(xIS2MDx_CFG_REG_A, (0x1<<7) + (0x0<<6) + (0x0<<5) + (0x0<<4) + (0x2<<2) + 0x0)
        # Enable BDU
        self.write(xIS2MDx_CFG_REG_C, (0x1<<4))

    def read(self, reg, length=1):
        if length == 1:
            return self._bus.readfrom_mem(self._addr, int(reg), 1)[0]
        else:
            return self._bus.readfrom_mem(self._addr, int(reg), int(length))

    def write(self, reg, value):
        self._bus.writeto_mem(self._addr, int(reg), bytes([int(value)]))

    def read_modify_write(self, reg, dat, mask):
        reg_to_write = (self.read(reg) & ~mask) | (dat & mask)
        self.write(reg, reg_to_write)

    def odr(self, odr=None):
        if (odr is None) or (odr == ''):
            return xIS2MDx_ODR[self._odr]
        else:
            odr = str(odr)
            if not odr in xIS2MDx_ODR:
                return -1
            self._odr = xIS2MDx_ODR.index(odr)
            self.read_modify_write(xIS2MDx_CFG_REG_A, self._odr<<2, 0x3<<2)
            return xIS2MDx_ODR[self._odr]

    def get_available_odr(self):
        return [x for x in xIS2MDx_ODR if not '']

    def read_mag_raw(self):
        mag_raw_val = self.read(xIS2MDx_OUTX_L_REG, 6)
        return list(struct.unpack('<hhh', bytearray(mag_raw_val))) # Little-endian

    def read_temp_raw(self):
        temp_raw_val = self.read(xIS2MDx_TEMP_OUT_L_REG, 2)
        return struct.unpack('<h', bytearray(temp_raw_val))[0] # Little-endian

    def convert_m(self, m):
        return 1.5*m/1000.0 # gauss (1.5 mgauss/LSB)

    def read_mag(self):
        mag_data = list(self.read_mag_raw())
        mag_data[0] = self.convert_m(mag_data[0])
        mag_data[1] = self.convert_m(mag_data[1])
        mag_data[2] = self.convert_m(mag_data[2])
        return mag_data

    def read_temperature(self):
        return self.read_temp_raw()/8.0 + 25

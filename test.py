# -*- encoding: utf-8 -*-

import serial
from enum import Enum


SERIAL_PORT = 'COM6'  # '/dev/ttyS0'


class OneWireRom(Enum):
    # Commands
    READ = 0x33
    SKIP = 0xcc
    MATCH = 0x55
    SEARCH = 0xf0
    ALARM = 0xec
    # Return codes
    SEARCH_FINISHED = 0x00
    SEARCH_FAILED = 0xff


class UartPattern(Enum):
    WRITE1 = 0xff
    WRITE0 = 0x00
    READ_BIT = 0xff
    RESET = 0xf0


class OneWire(object):
    ROM_LENGTH = 8

    def __init__(self, serial_name):
        self.rom = []
        self.port = serial.Serial(serial_name, baudrate=115200)
        self.port.flushInput()

    def reset(self):
        self.port.baudrate = 9600
        res = self.shift_bit(UartPattern.RESET)
        self.port.baudrate = 115200
        return res != [UartPattern.RESET, ]

    def receive(self):
        result = 0x00
        for i in range(8):
            result >>= 1
            bit = self.shift_bit(UartPattern.READ_BIT)
            if bit == UartPattern.READ_BIT:
                result |= 0x80
        return result

    def send(self, value):
        mask = 0x01
        for i in range(8):
            if value & mask:
                self.shift_bit(UartPattern.WRITE1)
            else:
                self.shift_bit(UartPattern.WRITE0)
            mask <<= 1

    def skip_rom(self):
        self.send(OneWireRom.SKIP)

    def read_rom(self):
        self.send(OneWireRom.READ)
        data = []
        for i in range(self.ROM_LENGTH):
            data.append(self.receive())
        self.rom = data

    def match_rom(self):
        self.send(OneWireRom.MATCH)
        for b in self.rom:
            self.send(b)

    def alarm_search(self):
        self.send(OneWireRom.ALARM)

    def shift_bit(self, value):
        self.port.write(value & 0xff)
        self.port.flushOutput()
        return self.port.read(size=1)[0]

    def close(self):
        if self.port.isOpen():
            self.port.close()

    def __del__(self):
        self.close()


class Ds18b20(object):
    FAMILY_ID = 0x10
    FAMILY_CODE = 0x28
    START_CONVERSION = 0x44
    WRITE_SCRACHPAD = 0x4e
    READ_SCRATCHPAD = 0xbe
    COPY_SCRATCHPAD = 0x48
    RECALL_E = 0xb8
    POWER_STATUS = 0xb4

    TIMEOUT = 0xffff

    def __init__(self, one_wire=None):
        self.onewire = one_wire or OneWire()
        self.__value = 0

    value = property(fget=lambda self: self.__value)

    def wait_ready(self):
        timeout = 0
        while timeout < self.TIMEOUT and not self.onewire.shift_bit(UartPattern.READ_BIT):
            timeout += 1
        if timeout == self.TIMEOUT:
            return False
        return True

    def read(self):
        if self.onewire.reset():
            self.onewire.skip_rom()
            self.onewire.send(self.START_CONVERSION)

            if self.wait_ready() and self.onewire.reset():
                self.onewire.skip_rom()
                self.onewire.send(self.READ_SCRATCHPAD)

                data_lo = self.onewire.receive() & 0xff
                data_hi = self.onewire.receive() & 0xff

                temperature = (data_hi << 8) | data_lo
                # TODO: handle minus sign: temperature & 0x8000 ? (temperature ^ 0xffff) + 1
                temperature &= 0x0fff
                fraction_part = temperature & 0x000f
                int_part = temperature >> 4

                self.__value = int_part + (fraction_part * 0.0625)

                return True
        return False


def main():
    try:
        proto = OneWire(SERIAL_PORT)
        sensor = Ds18b20(proto)
        while True:
            if sensor.read():
                print 'Temperature read: %s' % sensor.value
            else:
                print 'Failed to read temperature'
    finally:
        proto.close()


if __name__ == '__main__':
    main()

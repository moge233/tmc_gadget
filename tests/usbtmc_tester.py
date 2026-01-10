'''
USB TMC Tester

'''


import random
import time
import unittest
import warnings

import pyvisa


RUN_TESTS = True

warnings.filterwarnings('error')


def generate_long_string(count: int) -> str:
    ret = ''
    for _ in range(count):
        ret += str(random.randrange(0, 10))
    return ret


class UsbTmcTestBench(unittest.TestCase):

    def setUp(self):
        self.serial = 1234567
        self.rm = pyvisa.ResourceManager()
        self.resource_string = f'USB::0x05E6::0x7000::{self.serial}::INSTR'
        self.instr = self.rm.open_resource(self.resource_string)

    def test_idn(self):
        result = self.instr.query('*IDN?')
        assert result == \
               f'KEITHLEY INSTRUMENTS, INC.,MODEL 7000,{self.serial},INSTR'

    def test_create_table(self):
        self.instr.write('x = {}')
        self.instr.write('x[1] = 10')
        x = self.instr.query('print(x[1])')
        assert x == '1.00000e+01'

        self.instr.write('x[2] = 20')
        x = self.instr.query('print(x[2])')
        assert x == '2.00000e+01'

    def test_delete_table1(self):
        self.instr.write('x = nil')
        self.instr.write('collectgarbage()')
        x = self.instr.query('print(x)')
        assert x == 'nil'

    def test_long_string(self):
        self.instr.write('x = {}')
        for i in range(1, 11):
            s = generate_long_string(i*1000)
            cmd = f'x[{i}] = "{s}"'
            self.instr.write(cmd)
            x = self.instr.query(f'print(x[{i}])')
            assert x == s

    def test_delete_table2(self):
        self.instr.write('x = nil')
        self.instr.write('collectgarbage()')
        x = self.instr.query('print(x)')
        assert x == 'nil'

    def test_termchar(self):
        self.instr.write_termination = '\r'
        self.instr.read_termination = '\r'
        try:
            x = self.instr.query('print("hello")')
        except UserWarning:
            assert False

    def test_led(self):
        self.instr.write_termination = '\n'
        self.instr.read_termination = '\n'
        self.instr.write('led.state = true')
        x = self.instr.query('print(led.state)')
        assert x == 'true'
        time.sleep(2)
        self.instr.write('led.state = not led.state')
        x = self.instr.query('print(led.state)')
        assert x == 'false'

    def test_message_exact_endpoint_size_with_termchar(self):
        self.instr.write_termination = '\n'
        self.instr.read_termination = '\n'
        self.instr.write('x = {}')
        for i in range(1, 11):
            s = generate_long_string(500)
            assert len(s) == 500
            self.instr.write(f'x[{i}] = "{s}"')
            x = self.instr.query(f'print(x[{i}])')
            assert x == s
        self.instr.write('x = nil')
        self.instr.write('collectgarbage()')
        x = self.instr.query('print(x)')
        assert x == 'nil'

    def test_message_exact_endpoint_size_without_termchar(self):
        self.instr.write('x = {}')
        for i in range(1, 11):
            s = generate_long_string(500)
            assert len(s) == 500
            self.instr.write(f'x[{i}] = "{s}"')
            x = self.instr.query(f'print(x[{i}])')
            assert x == s
        self.instr.write('x = nil')
        self.instr.write('collectgarbage()')
        x = self.instr.query('print(x)')
        assert x == 'nil'

if __name__ == '__main__':

    if RUN_TESTS:
        test_harness = UsbTmcTestBench()
        unittest.main()
    else:
        serial = 1234567
        rm = pyvisa.ResourceManager()
        resource_string = f'USB::0x05E6::0x7000::{serial}::INSTR'
        with rm.open_resource(resource_string) as instr:
            instr.write_termination = '\n'
            instr.read_termination = '\n'
            d = int(500 / 4)
            s = generate_long_string(500)
            assert len(s) == 500
            instr.write('x = {}')
            instr.write(f'x[1] = "{s}"')
            x = instr.query('print(x[1])')
            assert x == s


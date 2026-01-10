#! python3


import random
import string
import time
import unittest

import pyvisa


def generate_long_string(n: int) -> str:
    s = ''
    for _ in range(n):
        s += random.choice(string.ascii_letters)
    return s


class TestUsbTmc(unittest.TestCase):

    def setUp(self):
        self.resource_manager = pyvisa.ResourceManager()
        self.instr = self.resource_manager.open_resource(self.resource_manager.list_resources()[0])

    def tearDown(self):
        self.instr.close()

    def test_led_state(self):
        for _ in range(10):
            self.instr.write('led.state = true')
            state = self.instr.query('print(led.state)')
            self.assertEqual(state, 'true', 'led.state != true after writing led.state = true')

            time.sleep(1)

            self.instr.write('led.state = false')
            state = self.instr.query('print(led.state)')
            self.assertEqual(state, 'false', 'led.state != false after writing led.state = false')

            time.sleep(1)

    def test_long_messages(self):
        self.instr.write('long_message_table = {}')
        long_message_tests = (1000, 2000, 5000, 9000)
        long_message_strings = []
        for i, test in enumerate(long_message_tests):
            s = generate_long_string(test)
            long_message_strings.append(s)
            msg = f'long_message_table[{i+1}] = "{s}"'
            print(f'test_long_messages writing {msg}')
            self.instr.write(msg)

        for i, test in enumerate(long_message_tests):
            s = long_message_strings[i]
            msg = f'print(long_message_table[{i+1}])'
            print(f'test_long_messages writing {msg}')
            s_comp = self.instr.query(msg)
            self.assertEqual(s, s_comp, f's != s_comp at i = {i}')

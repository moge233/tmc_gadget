#! python3


import random
import string
import time

import pyvisa


def generate_long_string(n: int) -> str:
    s = ''
    for _ in range(n):
        s += random.choice(string.ascii_letters)
    return s


if __name__ == '__main__':

    rm = pyvisa.ResourceManager()

    long_strings = []

    with rm.open_resource(rm.list_resources()[0]) as instr:
        print(instr.query('*IDN?'))
        msg = 'x = {}'
        instr.write(msg)

        for i in range(40):
            s = generate_long_string(500)
            msg = f'x[{i}] = "{s}"'
            instr.write(msg)

            msg = f'print(x[{i}])'
            s = instr.query(msg)

            print(s)

'''
        for i in range(5):
            s = generate_long_string(90)
            long_strings.append(s)
            msg = f'x[{i}] = "{s}"'
            instr.write(msg)

            msg = f'print(x[{i}])'
            s = instr.query(msg)

            print(f'x[{i}] = {s}\nlong_strings[{i}] = {long_strings[i]}')

            assert s == long_strings[i]
'''

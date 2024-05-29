"""
Helper script for generating serial commands for AD9959 DDS using myRIO
"""

import numpy as np

# Define pin mappings (adjust to match hardware connection)
DIO_PINS = {
    'RST':0,
    'IO_UPD':1,
    'CS':2,
    'SCLK':3,
    'SDIO3':4,
    'SDIO2':5,
    'SDIO1':6,
    'SDIO0':7,
}

SYS_CLK = 500e6 # 500 MHz (max value, confirm wiring)


def calc_FTW(freq_out):
    return int((freq_out * 2**32) / SYS_CLK)

# Helpers for bitwise ops
def int_to_bits(value, bits):
    return [int(x) for x in format(value, f'0{bits}b')]

def split_word(word, byte_count=3):
    return [(word >> (8 * i)) & 0xFF for i in reversed(range(byte_count))]

def split_byte_4wire(byte):
    bits = int_to_bits(byte, 8)
    sdio_bits = [
        (bits[0], bits[1], bits[2], bits[3]), # first nibble (D7-D4)
        (bits[4], bits[5], bits[6], bits[7]) # second nibble (D3-D0)
    ]
    return sdio_bits

def set_bits(value, bit_value, start_bit, num_bits):
    """
    Set specific bits in some original value - for adjusting specfic portions of registers
    """
    mask = (1 << num_bits) - 1
    value &= ~(mask << start_bit) # Clear bits in given range
    value |= (bit_value & mask) << start_bit # Set new bit value
    return value

class SerialSequencer:
    def __init__(self, pins):
        self.pins = DIO_PINS
        self.sequence = []

    def add_step(self, **pin_states):
        step = [0] * 8
        for pin, state in pin_states.items():
            if pin in self.pins:
                step[self.pins[pin]] = state
        self.sequence.append(step)

    def io_update(self):
        self.add_step(IO_UPD=1)
        self.add_step(IO_UPD=0)

    def build_sequence(self, data, mode='4-wire'):
        # self.add_step(CS=0) # CS low to start transmission
        if mode == '1-wire':
            for bit in data:
                self.add_step(SCLK=0, SDIO0=bit) # load data with SCLK low
                self.add_step(SCLK=1, SDIO0=bit) # transfer data on SCLK high
        elif mode == '4-wire':
            for byte in data:
                sdbits = split_byte_4wire(byte)
                for sdbit in sdbits:
                    self.add_step(SCLK=0, SDIO3=sdbit[0], SDIO2=sdbit[1],SDIO1=sdbit[2], SDIO0=sdbit[3])
                    self.add_step(SCLK=1, SDIO3=sdbit[0], SDIO2=sdbit[1],SDIO1=sdbit[2], SDIO0=sdbit[3])

    def get_array(self):
        self.io_update()
        output = np.packbits(np.array(self.sequence, dtype=np.uint8))
        self.sequence = []
        return output

    def write_freq(self, freq, channel=0):
        # write instruction byte, select CSR and enable channel
        self.build_sequence([0x00])
        data_byte = (1 << (4+channel)) | 0x06 # ensure 4wire sdio mode
        self.build_sequence([data_byte])
        # write instruction to select CFTW0 (0x04)
        self.build_sequence([0x04])
        f_word = calc_FTW(freq)
        self.build_sequence(split_word(f_word, 4))
        
# End class defintion

# Initialize sequencer
ser_seq = SerialSequencer(DIO_PINS)

# Startup sequence wrapper for LabView node
def init_DDS():
    # Reset DDS
    ser_seq.add_step(RST=1)
    ser_seq.add_step(RST=0) # Does this need a wait/delay?
    # Configure 4-wire serial
    ib = int_to_bits(0, 8)
    db = int_to_bits(6, 8)
    combo = ib+db
    ser_seq.build_sequence(combo, mode='1-wire')
    ser_seq.io_update()
    # Set PLL Divider Ratio
    fr1_ib = [0x01] # FR1 register address
    set_pll = split_word(set_bits(0, 20, 18, 5)) # PPL bits [22:18]
    ser_seq.build_sequence(fr1_ib)
    ser_seq.build_sequence(set_pll)

    return ser_seq.get_array()

# For visualizing SDIO bus on myRIO leds in LabView
def led_bits(array):
    return [(byte & 0x0F) for byte in array]

def led_test():
    return led_bits(init_DDS())

# print(init_DDS())
# print(led_test())

# Frequency Wrapper for LabView node
def lab_freq(freq, channel=0):
    ser_seq.write_freq(freq, channel)
    seq = ser_seq.get_array()
    print(len(seq))
    return led_bits(seq)

#test
lab_freq(1000,3)

import serial
from fastcrc import crc16

import struct
import threading
import queue
import time
import asyncio
import dataclasses
from typing import List

def get_crc(frame: bytes) -> int:
    return crc16.xmodem(bytes(frame))


@dataclasses.dataclass
class JointData:
    t_pos: List[float] = dataclasses.field(default_factory=list)
    t_vel: List[float] = dataclasses.field(default_factory=list)
    t_acc: List[float] = dataclasses.field(default_factory=list)
    t_tq: List[float] = dataclasses.field(default_factory=list)

    tq_P: List[float] = dataclasses.field(default_factory=list)
    tq_I: List[float] = dataclasses.field(default_factory=list)
    tq_D: List[float] = dataclasses.field(default_factory=list)
    tq_PID: List[float] = dataclasses.field(default_factory=list)
    tq_fr: List[float] = dataclasses.field(default_factory=list)
    tq_ID: List[float] = dataclasses.field(default_factory=list)

    c_pos: List[float] = dataclasses.field(default_factory=list)
    c_vel: List[float] = dataclasses.field(default_factory=list)
    c_tq: List[float] = dataclasses.field(default_factory=list)
    c_temp: List[float] = dataclasses.field(default_factory=list)

    time: List[float] = dataclasses.field(default_factory=list)

class SerialConnection:
    def __init__(self):
        self.port = serial.Serial(port='/dev/ttyACM0', baudrate=921600, parity=serial.PARITY_NONE,
                                  stopbits=1, bytesize=8, timeout=1)
        self.port.flush()
        self.read_bytes_buffer = b''
        self.read_bytes_buffer_mtx = threading.Lock()
        self.received_data = []
        self.data_queue = queue.Queue()  # ????

        self.read_serial_thread = threading.Thread(target=self.read_serial_buffer, daemon=True).start()
        self.process_serial_thread = threading.Thread(target=self.process_queue, daemon=True).start()

    def enable_debug(self):
        msg = struct.pack('>BB', 155, 1)
        crc = get_crc(msg)
        msg += struct.pack('>H', crc)
        self.port.write(msg)

    def read_serial_buffer(self):
        while True:
            new_buffer = self.port.read_all()
            # if len(new_buffer) > 500:
            #     print(f'\nBIG: {len(new_buffer)}\n')
            self.read_bytes_buffer += new_buffer
            self.read_st_frame()
            time.sleep(0.001)

    def read_st_frame(self):
        # Validate accumulated byte stream
        if len(self.read_bytes_buffer) < 4:
            return
        if len(self.read_bytes_buffer) > 10000:
            self.read_bytes_buffer = b''
            return
        # if (header_idx := self.read_bytes_buffer.find(155)) > 10000 - 4:
        #     self.read_bytes_buffer = b''
        #     return

        # if self.read_bytes_buffer[header_idx + 1] != params.Host_FT.JtcStatus.value:
        #     self.read_bytes_buffer = b''
        #     return
        header_idx = 0
        nd = int.from_bytes(self.read_bytes_buffer[header_idx + 2:header_idx + 4], 'big', signed=False)
        print(nd)
        if len(self.read_bytes_buffer) < (header_idx + nd):
            return
        if nd < 4:
            return
        if nd > 10000:
            self.read_bytes_buffer = b''
            return

        # Save valid buffer
        read_buffer = self.read_bytes_buffer[header_idx:header_idx + nd]

        # Validate CRC
        if (calc_crc := get_crc(read_buffer[:-2])) != struct.unpack('>H', read_buffer[-2:])[0]:
            print(f"Invalid CRC. Received: {struct.unpack('>H', read_buffer[-2:])[0]}, expected: {calc_crc}")
            self.read_bytes_buffer = b''
            return

        ##################################################
        # DEBUG
        # print(f'len(self.read_bytes): {len(self.read_bytes_buffer)}')
        # print(f'header_idx:           {header_idx}')
        # print(f'nd:                   {nd}')
        # print(f'header_idx + nd:      {header_idx + nd}')
        # print([int(read_buffer[i]) for i in range(10)])
        # print(f'new buffer: {len(new_buffer)}')
        ##################################################

        # send received frame to processing
        self.data_queue.put(read_buffer)
        self.read_bytes_buffer = self.read_bytes_buffer[header_idx + nd:]
        return

    def process_queue(self):
        while True:
            frame = self.data_queue.get()
            self.received_data.append(self.process_frame(frame))
            self.data_queue.task_done()

    def process_frame(self, frame):
        print(frame)


if __name__ == '__main__':
    ser = SerialConnection()

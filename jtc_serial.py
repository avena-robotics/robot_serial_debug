import serial
from fastcrc import crc16

import struct
import threading
import queue
import time
import asyncio
import dataclasses
from typing import List
from collections import deque
import math

JOINT_POSMAX = math.pi
JOINT_VELMAX = 2
JOINT_ACCMAX = 4 * math.pi
JOINT_TORQUEMAX = 360.0
MAX_INT16 = 32767.0


def get_crc(frame: bytes) -> int:
    return crc16.xmodem(bytes(frame))


@dataclasses.dataclass
class JointData:
    t_err: deque = dataclasses.field(default_factory=deque)
    t_pos: deque = dataclasses.field(default_factory=deque)
    t_vel: deque = dataclasses.field(default_factory=deque)
    t_acc: deque = dataclasses.field(default_factory=deque)
    t_tq: deque = dataclasses.field(default_factory=deque)
    tq_P: deque = dataclasses.field(default_factory=deque)
    tq_I: deque = dataclasses.field(default_factory=deque)
    tq_D: deque = dataclasses.field(default_factory=deque)
    tq_PID: deque = dataclasses.field(default_factory=deque)
    tq_fr: deque = dataclasses.field(default_factory=deque)
    tq_ID: deque = dataclasses.field(default_factory=deque)
    c_pos: deque = dataclasses.field(default_factory=deque)
    c_vel: deque = dataclasses.field(default_factory=deque)
    c_tq: deque = dataclasses.field(default_factory=deque)
    c_temp: deque = dataclasses.field(default_factory=deque)
    time: deque = dataclasses.field(default_factory=deque)


class SerialConnection:
    def __init__(self, vis):
        self.port = serial.Serial(port='/dev/ttyACM0', baudrate=921600, parity=serial.PARITY_NONE,
                                  stopbits=1, bytesize=8, timeout=1)
        self.port.flush()
        self.read_bytes_buffer = b''
        self.read_bytes_buffer_mtx = threading.Lock()
        self.received_data = vis
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
        if len(self.read_bytes_buffer) > 30000:
            self.read_bytes_buffer = b''
            return
        if (header_idx := self.read_bytes_buffer.find(bytes([155, 156, 157, 158]))) > 30000 - 4 or header_idx == -1:
            self.read_bytes_buffer = b''
            return

        # if self.read_bytes_buffer[header_idx + 1] != params.Host_FT.JtcStatus.value:
        #     self.read_bytes_buffer = b''
        #     return
        nd = int.from_bytes(self.read_bytes_buffer[header_idx + 4:header_idx + 6], 'big', signed=False)
        nb = int.from_bytes(self.read_bytes_buffer[header_idx + 6:header_idx + 8], 'big', signed=False)

        if len(self.read_bytes_buffer) < (header_idx + nb):
            return
        if nb < 4:
            return
        if nb > 30000:
            print("Can't keep up with reading!")
            self.read_bytes_buffer = b''
            return

        # Save valid buffer
        read_buffer = self.read_bytes_buffer[header_idx:header_idx + nb]

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
        offset = 8
        joint_data_len = int((nb - offset - 2) / nd)
        for i in range(nd):
            self.data_queue.put(read_buffer[offset + joint_data_len * i:offset + joint_data_len * (i + 1)])
        self.read_bytes_buffer = self.read_bytes_buffer[header_idx + nb:]
        return

    def process_queue(self):
        while True:
            frame = self.data_queue.get()
            self.process_frame(frame, self.received_data, 1)
            self.data_queue.task_done()

    # fml
    def process_frame(self, frame: bytearray, joint_data: JointData, joint_id: int):
        if len(joint_data.time) >= 50000:
            joint_data.time.rotate(-1)
            joint_data.t_pos.rotate(-1)
            joint_data.t_vel.rotate(-1)
            joint_data.t_acc.rotate(-1)
            joint_data.c_pos.rotate(-1)
            joint_data.c_vel.rotate(-1)
            joint_data.c_tq.rotate(-1)
            joint_data.c_temp.rotate(-1)
            joint_data.t_tq.rotate(-1)
            joint_data.tq_P.rotate(-1)
            joint_data.tq_I.rotate(-1)
            joint_data.tq_D.rotate(-1)
            joint_data.tq_PID.rotate(-1)
            joint_data.tq_fr.rotate(-1)
            joint_data.tq_ID.rotate(-1)
            joint_data.t_err.rotate(-1)
            joint_data.time.pop()
            joint_data.t_pos.pop()
            joint_data.t_vel.pop()
            joint_data.t_acc.pop()
            joint_data.c_pos.pop()
            joint_data.c_vel.pop()
            joint_data.c_tq.pop()
            joint_data.c_temp.pop()
            joint_data.t_tq.pop()
            joint_data.tq_P.pop()
            joint_data.tq_I.pop()
            joint_data.tq_D.pop()
            joint_data.tq_PID.pop()
            joint_data.tq_fr.pop()
            joint_data.tq_ID.pop()
            joint_data.t_err.pop()

        offset = joint_id * 27
        joint_data.time.append(struct.unpack(">f", frame[0:4])[0])
        joint_data.t_pos.append(struct.unpack(">h", frame[offset + 4:offset + 6])[0] / MAX_INT16 * JOINT_POSMAX)
        joint_data.t_vel.append(struct.unpack(">h", frame[offset + 6:offset + 8])[0] / MAX_INT16 * JOINT_VELMAX)
        joint_data.t_acc.append(struct.unpack(">h", frame[offset + 8:offset + 10])[0] / MAX_INT16 * JOINT_ACCMAX)
        joint_data.c_pos.append(struct.unpack(">h", frame[offset + 10:offset + 12])[0] / MAX_INT16 * JOINT_POSMAX)
        joint_data.c_vel.append(struct.unpack(">h", frame[offset + 12:offset + 14])[0] / MAX_INT16 * JOINT_VELMAX)
        joint_data.c_tq.append(struct.unpack(">h", frame[offset + 14:offset + 16])[0] / MAX_INT16 * JOINT_TORQUEMAX)
        joint_data.c_temp.append(struct.unpack(">B", frame[offset + 16:offset + 17])[0])
        joint_data.t_tq.append(struct.unpack(">h", frame[offset + 17:offset + 19])[0] / MAX_INT16 * JOINT_TORQUEMAX)
        joint_data.tq_P.append(struct.unpack(">h", frame[offset + 19:offset + 21])[0] / MAX_INT16 * JOINT_TORQUEMAX)
        joint_data.tq_I.append(struct.unpack(">h", frame[offset + 21:offset + 23])[0] / MAX_INT16 * JOINT_TORQUEMAX)
        joint_data.tq_D.append(struct.unpack(">h", frame[offset + 23:offset + 25])[0] / MAX_INT16 * JOINT_TORQUEMAX)
        joint_data.tq_PID.append(struct.unpack(">h", frame[offset + 25:offset + 27])[0] / MAX_INT16 * JOINT_TORQUEMAX)
        joint_data.tq_fr.append(struct.unpack(">h", frame[offset + 27:offset + 29])[0] / MAX_INT16 * JOINT_TORQUEMAX)
        joint_data.tq_ID.append(struct.unpack(">h", frame[offset + 29:offset + 31])[0] / MAX_INT16 * JOINT_TORQUEMAX)
        joint_data.t_err.append(joint_data.t_pos[-1]-joint_data.c_pos[-1])

if __name__ == '__main__':
    ser = SerialConnection()

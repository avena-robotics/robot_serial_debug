import serial
from fastcrc import crc16

import struct
import threading
import queue
import time
import asyncio
import dataclasses
from typing import List
import math

JOINT_POSMAX = math.pi
JOINT_VELMAX = 2 * math.pi
JOINT_ACCMAX = 4 * math.pi
JOINT_TORQUEMAX = 360.0
MAX_INT16 = 32767.0


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
        joint_data_len = int((nb-offset-2)/nd)
        for i in range(nd):
            self.data_queue.put(read_buffer[offset+joint_data_len*i:offset+joint_data_len*(i+1)])
        self.read_bytes_buffer = self.read_bytes_buffer[header_idx + nb:]
        return

    def process_queue(self):
        while True:
            frame = self.data_queue.get()
            self.process_frame(frame, self.received_data)
            self.data_queue.task_done()

    # fml
    def process_frame(self, frame: bytearray, joint_data: JointData):
        joint_data.time.append(struct.unpack(">f", frame[0:4])[0])
        joint_data.t_pos.append(struct.unpack(">H", frame[4:6])[0] / JOINT_POSMAX * MAX_INT16)
        joint_data.t_vel.append(struct.unpack(">H", frame[6:8])[0] / JOINT_VELMAX * MAX_INT16)
        joint_data.t_acc.append(struct.unpack(">H", frame[8:10])[0] / JOINT_ACCMAX * MAX_INT16)
        joint_data.c_pos.append(struct.unpack(">H", frame[10:12])[0] / JOINT_POSMAX * MAX_INT16)
        joint_data.c_vel.append(struct.unpack(">H", frame[12:14])[0] / JOINT_VELMAX * MAX_INT16)
        joint_data.c_tq.append(struct.unpack(">H", frame[14:16])[0] / JOINT_TORQUEMAX * MAX_INT16)
        joint_data.c_temp.append(struct.unpack(">B", frame[16:17])[0])
        joint_data.t_tq.append(struct.unpack(">H", frame[17:19])[0] / JOINT_TORQUEMAX * MAX_INT16)
        joint_data.tq_P.append(struct.unpack(">H", frame[19:21])[0] / JOINT_TORQUEMAX * MAX_INT16)
        joint_data.tq_I.append(struct.unpack(">H", frame[21:23])[0] / JOINT_TORQUEMAX * MAX_INT16)
        joint_data.tq_D.append(struct.unpack(">H", frame[23:25])[0] / JOINT_TORQUEMAX * MAX_INT16)
        joint_data.tq_PID.append(struct.unpack(">H", frame[25:27])[0] / JOINT_TORQUEMAX * MAX_INT16)
        joint_data.tq_fr.append(struct.unpack(">H", frame[27:29])[0] / JOINT_TORQUEMAX * MAX_INT16)
        joint_data.tq_ID.append(struct.unpack(">H", frame[29:31])[0] / JOINT_TORQUEMAX * MAX_INT16)


if __name__ == '__main__':
    ser = SerialConnection()

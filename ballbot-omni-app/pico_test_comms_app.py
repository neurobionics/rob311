from types import DynamicClassAttribute
import serial
import time
import logging
import numpy as np
import threading
from threading import Thread, Lock, current_thread
from MBot.Messages.message_defs import data_rpi_dtype
from MBot.SerialProtocol.protocol import SerialProtocol


if __name__ == "__main__":
    ser_dev = SerialProtocol()
    ser_dev.serializer_dict[101] = [lambda bytes: np.frombuffer(bytes, dtype=data_rpi_dtype), lambda data: data.tobytes()]
    ser_dev.serializer_dict[102] = [lambda bytes: np.frombuffer(bytes, dtype=data_rpi_dtype), lambda data: data.tobytes()]

    serial_read_thread = Thread(target = SerialProtocol.read_loop, args=(ser_dev,), daemon=True)
    serial_read_thread.start()

    test_data = np.zeros(1, dtype=data_rpi_dtype)
    ser_dev.send_topic_data(101, test_data)

    time.sleep(1.0) # give it time for new messages to arrive

    counter = 1
    while(True):
        print("current data loop {0}:".format(counter))
        counter += 1

        try:
            cur_data = ser_dev.get_cur_topic_data(102)[0]
        except KeyError:
            print("no data found, retrying!")
            ser_dev.send_topic_data(101, test_data)
            time.sleep(1.0)
            continue

        print(cur_data)
        print("post increment:")
        cur_data['imu_a_z'] += 1
        print(cur_data)

        print("sending_data..")
        ser_dev.send_topic_data(101, cur_data)

        time.sleep(1.0)
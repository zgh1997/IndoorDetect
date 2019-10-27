import os
import os.path
import copy
import time
import logging
import serial
import _thread
import threading
import binascii
from ctypes import *
import math
import datetime
import json

from collections import OrderedDict

data_port = serial.Serial()
user_port = serial.Serial()
data_buffer = []
dataReceiveThread = ""

# Data structure length
HEADER_SIZE = 52
FORMAT_TRANS = 2 # ascii字符串传输为16进制，index*2
TL_HEADER_SIZE = 8
POINT_UNIT_SIZE = 16
SINGLE_UART_POINT_DATA_SIZE = 6
SINGLE_TARGET_DATA_SIZE = 68
SINGLE_TARGET_INDEX_SIZE = 1
SINGLE_CLASSIFIER_DATA_SIZE = 8

EC_LEN = 9
asc2hex_float_len = 4 * FORMAT_TRANS
asc2hex_int8_len = 1 * FORMAT_TRANS
asc2hex_int16_len = 2 * FORMAT_TRANS

# Point data | Target data | (Target index) | Classifier data
uart_point_data = {"range":0.0, "azimuth":0.0, "doppler":0.0, "snr":0.0}
point_data_cartesian = {"x":0.0, "y":0.0}
target_data = {"tid": 0, "pos_x": 0.0, "pos_y": 0.0, "vel_x": 0.0, "vel_y": 0.0, "ec9": None, "g": 0.0}
classifier_data = {"activeTargetID": 0, "targetTag": 0}

def open_port():
    try:
        data_port.port = 'COM6'  # 端口号
        data_port.baudrate = 921600  # 波特率
        data_port.bytesize = 8  # 数据位
        data_port.stopbits = 1  # 停止位
        data_port.parity = "N"  # 校验位
        data_port.open()
        if data_port.isOpen():
            print("打开成功")
        else:
            print("打开失败")

        # global dataReceiveThread
        # dataReceiveThread = threading.Thread(target=data_receive_function, args=())
        # dataReceiveThread.start()

        user_port.port = "COM5"  # 端口号
        user_port.baudrate = 115200  # 波特率
        user_port.bytesize = 8  # 数据位
        user_port.stopbits = 1  # 停止位
        user_port.parity = "N"  # 校验位
        user_port.open()
        if user_port.isOpen():
            print("打开成功")
        else:
            print("打开失败")
        # _thread.start_new_thread(init_board, ())

        init_board()
        # data_receive_function()
        _thread.start_new_thread(data_receive_function(), ())

    except Exception as ex:
        print(ex)

def init_board():
    global time_start
    global time_start_flag
    current_dir = os.path.dirname(__file__)
    file = open(current_dir + "/../chirp_configs/indoor_false_det_68xx.cfg", "r+")
    if file is None:
        print("配置文件不存在!")
        return
    for text in file.readlines():

        if text == "sensorStart":
            if time_start_flag is False:
                time_start = time.time()
                time_start_flag = True
        print("send config:" + text)

        user_port.write(text.encode('utf-8'))
        user_port.write('\n'.encode('utf-8'))
        time.sleep(0.2)
    file.close()

def data_receive_function():
    ponit_data_frame_list = []  # 该列表用来存放点数据
    target_data_frame_list = [] # 该列表用来存放目标数据
    classifier_data_frame_list = [] # 该列表用来存放分类结果数据(人或者地板，有ID和type)
    count = 0
    pointCloudJson = OrderedDict()
    targetJson = OrderedDict()
    classifierJson = OrderedDict()

    while 1:
        if data_port is not None and data_port.isOpen():
            try:
                if data_port.in_waiting:
                    buffer = str(binascii.b2a_hex(data_port.read(data_port.in_waiting)))[2:-1]
                    valid_data = []
                    for i in range(len(buffer)):
                        valid_data.append((buffer[i]))
                    # print(valid_data)
                    data_buffer.extend(valid_data)
                    point_data_frame, target_data_frame, classifier_data_frame = process_data()
                    # print(point_data)
                    ponit_data_frame_list.extend(point_data_frame)
                    target_data_frame_list.extend(point_data_frame)
                    classifier_data_frame_list.extend(point_data_frame)
                    count += 1

                    if point_data_frame:
                        # Update point cloud file
                        pointCloudJson.update({count: point_data_frame})

                    if target_data_frame:
                        # Update point cloud file
                        targetJson.update({count: target_data_frame})

                    if classifier_data_frame:
                        # Update point cloud file
                        classifierJson.update({count: classifier_data_frame})


                # Store points and human info in json file
                with open('../Data/PointCloud.json', 'w') as f:
                    # f.write(str(pointCloudJson))
                    json.dump(pointCloudJson, f)

                with open('../Data/Target.json', 'w') as f:
                    json.dump(targetJson, f)

                with open('../Data/Classifier.json', 'w') as f:
                    json.dump(classifierJson, f)

            except TimeoutError:
                print('Time Out Error')
            except Exception as ex:
                print("EXCEPTION:", ex.__traceback__)
                print("EXCEPTION_detail:", ex)
        time.sleep(0.01)

def process_data():
    uart_point_data_frame, point_data_frame, target_data_frame, classifier_data_frame = None, None, None, None
    while len(data_buffer) >= HEADER_SIZE:
        frame_data = get_frame()
        if frame_data is None:
            return
        # (frame_data)

        if len(frame_data) == HEADER_SIZE*2:
            print("空数据帧！")
            continue


        index = HEADER_SIZE * 2
        # Split point cloud data、target data and classifier data
        # Get point cloud data
        tlv_type = int(convert_string("".join(frame_data[index:index + asc2hex_float_len])), 16)
        index += asc2hex_float_len
        point_cloud_len = int(convert_string("".join(frame_data[index:index + asc2hex_float_len])), 16) - 16
        print("Point cloud len: " + str(point_cloud_len))
        index += asc2hex_float_len
        index += POINT_UNIT_SIZE * FORMAT_TRANS
        point_cloud_len -= POINT_UNIT_SIZE * FORMAT_TRANS
        point_num = int(point_cloud_len / SINGLE_UART_POINT_DATA_SIZE)

        if tlv_type == 6:
            print("Point cloud: Exists " + str(point_num) + " points")

            uart_point_data_frame = [copy.deepcopy(uart_point_data) for i in range(point_num)]
            point_data_frame = [copy.deepcopy(point_data_cartesian) for i in range(point_num)]
            for i in range(point_num):
                azimuth = byte_to_float(convert_string("".join(frame_data[index:index + asc2hex_int8_len])))
                uart_point_data_frame[i]["azimuth"] = azimuth
                index += asc2hex_int8_len
                uart_point_data_frame[i]["doppler"] = byte_to_float(convert_string("".join(frame_data[index:index + asc2hex_int8_len])))
                index += asc2hex_int8_len
                _range = byte_to_float(convert_string("".join(frame_data[index:index + asc2hex_int16_len])))
                uart_point_data_frame[i]["range"] = _range
                index += asc2hex_int16_len
                snr = byte_to_float(convert_string("".join(frame_data[index:index + asc2hex_int16_len])))
                uart_point_data_frame[i]["snr"] = byte_to_float(convert_string("".join(frame_data[index:index + asc2hex_int16_len])))
                index += asc2hex_int16_len

                xs = _range * math.sin(azimuth)
                ys = _range * math.cos(azimuth)
                point_data_frame[i]["x"] = xs
                point_data_frame[i]["y"] = ys

            if index >= len(frame_data):
                return point_data_frame, None, None
        else:
            print("TLV type error.")
            return None, None, None

        # Get target data
        tlv_type = int(convert_string("".join(frame_data[index:index + asc2hex_float_len])), 16)

        index += asc2hex_float_len
        target_list_len = int(convert_string("".join(frame_data[index:index + asc2hex_float_len])), 16) - 16
        index += asc2hex_float_len
        target_num = int(target_list_len / SINGLE_TARGET_DATA_SIZE)
        if tlv_type == 7:

            print("Target num : Exists " + str(target_num) + " targets")
            target_data_frame = [copy.deepcopy(target_data) for i in range(target_num)]

            for i in range(target_num):
                # tid = int.from_bytes(bytearray(frame_data[index:index + 4]), signed=False)
                tid = int(convert_string("".join(frame_data[index:index + asc2hex_float_len])), 16)
                index += asc2hex_float_len
                pos_x = byte_to_float(convert_string("".join(frame_data[index:index + asc2hex_float_len])))
                index += asc2hex_float_len
                pos_y = byte_to_float(convert_string("".join(frame_data[index:index + asc2hex_float_len])))
                index += asc2hex_float_len
                vel_x = byte_to_float(convert_string("".join(frame_data[index:index + asc2hex_float_len])))
                index += asc2hex_float_len
                vel_y = byte_to_float(convert_string("".join(frame_data[index:index + asc2hex_float_len])))
                index += asc2hex_float_len
                index += EC_LEN * asc2hex_float_len
                index += asc2hex_float_len

                target_data_frame[tid] = target_data.copy()
                target_data_frame[tid]["tid"] = tid
                target_data_frame[tid]["pos_x"] = pos_x
                target_data_frame[tid]["pos_y"] = pos_y
                target_data_frame[tid]["vel_x"] = vel_x
                target_data_frame[tid]["vel_y"] = vel_y

            if index >= len(frame_data):
                return point_data_frame, target_data_frame, None


        # Ignore target index
        tlv_type = int(convert_string("".join(frame_data[index:index + asc2hex_float_len])), 16)
        index += asc2hex_float_len
        target_index_len = int(convert_string("".join(frame_data[index:index + asc2hex_float_len])), 16) - 16
        index += asc2hex_float_len
        target_num = int(target_index_len / SINGLE_TARGET_INDEX_SIZE)
        if tlv_type == 8:
            index += target_index_len
            if index >= len(frame_data):
                return point_data_frame, target_data_frame, None

        # Get classifier data
        tlv_type = int(convert_string("".join(frame_data[index:index + asc2hex_float_len])), 16)
        index += asc2hex_float_len
        classifier_list_len = int(convert_string("".join(frame_data[index:index + asc2hex_float_len])), 16) - 16
        index += asc2hex_float_len
        classifier_num = int(classifier_list_len / SINGLE_TARGET_DATA_SIZE)
        if tlv_type == 9:
            print("Classifier num : Exists " + str(classifier_num) + " targets")
            classifier_data_frame = [copy.deepcopy(classifier_data) for i in range(classifier_num)]

            for i in range(classifier_num):
                activeTargetID = byte_to_uint32(convert_string("".join(frame_data[index:index + asc2hex_float_len])))
                index += asc2hex_float_len
                targetTag = byte_to_int32(convert_string("".join(frame_data[index:index + asc2hex_float_len])))
                index += asc2hex_float_len

                classifier_data_frame[activeTargetID] = target_data.copy()
                classifier_data_frame[activeTargetID]["activeTargetID"] = activeTargetID
                classifier_data_frame[activeTargetID]["targetTag"] = targetTag

    return point_data_frame, target_data_frame, classifier_data_frame

def get_frame():
    data_str = "".join(data_buffer)
    # print("data_str:" + data_str)
    start_index = data_str.index("0201040306050807")
    if start_index == -1:
        return None
    start_index = int(start_index)
    del data_buffer[0:start_index]
    start_index = 0
    if len(data_buffer) < HEADER_SIZE:
        return None
    packet_len = int(convert_string("".join(data_buffer[start_index + 40:start_index + 48])), 16)
    tlv_num = int(convert_string("".join(data_buffer[start_index + 96:start_index + 100])), 16)
    print("数据包大小:" + str(packet_len))
    print("TLV num: " + str(tlv_num))
    if packet_len > 30000:
        print("数据包大小超过30000，丢弃帧")
        del data_buffer[0:24]
        return None
    if len(data_buffer) < packet_len:
        return None
    ret = copy.deepcopy(data_buffer[start_index: start_index + packet_len * 2])
    del data_buffer[start_index: start_index + packet_len * 2]

    return ret

def on_application_quit():
    try:
        if data_port is not None:
            user_port.close()
        if data_port is not None:
            data_port.close()
    except Exception as ex:
        print("on_application_quit" + ex.__context__)

def convert_string(string):
    try:
        # str1 = string[2:4] + string[0:2] + string[6:8] + string[4:6]
        str1 = string[6:8] + string[4:6] + string[2:4] + string[0:2]
        return str1
    except IndexError as idxerr:
        print("convert_string" + idxerr.__context__)

def byte_to_float(s):
    i = int(s, 16)
    cp = pointer(c_int(i))
    fp = cast(cp, POINTER(c_float))
    return fp.contents.value

def byte_to_uint32(s):
    i = int(s, 16)
    cp = pointer(c_uint32(i))
    return cp.contents.value

def byte_to_int32(s):
    i = int(s, 16)
    cp = pointer(c_int(i))
    return cp.contents.value

if __name__ == '__main__':
    open_port()

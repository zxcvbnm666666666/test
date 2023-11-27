import serial
import binascii
import pynmea2
import threading
import datetime
import time
import tkinter as tk
from tkinter_control.tkinter_op import tk_fun
# from mqtt_control.mqtt_op import Information
from pb_control.proto_send import proto_send
from cane_control.ringclass import RingClass
from mqtt_control.mqtt_op import mqtt_init
from multiprocessing import Process, Manager, Pool, Queue
from paho.mqtt import client as mqtt
from datetime import datetime as datetime1
count_oilPressure = 0
oilPressure_real_data_dict = {}
ring_buffer_zone_oilPressure = RingClass(100)
number_oil = 0

count_flow = 0
flow_real_data_dict = {}
ring_buffer_zone_flow = RingClass(100)
number_flow = 0

count_rotate = 0
rotate_real_data_dict = {}
ring_buffer_zone_rotate = RingClass(100)
number_rotate = 0

Information = {

    'MQTTHOST': "mqtt.gdaas-tea.com",

    'mqtt_publish': "cn.gdaas.zbs/145/report",

    'mqtt_subscribe': "cn.gdaas.zbs/145/cmd",

    'nodeId': '399',

    'address': '1',

    'channel': '1',

    'reboot_time': 5,

    'collect_time': 300,

    'start_time': 21,

    'end_time': 23,

    'serial_buad': 115200

}


def main():
    q1 = Queue()
    q2 = Queue()
    print('123')

    p1 = Process(target=threading_control, args=(q1, q2))
    p2 = Process(target=tk_fun, args=(q1, q2))

    # p2.start()
    p1.start()
    p1.join()
    # p2.join()
    print('1236')


def threading_control(q1, q2):
    t1 = threading.Thread(target=new_recv_data, args=())
    t2 = threading.Thread(target=angle_lon_lat_recv_data, args=(q1, q2))
    #t2.start()
    t1.start()
    t1.join()
    #t2.join()


def new_recv_data():

    client = mqtt.Client('990')  # 客户段对象，client_id必须要给出，即（time.strftime）
    client = mqtt_init(client)
    while True:
        a(client)
        b(client)





def a(client):
    global oilPressure_real_data_dict
    global count_oilPressure
    global number_oil
    oil_serial = serial.Serial("/dev/ttysWK0", 115200)
    ser_data_read = oil_serial.read(23)
    oil_data = str((binascii.b2a_hex(ser_data_read)).decode())
    if oil_data[0:4] != "55aa":
        location_55aa = oil_data.find("55aa")
        second_read = oil_serial.read(int(location_55aa / 2))
        ser_oilPressure_data_str = ''
    else:
        if len(oil_data) == 46:
            data_mark = "_oilPressure_ch"
            data_length = len(ser_receive_data)  # 余数只有0123
            times = int(((data_length - 6) / 4))  # 取整
            for i in range(times):
                j = i + 1
                if j == 9:  # 排出crc校验码数据,8是因为i从0开始算的
                    data_h = j * 4 + 2
                    data_l = data_h + 4
                else:
                    if j != 10:
                        data_channel_i = str(j) + data_mark  # 通道命名
                        data_h = j * 4 + 2
                        data_l = data_h + 4
                        data_16 = ser_receive_data[data_h:data_l]
                        data_10 = int(data_16, 16)  # 返回来处理过的数据

                        if data_10 <= 880:
                            data_10_ele = 4
                        if data_10 >= 4400:
                            data_10_ele = 20
                        if 880 < data_10 < 4400:
                            data_10_ele = data_10 / 220
                        data_true = 15.63 * data_10_ele - 62.52
                        sensor_type = 23000 + j
                        if sensor_type == 23001:
                            # 每个完整的数据先传过来的都是24002这个，因为最先解析的是它
                            del sensors_type_list[:]
                            del value1s_flow_list[:]
                            del value2s_flow_list[:]
                        sensors_type_list.append(sensor_type)
                        value1s_flow_list.append(str(data_true)[:4])
                        value2s_flow_list.append(0)
                        if len(sensors_type_list) == 8 and len(value1s_flow_list) == 8:
                            number_oil += 1
                            nodeid = Information['nodeId']
                            # print('89', number_mqtt)
                            result = proto_send().DValuesReq(nodeid, sensors_type_list, value1s_flow_list,
                                                             value2s_flow_list)

                            if number_oil == 20:
                                number_oil = 0
                                client.publish("cn.gdaas.zbs/145/report", result, qos=0)  # 上云
                                print("send_oilpressure_data success")
                        data_list.append(str(data_true)[:4])
                        channle_list.append(data_channel_i)
                        oilPressure_real_data_dict = dict(zip(channle_list, data_list))
                        dt = datetime1.now()
                        # print('000')
                        date = dt.strftime('%Y.%m.%d')
                        time1 = dt.strftime('%H:%M:%S')
                        # print('000')
                        channle_list.append("date")
                        # print('1235')
                        data_list.append(date)
                        channle_list.append("time")
                        data_list.append(time1)
                        # print('1235')
                        data_dict_history = dict(zip(channle_list, data_list))
                        if len(data_dict_history) == 11:

                            # print('1235')
                            flow_file_name = 'oilPressure_data.txt'

                            try:
                                count_oilPressure = count_oilPressure + 1
                                # print('1235',count_oilPressure )
                                # 可能要对这个data_list进行类型变换
                                ring_buffer_zone_oilPressure.append(data_dict_history)
                                print('1235', count_oilPressure)
                                if count_oilPressure == 100:
                                    print('1235', count_oilPressure)
                                    count_oilPressure = 0
                                    fopen = open(flow_file_name, 'a')
                                    print('start write oilPressure data now--------------')
                                    x_list = ring_buffer_zone_oilPressure.tolist()
                                    for i in x_list:
                                        fopen.write(str(i))
                                        fopen.write("\r")
                                    fopen.close()

                            except:
                                pass


def b(client):
    global flow_real_data_dict
    global count_flow
    global number_flow
    flow_serial = serial.Serial("/dev/ttysWK0", 115200)
    flow_data_read = flow_serial.read(23)
    flow_data = str((binascii.b2a_hex(flow_data_read)).decode())
    if oil_data[0:4] != "55aa":
        location_55aa = oil_data.find("55aa")
        second_read = oil_serial.read(int(location_55aa / 2))
        ser_oilPressure_data_str = ''
    else:
        if len(flow_data) == 46:
            ser_receive_data = data
            data_mark = "_flow_ch"
            data_length = len(ser_receive_data)  # 余数只有0123
            times = int(((data_length - 6) / 4))  # 取整
            # 扭矩传感器的times是10
            for i in range(times):
                # 其中j是从1到10的
                j = i + 1
                if j == 9:  # 排出crc校验码数据,8是因为i从0开始算的
                    data_h = j * 4 + 2
                    data_l = data_h + 4
                else:
                    if j != 10:
                        data_channel_i = str(j) + data_mark  # 通道命名
                        data_h = j * 4 + 2
                        data_l = data_h + 4
                        data_16 = ser_receive_data[data_h:data_l]
                        data_10 = int(data_16, 16)  # 返回来处理过的数据
                        if data_10 <= 880:
                            data_10_ele = 4
                        if data_10 >= 4400:
                            data_10_ele = 20
                        if 880 < data_10 < 4400:
                            data_10_ele = data_10 / 220

                        if j == 1:
                            # 每个完整的数据先传过来的都是24002这个，因为最先解析的是它
                            del sensors_type_list[:]
                            del value1s_flow_list[:]
                            del value2s_flow_list[:]
                            sensor_type = 26000 + j
                            data_true = 8.33 * data_10_ele - 33.33
                            sensors_type_list.append(sensor_type)
                            value1s_flow_list.append(str(data_true)[:4])
                            value2s_flow_list.append(0)
                            data_list.append(str(data_true)[:4])
                            channle_list.append(data_channel_i)
                        if j == 2:
                            sensor_type = 26000 + j
                            data_true = 8.33 * data_10_ele - 33.33
                            sensors_type_list.append(sensor_type)
                            value1s_flow_list.append(str(data_true)[:4])
                            value2s_flow_list.append(0)
                            data_list.append(str(data_true)[:4])
                            channle_list.append(data_channel_i)

                        if j == 3:
                            sensor_type = 26000 + j
                            data_true = 8.33 * data_10_ele - 33.33
                            sensors_type_list.append(sensor_type)
                            value1s_flow_list.append(str(data_true)[:4])
                            value2s_flow_list.append(0)
                            data_list.append(str(data_true)[:4])
                            channle_list.append(data_channel_i)

                        if j == 4:
                            sensor_type = 26000 + j
                            data_true = 8.33 * data_10_ele - 33.33
                            sensors_type_list.append(sensor_type)
                            value1s_flow_list.append(str(data_true)[:4])
                            value2s_flow_list.append(0)
                            data_list.append(str(data_true)[:4])
                            channle_list.append(data_channel_i)

                        if len(sensors_type_list) == 4 and len(value1s_flow_list) == 4:
                            number_flow += 1
                            nodeid = Information['nodeId']
                            # print('89', number_mqtt)
                            result = proto_send().DValuesReq(nodeid, sensors_type_list, value1s_flow_list,
                                                             value2s_flow_list)

                            if number_flow == 20:
                                number_flow = 0
                                client.publish("cn.gdaas.zbs/145/report", result, qos=0)  # 上云
                                print("send_flow_data success")

            flow_real_data_dict = dict(zip(channle_list, data_list))
            # print("90")
            dt = datetime1.now()
            # print('000')
            date = dt.strftime('%Y.%m.%d')
            time1 = dt.strftime('%H:%M:%S')
            # print('000')
            channle_list.append("date")
            # print('1235')
            data_list.append(date)
            channle_list.append("time")
            data_list.append(time1)
            # print('1235')
            data_dict_history = dict(zip(channle_list, data_list))

            if len(data_dict_history) == 11:
                flow_file_name = 'flow_data.txt'
                try:
                    count_flow = count_flow + 1
                    # print('1235',count_flow )
                    # 可能要对这个data_list进行类型变换
                    ring_buffer_zone_flow.append(data_dict_history)
                    print('1235', count_flow)
                    if count_flow == 100:
                        print('1235', count_flow)
                        count_flow = 0
                        fopen = open(flow_file_name, 'a')
                        print('start write flow data now--------------')
                        x_list = ring_buffer_zone_flow.tolist()
                        for i in x_list:
                            fopen.write(str(i))
                            fopen.write("\r")
                        fopen.close()

                except:
                    pass




AngleData = [0.0] * 8
lon_latData = [0.0] * 8
FrameState = 0  # 通过0x后面的值判断属于哪一种情况
Bytenum = 0  # 读取到这一段的第几位
CheckSum = 0  # 求和校验位
sensors_angle_list = []
value1s_angle_list = []
value2s_angle_list = []

sensors_lon_list = []
value1s_lon_list = []
value2s_lon_list = []

count_angle = 0
count_lon = 0
Angle = [0.0] * 3
lon_lat = [0, 0] * 2
Number_Angle = 0
Number_Lon = 0
ring_buffer_zone_angle = RingClass(100)
ring_buffer_zone_lon = RingClass(100)


# angle_lon_lat_recv_data是单独接入USB口的姿态角度和经纬度
def angle_lon_lat_recv_data(q1, q2):
    client = mqtt.Client('999')  # 客户段对象，client_id必须要给出，即（time.strftime）
    client = mqtt_init(client)

    global FrameState  # 在局部修改全局变量，要进行global的定义
    global Bytenum
    global CheckSum
    global lon_lat
    global Angle
    global Number_Angle
    global Number_Lon
    global count_angle
    global count_lon
    angle_data = []
    lon_data = []
    angle_lon_lat_serial = serial.Serial('/dev/ttyUSB0', 115200)  # 对应板子上的串口号
    while True:
        try:
            datahex = angle_lon_lat_serial.read(33)  # 读取33个字节
            # print('datahex', datahex.hex())
            for data in datahex:
                # 在输入的数据进行遍历
                # print('data',data)
                # Python2软件版本这里需要插入 data = ord(data)*****************************************************************************************************
                if FrameState == 0:  # 当未确定状态的时候，进入以下判断

                    if data == 0x55 and Bytenum == 0:  # 0x55位于第一位时候，开始读取数据，增大bytenum
                        CheckSum = data
                        Bytenum = 1

                        continue
                    elif data == 0x53 and Bytenum == 1:
                        CheckSum += data
                        FrameState = 3
                        Bytenum = 2
                    elif data == 0x57 and Bytenum == 1:
                        CheckSum += data
                        FrameState = 7
                        Bytenum = 2


                elif FrameState == 3:  # angle

                    if Bytenum < 10:
                        AngleData[Bytenum - 2] = data
                        CheckSum += data
                        Bytenum += 1
                    else:
                        if data == (CheckSum & 0xff):
                            rxl = AngleData[0]
                            rxh = AngleData[1]
                            ryl = AngleData[2]
                            ryh = AngleData[3]
                            rzl = AngleData[4]
                            rzh = AngleData[5]
                            k_angle = 180.0

                            angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
                            angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
                            angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
                            if angle_x >= k_angle:
                                angle_x -= 2 * k_angle
                            if angle_y >= k_angle:
                                angle_y -= 2 * k_angle
                            if angle_z >= k_angle:
                                angle_z -= 2 * k_angle

                            # print('angle_data', angle_x)
                            sensors_angle_list = []
                            value1s_angle_list = []
                            value2s_angle_list = []
                            value1s_angle_list = [angle_x, angle_y, angle_z]
                            nodeid = Information['nodeId']
                            print('nodeid', value1s_angle_list[0])
                            sensors_angle_list = [str(27001), str(27002), str(27003)]
                            value2s_angle_list = [0, 0, 0]
                            # print('1234')
                            result = proto_send().DValuesReq(nodeid, sensors_angle_list, value1s_angle_list,
                                                             value2s_angle_list)
                            # print('1234')
                            print('result', result)
                            client.publish("cn.gdaas.zbs/145/report", result, qos=1)  # 上云

                            angle_data = list(map(str, value1s_angle_list))
                            angle_list = ['jiaodu_x', 'jiaodu_y', 'jiaodu_z', 'date', 'time', 'Number']
                            print('1234')
                            from datetime import datetime as datetime1
                            dt = datetime1.now()
                            date = dt.strftime('%Y.%m.%d ')
                            time = dt.strftime('%H:%M:%S.%f')
                            angle_data.append(date)
                            angle_data.append(time)
                            Number_Angle = Number_Angle + 1
                            if Number_Angle == 6000:
                                Number_Angle = 0
                            angle_data.append(Number_Angle)
                            data_dict_history = dict(zip(angle_list, angle_data))
                            # print("数据",data_dict_history['jiaodu_x'])
                            # print("数据1",data_dict_history['date'])
                            flow_file_name = 'Angle_data.txt'
                            count_angle = count_angle + 1
                            # print('123',count_angle)
                            # 可能要对这个data_list进行类型变换
                            ring_buffer_zone_angle.append(data_dict_history)
                            if count_angle == 100:
                                count_angle = 0
                                fopen = open(flow_file_name, 'a')
                                print('start write angle data now--------------')
                                x_list = ring_buffer_zone_angle.tolist()
                                for i in x_list:
                                    fopen.write(str(i))
                                    fopen.write("\r")
                                fopen.close()

                        CheckSum = 0
                        Bytenum = 0
                        FrameState = 0

                elif FrameState == 7:  # 经纬度

                    if Bytenum < 10:
                        lon_latData[Bytenum - 2] = data
                        CheckSum += data
                        Bytenum += 1
                    else:
                        if data == (CheckSum & 0xff):
                            lon0 = lon_latData[0]
                            lon1 = lon_latData[1]
                            lon2 = lon_latData[2]
                            lon3 = lon_latData[3]
                            lat0 = lon_latData[4]
                            lat1 = lon_latData[5]
                            lat2 = lon_latData[6]
                            lat3 = lon_latData[7]
                            k_Data1 = 10000000
                            k_Data2 = 100000
                            Lon1 = ((lon3 << 24) | (lon2 << 16) | (lon1 << 8) | lon0)
                            Lat1 = ((lat3 << 24) | (lat2 << 16) | (lat1 << 8) | lat0)
                            Lon = Lon1 / k_Data1 + ((Lon1 % k_Data1) / k_Data2) / 60
                            Lat = Lat1 / k_Data1 + ((Lat1 % k_Data1) / k_Data2) / 60
                            # print("lon_lat", Lon)

                            sensors_lon_list = []
                            value1s_lon_list = []
                            value2s_lon_list = []
                            # print('1237')
                            value1s_lon_list = [Lon, Lat]
                            nodeid = Information['nodeId']
                            sensors_lon_list = [str(15001)]
                            value2s_lon_list = [0, 0]
                            # print('1237')
                            result_a = proto_send().DValuesReq(nodeid, sensors_lon_list, value1s_lon_list,
                                                               value2s_lon_list)
                            print('1237')
                            client.publish("cn.gdaas.zbs/145/report", result_a, qos=1)  # 上云

                            lon_data = list(map(str, value1s_lon_list))
                            lon_list = ['lon', 'lat', 'date', 'time', 'Number']
                            from datetime import datetime as datetime1
                            dt = datetime1.now()
                            date = dt.strftime('%Y.%m.%d ')
                            time = dt.strftime('%H:%M:%S.%f')
                            lon_data.append(date)
                            lon_data.append(time)
                            Number_Lon = Number_Lon + 1
                            if Number_Lon == 6000:
                                Number_Lon = 0
                            lon_data.append(Number_Lon)
                            data_dict_history = dict(zip(lon_list, lon_data))
                            # print("数据",data_dict_history['jiaodu_x'])
                            # print("数据1",data_dict_history['date'])
                            flow_file_name = 'lon_data.txt'
                            count_lon = count_lon + 1
                            # print('123',count_angle)
                            # 可能要对这个data_list进行类型变换
                            ring_buffer_zone_lon.append(data_dict_history)
                            if count_lon == 100:
                                count_lon = 0
                                fopen = open(flow_file_name, 'a')
                                print('start write lon data now--------------')
                                x_list = ring_buffer_zone_lon.tolist()
                                for i in x_list:
                                    fopen.write(str(i))
                                    fopen.write("\r")
                                fopen.close()
                        CheckSum = 0
                        Bytenum = 0
                        FrameState = 0
        except:
            pass

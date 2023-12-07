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
import sys
import signal
import functools


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

# 信号处理程序
def sig_handler(signum, frame, p1, p2):
    # 停止进程并在必要时进行清理
    p1.terminate()          #终止进程P1
    p2.terminate()
    # 在这里添加清理代码
    # ...
    sys.exit(0)


def main():
    
    oil = Queue()                #代表油压的队列
    flow = Queue()               #流量的队列
    rotate = Queue()             #转速的队列
    angle = Queue()              #角度的队列
    lon = Queue()                #经纬度的队列
    print('123')
    
    p1 = Process(target=threading_control, args=(oil, flow, rotate, angle, lon))
    p2 = Process(target=tk_fun, args=(oil, flow, rotate, angle, lon))
    handler = functools.partial(sig_handler, p1, p2)
    # 注册信号处理程序
    signal.signal(signal.SIGTSTP, handler) # Ctrl+Z
    signal.signal(signal.SIGINT, handler) # Ctrl+C
  
    p2.start()
    p1.start()
    p1.join()
    p2.join()
    print('1236')


def threading_control(oil, flow, rotate, angle, lon):
    t1 = threading.Thread(target=oil_Pressure_recv_data, args=(oil,))
    t2 = threading.Thread(target=flow_data_recv_data, args=(flow,))
    t3 = threading.Thread(target=rotate_data_recv_data, args=(rotate,))
    t4 = threading.Thread(target=angle_lon_lat_recv_data, args=(angle, lon))
    t4.start()
    t3.start()
    t2.start()
    t1.start()
    t1.join()
    t2.join()
    t3.join()
    t4.join()


def oil_Pressure_recv_data(oil):
    
    data_length = 46                                       # 信息采集节点板的数据长度，油压和流量的是55AA02开头,z脉冲（转速）的是55AA01开头
    oil_serial = serial.Serial("/dev/ttysWK0", 115200)     #RK3399上对应的串口号，更换板子时可更换串口

    oil_list = [0.0]*8                                   #用于存储写入队列的一个列表

    data_dict_history = {}                                #代表写入txt文件的一行字典数据
    ring_buffer_zone_oilPressure = RingClass(100)         #缓冲区，用来存储data_dict_history
    number_mqtt = 0                                       #上云的间隔
    count_oilPressure = 0                                 #写入txt文件的间隔

    data_list = []                                        #存储字典中的数据，例如：txt中的一个字典数据'1_oilPressure_ch': '0.0'。data_list用来存储'0.0'
    channle_list = []                                     #存储字典中的类型名称，例如：txt中的一个字典数据'1_oilPressure_ch': '0.0'。channle_list用来存储'1_oilPressure_ch'
    sensors_type_list = []                                #存储这pb协议定义的数据类型，如：23代表是油压，24代表的是流量
    value1s_list = []                                     #存储信息节点板上8个接口的数据
    value2s_list = []                                     
    

    while True:
        try:
            ser_data_read = oil_serial.read(23)
            # ASCII码转成16进制
            ser_data = str((binascii.b2a_hex(ser_data_read)).decode())
            if ser_data[0:4] != "55aa":
                # 找到55aa的位置下标，下次读取这个下标的字符，扔掉,这样读取的数据有变得完整了
                location_55aa = ser_data.find("55aa")
                second_read = oil_serial.read(int(location_55aa / 2))
                ser_oilPressure_data_str = ''
            else:
                ser_oilPressure_data_str = ser_data
            ser_receive_data = ser_oilPressure_data_str
            data = ser_receive_data
            #print("oil_pressure_recv_data:", data, "length:",len(data),"\n")
            if len(data) == data_length:
                # 如果数据长度跟正常一条数据的一样那么开始处理
                ser_receive_data = data
                data_mark = "_oilPressure_ch"
                data_length = len(ser_receive_data)  # 余数只有0123
                times = int(((data_length - 6) / 4))  # 取整
                number_mqtt = number_mqtt + 1         #用于计数上云的间隔数
                # 信息采集节点板的一条数据的times是10
                for i in range(times):
                    # 其中j是从1到10的
                    j = i + 1
                    if j == 9:  # 排出crc校验码数据,8是因为i从0开始算的
                        data_h = j * 4 + 2
                        data_l = data_h + 4
                    else:
                        if j == 10:
                            
                            data_channel_i = "Number"  # Number是区分txt文件中数据的先后顺序
                            data_h = j * 4 + 2
                            data_l = data_h + 4
                            data_16 = ser_receive_data[data_h:data_l]
                            data_10 = int(data_16, 16)
                            data_list.append(data_10)
                            channle_list.append(data_channel_i)
                        else:
                            # 这些位数的数据代表信息节点板上的不同通道的油压
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
                            oil_list[j-1] = data_true
                            if sensor_type == 23001:
                                # 每个完整的数据最先解析的是它
                                del sensors_type_list[:]
                                del value1s_list[:]
                                del value2s_list[:]
                            

                            sensors_type_list.append(sensor_type)
                            value1s_list.append(str(data_true)[:4])
                            value2s_list.append(0)
                            
                            if len(sensors_type_list) == 8 and len(value1s_list) == 8 and number_mqtt == 20: 
                                oil.put(oil_list)              #向油压队列放进数据
                                number_mqtt = 0
                                client = mqtt.Client('990') #客户段对象，client_id必须要给出，即（time.strftime）
                                client = mqtt_init(client)
                                nodeid = Information['nodeId']
                                result = proto_send().DValuesReq(nodeid, sensors_type_list, value1s_list,
                                                                 value2s_list)
                                client.publish("cn.gdaas.zbs/145/report", result, qos=0)  # 上云
                                print("send_oilpressure_data success")
                            
                            data_list.append(str(data_true)[:4])
                            channle_list.append(data_channel_i)
                    # 接受数据的时间，需要保存到本地文件当中

                data_dict_history = dict(zip(channle_list, data_list))
                dt = datetime1.now()
           
                date = dt.strftime('%Y.%m.%d')
                time1 = dt.strftime('%H:%M:%S')
             
                channle_list.append("date")
           
                data_list.append(date)
                channle_list.append("time")
                data_list.append(time1)
          
                data_dict_history = dict(zip(channle_list, data_list))
                
                if len(data_dict_history) == 11:

                    flow_file_name = 'oilPressure_data.txt'
                    
                    try:
                        count_oilPressure = count_oilPressure + 1
                       
                        ring_buffer_zone_oilPressure.append(data_dict_history)
                        #print('1235',count_oilPressure )
                        if count_oilPressure == 10:
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

        except:
            pass

       


def flow_data_recv_data(flow):
    
    data_length = 46  # 扭矩传感器的数据长度
    oil_serial = serial.Serial("/dev/ttysWK1", 115200)

    flow_list = [0.0]*4         #存储4位流量数据的列表

    data_dict_history = {}
    ring_buffer_zone = RingClass(100)
    number_mqtt = 0     #定义上传的数据间隔
    count_number = 0    #定义保存的数据间隔

    data_list = [] 
    channle_list = []
    sensors_type_list = []
    value1s_list = []
    value2s_list = []
    

    while True:
        try:
            ser_data_read = oil_serial.read(23)
            # ASCII码转成16进制
            ser_data = str((binascii.b2a_hex(ser_data_read)).decode())
            if ser_data[0:4] != "55aa":
                # 找到55aa的位置下标，下次读取这个下标的字符，扔掉,这样读取的数据有变得完整了
                location_55aa = ser_data.find("55aa")
                second_read = oil_serial.read(int(location_55aa / 2))
                ser_oilPressure_data_str = ''
            else:
                ser_oilPressure_data_str = ser_data
            ser_receive_data = ser_oilPressure_data_str
            data = ser_receive_data
            #print("oil_pressure_recv_data:", data, "length:",len(data),"\n")
            if len(data) == data_length:
                # 如果数据长度跟正常一条数据的一样那么开始处理
                ser_receive_data = data
                data_mark = "_oilPressure_ch"
                data_length = len(ser_receive_data)  # 余数只有0123
                times = int(((data_length - 6) / 4))  # 取整
                number_mqtt = number_mqtt + 1         #用于计数上云的间隔数
                
                for i in range(times):
                    # 其中j是从1到10的
                    j = i + 1
                    if j == 9:  # 排出crc校验码数据,8是因为i从0开始算的
                        data_h = j * 4 + 2
                        data_l = data_h + 4
                    else:
                        if j == 10:
                            
                            data_channel_i = "Number"  # 确定写在txt文件中的数据的顺序
                            data_h = j * 4 + 2
                            data_l = data_h + 4
                            data_16 = ser_receive_data[data_h:data_l]
                            data_10 = int(data_16, 16)
                            data_list.append(data_10)
                            channle_list.append(data_channel_i)
                        else:
                            # 这些位数的数据代表板子上不同通道的流量
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
                                # 每个完整的数据先传过来的是最先解析的是它
                                del sensors_type_list[:]
                                del value1s_list[:]
                                del value2s_list[:]

                                sensor_type = 24000 + j
                                data_true = 8.33 * data_10_ele - 33.33
                                sensors_type_list.append(sensor_type)
                                value1s_list.append(str(data_true)[:4])
                                value2s_list.append(0)
                                data_list.append(str(data_true)[:4])
                                channle_list.append(data_channel_i)
                                flow_list[j-1] = data_true
                           
                            if j == 2:
                                sensor_type = 24000 + j
                                data_true = 8.33 * data_10_ele - 33.33
                                sensors_type_list.append(sensor_type)
                                value1s_list.append(str(data_true)[:4])
                                value2s_list.append(0)
                                data_list.append(str(data_true)[:4])
                                channle_list.append(data_channel_i)
                                flow_list[j-1] = data_true

                            if j == 3:
                                sensor_type = 24000 + j
                                data_true = 8.33 * data_10_ele - 33.33
                                sensors_type_list.append(sensor_type)
                                value1s_list.append(str(data_true)[:4])
                                value2s_list.append(0)
                                data_list.append(str(data_true)[:4])
                                channle_list.append(data_channel_i)
                                flow_list[j-1] = data_true

                            if j == 4:
                                sensor_type = 24000 + j
                                data_true = 8.33 * data_10_ele - 33.33
                                sensors_type_list.append(sensor_type)
                                value1s_list.append(str(data_true)[:4])
                                value2s_list.append(0)
                                data_list.append(str(data_true)[:4])
                                channle_list.append(data_channel_i)
                                flow_list[j-1] = data_true
                            
                                
                            if len(sensors_type_list) == 4 and len(value1s_list) == 4 and number_mqtt == 20: 
                                flow.put(flow_list)          #队列通讯
                                number_mqtt = 0
                                client = mqtt.Client('990') #客户段对象，client_id必须要给出，即（time.strftime）
                                client = mqtt_init(client)
                                nodeid = Information['nodeId']
                                result = proto_send().DValuesReq(nodeid, sensors_type_list, value1s_list,
                                                                    value2s_list)
                                client.publish("cn.gdaas.zbs/145/report", result, qos=0)  # 上云
                                print("send_flow_data success")
                                del sensors_type_list[:]
                                del value1s_list[:]
                                del value2s_list[:]
                           
                    # 接受数据的时间，需要保存到本地文件当中
                # print('1235')
                data_dict_history = dict(zip(channle_list, data_list))
                dt = datetime1.now()
           
                date = dt.strftime('%Y.%m.%d')
                time1 = dt.strftime('%H:%M:%S')
             
                channle_list.append("date")
                data_list.append(date)
                channle_list.append("time")
                data_list.append(time1)
          
                data_dict_history = dict(zip(channle_list, data_list))
                
                if len(data_dict_history) == 7:

                    flow_file_name = 'flow_data.txt'
                    try:
                        count_number = count_number  + 1
                        # 可能要对这个data_list进行类型变换
                        ring_buffer_zone.append(data_dict_history)
                        if count_number == 10:
                            count_number = 0
                            fopen = open(flow_file_name, 'a')
                            print('start write flow data now--------------')
                            x_list = ring_buffer_zone.tolist()
                            for i in x_list:
                                fopen.write(str(i))
                                fopen.write("\r")
                            fopen.close()
                            
                    except:
                        pass

        except:
            pass




def rotate_data_recv_data(rotate):
    
    data_length = 46  # 扭矩传感器的数据长度
    oil_serial = serial.Serial("/dev/ttysWK2", 115200)

    rotate_list = [0.0] * 5

    data_dict_history = {}
    ring_buffer_zone = RingClass(100)
    number_mqtt = 0     #定义上传的数据间隔
    count_number = 0    #定义保存的数据间隔

    data_list = []  
    channle_list = []
    sensors_type_list = []
    value1s_list = []
    value2s_list = []
    

    while True:
        try:
            ser_data_read = oil_serial.read(23)
            # ASCII码转成16进制
            ser_data = str((binascii.b2a_hex(ser_data_read)).decode())
            if ser_data[0:4] != "55aa":
                # time.sleep(0.01),找到55aa的位置下标，下次读取这个下标的字符，扔掉,这样读取的数据有变得完整了
                location_55aa = ser_data.find("55aa")
                second_read = oil_serial.read(int(location_55aa / 2))
                ser_oilPressure_data_str = ''
            else:
                ser_oilPressure_data_str = ser_data
            ser_receive_data = ser_oilPressure_data_str
            data = ser_receive_data
            print("oil_pressure_recv_data:", data, "length:",len(data),"\n")
            if len(data) == data_length:
                # 如果数据长度跟正常一条数据的一样那么开始处理
                ser_receive_data = data
                data_mark = "_rotate_ch"
                data_length = len(ser_receive_data)  # 余数只有0123
                times = int(((data_length - 6) / 4))  # 取整
                number_mqtt = number_mqtt + 1         #用于计数上云的间隔数
                # 扭矩传感器的times是10
                for i in range(times):
                    # 其中j是从1到10的
                    j = i + 1
                    if j == 9:  # 排出crc校验码数据,8是因为i从0开始算的
                        data_h = j * 4 + 2
                        data_l = data_h + 4
                    else:
                        if j == 10:
                            
                            data_channel_i = "Number"  # 确定写在txt文件中的数据的顺序
                            data_h = j * 4 + 2
                            data_l = data_h + 4
                            data_16 = ser_receive_data[data_h:data_l]
                            data_10 = int(data_16, 16)
                            data_list.append(data_10)
                            channle_list.append(data_channel_i)
                        else:
                            # 这些位数的数据代表不同通道的扭矩，
                            data_channel_i = str(j) + data_mark  # 通道命名
                            data_h = j * 4 + 2
                            data_l = data_h + 4
                            data_16 = ser_receive_data[data_h:data_l]
                            data_10 = int(data_16, 16)  # 返回来处理过的数据
                            
                            if j == 1:
                                # 每个完整的数据先传过来的是最先解析的是它
                                del sensors_type_list[:]
                                del value1s_list[:]
                                del value2s_list[:]

                                sensor_type = 26000 + j
                                data_true = (data_10 / 4) * 60  #根切，单位是转/分钟
                                sensors_type_list.append(sensor_type)
                                value1s_list.append(str(data_true)[:4])
                                value2s_list.append(0)
                                data_list.append(str(data_true)[:4])
                                channle_list.append(data_channel_i)
                                rotate_list[j-1] = data_true
                           
                            if j == 2:
                                sensor_type = 26000 + j
                                data_true = (data_10 / 12) * 60  #切断，单位是转/分钟
                                sensors_type_list.append(sensor_type)
                                value1s_list.append(str(data_true)[:4])
                                value2s_list.append(0)
                                data_list.append(str(data_true)[:4])
                                channle_list.append(data_channel_i)
                                rotate_list[j-1] = data_true

                            if j == 3:
                                sensor_type = 26000 + j
                                data_true =  (data_10 / 12) * 60  #一级输送，单位是转/分钟
                                sensors_type_list.append(sensor_type)
                                value1s_list.append(str(data_true)[:4])
                                value2s_list.append(0)
                                data_list.append(str(data_true)[:4])
                                channle_list.append(data_channel_i)
                                rotate_list[j-1] = data_true
                            if j == 4:
                                sensor_type = 26000 + j
                                data_true =  (data_10 / 3) * 60  #风机，单位是转/分钟
                                sensors_type_list.append(sensor_type)
                                value1s_list.append(str(data_true)[:4])
                                value2s_list.append(0)
                                data_list.append(str(data_true)[:4])
                                channle_list.append(data_channel_i)
                                rotate_list[j-1] = data_true
                            if j == 5:
                                sensor_type = 26000 + j
                                data_true =  (data_10 / 21) * 0.36 * 4.4 * 60 #车速。m/min
                                sensors_type_list.append(sensor_type)
                                value1s_list.append(str(data_true)[:4])
                                value2s_list.append(0)
                                data_list.append(str(data_true)[:4])
                                channle_list.append(data_channel_i)
                                rotate_list[j-1] = data_true
                            
                                
                            if len(sensors_type_list) == 5 and len(value1s_list) == 5 and number_mqtt == 1: 

                                rotate.put(rotate_list)

                                number_mqtt = 0
                                client = mqtt.Client('990') #客户段对象，client_id必须要给出，即（time.strftime）
                                client = mqtt_init(client)
                                nodeid = Information['nodeId']
                                result = proto_send().DValuesReq(nodeid, sensors_type_list, value1s_list,
                                                                    value2s_list)
                                client.publish("cn.gdaas.zbs/145/report", result, qos=0)  # 上云
                                print("send_rotate_data success")
                                del sensors_type_list[:]
                                del value1s_list[:]
                                del value2s_list[:]
                           
                    # 接受数据的时间，需要保存到本地文件当中
                #print('1235')
                data_dict_history = dict(zip(channle_list, data_list))
                dt = datetime1.now()
           
                date = dt.strftime('%Y.%m.%d')
                time1 = dt.strftime('%H:%M:%S')
             
                channle_list.append("date")
                data_list.append(date)
                channle_list.append("time")
                data_list.append(time1)
          
                data_dict_history = dict(zip(channle_list, data_list))
                
                if len(data_dict_history) == 8:

                    flow_file_name = 'rotate_data.txt'
                    try:
                        count_number = count_number  + 1
                        # 可能要对这个data_list进行类型变换
                        ring_buffer_zone.append(data_dict_history)
                        if count_number == 10:
                            count_number = 0
                            fopen = open(flow_file_name, 'a')
                            print('start write rotate data now--------------')
                            x_list = ring_buffer_zone.tolist()
                            for i in x_list:
                                fopen.write(str(i))
                                fopen.write("\r")
                            fopen.close()
                            
                    except:
                        pass

        except:
            pass



# angle_lon_lat_recv_data是单独接入USB口的姿态角度和经纬度
def angle_lon_lat_recv_data(angle, lon):
    FrameState = 0  
    Bytenum = 0
    CheckSum = 0
    AngleData = [0.0] * 8
    lon_latData = [0.0] * 8 

    angle_queue = [0.0] * 3
    lon_queue = [0.0] * 2

    number_mqtt_a = 0      #用于计算角度上云的间隔数
    number_mqtt_b = 0      #用于计算经纬度上云的间隔数

    count_angle = 0  #用于计算写入角度文件的间隔数
    count_lon = 0    #用于计算写入经纬度文件的间隔数
    
    Number_Angle = 0     #用于表示角度数据的先后顺序
    Number_Lon = 0       #用于表示经纬度数据的先后顺序

    ring_buffer_zone_angle = RingClass(100)
    ring_buffer_zone_lon = RingClass(100)
    
    sensors_angle_list = []
    value1s_angle_list = []
    value2s_angle_list = []

    sensors_lon_list = []
    value1s_lon_list = []
    value2s_lon_list = []
    
   
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
                            angle_queue[0] = angle_x
                            angle_queue[1] = angle_y
                            angle_queue[2] = angle_z
                            if angle_x >= k_angle:
                                angle_x -= 2 * k_angle
                            if angle_y >= k_angle:
                                angle_y -= 2 * k_angle
                            if angle_z >= k_angle:
                                angle_z -= 2 * k_angle

                            # print('angle_data', angle_x)
                            number_mqtt_a = number_mqtt_a + 1
                            del sensors_angle_list[:]
                            del value1s_angle_list[:]
                            del value2s_angle_list[:]
                            value1s_angle_list = [angle_x, angle_y, angle_z]
                            nodeid = Information['nodeId']
                            
                            sensors_angle_list = [str(27001), str(27002), str(27003)]
                            value2s_angle_list = [0, 0, 0]
                            if number_mqtt_a == 10:
                                angle.put(angle_queue)
                                number_mqtt_a = 0
                                # print('1234')
                                client = mqtt.Client('999')  # 客户段对象，client_id必须要给出，即（time.strftime）
                                client = mqtt_init(client)
                                result = proto_send().DValuesReq(nodeid, sensors_angle_list, value1s_angle_list,
                                                                value2s_angle_list)
                                
                                client.publish("cn.gdaas.zbs/145/report", result, qos=0)  # 上云
                                print('send angle_data success')


                            angle_data = list(map(str, value1s_angle_list))
                            angle_list = ['jiaodu_x', 'jiaodu_y', 'jiaodu_z', 'date', 'time', 'Number']
                     
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
                            if count_angle == 10:
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
                            lon_queue[0] = Lon
                            lon_queue[1] = Lat
                            # print("lon_lat", Lon)
                            number_mqtt_b = number_mqtt_b + 1

                            del sensors_lon_list[:]
                            del value1s_lon_list[:]
                            del value2s_lon_list[:]
                            # print('1237')
                            value1s_lon_list = [Lon, Lat]
                            nodeid = Information['nodeId']
                            sensors_lon_list = [str(15001)]
                            value2s_lon_list = [0, 0]
                            # print('1237')
                            if number_mqtt_b == 10:
                                lon.put(lon_queue)
                                number_mqtt_b = 0
                                client = mqtt.Client('999')  # 客户段对象，client_id必须要给出，即（time.strftime）
                                client = mqtt_init(client)
                                result_a = proto_send().DValuesReq(nodeid, sensors_lon_list, value1s_lon_list,
                                                                    value2s_lon_list)
                                client.publish("cn.gdaas.zbs/145/report", result_a, qos=0)  # 上云
                                print('send lon_data success')
                            lon_data = list(map(str, value1s_lon_list))
                            lon_list = ['lon', 'lat', 'date', 'time', 'Number']
                            
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
                            if count_lon == 10:
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

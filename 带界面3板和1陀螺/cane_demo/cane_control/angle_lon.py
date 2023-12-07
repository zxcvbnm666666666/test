from ringclass import RingClass
import serial

from datetime import datetime


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
lon_lat = [0,0] * 2
Number_Angle = 0
Number_Lon = 0
ring_buffer_zone_angle = RingClass(100)
ring_buffer_zone_lon = RingClass(100)

def angle_lon_lat_recv_data():
    client = mqtt.Client(time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))) #客户段对象，client_id必须要给出，即（time.strftime）
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
    angle_lon_lat_serial = serial.Serial('/dev/ttyUSB0', 115200)#对应板子上的串口号
    while True:
        try:
            datahex = angle_lon_lat_serial.read(33)  # 读取33个字节
            #print('datahex', datahex.hex())
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

                            print('angle_data', angle_x)

                            value1s_angle_list = [angle_x, angle_y, angle_z]
                            '''nodeid = "0001" #Information['nodeId']
                            sensors_angle_list = [str(27001), str(27002), str(27003)]
                            value2s_angle_list = [0, 0, 0]
                            #result = proto_send().DValuesReq(node_id, sensors_angle_list, value1s_angle_list,value2s_angle_list)
                            #client.publish("cn.gdaas.zbs/145/report", result, qos=1)  # 上云'''

                            angle_data = list(map(str, value1s_angle_list))
                            angle_list = ['jiaodu_x','jiaodu_y','jiaodu_z','date','time','Number']
                            dt = datetime.now()
                            date = dt.strftime('%Y.%m.%d ')
                            time = dt.strftime('%H:%M:%S.%f')
                            angle_data.append(date)
                            angle_data.append(time)
                            Number_Angle = Number_Angle + 1
                            if Number_Angle == 6000:
                                Number_Angle = 0
                            angle_data.append(Number_Angle)
                            data_dict_history = dict(zip(angle_list, angle_data))
                            #print("数据",data_dict_history['jiaodu_x'])
                            #print("数据1",data_dict_history['date'])
                            flow_file_name = 'Angle_data.txt'
                            count_angle = count_angle + 1
                            #print('123',count_angle)
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
                            print("lon_lat", Lon)

                            value1s_lon_list = [Lon, Lat]
                            '''nodeid = "0001" #Information['nodeId']
                            sensors_angle_list = [str(27001), str(27002), str(27003)]
                            value2s_angle_list = [0, 0, 0]
                            #result = proto_send().DValuesReq(node_id, sensors_angle_list, value1s_angle_list,value2s_angle_list)
                            #client.publish("cn.gdaas.zbs/145/report", result, qos=1)  # 上云'''

                            lon_data = list(map(str, value1s_lon_list))
                            lon_list = ['lon', 'lat', 'date', 'time', 'Number']
                            dt = datetime.now()
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




if __name__ == '__main__':
    angle_lon_lat_recv_data()
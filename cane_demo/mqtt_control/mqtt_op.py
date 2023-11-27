

import os
import datetime
import binascii
import threading
import time
#from paho.mqtt import client as mqtt



basedir = os.path.abspath(os.path.dirname(__file__))


Information = {
    'MQTTHOST': "mqtt.gdaas-tea.com",
    'mqtt_publish': "cn.gdaas.zbs/145/report",
    'mqtt_subscribe': "cn.gdaas.zbs/145/cmd",
    'nodeId': '339',
    'address': '1',
    'channel': '1',

    'reboot_time': 5,
    'collect_time': 300,
    'start_time': 21,
    'end_time': 23,


    'serial_name': "/dev/ttyS1",
    #'serial_name': "COM1",
    #'serial_buad': 9600
    'serial_buad': 115200

}


#client = mqtt.Client(time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))) #客户段对象，client_id必须要给出，即（time.strftime）


def on_connect(client, userdata, flags, rc):
    try:
        rc_status = ["连接成功", "协议版本错误", "无效的客户端标识", "服务器无法使用", "用户名或密码错误", "无授权"]
        #client.subscribe(Information['mqtt_subscribe'])
        print("connect：", rc_status[rc])

    except Exception as erro:
        print('-'*50, 'mqtt.on_connect出现异常', '-'*50)
        print(erro)
        pass


def on_message(client, userdata, msg):
    try:
        print('-'*50, 'mqtt接收到信息', '-'*50)

    except:
        pass


def mqtt_init(mqttClient):
    mqttClient.on_connect = on_connect
    mqttClient.on_message = on_message
    MQTTHOST = Information['MQTTHOST']
    MQTTPORT = 1883
    mqttClient.username_pw_set("", "")
    mqttClient.connect(MQTTHOST, MQTTPORT, 60)
    mqttClient.loop_start()

    return mqttClient





'''if __name__ == '__main__':
    client = mqtt_init(client)
    client.subscribe("cn.gdaas.zbs/145/cmd")
    while True:
        pass
'''
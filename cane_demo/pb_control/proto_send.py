import pb_control.monitor_pb2 as proto
import time
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

class proto_send():
	
		
	def SensorValue(self, nodeid, sensors, value1, value2):
		format = proto.GateSend()
		sid = 1
		format.sid = int(SensorValueId)
		payload = format.sensorValue
		node = payload.node
		sensor_list = []
		sensor1 = 8
		for index in sensor1:
			sensor_list.append(int(index.sensor_type))  
		node.nodeId = nodeid
		node.sensors.extend(sensor_list)
		node.address =address  
		node.channel =channel
		payload.sensor = int(sensor)     
		payload.value1 = float(value1)      
		payload.value2 = float(value2)      
		return format.SerializeToString()
	def DValuesReq(self, nodeid, sensors, value1s, value2s):
		format = proto.GateSend()
		
		#print('999',len(sensors))
		for a in range(0,len(sensors)):
			
			#print('999',len(sensors))
			payload = format.values
			data = payload.values.add()
			node = data.node	
			sensor_list = []
			#print('999',a)
			#sensor_list.append(1)
			#print('999',nodeid)
			node.nodeId = int(nodeid)
			#print('345')
			node.sensors.extend(sensor_list)
			node.address =  int(Information['address'])
			node.channel = int(Information['channel'])
			data.sensor = int(sensors[a])
			data.value1 = float(value1s[a])
			data.value2 = float(value2s[a])
			
			#print('999',a)
		return format.SerializeToString()


	def DQueryNodeReq(self):
		pass
		
	
	def DNodeManageRsp(self):
		pass
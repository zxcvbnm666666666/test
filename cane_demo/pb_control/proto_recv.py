import time,binascii
from app.CloudConn.analysis_C import analysis_C
import app.CloudConn.monitor_pb2 as proto
from app import db
from app.models.node import Node,Query
from app.models.command import Command
from app.models.serial_command import Serial
from app.models.base_info import Base_info
from app.models.sensor import Sensor
from app.models.control_status import Control_Status
from app.control.count_thread import cotrol_command


class proto_recv():
	def SRegisterRsp(self,format):
		RegisterId = int(Base_info.query.filter_by(name = 'RegisterId').first().content)
		if format.sid == RegisterId:
			Topic_Subscribe = format.register.scribeTopic
			Topic_Publish = format.register.sendTopic
			Longitude  = format.register.gate.longitude
			Latitude  = format.register.gate.latitude
			Gate_ID  = format.register.gate.gateId
			Gate_Address  = format.register.gate.address
			Gate_Channel  = format.register.gate.channel
			NetType  = format.register.gate.netType
			print("NetType1111111111111111:",NetType)
			if NetType == 1:
				NetType = "ZigBee"
			else:
				NetType = "LoRa"
			return [Topic_Subscribe,Topic_Publish,Longitude,Latitude,\
				Gate_ID,Gate_Address,Gate_Channel,NetType]
		else:
			return False
		
	def SCycleSetReq(self,format):
		print("node info begin updating")
		a = Base_info.query.filter_by(name = 'CycleSetId').first()
		a.content = str(format.sid)
		db.session.add(a)
		db.session.commit()

		cycleset = format.cycleSet
		print(cycleset)
		if True:
			for cycle in cycleset.cycle:
				ID = hex(cycle.node.address)[2:].zfill(4)
				Channel = str(cycle.node.channel).zfill(2)
				Node_info = ID + Channel
				latitude = cycle.node.latitude 
				longitude = cycle.node.longitude
				nodeId = cycle.node.nodeId
				sensors = (str(cycle.node.sensors))[1:-1]
				sensors = sensors.replace(" ","") 
				try:
					x = str(cycle.node.x)
					y = str(cycle.node.y)
				except:
					x = y = "0"
				node = Node(ID,Channel,sensors,latitude,longitude,xRcs=x,yRcs=y,nodeId=nodeId)
				db.session.add(node)
				db.session.commit()
				for index in cycle.cycle:
					if True:
						sensor = index.sensor
						sensor_type = str(int(sensor/1000)).zfill(2)
						if sensor_type in ("08","09","16"):
							pass
						else:
							sensor_account = str(sensor)[-2:]
							#print("sensor_type:",sensor_type,sensor_account)
							CMD = analysis_C().Build_CMD(Node_info,sensor_type,sensor_account,index.interval)	
							cmd = Command(ID,sensor,CMD,index.interval)
							db.session.add(cmd)
							db.session.commit()
							query = Query(sensor,index.interval,ID)
							db.session.add(query)
							db.session.commit()
							cmd = Serial(ID,sensor,CMD,'1')
							db.session.add(cmd)
							db.session.commit()
			print("finish node info update") 
			return True
			
	def SSensorCtrlReq(self,format):
		print("format",format)
		SensorCtrlId = str(format.sid)
		node_info = format.sensorCtrl.node
		node_address = (str(node_info.address)).zfill(4)
		node_channel = (str(node_info.channel)).zfill(2)
		Node = node_address + node_channel
		sensor_type = str(format.sensorCtrl.sensor)
		sensor_function = (str(int((int(sensor_type))/1000))).zfill(2)
		sensor_account = sensor_type[-2:]
		cmd = (str(format.sensorCtrl.cmd)).zfill(2)
		try:
			params = format.sensorCtrl.params
			message_hex2asc =str(binascii.b2a_hex(params))
		except:	
			pass
		a = Base_info.query.filter_by(name = 'SensorCtrlId').first()
		a.content = SensorCtrlId
		db.session.add(a)
		db.session.commit()

		if cmd == "02":  #打开设备
			cmd = "ff"
			cmd =  analysis_C().Build_CMD_ctrl(Node,sensor_function,sensor_account,cmd)
			cotrol_command(cmd)
				
		elif cmd == "03":  #关闭设备
			cmd = "00"  #开空指令
			cmd =  analysis_C().Build_CMD_ctrl(Node,sensor_function,sensor_account,cmd)
			cotrol_command(cmd)
		elif cmd == "04" :#设置  和开空调
			cmd = "ff"
			cmd =  analysis_C().Build_CMD_ctrl(Node,sensor_function,sensor_account,cmd)
			cotrol_command(cmd)
			if len(params) == 2 :#自动
				if message_hex2asc[-3:-1] in ["00","01","02","03","04"]:  #几种模式
					sensor_account = "02"
					cmd = message_hex2asc[-3:-1] #模式设置指令
					cmd =  analysis_C().Build_CMD_ctrl(Node,sensor_function,sensor_account,cmd)
					cotrol_command(cmd)
				else:
					sensor_account = "03"
					cmd = message_hex2asc[-3:-1] #温度设置指令
					cmd =  analysis_C().Build_CMD_ctrl(Node,sensor_function,sensor_account,cmd)
					cotrol_command(cmd)
			elif len(params) == 4 :#非自动
				sensor_account = "02" #模式设置指令
				cmd = message_hex2asc[-7:-5] #模式设置指令
				cmd =  analysis_C().Build_CMD_ctrl(Node,sensor_function,sensor_account,cmd)
				cotrol_command(cmd)
				sensor_account = "03" #模式设置指令
				cmd = message_hex2asc[-3:-1] #温度设置指令
				cmd =  analysis_C().Build_CMD_ctrl(Node,sensor_function,sensor_account,cmd)
				cotrol_command(cmd)
			
	def Heartbeat(self,format):
		HeartbeatID = Base_info.query.filter_by(name='HeartbeatId').first().content
		if format.sid == int(HeartbeatID):
			print("Heartbeat_Rsp")
		return False
	
	def SQueryNodeRsp(self,format):
		print("ServerRsp")
		
	def SQueryCycleRsp(self,format):
		pass
		
	def SNodeManageReq(self,format):
		pass
		
	def SSensorValueRsp(self,format):
		pass


Dict_Rsp = {\
	"register" : proto_recv().SRegisterRsp,\
	"cycleSet" : proto_recv().SCycleSetReq,\
	"sensorCtrl" : proto_recv().SSensorCtrlReq,\
	"heartbeat" : proto_recv().Heartbeat,\
	"sensorValue" : proto_recv().SSensorValueRsp,\
	"queryCycle" : proto_recv().SQueryCycleRsp\
}

class control_state:
	def Debug(self,format):
		try:
			if format[2:6] == 'cccc': #通讯猫单独发送指令
				cmd = format[6:-1]    #完整的指令  
				sensor_type = cmd[-10:-8] #电磁阀个数  空调开关/模式/温度标志
				sensor_status = cmd[-8:-6]
				#AC_model = AC_temperature = device_status = cmd[-6:-4] 
				if sensor_type == "16": #继电器
					cotrol_command(cmd)
				elif sensor_type == "09": #电磁阀
					cotrol_command(cmd)
				elif sensor_type == "08": #空调
					if sensor_status == "01":  #开关状态
						cotrol_command(cmd)
					elif sensor_status == "02": #模式选择
						self.AC_set(sensor_status,cmd)						
					elif sensor_status == "03": #温度设置
						self.AC_set(sensor_status,cmd)
				else:
					cotrol_command(cmd)  #其他指令直接发送到待发区域
		except:
			return False
	def debug_open_device(self,cmd):
		open_device = cmd
		open_device_cmd=open_device.replace(open_device[-8:-4],"01ff")
		cotrol_command(open_device_cmd)#开指令，传递到串口
	def AC_set(self,sensor_status,cmd):
		try:
			device_status = Control_Status.query.filter_by(node_id=node_id,sensor_type=sensor_type,sensor_status=sensor_status).first().device_status
			if device_status == "00":#当为关的时候,先打开设备
				self.debug_open_device(cmd)
		except:
			self.debug_open_device(cmd)
		finally:
			cotrol_command(cmd)	
    	

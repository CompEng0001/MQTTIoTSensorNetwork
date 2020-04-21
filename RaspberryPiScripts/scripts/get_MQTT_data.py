#! /usr/bin/python

import paho.mqtt.client as mqtt
import time 
from time import sleep
from datetime import datetime
import os

#Dont forget to change the variables for the MQTT Broker

mqtt_username = "mayfield" 
mqtt_password = "IoTNetwork"

topics = ["livingroom/","kitchen/","garage/","outside/"]
topicIndex = 0

humidity_topic = "humidity"
temperature_topic = "temperature"
pressure_topic = "pressure"
iaq_topic = "iaq"
mq5_topic = "mq5"
dust_topic = "dust"

mqtt_broker_ip = "192.168.1.171"
mqtt_port = 1883
Keep_Alive_Interval = 60


removelines = "python IoTHub/scripts/remove_lines.py"
sensorData = " "

def on_connect(self, mosq, obj, rc):
	for x in range(4):
		self.subscribe(str(topics[x]) + temperature_topic, 0)
		self.subscribe(str(topics[x]) + humidity_topic, 0)
		self.subscribe(str(topics[x]) + pressure_topic, 0)
		self.subscribe(str(topics[x]) + iaq_topic, 0)
		self.subscribe(str(topics[x]) + mq5_topic, 0)
		self.subscribe(str(topics[x]) + dust_topic, 0)
	
	# rc is the error code returned when connecting to the broker
	print("Connected! " + str(rc))
	
def on_message(mosq, obj, msg):
	global sensorData
	topic = str(msg.topic)
	message = float(msg.payload)
	sensorData += str(message) + ","
	#print(str(topic) + ': ' +str(message))
	 	
def on_subscribe(mosq, obj, mid, granted_qos):
	pas	

def log_sensorData_to_file():
	global topics
	global topicIndex
	global sensorData
	missingData = "NaN,NaN,NaN,NaN,NaN,NaN"
	os.system(removelines + " %s" %topicIndex)
	now = time.strftime("%c")
	file = open("IoTHub/" +topics[topicIndex] + "data_log.csv","a")
    	if len(sensorData) > 20:
		file.write(str(now)+","+ "%s \n" % (sensorData))
	else:
		file.write(str(now)+"," +"%s \n" % (missingData))
        file.flush
        time.sleep(5)
        file.close()

mqttc = mqtt.Client()
mqttc.username_pw_set(mqtt_username,mqtt_password)

mqttc.on_connect = on_connect
mqttc.on_message = on_message

mqttc.connect(mqtt_broker_ip, int(mqtt_port), int(Keep_Alive_Interval))
time.sleep(3)
#now = time.strftime("%c")
 
while 1:
	for x in range(4):
		mqttc.loop_start()
		now = time.strftime("%c")
		print("Current time %s" % now )
		topicIndex = x
		mqttc.publish(str(topics[topicIndex])," ")
		print("Requesting data from: %s" % topics[topicIndex])
		time.sleep(20)
		log_sensorData_to_file()
		print("sensor data: %s" % (sensorData))
		sensorData = " "
		mqttc.loop_stop()
mqttc.disconnect()

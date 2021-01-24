#!/usr/bin/python

MQTT_ServerIP     = "192.168.5.248"
MQTT_ServerPort   = 1883
serialPortDevice  = '/dev/ttyACM0'
serialPortBaudrate = 57600
LOG_FILENAME      = "/home/pi/log/lora_mqtt.log"
#LOG_FILENAME      = "lora_mqtt.log"

MQTT_TOPIC_OUT       = 'huis/LoRa/+/out'

#Omgeschakeld in script:
MQTT_TOPIC_HOMELOGIC_BEDIENING     = 'huis/HomeAssistant/Actief-WildDetector-PIR/bediening'

MQTT_TOPIC_CHECK     = "huis/LoRa/RPiHome/check"
MQTT_TOPIC_REPORT    = "huis/LoRa/RPiHome/report"

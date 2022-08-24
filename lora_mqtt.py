#!/usr/bin/python3

import os
import signal
import time
import serial
import _thread
import traceback
import json
import struct

from queue import Queue
import paho.mqtt.publish as mqtt_publish
import paho.mqtt.client as mqtt_client

# external files/classes
import logger
import serviceReport
import settings

DEVICE_DISCONNECTED = -127

wildDetectorActive = True
chickenDoorOpenActive = False

sendQueue = Queue(maxsize=0)

exitThread = False
serialPort = None
LoRaRSSI = False


def current_sec_time():
    return int(round(time.time()))


def signal_handler(_signal, frame):
    global exitThread

    print('You pressed Ctrl+C!')
    exitThread = True


def on_message(_client, userdata, msgJson):
    print(('ERROR: Received ' + msgJson.topic + ' in on_message function' + str(msgJson.payload)))


def on_message_out(_client, userdata, msg):
    # print(("on_message_out:" + msg.topic + " " + str(msg.payload)))
    # print(msg.topic + " " + str(msg.payload))
    topics = msg.topic.split("/")

    deviceName = topics[2]  # huis/LoRa/FS20ST-1/out
    cmnd = deviceName.split("-")    # KaKu-12

    if cmnd[0] == "FS20ST":
        pass
    #     #print("Activate FS20ST WCD: %s" % cmnd[1])
    #     setFS20ST(int(cmnd[1]), msg.payload)


def on_message_homelogic_bediening(_client, userdata, msg):
    global chickenDoorOpenActive
    global wildDetectorActive

    # print("on_message_homelogic_bediening:" + msg.topic + " " + str(msg.payload))
    # print(msg.topic + " " + str(msg.payload))

    topc = msg.topic.split('/')
    deviceName = topc[2]
    # print(deviceName)
    if deviceName == 'Actief-Kippenhok-Deurtje':
        if int(msg.payload) == 1:
            chickenDoorOpenActive = True
        else:
            chickenDoorOpenActive = False
        print("on_message_homelogic_bediening: Actief Kippenhok Deurtje=%d" % chickenDoorOpenActive)

    elif deviceName == 'Actief-WildDetector-PIR':
        if int(msg.payload) == 1:
            wildDetectorActive = True
        else:
            wildDetectorActive = False
        print("on_message_homelogic_bediening: Actief WildDetector=%d" % wildDetectorActive)


def openSerialPort():
    global exitThread
    try:
        ser = serial.Serial(port=settings.serialPortDevice,
                            baudrate=settings.serialPortBaudrate,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS,
                            timeout=1)  # 1=1sec 0=non-blocking None=Blocked

        if ser.isOpen():
            print(("lora_mqtt: Successfully connected to serial port %s" % settings.serialPortDevice))

        return ser

    # Handle other exceptions and print the error
    except Exception as arg:
        print("%s" % str(arg))
        # traceback.print_exc()

        # Report failure to Home Logic system check
        serviceReport.sendFailureToHomeLogic(serviceReport.ACTION_NOTHING, 'Serial port open failure on port %s, wrong port or USB cable missing' % settings.serialPortDevice)

        # Suppress restart loops
        time.sleep(900)  # 15 min
        exitThread = True


def closeSerialPort(ser):
    ser.close()


def initLoRa(ser):
    ser.write("1n".encode())  # NodeId=1
    if LoRaRSSI:
        ser.write("1r".encode())  # RSSI=1
    else:
        ser.write("0r".encode())  # RSSI=0


def serialPortThread():
    global serialPort
    global exitThread

    # signalStrength = 0

    serialPort = openSerialPort()

    while not exitThread:
        try:
            if serialPort.isOpen():
                serInLine = serialPort.readline().decode()
            else:
                serInLine = ""

            if serInLine != "":
                serInLine = serInLine.rstrip("\r\n")
                # print('LoRa: %s' % (serInLine))
                msg = serInLine.split(' ')
#                print(msg)
                # LoRa is started: Init it
                if msg[0] == "[LORA_GATEWAY]":
                    initLoRa(serialPort)
                    print("LoRa: [LORA_GATEWAY]: Adapter Initialized!...")

                # Only handle messages starting with 'OK' from LoRa
                if msg[0] == 'OK':
                    # Reset the Rx timeout timer
                    serviceReport.systemWatchTimer = current_sec_time()
                    # print("Update systemWatchTimer")
                    # print 'OK found!'
                    del msg[0]  # remove 'OK' from list

                    nodeId = int(msg[0]) & 0x1F  # NodeId 0..31
                    del msg[0]  # remove NodeId from list

                    # if available, gt the rssi signal strength
                    if LoRaRSSI:
                        # signalStrength = msg[-1]
                        del msg[-1]     # signalLevel dB
                        del msg[-1]     # "RSSI"
                    #     print("NodeId: %d Msg: %s (RSSI: %s)" % (nodeId, msg, signalStrength))
                    # else:
                    #     print("NodeId: %d Msg: %s" % (nodeId, msg))

                    # nodeId 2: Henhouse door open/close
                    if nodeId == 2:
                        msgId = int(msg[0])

                        # # MSG_ID_NODE_STARTUP=1: Node startup notification
                        # if msgId == 1:
                        #     # This sensor is still available
                        #     mqtt_publish.single("huis/LoRa/PIRB-Wild-Detector/pirb", 0, qos=1, hostname=settings.MQTT_ServerIP)
                        # # MSG_ID_STILL_ALIVE=2: Node still alive
                        # elif msgId == 2:
                        #     # This sensor is still available
                        #     mqtt_publish.single("huis/LoRa/PIRB-Wild-Detector/pirb", 0, qos=1, hostname=settings.MQTT_ServerIP)
                        # MSG_ID_CMND_REQUEST=3: Node wakeup/cmnd request
                        if msgId == 3:
                            if chickenDoorOpenActive:
                                # Set WildDetector PIR: off
                                putMsg = "3,1,%ds" % nodeId
                                print("LoRa: Open chicken door")
                            else:
                                # Set WildDetector PIR: off
                                putMsg = "3,0,%ds" % nodeId
                                print("LoRa: Close chicken door")
                            sendQueue.put(putMsg.encode())
                        else:
                            print("LoRa: Received unknown msgId:%d from NodeId %d" % (msgId, nodeId))

                    # # nodeId 3: Wild detector
                    # elif nodeId == 3:
                    #     msgId = int(msg[0])

                    #     # MSG_ID_NODE_STARTUP=1: Node startup notification
                    #     if msgId == 1:
                    #         # This sensor is still available
                    #         mqtt_publish.single("huis/LoRa/PIRB-Wild-Detector/pirb", 0, qos=1, hostname=settings.MQTT_ServerIP)
                    #     # MSG_ID_STILL_ALIVE=2: Node still alive
                    #     elif msgId == 2:
                    #         # This sensor is still available
                    #         mqtt_publish.single("huis/LoRa/PIRB-Wild-Detector/pirb", 0, qos=1, hostname=settings.MQTT_ServerIP)
                    #     # MSG_ID_CMND_REQUEST=3: Node wakeup/cmnd request
                    #     elif msgId == 3:
                    #         if wildDetectorActive:
                    #             # Set WildDetector PIR: off
                    #             sendQueue.put("3,1,%ds".encode() % nodeId)
                    #             print("LoRa: Activate WildDetector PIR")
                    #         else:
                    #             # Set WildDetector PIR: off
                    #             sendQueue.put("3,0,%ds".encode() % nodeId)
                    #             print("LoRa: De-activate WildDetector PIR")
                    #     # MSG_ID_PIR_MOVEMENT=4: PIR detected movement
                    #     elif msgId == 4:
                    #         mqtt_publish.single("huis/LoRa/PIRB-Wild-Detector/pirb", 1, qos=1, hostname=settings.MQTT_ServerIP)
                    #     else:
                    #         print("LoRa: Received unknown msgId:%d from NodeId %d" % (msgId, nodeId))

                    # nodeId 4: Weerstation
                    elif nodeId == 4:
                        # byte nodeId;
                        # byte msgId;
                        # byte voltageVcc;				getVcc 1.0V=0, 1.8V=40, 3,0V=100, 3.3V=115, 5.0V=200, 6.0V=250
                        # int temperature;
                        # int humidity;
                        # int pressure;
                        # byte windDirection;
                        # unsigned int windPulses;		Total nr of wind pulses in 5 min: 39294 pulses by 160km/h
                        # unsigned int windGust; 		Max nr of measured pulses in an 8 sec frame
                        # unsigned int rainPulses; 		Total nr of rain pulses in 5 min

                        msgId = int(msg[0])
                        del msg[0]  # remove msgId from list

                        # MSG_ID_NODE_STARTUP=1: Node startup notification
                        if msgId == 1:
                            print("Received Weather station node startup notification")
                        # MSG_ID_TEMPERATURE=4: Temperature measurement
                        elif msgId == 4:
                            sensorData = {}

                            # Vcc: msg[0] -> 1.0V=0, 1.8V=40, 3,0V=100, 3.3V=115, 5.0V=200, 6.0V=250
                            voltageVcc = float(((int(msg[0]) * 2) + 100)) / 100
                            sensorData['Vcc'] = round(voltageVcc, 2)

                            # Temp (int)
                            byteVal = bytearray([int(msg[2]), int(msg[1])])
                            val = struct.unpack(">h", byteVal)[0]
                            sensorData['Temperature'] = round(float(val) / 100, 1)
                            print("Temp Buiten %1.1f C" % (val / 100))

                            # Humidity (int)
                            byteVal = bytearray([int(msg[4]), int(msg[3])])
                            val = struct.unpack(">h", byteVal)[0]
                            sensorData['Humidity'] = round(float(val) / 100, 1)

                            mqtt_publish.single("huis/LoRa/Weerstation/temp", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)

                            # Pressure (int)
                            byteVal = bytearray([int(msg[6]), int(msg[5])])
                            val = struct.unpack(">h", byteVal)[0]
                            sensorData['Pressure'] = round(float(val) / 10, 1)

                            altituteInMeter = 500  # 500 m
                            pressureSealevel = val / pow((1.0 - float(altituteInMeter) / float(44330)), 5.255)
                            sensorData['PressureSealevel'] = round(pressureSealevel / 10, 1)
                        
                            # 8 bits=360 degrees
                            windContactToDegrees = {
                                0x01: 0,        # 0000 0001 - 0x01 -   1 - 0°    (North)
                                0x03: 22.5,     # 0000 0011 - 0x03 -   3 - 22,5°
                                0x02: 45,       # 0000 0010 - 0x02 -   2 - 45°   (North-East)
                                0x06: 67.5,     # 0000 0110 - 0x06 -   6 - 67,5°
                                0x04: 90,       # 0000 0100 - 0x04 -   4 - 90°   (East)
                                0x0C: 112.5,    # 0000 1100 - 0x0C -  12 - 112,5°
                                0x08: 135,      # 0000 1000 - 0x08 -   8 - 135°  (South-East)
                                0x18: 157.5,    # 0001 1000 - 0x18 -  24 - 157,5°
                                0x10: 180,      # 0001 0000 - 0x10 -  16 - 180°  (South)
                                0x30: 202.5,    # 0011 0000 - 0x30 -  48 - 202,5°
                                0x20: 225,      # 0010 0000 - 0x20 -  32 - 225°  (South-West)
                                0x60: 247.5,    # 0110 0000 - 0x60 -  96 - 247,5°
                                0x40: 270,      # 0100 0000 - 0x40 -  64 - 270°  (West)
                                0xC0: 292.5,    # 1100 0000 - 0xC0 - 192 - 292,5°
                                0x80: 315,      # 1000 0000 - 0x80 - 128 - 315°  (North-West)
                                0x81: 337.5     # 1000 0001 - 0x81 - 129 - 337,5°
                            }
                            try:
                                # Wind direction (byte)
                                sensorData['WindDirection'] = windContactToDegrees[int(msg[7])]
                            except KeyError:
                                sensorData['WindDirection'] = 0
                                print("Value: 0x%02x not found in windContactToDegrees table" % (int(msg[7])))
                            # sensorData['WindDirection'] = int(msg[7])

                            # Windspeed in past 5 min (unsigned int)
                            # 1 rotation of wind of the anemometer (circumference) is 678.6mm = 0.6786m and gives 2 pulses
                            # Time measurement frame of the wind pulses is 5 min = 300 sec
                            # Rotations = windPulses / 2
                            # distance = 0.6786m * rotations
                            # windspeed = distance / 300
                            #
                            # windSpeed [m/s] = windPulses / 2 * 0.6786 / 300 = windPulses * 0.001131
                            # anemometer correction factor = 1.18 -> 0.001131 * 1.18 = 0.00133458
                            # Conversion m/s to km/h = m/s * 3.6
                            # windSpeed [km/h] = 0.00133458 * 3.6 = 0.004804488
                            byteVal = bytearray([int(msg[9]), int(msg[8])])
                            val = struct.unpack(">H", byteVal)[0]
                            sensorData['WindPulses'] = int(val)
                            # windFactor = 0.004804488 # pulses per km/h
                            windFactor = 0.004804488 * 3  # pulses per km/h ()
                            sensorData['WindSpeed'] = round(float(val) * windFactor, 1)

                            # Wind gust past 5 min (8 sec) (unsigned int)
                            byteVal = bytearray([int(msg[11]), int(msg[10])])
                            val = struct.unpack(">H", byteVal)[0]
                            sensorData['WindGustPulses'] = int(val)
                            # WindGust is in an 8 sec time frame. windSpeed factor is in 300s time frame
                            # 300s / 8s = 37.5
                            sensorData['WindGust'] = round(float(val) * windFactor * 37.5, 1)
                
                            # Nr of rain pulses in past 5 min (unsigned int)
                            # 1 liter of water on a m2 is 1 mm of rain
                            # 1 liter of water in the rain bucket gives 315 pulses
                            # The rain bucket is 180mm in diameter and that gives an area of 25446.9mm2=0.0254469m2
                            # This area is 39.2975168 times smaller than a m2
                            # 315 pulses is 1 liter and 1 mm of rain/m2, 1 pulse is 1/315mm multiplied by the size factor
                            # So every pulse = 1/315 * 39.2975168 = 0.124754mm/pulse
                            byteVal = bytearray([int(msg[13]), int(msg[12])])
                            val = struct.unpack(">H", byteVal)[0]
                            sensorData['RainPulses'] = int(val)
                            sensorData['RainFall'] = float(val) * 0.124754
                            # rainRate is mm/h, measurement frame is 5 min. 60min / 5 = 12
                            sensorData['RainRate'] = round(float(val) * 0.124754 * 12, 1)

                            mqtt_publish.single("huis/LoRa/Weerstation/weer", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)
                        else:
                            print("LoRa: Received unknown msgId:%d from NodeId %d" % (msgId, nodeId))

                    # nodeId 5: Temp Chata
                    # nodeId 6: Temp Chata Kelder
                    # nodeId 7: Temp Hottub
                    # nodeId 8: Temp Schuur
                    # nodeId 9: Temp Sauna
                    # nodeId 10: Temp Vriezer Werkplaats
                    # nodeId 11: Temp Vriezer Keuken
                    # nodeId 12: Temp Vriezer Technische ruimte
                    # nodeId 13..29 Voor toekomstige tempsensoren
                    elif (nodeId >= 5) and (nodeId <= 29):
                        msgId = int(msg[0])
                        del msg[0]  # remove msgId from list

                        # MSG_ID_NODE_STARTUP=1: Node startup notification
                        if msgId == 1:
                            print("Received Temperature node (id %d) startup notification" % nodeId)
                        # MSG_ID_TEMPERATURE=4: Temperature measurement
                        elif msgId == 4:
                            sensorData = {}

                            # msg[0] -> 1.0V=0, 1.8V=40, 3,0V=100, 3.3V=115, 5.0V=200, 6.0V=250
                            voltageVcc = float(((int(msg[0]) * 2) + 100)) / 100
                            sensorData['Vcc'] = round(voltageVcc, 2)

                            byteVal = bytearray([int(msg[2]), int(msg[1])])
                            val = struct.unpack(">h", byteVal)[0]
                            if val != DEVICE_DISCONNECTED:
                                sensorData['Temperature'] = round(float(val) / 10, 1)
                            else:
                                # Sensor device disconnected
                                sensorData['Temperature'] = DEVICE_DISCONNECTED

                            if nodeId == 5:
                                print("Temp Chata %1.1f C" % (val / 10))
                                mqtt_publish.single("huis/LoRa/Temp-Chata/temp", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)
                            elif nodeId == 6:
                                print("Temp Chata Kelder %1.1f C" % (val / 10))
                                mqtt_publish.single("huis/LoRa/Temp-Chata-Kelder/temp", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)
                            elif nodeId == 7:
                                print("Temp Hottub Water Boven %1.1f C" % (val / 10))
                                byteVal = bytearray([int(msg[4]), int(msg[3])])
                                val = struct.unpack(">h", byteVal)[0]
                                if val != DEVICE_DISCONNECTED:
                                    sensorData['TempWaterBeneden'] = round(float(val) / 10, 1)
                                else:
                                    # Sensor device disconnected
                                    sensorData['TempWaterBeneden'] = DEVICE_DISCONNECTED
                                print("Temp Hottub Water Beneden %1.1f C" % (val / 10))

                                byteVal = bytearray([int(msg[6]), int(msg[5])])
                                val = struct.unpack(">h", byteVal)[0]
                                if val != DEVICE_DISCONNECTED:
                                    sensorData['TempFiltratieKast'] = round(float(val) / 10, 1)
                                else:
                                    # Sensor device disconnected
                                    sensorData['TempFiltratieKast'] = DEVICE_DISCONNECTED
                                print("Temp Hottub FiltratieKast %1.1f C" % (val / 10))
                                
                                mqtt_publish.single("huis/LoRa/Temp-Hottub/temp", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)
                            elif nodeId == 8:
                                print("Temp Schuur %1.1f C" % (val / 10))
                                mqtt_publish.single("huis/LoRa/Temp-Schuur/temp", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)
                            elif nodeId == 9:
                                print("Temp Sauna %1.1f C" % (val / 10))
                                mqtt_publish.single("huis/LoRa/Temp-Sauna/temp", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)
                            elif nodeId == 10:
                                print("Temp Vriezer-Werkplaats %1.1f C" % (val / 10))
                                mqtt_publish.single("huis/LoRa/Temp-Vriezer-Werkplaats/temp", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)
                            elif nodeId == 11:
                                print("Temp Vriezer-Keuken %1.1f C" % (val / 10))
                                mqtt_publish.single("huis/LoRa/Temp-Vriezer-Keuken/temp", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)
                            elif nodeId == 12:
                                print("Temp Vriezer TechnRuimte %1.1f C" % (val / 10))
                                mqtt_publish.single("huis/LoRa/Temp-Vriezer-TechnRuimte/temp", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)
                        else:
                            print("LoRa: Received unknown msgId:%d from NodeId %d" % (msgId, nodeId))
                    else:
                        print("LoRa: Received an unknown message from NodeID %d" % nodeId)

            # Check if there is any message to send via LoRa
            if not sendQueue.empty():
                sendMsg = sendQueue.get_nowait()
                # print(("SendMsg: %s" % sendMsg))
                if sendMsg != "":
                    serialPort.write(sendMsg)

        # In case the message contains unusual data
        except ValueError as arg:
            print(arg)
            traceback.print_exc()
            time.sleep(1)

        # Quit the program by Ctrl-C
        except KeyboardInterrupt:
            print("Program aborted by Ctrl-C")
            exit()

        # Handle other exceptions and print the error
        except Exception as arg:
            print("%s" % str(arg))
            traceback.print_exc()
            time.sleep(120)


def print_time(delay):
    count = 0
    while count < 5:
        time.sleep(delay)
        count += 1
        print("%s" % (time.ctime(time.time())))


# The callback for when the client receives a CONNACK response from the server.
def on_connect(_client, userdata, flags, rc):
    if rc == 0:
        print("MQTT Client connected successfully")
        _client.subscribe([(settings.MQTT_TOPIC_OUT, 1), (settings.MQTT_TOPIC_HOMELOGIC_BEDIENING, 1), (settings.MQTT_TOPIC_CHECK, 1)])
    else:
        print(("ERROR: MQTT Client connected with result code %s " % str(rc)))


###
# Initalisation ####
###
logger.initLogger(settings.LOG_FILENAME)

# Init signal handler, because otherwise Ctrl-C does not work
signal.signal(signal.SIGINT, signal_handler)

# Make the following devices accessable for user
os.system("sudo chmod 666 %s" % settings.serialPortDevice)

# Give Mosquitto and Home_logic the time to startup
time.sleep(6)

# First start the MQTT client
client = mqtt_client.Client()
client.message_callback_add(settings.MQTT_TOPIC_OUT,       on_message_out)
client.message_callback_add(settings.MQTT_TOPIC_HOMELOGIC_BEDIENING,     on_message_homelogic_bediening)
client.message_callback_add(settings.MQTT_TOPIC_CHECK,     serviceReport.on_message_check)
client.on_connect = on_connect
client.on_message = on_message
client.connect(settings.MQTT_ServerIP, settings.MQTT_ServerPort, 60)
client.loop_start()

# Create the serialPortThread
try:
    # thread.start_new_thread( print_time, (60, ) )
    _thread.start_new_thread(serialPortThread, ())
except Exception as e:
    print("Error: unable to start the serialPortThread")
    print("Exception: %s" % str(e))
    traceback.print_exc()


# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.


while not exitThread:
    time.sleep(30)  # 30s

if serialPort is not None:
    closeSerialPort(serialPort)
    print('Closed serial port')

print("Clean exit!")

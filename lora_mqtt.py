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

# Node ID to node name tabel
idToNodeName = {
    # NodeID 1: Used by the LoRa Gateway (master node)
    2: "Deurtje CEZ-meter",
    3: "Brievenbus post",
    4: "Weerstation",
    5: "Temp Chata",
    6: "Temp Chata Kelder",
    7: "Temp Hottub",
    8: "Temp Schuur",
    9: "Temp Sauna",
    10: "Temp Vriezer Werkplaats",
    11: "Temp Vriezer Keuken",
    12: "Temp Vriezer Technische ruimte",
    13: "Temp Zwembad ruimte en water",
    # nodeId 13..29 Voor toekomstige Temp tempsensoren
    30: "Chata CEZ-meter    "
}

sendQueue = Queue(maxsize=0)

exitThread = False
serialPort = None
LoRaRSSI = False

ignorePowerReading = True
oldCheckPulses = None
diffPulseTime = 0
oldPulseTimer = 0


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
    command = deviceName.split("-")    # KaKu-12

    if command[0] == "FS20ST":
        pass
    #     #print("Activate FS20ST WCD: %s" % command[1])
    #     setFS20ST(int(command[1]), msg.payload)


def on_message_homelogic_bediening(_client, userdata, msg):

    # print("on_message_homelogic_bediening:" + msg.topic + " " + str(msg.payload))
    # print(msg.topic + " " + str(msg.payload))

    topc = msg.topic.split('/')
    deviceName = topc[2]
    # print(deviceName)


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


# noinspection PyTypedDict
def serialPortThread():
    global serialPort
    global exitThread
    global ignorePowerReading
    global oldCheckPulses
    global oldPulseTimer
    global diffPulseTime

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
                # print('LoRa: %s' % serInLine)
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

                    msgId = int(msg[0])
                    del msg[0]  # remove msgId from list

                    # For debug purposes
                    # if nodeId == 30:
                    #     print('LoRa: %s' % serInLine)

                    # if available, gt the rssi signal strength
                    if LoRaRSSI:
                        # signalStrength = msg[-1]
                        del msg[-1]     # signalLevel dB
                        del msg[-1]     # "RSSI"
                    #     print("NodeId: %d Msg: %s (RSSI: %s)" % (nodeId, msg, signalStrength))
                    # else:
                    #     print("NodeId: %d Msg: %s" % (nodeId, msg))

                    # nodeId 1: Used by the LoRa Gateway (master node)

                    # nodeId 2: Sabotage schakelaar op deurtje CEZ-meter
                    # nodeId 3: Brievenbus post notificatie
                    if (nodeId == 2) or (nodeId == 3):
                        sensorData = {}

                        # msg[0] -> 1.0V=0, 1.8V=40, 3,0V=100, 3.3V=115, 5.0V=200, 6.0V=250
                        voltageVcc = float(((int(msg[0]) * 2) + 100)) / 100
                        sensorData['Vcc'] = round(voltageVcc, 2)
                        sensorData['msgId'] = msgId
                        status = int(msg[1])
                        if nodeId == 2:
                            # Deurtje is open als status=1
                            status = not status
                            sensorData['status'] = status
                            # print("Sabotage schakelaar CEZ kastje: %d" % sensorData['status'])
                            mqtt_publish.single("huis/LoRa/Sabotage-Deurtje-bij-meter/sabotage", json.dumps(sensorData, separators=(', ', ':')), qos=1, hostname=settings.MQTT_ServerIP)
                        else:
                            # Klep is open als status=0
                            status = not status
                            sensorData['status'] = status
                            # print("Brievenbus post notificatie: doorOpen: %d, batterij:%1.1f" % (sensorData['status'], voltageVcc))
                            mqtt_publish.single("huis/LoRa/Klep-Postbus-Open/postbus", json.dumps(sensorData, separators=(', ', ':')), qos=1, hostname=settings.MQTT_ServerIP)

                        # MSG_ID_NODE_STARTUP=1: Node startup notification
                        if msgId == 1:
                            print("Received node (id %d) startup notification" % nodeId)
                            if nodeId == 2:
                                serviceReport.reportNodeStatus("Node Sabotage schakelaar op deurtje CEZ meter (ID:%d) is opgestart" % nodeId)
                            else:
                                serviceReport.reportNodeStatus("Node Brievenbus post notificatie (ID:%d) is opgestart" % nodeId)
                            # This sensor is still available
                        # MSG_ID_STILL_ALIVE=2: Node still alive
                        elif msgId == 2:
                            pass
                            # This sensor is still available
                            # print("Received node (id %d) still available" % nodeId)

                        # MSG_ID_SWITCH_CHANGE=4: Switch change detected
                        elif (msgId == 3) or (msgId == 4):
                            pass
                        else:
                            print("LoRa: Received unknown msgId:%d from NodeId %d" % (msgId, nodeId))
                            serviceReport.reportNodeStatus("Onbekend bericht ontvangen van node %s (ID:%d)" % (idToNodeName[nodeId], nodeId))

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

                        # MSG_ID_NODE_STARTUP=1: Node startup notification
                        if msgId == 1:
                            print("Received Weather station node startup notification")
                            serviceReport.reportNodeStatus("Weerstation (ID:%d) is opgestart" % nodeId)

                        elif msgId == 5:
                            print("Received Weather station node: start raining message")
                            mqtt_publish.single("huis/HomeLogic/Actief-Regen/bediening", 1, hostname=settings.MQTT_ServerIP, retain=True)
                            serviceReport.reportNodeStatus("Node Weerstation (ID:%d): Start regen bericht ontvangen van " % nodeId)

                        elif msgId == 6:
                            print("Received Weather station node: reboot ")
                            serviceReport.reportNodeStatus("Node Weerstation (ID:%d) wordt opnieuw herstart" % nodeId)

                        # MSG_ID_TEMPERATURE=4: Temperature measurement
                        elif msgId == 4:
                            sensorData = {}

                            # Vcc: msg[0] -> 1.0V=0, 1.8V=40, 3,0V=100, 3.3V=115, 5.0V=200, 6.0V=250
                            voltageVcc = float(((int(msg[0]) * 2) + 100)) / 100
                            sensorData['Vcc'] = round(voltageVcc, 2)
                            failure = int(msg[1])
                            if failure != 0:
                                if failure & 0x01:
                                    print("WARNING: BME280 device (for temp/humidity/pressure) not found")
                                    serviceReport.reportNodeStatus("ERROR: Node Weerstation BME280 (temp/vocht/luchtdruk) niet gevonden")
                                if failure & 0x02:
                                    print("WARNING: PCF8574 device (for wind direction) not found")
                                    serviceReport.reportNodeStatus("ERROR: Node Weerstation PCF8574 (wind richting) niet gevonden")

                            # Temp (int)
                            byteVal = bytearray([int(msg[3]), int(msg[2])])
                            val = struct.unpack(">h", byteVal)[0]
                            sensorData['Temperature'] = round(float(val) / 100, 1)
                            # print("Temp Buiten %1.1f C" % (val / 100))

                            # Humidity (int)
                            byteVal = bytearray([int(msg[5]), int(msg[4])])
                            val = struct.unpack(">h", byteVal)[0]
                            sensorData['Humidity'] = round(float(val) / 100, 1)

                            mqtt_publish.single("huis/LoRa/Weerstation/temp", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)

                            # Pressure (int)
                            byteVal = bytearray([int(msg[7]), int(msg[6])])
                            val = struct.unpack(">h", byteVal)[0]
                            sensorData['Pressure'] = round(float(val) / 10, 1)

                            altituteInMeter = 500  # 500 m
                            pressureSealevel = val / pow((1.0 - float(altituteInMeter) / float(44330)), 5.255)
                            sensorData['PressureSealevel'] = round(pressureSealevel / 10, 1)
                        
                            # # 8 bits=360 degrees
                            # windContactToDegrees = {
                            #     0x01: 0,        # 0000 0001 - 0x01 -   1 - 0°    (North)
                            #     0x03: 22.5,     # 0000 0011 - 0x03 -   3 - 22,5°
                            #     0x02: 45,       # 0000 0010 - 0x02 -   2 - 45°   (North-East)
                            #     0x06: 67.5,     # 0000 0110 - 0x06 -   6 - 67,5°
                            #     0x04: 90,       # 0000 0100 - 0x04 -   4 - 90°   (East)
                            #     0x0C: 112.5,    # 0000 1100 - 0x0C -  12 - 112,5°
                            #     0x08: 135,      # 0000 1000 - 0x08 -   8 - 135°  (South-East)
                            #     0x18: 157.5,    # 0001 1000 - 0x18 -  24 - 157,5°
                            #     0x10: 180,      # 0001 0000 - 0x10 -  16 - 180°  (South)
                            #     0x30: 202.5,    # 0011 0000 - 0x30 -  48 - 202,5°
                            #     0x20: 225,      # 0010 0000 - 0x20 -  32 - 225°  (South-West)
                            #     0x60: 247.5,    # 0110 0000 - 0x60 -  96 - 247,5°
                            #     0x40: 270,      # 0100 0000 - 0x40 -  64 - 270°  (West)
                            #     0xC0: 292.5,    # 1100 0000 - 0xC0 - 192 - 292,5°
                            #     0x80: 315,      # 1000 0000 - 0x80 - 128 - 315°  (North-West)
                            #     0x81: 337.5     # 1000 0001 - 0x81 - 129 - 337,5°
                            # }
                            # try:
                            #     # Wind direction (byte)
                            #     sensorData['WindDirection'] = windContactToDegrees[int(msg[8])]
                            # except KeyError:
                            #     sensorData['WindDirection'] = 0
                            #     print("WARNING: Value: 0x%02x not found in windContactToDegrees table" % (int(msg[8])))
                            # # sensorData['WindDirection'] = int(msg[8])

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
                            byteVal = bytearray([int(msg[10]), int(msg[9])])
                            val = struct.unpack(">H", byteVal)[0]
                            sensorData['WindPulses'] = int(val)
                            # windFactor = 0.004804488 # pulses per km/h
                            windFactor = 0.004804488 * 3  # pulses per km/h ()
                            sensorData['WindSpeed'] = round(float(val) * windFactor, 1)

                            # Wind gust past 5 min (8 sec) (unsigned int)
                            byteVal = bytearray([int(msg[12]), int(msg[11])])
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
                            byteVal = bytearray([int(msg[14]), int(msg[13])])
                            val = struct.unpack(">H", byteVal)[0]
                            sensorData['RainPulses'] = int(val)
                            sensorData['RainFall'] = float(val) * 0.124754
                            # rainRate is mm/h, measurement frame is 5 min. 60min / 5 = 12
                            sensorData['RainRate'] = round(float(val) * 0.124754 * 12, 1)

                            mqtt_publish.single("huis/LoRa/Weerstation/weer", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)
                        else:
                            print("LoRa: Received unknown msgId:%d from NodeId %d" % (msgId, nodeId))
                            serviceReport.reportNodeStatus("Node %s (ID:%d): Onbekend bericht ontvangen" % (idToNodeName[nodeId], nodeId))

                    # nodeId 5: Temp Chata
                    # nodeId 6: Temp Chata Kelder
                    # nodeId 7: Temp Hottub
                    # nodeId 8: Temp Schuur
                    # nodeId 9: Temp Sauna
                    # nodeId 10: Temp Vriezer Werkplaats
                    # nodeId 11: Temp Vriezer Keuken
                    # nodeId 12: Temp Vriezer Technische ruimte
                    # nodeId 13: Temp Zwembad ruimte en water
                    # nodeId 14..29 Voor toekomstige Temp tempsensoren
                    elif (nodeId >= 5) and (nodeId <= 13):
                        # MSG_ID_NODE_STARTUP=1: Node startup notification
                        if msgId == 1:
                            print("Received Temperature node (id %d) startup notification" % nodeId)
                            serviceReport.reportNodeStatus("Node %s (ID:%d) is opgestart" % (idToNodeName[nodeId], nodeId))
                        # MSG_ID_TEMPERATURE=4: Temperature measurement
                        elif msgId == 4:
                            sensorData = {}

                            # msg[0] -> 1.0V=0, 1.8V=40, 3,0V=100, 3.3V=115, 5.0V=200, 6.0V=250
                            voltageVcc = float(((int(msg[0]) * 2) + 100)) / 100
                            sensorData['Vcc'] = round(voltageVcc, 2)

                            if len(msg) == 1:
                                # ERROR: No temperature device found in Node
                                serviceReport.reportNodeStatus("Node %s (ID:%d): geen DS18b20 gevonden" % (idToNodeName[nodeId], nodeId))
                            else:
                                byteVal = bytearray([int(msg[2]), int(msg[1])])
                                val = struct.unpack(">h", byteVal)[0]
                                if val != DEVICE_DISCONNECTED:
                                    sensorData['Temperature'] = round(float(val) / 10, 1)
                                else:
                                    # Sensor device disconnected
                                    sensorData['Temperature'] = DEVICE_DISCONNECTED

                                if nodeId == 5:
                                    # print("Temp Chata %1.1f C" % (val / 10))
                                    mqtt_publish.single("huis/LoRa/Temp-Chata/temp", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)
                                elif nodeId == 6:
                                    # print("Temp Chata Kelder %1.1f C" % (val / 10))
                                    mqtt_publish.single("huis/LoRa/Temp-Chata-Kelder/temp", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)
                                elif nodeId == 7:
                                    # print("Temp Hottub Water Boven %1.1f C" % (val / 10))
                                    byteVal = bytearray([int(msg[4]), int(msg[3])])
                                    val = struct.unpack(">h", byteVal)[0]
                                    if val != DEVICE_DISCONNECTED:
                                        sensorData['TempWaterBeneden'] = round(float(val) / 10, 1)
                                    else:
                                        # Sensor device disconnected
                                        sensorData['TempWaterBeneden'] = DEVICE_DISCONNECTED
                                    # print("Temp Hottub Water Beneden %1.1f C" % (val / 10))

                                    byteVal = bytearray([int(msg[6]), int(msg[5])])
                                    val = struct.unpack(">h", byteVal)[0]
                                    if val != DEVICE_DISCONNECTED:
                                        sensorData['TempFiltratieKast'] = round(float(val) / 10, 1)
                                    else:
                                        # Sensor device disconnected
                                        sensorData['TempFiltratieKast'] = DEVICE_DISCONNECTED
                                    # print("Temp Hottub FiltratieKast %1.1f C" % (val / 10))

                                    mqtt_publish.single("huis/LoRa/Temp-Hottub/temp", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)
                                elif nodeId == 8:
                                    # print("Temp Schuur %1.1f C" % (val / 10))
                                    mqtt_publish.single("huis/LoRa/Temp-Schuur/temp", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)
                                elif nodeId == 9:
                                    # print("Temp Sauna %1.1f C" % (val / 10))
                                    mqtt_publish.single("huis/LoRa/Temp-Sauna/temp", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)
                                elif nodeId == 10:
                                    # print("Temp Vriezer-Werkplaats %1.1f C" % (val / 10))
                                    mqtt_publish.single("huis/LoRa/Temp-Vriezer-Werkplaats/temp", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)
                                elif nodeId == 11:
                                    # print("Temp Vriezer-Keuken %1.1f C" % (val / 10))
                                    mqtt_publish.single("huis/LoRa/Temp-Vriezer-Keuken/temp", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)
                                elif nodeId == 12:
                                    # print("Temp Vriezer TechnRuimte %1.1f C" % (val / 10))
                                    mqtt_publish.single("huis/LoRa/Temp-Vriezer-TechnRuimte/temp", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)
                                elif nodeId == 13:
                                    # print("Temp Zwembad Ruimte %1.1f C" % (val / 10))
                                    byteVal = bytearray([int(msg[4]), int(msg[3])])
                                    val = struct.unpack(">h", byteVal)[0]
                                    if val != DEVICE_DISCONNECTED:
                                        sensorData['TempWater'] = round(float(val) / 10, 1)
                                    else:
                                        # Sensor device disconnected
                                        sensorData['TempWater'] = DEVICE_DISCONNECTED
                                    # print("Temp Zwembad Water %1.1f C" % (val / 10))

                                    mqtt_publish.single("huis/LoRa/Temp-Zwembad/temp", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)
                        else:
                            print("LoRa: Received unknown msgId:%d from NodeId %d" % (msgId, nodeId))
                            serviceReport.reportNodeStatus("Node %s (ID:%d): Onbekend bericht ontvangen" % (idToNodeName[nodeId], nodeId))

                    # nodeId 30: Chata CEZ pulse counter
                    elif nodeId == 30:
                        # MSG_ID_NODE_STARTUP=1: Node startup notification
                        if msgId == 1:
                            print("Received node (id %d) startup notification" % nodeId)
                            serviceReport.reportNodeStatus("Node CEZ pulse counter bij Chata (ID:%d) is opgestart" % nodeId)
                            # Sensor is restarted and so the nr of counted pulses, reset the diffPulseTime
                            oldPulseTimer = current_sec_time()  # Init oldPulseTimer for the next measurement
                            ignorePowerReading = False
                            oldCheckPulses = None  # Do not recover the pulses after this, node is restarted

                        # MSG_ID_COUNTED_PULSES=4: Switch change detected
                        elif msgId == 4:
                            sensorData = {}

                            # msg[0] -> 1.0V=0, 1.8V=40, 3,0V=100, 3.3V=115, 5.0V=200, 6.0V=250
                            voltageVcc = round(float(((int(msg[0]) * 2) + 100)) / 100, 2)

                            pulses = int(msg[1]) + int(msg[2]) * 256
                            checkPulses = int(msg[3]) + int(msg[4]) * 256

                            if oldCheckPulses is not None:
                                # Check if we missed 1 or more messages and so missing pulses
                                if oldCheckPulses != (checkPulses - pulses):
                                    ignorePowerReading = True   # Do not calculate the power
                                    newPulses = checkPulses - oldCheckPulses
                                    if newPulses > 10000:
                                        print("WARNING: Something is wrong, using more than 50kW, don´t use newPulses")
                                        pulses = 0
                                    else:
                                        print("We missed some msgs, recover the number of pulses")
                                        print("oldCheckPulses: %d, checkPulses: %d, pulses: %d" % (oldCheckPulses, checkPulses, pulses))
                                        # Set the recovered amount of pulses
                                        pulses = newPulses
                                        print('Recovered amount of pulses: %d' % pulses)
                            else:
                                if checkPulses != 0:
                                    print("oldCheckPulses was not initialized yet, lora_mqtt.py service is restarted")
                                else:
                                    print("Node and lora_mqtt.py service are restarted")
                            oldCheckPulses = checkPulses

                            diffPulseTime = current_sec_time() - oldPulseTimer
                            oldPulseTimer = current_sec_time()  # Init oldPulseTimer for the next measurement

                            power = 0
                            if ignorePowerReading:
                                # In case this lora_mqtt.py was restarted,
                                ignorePowerReading = False
                                print("Ignore the first power reading")
                                diffPulseTime = 0  # Block the first power calculation, because the time isn't right, which results in high power peak result
                            else:
                                if diffPulseTime != 0:
                                    # Don't calculate the power if the previous msg is less 1 sec ago, otherwise divide by zero
                                    # P [Watt] = pulses * 360 [Ws] / t [sec]
                                    # CEZ meter Chata: 10.000 pulses/kWh
                                    pulsesInTime = pulses * 360  # 360=10.000 pulses/kWh and 3600=1.000 pulses/kWh
                                    power = pulsesInTime / diffPulseTime

                                senorData = {}
                                sensorData['Vcc'] = voltageVcc
                                sensorData['pulses'] = pulses
                                sensorData['power'] = round(power, 1)

                                mqtt_publish.single("huis/LoRa/Chata-CEZ-meter/power", json.dumps(sensorData, separators=(', ', ':')), qos=1, hostname=settings.MQTT_ServerIP)

                            print(("power: %d W (added %d pulses, time %d sec)" % (power, pulses, diffPulseTime)))

                        else:
                            print("LoRa: Received unknown msgId:%d from NodeId %d" % (msgId, nodeId))
                    else:
                        print("LoRa: Received an message from unknown node (ID:%d, msgID: %d)" % (nodeId, msgId))
                        serviceReport.reportNodeStatus("Node %s (ID:%d, msgID:%d): Onbekend bericht ontvangen" % (idToNodeName[nodeId], msgId, nodeId))

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
# Initialisation ####
###
logger.initLogger(settings.LOG_FILENAME)

# Init signal handler, because otherwise Ctrl-C does not work
signal.signal(signal.SIGINT, signal_handler)

# Make the following devices accessible for user
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

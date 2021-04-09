#!/usr/bin/python3

import os
import signal
import time
import serial
import _thread
import traceback
import json

from queue import Queue
import paho.mqtt.publish as mqtt_publish
import paho.mqtt.client as mqtt_client

# external files/classes
import logger
import serviceReport
import settings

wildDetectorActive = True
chickenDoorOpenActive = False

sendQueue = Queue(maxsize=0)
current_sec_time = lambda: int(round(time.time()))
current_milli_time = lambda: int(round(time.time() * 1000))

exit = False
serialPort = None
LoRaRSSI = False


def signal_handler(_signal, frame):
    global exit

    print('You pressed Ctrl+C!')
    exit = True


def on_message(client, userdata, msgJson):
    print(('ERROR: Received ' + msgJson.topic + ' in on_message function' + str(msgJson.payload)))


def on_message_out(client, userdata, msg):
    # print(("on_message_out:" + msg.topic + " " + str(msg.payload)))
    #print(msg.topic + " " + str(msg.payload))
    topics = msg.topic.split("/")

    deviceName = topics[2] #huis/LoRa/FS20ST-1/out
    cmnd = deviceName.split("-") #KaKu-12

    # if cmnd[0] == "FS20ST":
    #     #print("Activate FS20ST WCD: %s" % cmnd[1])
    #     setFS20ST(int(cmnd[1]), msg.payload)


def on_message_homelogic_bediening(client, userdata, msg):
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
    try:
        ser = serial.Serial(port=settings.serialPortDevice,
                            baudrate=settings.serialPortBaudrate,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS,
                            timeout=1)  # 1=1sec 0=non-blocking None=Blocked

        if ser.isOpen():
            print(("lora_mqtt: Successfully connected to serial port %s" % (settings.serialPortDevice)))

        return ser

    # Handle other exceptions and print the error
    except Exception as arg:
        print("%s" % str(arg))
        traceback.print_exc()

        #Report failure to Home Logic system check
        serviceReport.sendFailureToHomeLogic(serviceReport.ACTION_NOTHING, 'Serial port open failure on port %s, wrong port or USB cable missing' % (settings.serialPortDevice))


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

    global totalsInitialised
    global firstPulsesMsg
    global totalKWHpulsesI   # High tarif
    global totalKWHpulsesII  # Low tarif
    global activeHighTarif
    global oldPulseTimer
    global oldPulses
#    global oldPulsesCEZ
    global current_sec_time
    global raining
    global oldRaining
    global oldMmRain
    global rainTimer
    global firstRainPulsesMsg

    # Waterpomp regeling
    global oldPompAanRelais

    global checkMsg
    global somethingWrong

    signal = 0
    oldTimeout = current_sec_time()

    serialPort = openSerialPort()

    mqtt_publish.single("huis/HomeLogic/Get-kWh-Totals/command", 1, qos=1, hostname=settings.MQTT_ServerIP)

    while True:
        try:
            if serialPort.isOpen():
                serInLine = serialPort.readline().decode()
            else:
                serInLine = ""

            if serInLine != "":
                serInLine = serInLine.rstrip("\r\n")
                # print('LoRa: %s' % (serInLine))
                msg = serInLine.split(' ')
                #print(msg)
                # LoRa is started: Init it
                if msg[0] == "[LORA_GATEWAY]":
                    initLoRa(serialPort)
                    print("LoRa: [LORA_GATEWAY]: Adapter Initialized!...")

                # Check the LoRa Rx timeout
                if (current_sec_time() - oldTimeout) > 2000:
                    # Reset the LoRa Rx timeout timer
                    oldTimeout = current_sec_time()

                    #Report failure to Home Logic system check
                    serviceReport.sendFailureToHomeLogic(serviceReport.ACTION_RESTART, 'Serial port timeout (35 min no data received)!')

                # Only handle messages starting with 'OK' from LoRa
                if msg[0] == 'OK':
                    # Reset the LoRa Rx timeout timer
                    oldTimeout = current_sec_time()

                    # print 'OK found!'
                    del msg[0]  # remove 'OK' from list

                    nodeId = int(msg[0]) & 0x1F  # NodeId 0..31
                    del msg[0]  # remove NodeId from list

                    # if available, gt the rssi signal strength
                    if LoRaRSSI:
                        signal = msg[-1]
                        del msg[-1] #signalLevel dB
                        del msg[-1] # "RSSI"
                        print("NodeId: %d Msg: %s (RSSI: %s)" % (nodeId, msg, signal))
                    else:
                        print("NodeId: %d Msg: %s" % (nodeId, msg))

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
                                putMsg = "3,1,%ds" % (nodeId)
                                print("LoRa: Open chicken door")
                            else:
                                # Set WildDetector PIR: off
                                putMsg = "3,0,%ds" % (nodeId)
                                print("LoRa: Close chicken door")
                            sendQueue.put(putMsg.encode())
                        else:
                            print("LoRa: Received unknown msgId:%d from NodeId %d" % (msgId, nodeId))
                    # nodeId 3: Wild detector
                    elif nodeId == 3:
                        msgId = int(msg[0])

                        # MSG_ID_NODE_STARTUP=1: Node startup notification
                        if msgId == 1:
                            # This sensor is still available
                            mqtt_publish.single("huis/LoRa/PIRB-Wild-Detector/pirb", 0, qos=1, hostname=settings.MQTT_ServerIP)
                        # MSG_ID_STILL_ALIVE=2: Node still alive
                        elif msgId == 2:
                            # This sensor is still available
                            mqtt_publish.single("huis/LoRa/PIRB-Wild-Detector/pirb", 0, qos=1, hostname=settings.MQTT_ServerIP)
                        # MSG_ID_CMND_REQUEST=3: Node wakeup/cmnd request
                        elif msgId == 3:
                            if wildDetectorActive:
                                # Set WildDetector PIR: off
                                sendQueue.put("3,1,%ds".encode() % nodeId)
                                print("LoRa: Activate WildDetector PIR")
                            else:
                                # Set WildDetector PIR: off
                                sendQueue.put("3,0,%ds".encode() % nodeId)
                                print("LoRa: De-activate WildDetector PIR")
                        # MSG_ID_PIR_MOVEMENT=4: PIR detected movement
                        elif msgId == 4:
                            mqtt_publish.single("huis/LoRa/PIRB-Wild-Detector/pirb", 1, qos=1, hostname=settings.MQTT_ServerIP)
                        else:
                            print("LoRa: Received unknown msgId:%d from NodeId %d" % (msgId, nodeId))
                    else:
                        print("LoRa: Received an unknown message from NodeID %d" % nodeId)

                serviceReport.systemWatchTimer = current_sec_time()

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
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("MQTT Client connected successfully")
        client.subscribe([(settings.MQTT_TOPIC_OUT, 1), (settings.MQTT_TOPIC_HOMELOGIC_BEDIENING, 1), (settings.MQTT_TOPIC_CHECK, 1)])
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


while not exit:
    time.sleep(2)  # 60s

if serialPort is not None:
    closeSerialPort(serialPort)
    print('Closed serial port')

print("Clean exit!")

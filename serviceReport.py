import paho.mqtt.publish as mqtt_publish
import json
import time

# external files/classes
import settings

# System check
ACTION_NOTHING = 0
ACTION_RESTART = 1

# Node msg IDs
NODE_STARTUP = 1
NODE_REBOOT = 2

checkMsg = 'OK'
checkFail = False
checkAction = ACTION_NOTHING
checkReport = {}


def current_sec_time():
    return int(round(time.time()))


systemWatchTimer = current_sec_time()


# Called by mqtt
def on_message_check(client, userdata, msgJson):
    if (current_sec_time() - systemWatchTimer) > 2100:
        sendCheckReportToHomeLogic(True, ACTION_RESTART, "Timeout systemWatchTimer, 35 min no activity in serialPortThread-thread loop")
        # print("on_message_check: " + msgJson.topic + ": " + str(msgJson.payload))
        print("on_message_check: systemWatchTimer > 2100 sec (%d sec)" % (current_sec_time() - systemWatchTimer))
    else:
        # print("on_message_check: " + msgJson.topic + ": " + str(msgJson.payload))
        sendCheckReportToHomeLogic(checkFail, checkAction, checkMsg)


def reportNodeStatus(msgText):
    checkReport['checkFail'] = False
    checkReport['checkAction'] = ACTION_NOTHING
    checkReport['checkMsg'] = msgText
    mqtt_publish.single(settings.NODE_STATUS_TOPIC_REPORT, json.dumps(checkReport), qos=1, hostname=settings.MQTT_ServerIP)


# Send the report to the Home Logic system checker
def sendCheckReportToHomeLogic(fail, action, msg):
    global checkMsg
    global checkFail

    checkMsg = msg
    checkFail = fail
    checkReport['checkFail'] = checkFail
    checkReport['checkAction'] = checkAction
    checkReport['checkMsg'] = checkMsg
    mqtt_publish.single(settings.MQTT_TOPIC_REPORT, json.dumps(checkReport), qos=1, hostname=settings.MQTT_ServerIP)


# Don't wait for the Home Logic system checker, report it directly
def sendFailureToHomeLogic(_checkAction, _checkMsg):
    sendCheckReportToHomeLogic(True, _checkAction, _checkMsg)

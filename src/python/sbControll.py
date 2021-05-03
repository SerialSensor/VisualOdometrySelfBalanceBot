import socket
import select
from collections import deque
from typing import Deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import keyboard
import os
import threading
import errno
import math

# General config / constants
SEND_IP = "192.168.0.0"  # Change to your phone's ip
RECEIVE_IP = "192.168.0.0"  # Change to your pc's ip
UDP_PORT = 8888  # (Default) Only change to the port configured inside the app
MESSAGE_LENGTH = 13  # data frame has 13 bytes
NO_QUANTIZATION = 1
TILT_CONTROLLER_P_QUANTIZATION = 0.05
TILT_CONTROLLER_I_QUANTIZATION = 0.1
TILT_CONTROLLER_D_QUANTIZATION = 0.0005
POS_CONTROLLER_QUANTIZATION = 0.05
YAW_CONTROLLER_QUANTIZATION = 0.05
DESIRED_TILT_ANDGLE_QUANTIZATION = -0.003491
MIN_DATA_INDEX = 1
MIN_DATA_VALUE = 0
MAX_DATA_VALUE = 255
PLOT_UPDATE_IN_MS = 10  # Update rate of animation
DATA_STORAGE_IN_MS = 120000  # Last x ms which are shown in the plot
POSITION_SENSOR_ID = 101
PARAMETER_SENSOR_ID = 103


def getNormedValue(val, norm):
    return int(val/norm + 0.5)


INTIAL_VALUE_DESRIED_ANGLE = getNormedValue(
    -0.445, DESIRED_TILT_ANDGLE_QUANTIZATION)
INTIAL_VALUE_P_TILT_CONTROLLER = getNormedValue(
    2.3, TILT_CONTROLLER_P_QUANTIZATION)
INTIAL_VALUE_I_TILT_CONTROLLER = getNormedValue(
    18.0, TILT_CONTROLLER_I_QUANTIZATION)
INTIAL_VALUE_D_TILT_CONTROLLER = getNormedValue(
    0.04, TILT_CONTROLLER_D_QUANTIZATION)
INTIAL_VALUE_P_POSITION_CONTROLLER = getNormedValue(
    0.5, POS_CONTROLLER_QUANTIZATION)
INTIAL_VALUE_I_POSITION_CONTROLLER = getNormedValue(
    0.5, POS_CONTROLLER_QUANTIZATION)
INTIAL_VALUE_D_POSITION_CONTROLLER = getNormedValue(
    0.5, POS_CONTROLLER_QUANTIZATION)
INTIAL_VALUE_P_YAW_CONTROLLER = getNormedValue(
    0.2, YAW_CONTROLLER_QUANTIZATION)
INCREASE_POSITION_OR_ROTATION = 2
NO_POSITION_OR_ROTATION = 1
DECREASE_POSITION_OR_ROTATION = 0
POSITION_CONTROL_INDEX = 11
ROTATION_CONTROL_INDEX = 12
INTIAL_VALUE_DRIVING_DIRECTION = getNormedValue(
    NO_POSITION_OR_ROTATION, NO_QUANTIZATION)
INTIAL_VALUE_ROTATION_DIRECTION = getNormedValue(
    NO_POSITION_OR_ROTATION, NO_QUANTIZATION)
outputData = bytearray([PARAMETER_SENSOR_ID, 0, 0, INTIAL_VALUE_DESRIED_ANGLE, INTIAL_VALUE_P_TILT_CONTROLLER, INTIAL_VALUE_I_TILT_CONTROLLER, INTIAL_VALUE_D_TILT_CONTROLLER,
                        INTIAL_VALUE_P_POSITION_CONTROLLER, INTIAL_VALUE_I_POSITION_CONTROLLER, INTIAL_VALUE_D_POSITION_CONTROLLER, INTIAL_VALUE_P_YAW_CONTROLLER, INTIAL_VALUE_DRIVING_DIRECTION, INTIAL_VALUE_ROTATION_DIRECTION])
defaultOutputData = outputData
dataQuantization = [NO_QUANTIZATION, NO_QUANTIZATION, NO_QUANTIZATION, DESIRED_TILT_ANDGLE_QUANTIZATION, TILT_CONTROLLER_P_QUANTIZATION, TILT_CONTROLLER_I_QUANTIZATION,
                    TILT_CONTROLLER_D_QUANTIZATION, POS_CONTROLLER_QUANTIZATION, POS_CONTROLLER_QUANTIZATION, POS_CONTROLLER_QUANTIZATION, YAW_CONTROLLER_QUANTIZATION, NO_QUANTIZATION, NO_QUANTIZATION]
currentDataIndex = MIN_DATA_INDEX
indexDataMeaning = ["SensorID", "PWM Offset Left", "PWM Offset Right", "Tilt-Controller desired value", "Tilt-Controller P value", "Tilt-Controller I value",
                    "Tilt-Controller D value", "Position-Controller P value", "Position-Controller I value", "Position-Controller D value", "Heading-Controller P value", "Driving direction", "Rotation direction"]
updateParameterData = True
# Prepare UDP
print("---------Settings---------")
print("Please change inside the script to your IPs and Port!")
print("Receiving on IP: ", RECEIVE_IP)
print("Sending to IP: ", SEND_IP)
print("Sending to Port: ", UDP_PORT)
# Any data sent to ssock shows up on rsock
rSock, sSock = socket.socketpair()
mainSocket = socket.socket(socket.AF_INET,
                           socket.SOCK_DGRAM)
mainSocket.setblocking(False)
mainSocket.bind((RECEIVE_IP, UDP_PORT))
sendQueue = deque(outputData, maxlen=1)
# Prepare Plots
fig, ax = plt.subplots()
ax.ticklabel_format(useOffset=False)
ax.set_ylabel('Position z in m')
ax.set_xlabel('Position x in m')
ax.set_xlim(2, -2)
ax.set_ylim(-2, 2)
maxDataLength = DATA_STORAGE_IN_MS // (PLOT_UPDATE_IN_MS)
lastReceivedPositionData = deque([bytes(MESSAGE_LENGTH)], maxlen=1)
plotData = deque([(0, 0)], maxlen=maxDataLength)
line = plt.plot(*zip(*plotData), c='black')[0]

# Key stuff


def getGreenString(val):
    return '\033[32m' + val + '\033[m'


def printUsage():
    keyboard.unhook_all()
    setupBaseKeys()
    print("---------How To---------")
    print("Press p for parameter mode. Navigate with arrow left and right through parameters, increase and decrease parameter with arrow up and down.")
    print("Press d for setting and sending default parameters.")
    print("Press m for manual drive mode. Drive in certain direction by holding a arrow key.")
    #print("Press " + getGreenString("b") + " for opening the birds eye view mode.")
    print("Press h for showing the how-to again.")
    print("Press esc to end this script.")
    print("Waiting for Input...")


def waitForKey():
    while True:
        keyboard.wait()


def getCurrentValue():
    return format(outputData[currentDataIndex]*dataQuantization[currentDataIndex], '.4f')


def sendToMainThread(data):
    # put data to queue and signal main thread
    sendQueue.append(data)
    sSock.send(b"\x00")


def roateLeft(event):
    if event.event_type == keyboard.KEY_DOWN:
        outputData[ROTATION_CONTROL_INDEX] = INCREASE_POSITION_OR_ROTATION
    else:
        outputData[ROTATION_CONTROL_INDEX] = NO_POSITION_OR_ROTATION
    sendToMainThread(outputData)


def roateRight(event):
    if event.event_type == keyboard.KEY_DOWN:
        outputData[ROTATION_CONTROL_INDEX] = DECREASE_POSITION_OR_ROTATION
    else:
        outputData[ROTATION_CONTROL_INDEX] = NO_POSITION_OR_ROTATION
    sendToMainThread(outputData)


def driveForward(event):
    if event.event_type == keyboard.KEY_DOWN:
        outputData[POSITION_CONTROL_INDEX] = INCREASE_POSITION_OR_ROTATION
    else:
        outputData[POSITION_CONTROL_INDEX] = NO_POSITION_OR_ROTATION
    sendToMainThread(outputData)


def driveBackward(event):
    if event.event_type == keyboard.KEY_DOWN:
        outputData[POSITION_CONTROL_INDEX] = DECREASE_POSITION_OR_ROTATION
    else:
        outputData[POSITION_CONTROL_INDEX] = NO_POSITION_OR_ROTATION
    sendToMainThread(outputData)


def decreaseAndSend(event):
    outputData[currentDataIndex] = max(
        outputData[currentDataIndex]-1, MIN_DATA_VALUE)
    print("Sent value:" + str(getCurrentValue()))
    sendToMainThread(outputData)


def increaseAndSend(event):
    outputData[currentDataIndex] = min(
        outputData[currentDataIndex]+1, MAX_DATA_VALUE)
    print("Sent value:" + str(getCurrentValue()))
    sendToMainThread(outputData)


def decreaseDataIndex(event):
    global currentDataIndex
    currentDataIndex = max(currentDataIndex - 1, MIN_DATA_INDEX)
    print("---------Switched Parameter---------")
    print(indexDataMeaning[currentDataIndex])
    print("Current value:" + str(getCurrentValue()))


def increaseDataIndex(event):
    global currentDataIndex
    currentDataIndex = min(currentDataIndex + 1, MESSAGE_LENGTH-3)
    print("---------Switched Parameter---------")
    print(indexDataMeaning[currentDataIndex])
    print("Current value:" + str(getCurrentValue()))


def myExit():
    os._exit(1)


def changeToDriverMode():
    print("Entered drive mode.")
    keyboard.unhook_all()
    setupBaseKeys()
    keyboard.hook_key('up', driveForward)
    keyboard.hook_key('down', driveBackward)
    keyboard.hook_key('left', roateLeft)
    keyboard.hook_key('right', roateRight)


def changeToParameterMode():
    print("Entered parameter mode.")
    keyboard.unhook_all()
    setupBaseKeys()
    keyboard.on_press_key('left', decreaseDataIndex)
    keyboard.on_press_key('right', increaseDataIndex)
    keyboard.on_press_key('down', decreaseAndSend)
    keyboard.on_press_key('up', increaseAndSend)


def sendDefaultParameters():
    print("Default parameters set and sent!")
    sendToMainThread(defaultOutputData)


def showHidePlot():  # currently not working due to thread things
    if plt.fignum_exists(fig.number):
        plt.close()
    else:
        plt.show()


def setupBaseKeys():
    keyboard.add_hotkey('p', changeToParameterMode)
    keyboard.add_hotkey('m', changeToDriverMode)
    keyboard.add_hotkey('d', sendDefaultParameters)
    keyboard.add_hotkey('h', printUsage)
    keyboard.add_hotkey('esc', myExit)


printUsage()
threadKey = threading.Thread(target=waitForKey)
threadKey.start()

# Input and output


def receiveAndSend():
    global updateParameterData, outputData
    while True:
        # When either mainSocket has data or rSock has data, select.select will return
        rlist, _, _ = select.select([mainSocket, rSock], [], [])
        for ready_socket in rlist:
            if ready_socket is mainSocket:
                # get latest data
                keepReceiving = True
                while keepReceiving:
                    try:
                        data, fromAddr = mainSocket.recvfrom(MESSAGE_LENGTH)
                        if data[0] == POSITION_SENSOR_ID:
                            lastReceivedPositionData.append(data)
                        elif data[0] == PARAMETER_SENSOR_ID and updateParameterData:
                            updateParameterData = False
                            outputData = bytearray(data)
                    except socket.error as why:
                        if why.args[0] == errno.EWOULDBLOCK:
                            keepReceiving = False
                        else:
                            raise why
            else:
                # Ready_socket is rsock
                rSock.recv(1)  # Dump the ready mark
                # Send data from queue
                mainSocket.sendto(sendQueue[0], (SEND_IP, UDP_PORT))


threadReceiveAndSend = threading.Thread(target=receiveAndSend)
threadReceiveAndSend.start()

# Plot stuff


def getFloatFrom3Bytes(lastReceivedPositionData, offset):
    val = int.from_bytes([lastReceivedPositionData[1 + offset], lastReceivedPositionData[2 +
                          offset], lastReceivedPositionData[3 + offset]], "little", signed=True)
    return val / float(65536)


def update(frame):
    localPositionX = getFloatFrom3Bytes(lastReceivedPositionData[0], 0)
    localPositionZ = getFloatFrom3Bytes(lastReceivedPositionData[0], 3)
    longVelocity = getFloatFrom3Bytes(lastReceivedPositionData[0], 6)
    heading = -getFloatFrom3Bytes(lastReceivedPositionData[0], 9)
    globalPositionX = math.cos(heading)*localPositionX - \
        math.sin(heading)*localPositionZ
    globalPositionZ = math.sin(heading)*localPositionX + \
        math.cos(heading)*localPositionZ
    plotData.append((globalPositionX, globalPositionZ))
    line.set_data(*zip(*plotData))
    ax.relim()
    ax.autoscale_view(True, True, True)
    return line,


ani = animation.FuncAnimation(
    fig, update, interval=PLOT_UPDATE_IN_MS, blit=False)

plt.show()

#!/usr/bin/env python

import rospy
from vicon_data_publisher.msg import Force_plate_data
from queue import Queue
import copy

publisher = None

window_size = 1000
queue = Queue(window_size)
average = Force_plate_data()
offset = Force_plate_data()
previous = Force_plate_data()

#############################################

def callback(data):
    process_and_publish(data)

#############################################
def process_and_publish(data):
    global average
    global offset
    global previous

    # Masse bestimmen
    data.fz_N = data.fz_N / 9.81

    # Nullen filtern
    if (abs(data.fx_N) < 0.5):
        data.fx_N = previous.fx_N

    if (abs(data.fy_N) < 0.5):
        data.fy_N = previous.fy_N

    if (abs(data.fz_N) < 0.5):
        data.fz_N = previous.fz_N

    if (abs(data.mx_Nm) < 0.5):
        data.mx_Nm = previous.mx_Nm

    if (abs(data.my_Nm) < 0.5):
        data.my_Nm = previous.my_Nm

    if (abs(data.mz_Nm) < 0.5):
        data.mz_Nm = previous.mz_Nm

    previous = copy.deepcopy(data)

    if (abs(data.fz_N) < 0.5):
        rospy.loginfo(f"{data.fz_N}\n")

    # Averaging Fenster initialisieren
    if (queue.qsize() < window_size):
        rospy.loginfo(f"{queue.qsize()}")
        # Nullwerte am Anfang nicht in das Offset einfliessen lassen
        if (abs(data.fz_N) < 0.5):
            return
        queue.put(data)
        average.fx_N = average.fx_N + data.fx_N / window_size
        average.fy_N = average.fy_N + data.fy_N / window_size
        average.fz_N = average.fz_N + data.fz_N / window_size
        average.mx_Nm = average.mx_Nm + data.mx_Nm / window_size
        average.my_Nm = average.my_Nm + data.my_Nm / window_size
        average.mz_Nm = average.mz_Nm + data.mz_Nm / window_size
        # Tarierung bestimmen
        offset = copy.deepcopy(average)
        return

    # Tarierung anwenden
    data.fx_N = data.fx_N - offset.fx_N
    data.fy_N = data.fy_N - offset.fy_N
    data.fz_N = data.fz_N - offset.fz_N
    data.mx_Nm = data.mx_Nm - offset.mx_Nm
    data.my_Nm = data.my_Nm - offset.my_Nm
    data.mz_Nm = data.mz_Nm - offset.mz_Nm

    # Averaging Fenster weiter ziehen
    old = queue.get()
    queue.put(data)

    # Averaging durchfueren
    average.fx_N = average.fx_N + (data.fx_N - old.fx_N) / window_size
    average.fy_N = average.fy_N + (data.fy_N - old.fy_N) / window_size
    average.fz_N = average.fz_N + (data.fz_N - old.fz_N) / window_size
    average.mx_Nm = average.mx_Nm + (data.mx_Nm - old.mx_Nm) / window_size
    average.my_Nm = average.my_Nm + (data.my_Nm - old.my_Nm) / window_size
    average.mz_Nm = average.mz_Nm + (data.mz_Nm - old.mz_Nm) / window_size

    # Publishing
    publisher.publish(average)

#############################################
def transceiver():
    rospy.init_node('force_plate_data_transceiver', anonymous=True)

    global publisher
    publisher = rospy.Publisher('Force_plate_data_sma', Force_plate_data, queue_size = 1000)
    rospy.Subscriber("Force_plate_data", Force_plate_data, callback)

    rospy.loginfo(f"Tranceiver started.")

    rospy.spin()

#############################################
if __name__ == '__main__':
    transceiver()

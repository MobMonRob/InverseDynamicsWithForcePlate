#!/usr/bin/env python

import rospy
from force_plate_data_publisher.msg import Vicon_compound_data;
from force_plate_data_publisher.msg import Force_plate_data;
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
    for dataPoint in data.force_plate_data :
        process_and_publish(dataPoint)

#############################################
def process_and_publish(data):
    global average
    global offset
    global previous

    # Masse bestimmen
    data.f_z = data.f_z / 9.81

    # Nullen filtern
    if (abs(data.f_x) < 0.5):
        data.f_x = previous.f_x

    if (abs(data.f_y) < 0.5):
        data.f_y = previous.f_y

    if (abs(data.f_z) < 0.5):
        data.f_z = previous.f_z

    if (abs(data.m_x) < 0.5):
        data.m_x = previous.m_x

    if (abs(data.m_y) < 0.5):
        data.m_y = previous.m_y

    if (abs(data.m_z) < 0.5):
        data.m_z = previous.m_z

    previous = copy.deepcopy(data)

    if (abs(data.f_z) < 0.5):
        rospy.loginfo(f"{data.f_z}\n")

    #publisher.publish(data)
    #return

    # Averaging Fenster initialisieren
    if (queue.qsize() < window_size):
        rospy.loginfo(f"{queue.qsize()}")
        # Nullwerte am Anfang nicht in das Offset einfliessen lassen
        if (abs(data.f_z) < 0.5):
            return
        queue.put(data)
        average.f_x = average.f_x + data.f_x / window_size
        average.f_y = average.f_y + data.f_y / window_size
        average.f_z = average.f_z + data.f_z / window_size
        average.m_x = average.m_x + data.m_x / window_size
        average.m_y = average.m_y + data.m_y / window_size
        average.m_z = average.m_z + data.m_z / window_size
        # Tarierung bestimmen
        offset = copy.deepcopy(average)
        return

    # Tarierung anwenden
    data.f_x = data.f_x - offset.f_x
    data.f_y = data.f_y - offset.f_y
    data.f_z = data.f_z - offset.f_z
    data.m_x = data.m_x - offset.m_x
    data.m_y = data.m_y - offset.m_y
    data.m_z = data.m_z - offset.m_z

    # Averaging Fenster weiter ziehen
    old = queue.get()
    queue.put(data)

    # Averaging durchfueren
    average.f_x = average.f_x + (data.f_x - old.f_x) / window_size
    average.f_y = average.f_y + (data.f_y - old.f_y) / window_size
    average.f_z = average.f_z + (data.f_z - old.f_z) / window_size
    average.m_x = average.m_x + (data.m_x - old.m_x) / window_size
    average.m_y = average.m_y + (data.m_y - old.m_y) / window_size
    average.m_z = average.m_z + (data.m_z - old.m_z) / window_size

    # Publishing
    publisher.publish(average)

#############################################
def transceiver():
    rospy.init_node('force_plate_data_transceiver', anonymous=True)

    global publisher
    publisher = rospy.Publisher('Force_plate_data_sma', Force_plate_data, queue_size = 1000)
    rospy.Subscriber("Vicon_compound_data", Vicon_compound_data, callback)

    rospy.loginfo(f"Tranceiver started.")

    rospy.spin()

#############################################
if __name__ == '__main__':
    transceiver()

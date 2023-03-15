#!/usr/bin/env python3

import socket
import struct
import rospy
import numpy as np
import cv2
import pickle
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage


#Server fuer Datenuebertragung erstellen und an IP binden

# TCP_IP = ''
TCP_IP = ''
TCP_PORT = 5005

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((TCP_IP, TCP_PORT))
server_socket.listen(1)

#Variablen

Video_Connection = False
encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
uebertragungsfrequenz = 1/60


#Rospy Publisher einrichten

rospy.init_node('SumControl', anonymous=True)
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
vel_msg = Twist()
rate = rospy.Rate(30)


#Rospy Subscriber mit eigener Klasse zum speichern der Frames einrichten

class CamSubscriber:

    def __init__(self):
        print("Camsubscriber erstellt")
        self.cv2_img = None
        
    def image_callback(self, msg):
        np_image_original = np.frombuffer(msg.data, np.uint8)
        self.cv2_img = cv2.imdecode(np_image_original, cv2.IMREAD_COLOR)

    def frame_return(self):
        return self.cv2_img


camsubscriber = CamSubscriber()
rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, camsubscriber.image_callback)


while True:

    conn, addr = server_socket.accept()
    print("Connection address:", addr)

    VIDEO_IP = addr[0]
    VIDEO_PORT = 5555
    print(VIDEO_IP)

    video_client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    print("Client laeuft")
    

    while True:
        print("test")
        try:
            video_client.connect((VIDEO_IP, VIDEO_PORT))
            print("Videoverbindung hergestellt")
            Video_Connection = True
            break
        except:
            pass


    while True:

        try:
            daten, addr = conn.recvfrom(1024) #Versuch Daten vom Smartphone zu empfangen
        except:
            break
        
        if daten:

            #cmd = struct.unpack('B', bytes(daten[0]))[0] #Der erste Wert des ByteArrays beschreibt die Art der gesendeten Daten (aktuell nur Geschwindigkeit, andere Befehle moeglich)
            cmd = daten[0]
            #print(daten[1])
            

            if cmd == 1: #Geschwindigkeit
                #lVel = float(struct.unpack('B', bytes(daten[1]))[0])
                #aVel = float(struct.unpack('B', bytes(daten[2]))[0])

                lVel = daten[1]
                aVel = daten[2]
                
                lVel = (lVel-128)/50
                aVel = (aVel-128)/10

                vel_msg.linear.x = lVel
                vel_msg.angular.z = aVel

                velocity_publisher.publish(vel_msg)
                rate.sleep()
        
            elif cmd == 2: #Platz fuer andere Befehle, die vom Smartphone gesendet werden
                pass

        elif not daten: break

        #Videouebertragung:
        #Das CompressedImage, das vom Subscriper in der Instanz camsubscriber gespeichert wird, 
        #wird zuerst in ein Numpyarray und anschliessend in ein CV2 Image umgewandelt.
        #Nach dem Encoden als JPG und Umwandeln in Bytecode wird das Bild an das Smartphone gesendet
       

        #np_arr = np.frombuffer(bytes(camsubscriber.frame_return()), np.uint8)
        #img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        frame = cv2.resize(camsubscriber.frame_return(), (780, 360))          #Bildgroesse anpassen
        result, frame1 = cv2.imencode('.jpg', frame, encode_param)
        frame_data = pickle.dumps(frame1, 0)


        message_size = struct.pack(">L", len(frame_data))
        try:
            video_client.sendall(message_size + frame_data)
            
        except:
            break

        time.sleep(uebertragungsfrequenz)


    conn.close()
    video_client.close()
    Video_Connection = False

server_socket.close()
del server_socket


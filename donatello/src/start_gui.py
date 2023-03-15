#!/usr/bin/env python3
# -- TabTess_start_gui.py --
# GUI to control all the Launch Files etc. in the RTC22 course
# edited WHS, OJ , 04.11.2022
# now with Tabs and more options
# usage
# $rosrun rtc start_gui.py
# 

from PyQt5.QtWidgets import (QWidget,
                             QApplication,
                             QPushButton,
                             QHBoxLayout,
                             QVBoxLayout,
                             QLabel,
                             QTabWidget,
                             QLineEdit,
                             QComboBox,
                             QStyle,
                             QFileDialog,
                             )
from PyQt5.QtGui import QPixmap
import sys
import os




class MainWindow(QTabWidget):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        self.mode = 0;

        # https://www.tutorialspoint.com/pyqt/pyqt_qtabwidget.htm

        self.tab1 = QWidget()
        self.tab2 = QWidget()
        self.tab3 = QWidget()
        self.tab4 = QWidget()

        self.addTab(self.tab4, "Tab 4")
        self.addTab(self.tab3, "Tab 3")
        self.addTab(self.tab1, "Tab 1")
        self.addTab(self.tab2, "Tab 2")
        
        self.tab3UI()
        self.tab1UI()
        self.tab2UI()
        self.tab4UI()

        # --- Window konfigurieren und starten
        self.setGeometry(100, 100, 480, 250)
        self.setWindowTitle('RTC22 - Starthilfe ')
        # self.show()

    def tab1UI(self):  # --- Simulation ----
        layout = QVBoxLayout()
        # --- roscore ---
        self.myPb_roscore = QPushButton(self)
        self.myPb_roscore.setText(' ROS-Master ')
        # self.myPb_roscore.setGeometry(10, 10, 300, 40)  # x,y,w,h
        self.myPb_roscore.clicked.connect(self.slot_roscore)
        layout.addWidget(self.myPb_roscore)

        # --- Empty-Gazebo  ---
        self.myPb_gzb_empty = QPushButton(self)
        self.myPb_gzb_empty.setText(' TB3 in Empty-Gazebo_World !')
        # self.myPb_gzb_empty.setGeometry(10, 50, 300, 40)  # x,y,w,h
        self.myPb_gzb_empty.clicked.connect(self.slot_empty_world)
        layout.addWidget(self.myPb_gzb_empty)

        # --- Gazebo-House ---
        self.myPb_gzb_house = QPushButton(self)
        self.myPb_gzb_house.setText(' TB3 in Gazebo_House !')
        # self.myPb_gzb_house.setGeometry(10, 90, 300, 40)  # x,y,w,h
        self.myPb_gzb_house.clicked.connect(self.slot_gazebo_house)
        layout.addWidget(self.myPb_gzb_house)

        # --- Starte RViz ---
        self.myPb_rviz = QPushButton(self)
        self.myPb_rviz.setText(' RViz ')
        # self.myPb_rviz.setGeometry(10, 130, 300, 40)  # x,y,w,h
        self.myPb_rviz.clicked.connect(self.slot_rviz)
        layout.addWidget(self.myPb_rviz)

        # --- Starte Mapping ---
        self.myPb_gmap = QPushButton(self)
        self.myPb_gmap.setText(' Gmapping ')
        # self.myPb_gmap.setGeometry(10, 170, 300, 40)  # x,y,w,h
        self.myPb_gmap.clicked.connect(self.slot_gmapping)
        layout.addWidget(self.myPb_gmap)

        # --- Start Action Server Script ---
        self.myPb_action = QPushButton(self)
        self.myPb_action.setText('Action Server')
        # self.myPb_action.setGeometry(10, 210, 300, 40)  # x,y,w,h
        self.myPb_action.clicked.connect(self.slot_action_server)
        layout.addWidget(self.myPb_action)

        # --- Client ---
        self.myPb_client = QPushButton(self)
        self.myPb_client.setText(' Action Client - path from file')
        # self.myPb_client.setGeometry(10, 250, 300, 40)  # x,y,w,h
        self.myPb_client.clicked.connect(self.slot_action_client)
        layout.addWidget(self.myPb_client)

        self.setTabText(1, "Simulation")
        self.tab1.setLayout(layout)

    def tab2UI(self):  # --- real TB3 ----
        # --- Map Saver Button ---
        self.myPb_map_save = QPushButton(self)
        self.myPb_map_save.setText('Save Map')
        self.myPb_map_save.clicked.connect(self.slot_save_map)
        # --- Navigation Start Button ---
        self.myPb_navigate = QPushButton(self)
        self.myPb_navigate.setText('Navigate to RViz Goal')
        self.myPb_navigate.clicked.connect(self.slot_navigate_to_goal)
        # --- Teleop Start Button ---
        self.myPb_teleop = QPushButton(self)
        self.myPb_teleop.setText(' Teleop Keyboard ')
        self.myPb_teleop.clicked.connect(self.slot_teleop)
        # --- Starte SSH Button ---
        self.myPb_ssh = QPushButton(self)
        self.myPb_ssh.setText('SSH - Start')
        self.myPb_ssh.clicked.connect(self.slot_ssh)

        # --- Grid Layout ---
        vbox = QVBoxLayout()
        # --- 1. Line ---
        hbox1 = QHBoxLayout()
        hbox1.addWidget(QLabel("IP TurtleBot3 im WLAN eintragen"))
        hbox1.addWidget(self.myPb_teleop)
        vbox.addLayout(hbox1)
        # --- 2. Line ---
        hbox2 = QHBoxLayout()
        hbox2.addWidget(QLabel("SSH"))
        hbox2.addWidget(self.myPb_ssh)
        vbox.addLayout(hbox2)
        # --- 3. Line ---
        hbox3 = QHBoxLayout()
        hbox3.addWidget(self.myPb_map_save)
        vbox.addLayout(hbox3)
        # --- 4. Line ---
        hbox4 = QHBoxLayout()
        hbox4.addWidget(self.myPb_navigate)
        vbox.addLayout(hbox4)

        self.setTabText(2, "real TurtleBot3 - SSH")
        self.tab2.setLayout(vbox)

    def tab3UI(self):  # --- Logo --
        self.label = QLabel(" Label ")
        # self.label2 = QLabel("Pixmap?")
        # self.pixmap = QPixmap("/home/oj/catkin_ws/src/rtc/nodes/gui/rtc_logo.png")

        myFilePath = os.path.dirname(os.path.abspath(__file__))
        print(myFilePath)
        pixPath = os.path.join(myFilePath, "rtc_logo_600px.png")
        print(pixPath)

        self.pixmap = QPixmap(pixPath)
        self.label.setPixmap(self.pixmap)  # Funkt bei korrektem Pfad

        self.Line_Edit_IP = QLineEdit("192.168.178.45")
        self.Line_Edit_IP.setFixedSize(120, 26)

        # --- roscore ---
        self.myPb_roscore = QPushButton(self)
        self.myPb_roscore.setText(' starte ROS-Master ')
        self.myPb_roscore.clicked.connect(self.slot_roscore)
        # --- Starte SSH Button ---
        self.myPb_ssh = QPushButton(self)
        self.myPb_ssh.setText('SSH - Start')
        self.myPb_ssh.clicked.connect(self.slot_ssh)

        vbox = QVBoxLayout()

        hbox = QHBoxLayout()
        hbox.addWidget(self.myPb_roscore)
        hbox.addWidget(self.myPb_ssh)
        hbox.addWidget(self.Line_Edit_IP)

        vbox.addLayout(hbox)

        hbox2 = QHBoxLayout()
        hbox2.addWidget(self.label)
        vbox.addLayout(hbox2)

        self.setTabText(3, "Logo")
        self.tab3.setLayout(vbox)

    
    def tab4UI(self):
        # layout = QVBoxLayout(self)
        self.setTabText (0, "Donatello")

        # --- Grid-Layout ----
        vbox = QVBoxLayout()

        # --- Joystick ---
        hbox1 = QHBoxLayout()
        hbox1.addWidget(QLabel("JoyStick Port:"))
        self.portInput = QComboBox()
        self.portInput.addItems(["0","1","2","3","4","5","6","7","8","9"])
        self.portInput.setFixedSize(40,26)
            # evtl. noch automatisches raussuchen der JS hinzufügen
        hbox1.addWidget(self.portInput)

        hbox1.addWidget(QLabel("Modus:"))

        self.modeInput = QComboBox()
        self.modeInput.addItems(["XBox Wireless", "Logitech USB"])
        self.modeInput.setFixedSize(130,26)
        self.modeInput.currentIndexChanged.connect(self.modeChange)
        hbox1.addWidget(self.modeInput)

        myPb_joystick = QPushButton(self)
        myPb_joystick.setText("Joystick")
        myPb_joystick.clicked.connect(self.slot_joystick)
        hbox1.addWidget(myPb_joystick)

        hbox1.addStretch()

        vbox.addLayout(hbox1)

        # --- Navigation with Mapfile

        hbox2 = QHBoxLayout()
        hbox2.addWidget(QLabel("Aktuelle Karte:"))
        vbox.addLayout(hbox2)

        hbox3 = QHBoxLayout()
        self.mapInput = QLineEdit("/home/soren/catkin_ws/src/maps/test3.yaml")
        hbox3.addWidget(self.mapInput)
        vbox.addLayout(hbox3)


        hbox4 = QHBoxLayout()

        myPb_map = QPushButton(self)
        myPb_map.setText("Karte auswählen")
        myPb_map.clicked.connect(self.slot_map)
   
        hbox4.addWidget(myPb_map)

        myPb_nav = QPushButton(self)
        myPb_nav.setText("Navigation")
        myPb_nav.clicked.connect(self.slot_nav)
        hbox4.addWidget(myPb_nav)

        vbox.addLayout(hbox4)

        hbox5 = QHBoxLayout()
        myPb_slam = QPushButton("SLAM")
        myPb_slam.clicked.connect(self.slot_slam)
        hbox5.addWidget(myPb_slam)

        vbox.addLayout(hbox5)

        hbox6 =QHBoxLayout()

        hbox6.addWidget(QLabel("Name:"))
        self.mapName = QLineEdit("map")
        hbox6.addWidget(self.mapName)
        myPb_saveMap = QPushButton("Save Map")
        myPb_saveMap.clicked.connect(self.slot_saveMap)
        hbox6.addWidget(myPb_saveMap)

        vbox.addLayout(hbox6)

        vbox.addStretch()
        
        self.tab4.setLayout(vbox)



    # --- Die  Slot-Methoden ---
    def slot_roscore(self):
        os.system('gnome-terminal --tab -- /bin/bash -c "roscore; exec bash"')

    def slot_empty_world(self):
        os.system('gnome-terminal --tab -- /bin/bash -c \
                  "roslaunch turtlebot3_gazebo\
                  turtlebot3_empty_world.launch;\
                  exec bash"')

    def slot_gazebo_house(self):
        os.system('gnome-terminal --tab -- /bin/bash -c \
                  "roslaunch turtlebot3_gazebo turtlebot3_house.launch;\
                  exec bash"')

    def slot_rviz(self):
        os.system('gnome-terminal --tab -- /bin/bash -c\
                  "roslaunch turtlebot3_gazebo\
                   turtlebot3_gazebo_rviz.launch;\
                  exec bash"')

    def slot_gmapping(self):
        os.system('gnome-terminal --tab -- /bin/bash -c\
                  "roslaunch turtlebot3_slam\
                   turtlebot3_gmapping.launch ;\
                  exec bash"')

    def slot_action_server(self):
        os.system('gnome-terminal --tab -- /bin/bash -c\
                  "rosrun rtc\
                  turtlebot3_server_path.py;\
                  exec bash"')

    def slot_action_client(self):
        os.system('gnome-terminal --tab -- /bin/bash -c\
                  "rosrun rtc\
                  turtlebot3_client_path_from_file.py;\
                  exec bash"')

    def slot_ssh(self):
        # https://manpages.ubuntu.com/manpages/jammy/man1/gnome-terminal.1.html
        # https://wiki.gnome.org/Apps/Terminal
        # gnome String lässt sich nicht zusammenbauen, oder doch?
        # os.system('gnome-terminal --tab -- /bin/bash -c\
        #          "ssh ubuntu@192.168.1.81; exec bash "')
        ip_str = self.Line_Edit_IP.text()
        terminal_str = "gnome-terminal --tab -- /bin/bash -c \"ssh ubuntu@"\
                       + ip_str + ";  exec bash \""
        print(terminal_str)
        os.system(terminal_str)
        # os.system("turtlebot")  # password

    # absoluter Pfad notwendig "~"" does not work, evtl $HOME ?
    def slot_save_map(self):
        os.system('gnome-terminal --tab -- /bin/bash -c\
                  "cd ~/catkin_ws/src/rtc/rtc_maps &&\
                  rosrun map_server map_saver -f \
                  my_map;\
                  exec bash"')

    def slot_navigate_to_goal(self):
        os.system('gnome-terminal --tab -- /bin/bash -c\
                  "cd ~/catkin_ws/src/rtc/rtc_maps &&\
                  roslaunch turtlebot3_navigation\
                  turtlebot3_navigation.launch\
                  map_file:=$HOME/catkin_ws/src/rtc/rtc_maps/my_map.yaml;\
                  exec bash"')
   
    def slot_teleop(self):
        os.system('gnome-terminal -- /bin/bash -c\
                  "rosrun teleop_twist_keyboard teleop_twist_keyboard.py;\
                  exec bash"')

    def slot_teleop(self):
        os.system('gnome-terminal -- /bin/bash -c\
                  "rosrun teleop_twist_keyboard teleop_twist_keyboard.py;\
                  exec bash"')

    def slot_joystick(self):
        
        terminal_str = 'gnome-terminal -- /bin/bash -c\
                    "roslaunch donatello gamepad.launch port:=' + str(self.portInput.currentIndex()) + ' js_mode:=' + str(self.modeInput.currentIndex()) + ';\
                  exec bash"'
       
        os.system(terminal_str)

    def modeChange(self):
        pass

    def slot_map(self):
        self.myFD_map = QFileDialog.getOpenFileName(self, 'Open Map', '//home//soren//catkin_ws//src//maps', "Map File (*.yaml)")
        self.mapInput.setText(self.myFD_map[0])

    def slot_nav(self):
        os.system('gnome-terminal --tab -- /bin/bash -c\
                  "roslaunch turtlebot3_navigation\
                  turtlebot3_navigation.launch\
                  map_file:='+ self.mapInput.text() +';\
                  exec bash"')
        
    def slot_slam(self):
        os.system('gnome-terminal --tab -- /bin/bash -c\
                  "roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping;\
                  exec bash"')
        
    def slot_saveMap(self):
        os.system('gnome-terminal --tab -- /bin/bash -c\
                  "rosrun map_server map_saver -f $HOME/catkin_ws/src/maps/'+ self.mapName.text() +';\
                  exec bash"')
        


if __name__ == '__main__':

    app = QApplication(sys.argv)
    mw = MainWindow()
    mw.show()
    sys.exit(app.exec_())

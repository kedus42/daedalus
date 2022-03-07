#!/usr/bin/env python3
import rospy, sys, rospkg
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QInputDialog
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64

rospy.init_node("basic_gui")

velocity_pub = rospy.Publisher("/daedalus_battery_velocity_controller/command", Float64MultiArray, queue_size=1)
steering_pub = rospy.Publisher("/daedalus_battery_steer_controller/command", Float64, queue_size=1)
left_pub = rospy.Publisher("/daedalus_left_controller/command", Float64MultiArray, queue_size=1)
right_pub = rospy.Publisher("/daedalus_right_controller/command", Float64MultiArray, queue_size=1)
target_pub = rospy.Publisher("/daedalus/target", Float64, queue_size=1)

ext_command = Float64MultiArray()
ext_command.data=([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
ret_command = Float64MultiArray()
ret_command.data=([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
fwd_command = Float64MultiArray()
fwd_command.data=([40])
bwd_command = Float64MultiArray()
bwd_command.data=([-40])
stop_command = Float64MultiArray()
stop_command.data=([0, 0])
right_leg_control = Float64MultiArray()
right_leg_control.data=([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
left_leg_control = Float64MultiArray()
left_leg_control.data=([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
left_drive_pos=0
right_drive_pos=0

def left_pos_update(latest):
    global left_drive_pos
    left_drive_pos=latest.data

def right_pos_update(latest):
    global right_drive_pos
    right_drive_pos=latest.data

right_encoder_sub = rospy.Subscriber("/daedalus/right_drive_pos", Int64, right_pos_update)
left_encoder_sub = rospy.Subscriber("/daedalus/left_drive_pos", Int64, left_pos_update)

rospack=rospkg.RosPack()
path=rospack.get_path("daedalus")

class window(QMainWindow):
    def __init__(self):
        super(window, self).__init__()
        self.setGeometry(400, 150, 400, 490)
        self.setStyleSheet("background : grey")
        self.icon=QtGui.QIcon()
        self.icon.addFile(path+"/imgs/icon.jpg", QtCore.QSize(150,150))
        self.setWindowIcon(self.icon)

        self.a1 = QtWidgets.QPushButton(self)
        self.a1.setGeometry(70, 60, 50, 50)
        self.a1.setText("^")
        self.a1.clicked.connect(self.forward)

        self.a2 = QtWidgets.QPushButton(self)
        self.a2.setGeometry(70, 180, 50, 50)
        self.a2.setText("v")
        self.a2.clicked.connect(self.backward)

        self.a3 = QtWidgets.QPushButton(self)
        self.a3.setGeometry(128, 120, 50, 50)
        self.a3.setText(">")
        self.a3.clicked.connect(self.right)

        self.a4 = QtWidgets.QPushButton(self)
        self.a4.setGeometry(12, 120, 50, 50)
        self.a4.setText("<")
        self.a4.clicked.connect(self.left)

        self.stop_button= QtWidgets.QPushButton(self)
        self.stop_button.setGeometry(70, 120, 50, 50)
        self.stop_button.setText("Stop")
        self.stop_button.clicked.connect(self.stop)

        self.b1 = QtWidgets.QPushButton(self)
        self.b1.setGeometry(12, 260, 175, 60)
        self.b1.setText("Extend left rods")
        self.b1.clicked.connect(self.ext_left)

        self.b2 = QtWidgets.QPushButton(self)
        self.b2.setGeometry(12, 330, 175, 60)
        self.b2.setText("Retract left rods")
        self.b2.clicked.connect(self.ret_left)

        self.b3 = QtWidgets.QPushButton(self)
        self.b3.setGeometry(213, 260, 175, 60)
        self.b3.setText("Extend right rods")
        self.b3.clicked.connect(self.ext_right)

        self.b4 = QtWidgets.QPushButton(self)
        self.b4.setGeometry(213, 330, 175, 60)
        self.b4.setText("Retract right rods")
        self.b4.clicked.connect(self.ret_right)

        self.b7 = QtWidgets.QPushButton(self)
        self.b7.setGeometry(12, 400, 175, 60)
        self.b7.setText("Extend back rods")
        self.b7.clicked.connect(self.ext_back)

        self.b8 = QtWidgets.QPushButton(self)
        self.b8.setGeometry(213, 400, 175, 60)
        self.b8.setText("Extend front rods")
        self.b8.clicked.connect(self.ext_front)

        self.b5 = QtWidgets.QPushButton(self)
        self.b5.setGeometry(213, 80, 175, 30)
        if (rospy.get_param("/controller_on")):
            self.b5.setText("Turn off course control")
        else:
            self.b5.setText("Turn on course control")
        self.b5.clicked.connect(self.toggle_course_control)

        self.b6 = QtWidgets.QPushButton(self)
        self.b6.setGeometry(213, 120, 175, 30)
        self.b6.setText("Set new target")
        self.b6.clicked.connect(self.set_target)

    def forward(self):
        velocity_pub.publish(fwd_command)
    def backward(self):
        velocity_pub.publish(bwd_command)
    def right(self):
        steering_pub.publish(-0.4)
    def left(self):
        steering_pub.publish(0.4)
    def stop(self):
        velocity_pub.publish(stop_command)
        steering_pub.publish(0)

    def ext_right(self):
        right_pub.publish(ext_command)
    def ext_left(self):
        left_pub.publish(ext_command)
    def ret_right(self):
        right_pub.publish(ret_command)
    def ret_left(self):
        left_pub.publish(ret_command)

    def ext_back(self):
        # joint_states=rospy.wait_for_message("/joint_states", JointState)
        # right_drive_pos=joint_states.position[53]
        # left_drive_pos=joint_states.position[54]
        # right_drive_pos=right_drive_pos%6.28
        # left_drive_pos=left_drive_pos%6.28
        # right_drive_pos=int(right_drive_pos*7/6.28)
        # left_drive_pos=int(left_drive_pos*7/6.28)
        iterator = 0
        #int offset = 0;
        #offset = (int((*latest_orientation).y) - LEG_PLANTING_THRESHHOLD)/10;
        while iterator<3:
            if (5+iterator-left_drive_pos>=0):
                rospy.loginfo("#1 left: actuating "+str((5+iterator)*3-left_drive_pos*3)+" through "+str((5+iterator)*3-left_drive_pos*3+2))
                left_leg_control.data[(5+iterator)*3-left_drive_pos*3]=1
                left_leg_control.data[(5+iterator)*3-left_drive_pos*3+1]=1
                left_leg_control.data[(5+iterator)*3-left_drive_pos*3+2]=1
            else:
                rospy.loginfo("#2 left: actuating "+str(8*3+(5+iterator)*3-left_drive_pos*3)+" through "+str(8*3+(5+iterator)*3-left_drive_pos*3+2))
                left_leg_control.data[8*3+((5+iterator)*3-left_drive_pos*3)]=1
                left_leg_control.data[8*3+((5+iterator)*3-left_drive_pos*3)+1]=1
                left_leg_control.data[8*3+((5+iterator)*3-left_drive_pos*3)+2]=1  
            if (5+iterator-right_drive_pos>=0):
                rospy.loginfo("#3 right: actuating "+str((5+iterator)*3-right_drive_pos*3)+" through "+str((5+iterator)*3-right_drive_pos*3+2))
                right_leg_control.data[(5+iterator)*3-right_drive_pos*3]=1
                right_leg_control.data[(5+iterator)*3-right_drive_pos*3+1]=1
                right_leg_control.data[(5+iterator)*3-right_drive_pos*3+2]=1
            else:
                rospy.loginfo("#4 right: actuating "+str(8*3+(5+iterator)*3-right_drive_pos*3)+" through "+str(8*3+(5+iterator)*3-right_drive_pos*3+2))
                right_leg_control.data[8*3+((5+iterator)*3-right_drive_pos*3)]=1
                right_leg_control.data[8*3+((5+iterator)*3-right_drive_pos*3)+1]=1
                right_leg_control.data[8*3+((5+iterator)*3-right_drive_pos*3)+2]=1  
            iterator+=1
        right_pub.publish(right_leg_control)
        left_pub.publish(left_leg_control)
        iterator=0
        while iterator<24:
            right_leg_control.data[iterator]=0
            left_leg_control.data[iterator]=0
            iterator+=1

    def ext_front(self):
        # joint_states=rospy.wait_for_message("/joint_states", JointState)
        # right_drive_pos=joint_states.position[53]
        # left_drive_pos=joint_states.position[54]
        # right_drive_pos=right_drive_pos%6.28
        # left_drive_pos=left_drive_pos%6.28
        # right_drive_pos=int(right_drive_pos*7/6.28)
        # left_drive_pos=int(left_drive_pos*7/6.28)
        iterator = 0
        #int offset = 0;
        #offset = (int((*latest_orientation).y) - LEG_PLANTING_THRESHHOLD)/10;
        while iterator<3 :
            if (1+iterator-left_drive_pos>=0):
                rospy.loginfo("#5 left: actuating "+str((1+iterator)*3-left_drive_pos*3)+" through "+str((1+iterator)*3-left_drive_pos*3+2))
                left_leg_control.data[(1+iterator)*3-left_drive_pos*3]=1
                left_leg_control.data[(1+iterator)*3-left_drive_pos*3+1]=1
                left_leg_control.data[(1+iterator)*3-left_drive_pos*3+2]=1
            else:
                rospy.loginfo("#6 left: actuating "+str(8*3+(1+iterator)*3-left_drive_pos*3)+" through "+str(8*3+(1+iterator)*3-left_drive_pos*3+2))
                left_leg_control.data[8*3+((1+iterator)*3-left_drive_pos*3)]=1
                left_leg_control.data[8*3+((1+iterator)*3-left_drive_pos*3)+1]=1
                left_leg_control.data[8*3+((1+iterator)*3-left_drive_pos*3)+2]=1  
            if (1+iterator-right_drive_pos>=0):
                rospy.loginfo("#7 right: actuating "+str((1+iterator)*3-right_drive_pos*3)+" through "+str((1+iterator)*3-right_drive_pos*3+2))
                right_leg_control.data[(1+iterator)*3-right_drive_pos*3]=1
                right_leg_control.data[(1+iterator)*3-right_drive_pos*3+1]=1
                right_leg_control.data[(1+iterator)*3-right_drive_pos*3+2]=1
            else:
                rospy.loginfo("#8 right: actuating "+str(8*3+(1+iterator)*3-right_drive_pos*3)+" through "+str(8*3+(1+iterator)*3-right_drive_pos*3+2))
                right_leg_control.data[8*3+((1+iterator)*3-right_drive_pos*3)]=1
                right_leg_control.data[8*3+((1+iterator)*3-right_drive_pos*3)+1]=1
                right_leg_control.data[8*3+((1+iterator)*3-right_drive_pos*3)+2]=1  
            iterator+=1
        right_pub.publish(right_leg_control)
        left_pub.publish(left_leg_control)
        iterator=0
        while iterator<24:
            right_leg_control.data[iterator]=0
            left_leg_control.data[iterator]=0
            iterator+=1

    def toggle_course_control(self):
        if (rospy.get_param("/controller_on")):
            rospy.set_param("/controller_on", False)
            self.b5.setText("Turn on course control")
        else:
            rospy.set_param("/controller_on", True)
            self.b5.setText("Turn off course control")

    def set_target(self):
        self.target_dialogue=QInputDialog(self)
        new_input=False
        new_target, new_input=self.target_dialogue.getDouble(self, "New target", "Enter a target relative to the sphere", 0, -180, 180, 0)
        if new_input:
            target_pub.publish(new_target)
    
app=QApplication(sys.argv)
win=window()
win.show()
sys.exit(app.exec_())
rospy.spin()
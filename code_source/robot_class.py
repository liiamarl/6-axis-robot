from position_kinematics import *
from frames import *
from dynamixel_control import *
import math as mt
import numpy as np
import random as rd
import time as t
import keyboard as kb


def angles_to_dynamixel(angles):
    # converts joint angles (-150 to 150 degrees) into dynamixel servos position (0 to 1023)
    dynamixel_pos = np.array([0, 0, 0, 0, 0, 0])
    dynamixel_pos[0] = angles[0] * 1024 / 300 + 512.5
    dynamixel_pos[1] = 2048.5 - (angles[1] -65.62) * 4096 / 360
    dynamixel_pos[2] = (angles[2] + 60) * 1024 / 300 + .5
    dynamixel_pos[3] = (angles[3] + 45) * 4096 / 360 + 2048.5
    dynamixel_pos[4] = 2048.5 - angles[4] * 4096 / 360
    dynamixel_pos[5] = angles[5] * 4096 / 360 + 2048.5

    return dynamixel_pos

def dynamixel_to_angles(dynamixel_pos):
    angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    angles[0] = 1.0 * (dynamixel_pos[0] - 512) * 300 / 1024
    angles[1] = 1.0 * (2048 - dynamixel_pos[1]) * 360 / 4096 + 65.62
    angles[2] = 1.0 * dynamixel_pos[2] * 300 / 1024 - 60
    angles[3] = 1.0 * (dynamixel_pos[3] - 2048) * 360 / 4096 - 45
    angles[4] = 1.0 * (2048 - dynamixel_pos[4]) * 360 / 4096
    angles[5] = 1.0 * (dynamixel_pos[5] - 2048) * 360 / 4096

    return angles

def distance(pos1, pos2):
    vector_1_2 = [pos2.tool_frame[0][0] - pos1.tool_frame[0][0], pos2.tool_frame[0][1] - pos1.tool_frame[0][1], pos2.tool_frame[0][2] - pos1.tool_frame[0][2]]
    norm = mt.sqrt(pow(vector_1_2[0], 2) + pow(vector_1_2[1], 2) + pow(vector_1_2[2], 2))
    return norm
    

class robot:

    def get_dynamixel_pos(self):
        #asks to every dynamixel their position and returns it
        return [get_servo_position(1), get_servo_position(2), get_servo_position(3), get_servo_position(4), get_servo_position(5), get_servo_position(6)]

    def update_current_pos(self):
        self.current_position = position(dynamixel_to_angles(self.get_dynamixel_pos()), self.TCP_offset, self.user_frame)


    def go_to_joint(self, target, bypass = False):
        #makes a joint motion from current position to target
        
        if target.position_ok or bypass:
            self.goal_position = target
            dynamixel_pos = angles_to_dynamixel(target.joint_angles)
            for i in range(6):
                move_servo(i + 1, dynamixel_pos[i])
        else:
            print("POSITION NOT OK")
            
    def go_to_joint_timed(self, target, time, bypass = False):
        self.update_current_pos()
        for j in range(1, 7):
            setup_speed(j, int(mt.fabs(self.current_position.joint_angles[j - 1] - target.joint_angles[j - 1]) / time))
        self.go_to_joint(target, bypass)
    
    def get_program_manual(self):
        disable_torque()
        program = list()
        while True:
            print ("press p to record a position or q to end program")
            if kb.read_key() == "p":
                self.update_current_pos()
                program.append(position(self.current_position.joint_angles, self.TCP_offset, self.user_frame))
                print("Position enregistrée")
            if kb.read_key() == "q":
                print("programme terminé")
                break
        return program
    
    def run_program_joint(self, program_pos, time, precision):
        enable_torque()
        for pos in program_pos:
            self.go_to_joint_timed(pos, time, True)
            self.update_current_pos()
            while distance(pos, self.current_position) > precision:
                self.update_current_pos()
                sleep(.1)
                print(self.current_position.tool_frame[0])
            
    
    

    def get_linear_path(self, pos1, pos2, resolution):
        # returns a list of positions on a line from pos1 to pos2 with a distance of resolution between them
        #pos1 and pos2 must have been created with the same tool offset and user frame
        vector_1_2 = [pos2.tool_frame[0][0] - pos1.tool_frame[0][0], pos2.tool_frame[0][1] - pos1.tool_frame[0][1], pos2.tool_frame[0][2] - pos1.tool_frame[0][2]]
        norm = mt.sqrt(pow(vector_1_2[0], 2) + pow(vector_1_2[1], 2) + pow(vector_1_2[2], 2))
        n_step = int(norm / resolution)
        vector_1_2[0] /= n_step
        vector_1_2[1] /= n_step
        vector_1_2[2] /= n_step
        linear_path = list([pos1])
        for i in range(1, n_step):
            coordinates = np.array([[pos1.tool_frame[0][0] + i *  vector_1_2[0], pos1.tool_frame[0][1] + i * vector_1_2[1], pos1.tool_frame[0][2] + i * vector_1_2[2]], pos1.tool_frame[1]])
            linear_path.append(position(coordinates, pos1.tool_offset, pos1.frame_offset))
        linear_path.append(pos2)
        return linear_path 

    def go_to_linear(self, target, resolution):
        #makes a linear motion from current position to target
        #just makes a lot of small joint movements along a linear path, setting movement n+1 just before movement n is completed
        pass

    def open_gripper(self):
        pass

    def close_gripper(self):
        pass


    def set_TCP(self, TCP_offset):
        self.TCP_offset = TCP_offset

    def set_user_frame(self, frame):
        self.user_frame = frame




    def __init__(self):
        setup_ports()
        self.user_frame = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]) #default is world frame
        self.TCP_offset = np.array([0.0, 0.0, 50.0]) #default is wrist mounting point
        self.update_current_pos()
        self.current_motors_speed = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        self.goal_position = self.current_position
        

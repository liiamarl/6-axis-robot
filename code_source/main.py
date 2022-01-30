from robot_class import *
import keyboard as kb
from time import sleep
import numpy as np





def move_routine(robot, epsilon, theta, time):
    time_c = .2
    if kb.is_pressed("x") and kb.is_pressed("+"):
        new_pos = position(np.add(robot.goal_position.tool_frame, np.array([[epsilon, 0.0, 0.0], [0.0, 0.0, 0.0]])), robot.TCP_offset, robot.user_frame)
        robot.go_to_joint_timed(new_pos, time)
        sleep(time - time_c)
    if kb.is_pressed("x") and kb.is_pressed("-"):
        new_pos = position(np.add(robot.goal_position.tool_frame, np.array([[-epsilon, 0.0, 0.0], [0.0, 0.0, 0.0]])), robot.TCP_offset, robot.user_frame)
        robot.go_to_joint_timed(new_pos, time)
        sleep(time - time_c)
    if kb.is_pressed("y") and kb.is_pressed("+"):
        new_pos = position(np.add(robot.goal_position.tool_frame, np.array([[0.0, epsilon, 0.0], [0.0, 0.0, 0.0]])), robot.TCP_offset, robot.user_frame)
        robot.go_to_joint_timed(new_pos, time)
        sleep(time - time_c)
    if kb.is_pressed("y") and kb.is_pressed("-"):
        new_pos = position(np.add(robot.goal_position.tool_frame, np.array([[0.0, -epsilon, 0.0], [0.0, 0.0, 0.0]])), robot.TCP_offset, robot.user_frame)
        robot.go_to_joint_timed(new_pos, time)
        sleep(time - time_c)
    if kb.is_pressed("z") and kb.is_pressed("+"):
        new_pos = position(np.add(robot.goal_position.tool_frame, np.array([[0.0, 0.0, epsilon], [0.0, 0.0, 0.0]])), robot.TCP_offset, robot.user_frame)
        robot.go_to_joint_timed(new_pos, time)
        sleep(time - time_c)
    if kb.is_pressed("z") and kb.is_pressed("-"):
        new_pos = position(np.add(robot.goal_position.tool_frame, np.array([[0.0, 0.0, -epsilon], [0.0, 0.0, 0.0]])), robot.TCP_offset, robot.user_frame)
        robot.go_to_joint_timed(new_pos, time)
        sleep(time - time_c)
    if kb.is_pressed("1") and kb.is_pressed("+"):
        new_pos = position(np.add(robot.goal_position.joint_angles, np.array([theta, 0.0, 0.0, 0.0, 0.0, 0.0])), robot.TCP_offset, robot.user_frame)
        robot.go_to_joint_timed(new_pos, time)
        sleep(time - time_c)
    if kb.is_pressed("1") and kb.is_pressed("-"):
        new_pos = position(np.add(robot.goal_position.joint_angles, np.array([-theta, 0.0, 0.0, 0.0, 0.0, 0.0])), robot.TCP_offset, robot.user_frame)
        robot.go_to_joint_timed(new_pos, time)
        sleep(time - time_c)
    if kb.is_pressed("2") and kb.is_pressed("+"):
        new_pos = position(np.add(robot.goal_position.joint_angles, np.array([0.0, theta, 0.0, 0.0, 0.0, 0.0])), robot.TCP_offset, robot.user_frame)
        robot.go_to_joint_timed(new_pos, time)
        sleep(time - time_c)
    if kb.is_pressed("2") and kb.is_pressed("-"):
        new_pos = position(np.add(robot.goal_position.joint_angles, np.array([0.0, -theta, 0.0, 0.0, 0.0, 0.0])), robot.TCP_offset, robot.user_frame)
        robot.go_to_joint_timed(new_pos, time)
        sleep(time - time_c)
    if kb.is_pressed("3") and kb.is_pressed("+"):
        new_pos = position(np.add(robot.goal_position.joint_angles, np.array([0.0, 0.0, theta, 0.0, 0.0, 0.0])), robot.TCP_offset, robot.user_frame)
        robot.go_to_joint_timed(new_pos, time)
        sleep(time - time_c)
    if kb.is_pressed("3") and kb.is_pressed("-"):
        new_pos = position(np.add(robot.goal_position.joint_angles, np.array([0.0, 0.0, -theta, 0.0, 0.0, 0.0])), robot.TCP_offset, robot.user_frame)
        robot.go_to_joint_timed(new_pos, time)
        sleep(time - time_c)
    if kb.is_pressed("4") and kb.is_pressed("+"):
        new_pos = position(np.add(robot.goal_position.joint_angles, np.array([0.0, 0.0, 0.0, theta, 0.0, 0.0])), robot.TCP_offset, robot.user_frame)
        robot.go_to_joint_timed(new_pos, time)
        sleep(time - time_c)
    if kb.is_pressed("4") and kb.is_pressed("-"):
        new_pos = position(np.add(robot.goal_position.joint_angles, np.array([0.0, 0.0, 0.0, -theta, 0.0, 0.0])), robot.TCP_offset, robot.user_frame)
        robot.go_to_joint_timed(new_pos, time)
        sleep(time - time_c)
    if kb.is_pressed("5") and kb.is_pressed("+"):
        new_pos = position(np.add(robot.goal_position.joint_angles, np.array([0.0, 0.0, 0.0, 0.0, theta, 0.0])), robot.TCP_offset, robot.user_frame)
        robot.go_to_joint_timed(new_pos, time)
        sleep(time - time_c)
    if kb.is_pressed("5") and kb.is_pressed("-"):
        new_pos = position(np.add(robot.goal_position.joint_angles, np.array([0.0, 0.0, 0.0, 0.0, -theta, 0.0])), robot.TCP_offset, robot.user_frame)
        robot.go_to_joint_timed(new_pos, time)
        sleep(time - time_c)
    if kb.is_pressed("6") and kb.is_pressed("+"):
        new_pos = position(np.add(robot.goal_position.joint_angles, np.array([0.0, 0.0, 0.0, 0.0, 0.0, theta])), robot.TCP_offset, robot.user_frame)
        robot.go_to_joint_timed(new_pos, time)
        sleep(time - time_c)
    if kb.is_pressed("6") and kb.is_pressed("-"):
        new_pos = position(np.add(robot.goal_position.joint_angles, np.array([0.0, 0.0, 0.0, 0.0, 0.0, -theta])), robot.TCP_offset, robot.user_frame)
        robot.go_to_joint_timed(new_pos, time)
        sleep(time - time_c)
        
    

setup_ports()
setup_regul_parameters()
RB = robot()
program = RB.get_program_manual()
RB.run_program_joint(program, .5, 7)
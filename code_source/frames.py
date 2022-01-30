from position_kinematics import *
import numpy as np
import math as mt
import random as rd



def get_TCP(p1, p2, p3, p4):
    #p1, p2, p3, p4 must be taken with a tool offset of [0, 0, 0]

    l1 = np.array([2 * (p1.wrist_position[0] - p2.wrist_position[0]), 2 * (p1.wrist_position[1] - p2.wrist_position[1]), 2 * (p1.wrist_position[2] - p2.wrist_position[2])])
    l2 = np.array([2 * (p2.wrist_position[0] - p3.wrist_position[0]), 2 * (p2.wrist_position[1] - p3.wrist_position[1]), 2 * (p2.wrist_position[2] - p3.wrist_position[2])])
    l3 = np.array([2 * (p3.wrist_position[0] - p4.wrist_position[0]), 2 * (p3.wrist_position[1] - p4.wrist_position[1]), 2 * (p3.wrist_position[2] - p4.wrist_position[2])])
    system_matrix = np.array([l1, l2, l3])
    system_vector = np.array([0.0, 0.0, 0.0])
    system_vector[0] = mt.pow(p1.wrist_position[0], 2) + mt.pow(p1.wrist_position[1], 2) + mt.pow(p1.wrist_position[2], 2) - mt.pow(p2.wrist_position[0], 2) - mt.pow(p2.wrist_position[1], 2) - mt.pow(p2.wrist_position[2], 2)
    system_vector[1] = mt.pow(p2.wrist_position[0], 2) + mt.pow(p2.wrist_position[1], 2) + mt.pow(p2.wrist_position[2], 2) - mt.pow(p3.wrist_position[0], 2) - mt.pow(p3.wrist_position[1], 2) - mt.pow(p3.wrist_position[2], 2)
    system_vector[2] = mt.pow(p3.wrist_position[0], 2) + mt.pow(p3.wrist_position[1], 2) + mt.pow(p3.wrist_position[2], 2) - mt.pow(p4.wrist_position[0], 2) - mt.pow(p4.wrist_position[1], 2) - mt.pow(p4.wrist_position[2], 2)

    center_point = np.linalg.solve(system_matrix, system_vector)
    center_point_in_tool = p1.world_to_tool(np.array([center_point, [0.0, 0.0, 0.0]]))
    return center_point_in_tool[0]






def get_frame_offset(p1, p2, p3):
    #p1, p2, p3 must be taken in world frame, so frame offset [[0, 0, 0], [0, 0, 0]]
    x_vector = np.array([p2[0][0] - p1[0][0], p2[0][1] - p1[0][1], p2[0][2] - p1[0][2]])
    #normalisation of x
    x_norm = mt.sqrt(mt.pow(x_vector[0], 2) + mt.pow(x_vector[1], 2) + mt.pow(x_vector[2], 2))
    x_vector[0] /= x_norm
    x_vector[1] /= x_norm
    x_vector[2] /= x_norm

    y_vector = np.array([p3[0][0] - p1[0][0], p3[0][1] - p1[0][1], p3[0][2] - p1[0][2]])
    y_norm = mt.sqrt(mt.pow(y_vector[0], 2) + mt.pow(y_vector[1], 2) + mt.pow(y_vector[2], 2))
    #not the actual y vector, but in the same plane and not collinear to x
    #let alpha be the angle between x and y, we can get it using scalar ptoduct formula
    alpha = mt.acos((x_vector[0] * y_vector[0] + x_vector[1] * y_vector[1] + x_vector[2] * y_vector[2]) / y_norm)
    #we can now get the projection of y on x
    projected_y = np.array([x_vector[0] * mt.cos(alpha) * y_norm, x_vector[1] * mt.cos(alpha) * y_norm, x_vector[2] * mt.cos(alpha) * y_norm])
    #we can now get the y vector orthogonal to x and normalise it
    y_vector[0] -= projected_y[0]
    y_vector[1] -= projected_y[1]
    y_vector[2] -= projected_y[2]
    y_norm = mt.sqrt(mt.pow(y_vector[0], 2) + mt.pow(y_vector[1], 2) + mt.pow(y_vector[2], 2))
    y_vector[0] /= y_norm
    y_vector[1] /= y_norm
    y_vector[2] /= y_norm
    #we now have the x and y vectors coordinates in the world frame
    #the z vector is a 90 degrees rotation of y around x
    rot_matrix = np.array([[pow(x_vector[0], 2), x_vector[0] * x_vector[1] - x_vector[2], x_vector[0] * x_vector[2] + x_vector[1]], [x_vector[0] * x_vector[1] + x_vector[2], pow(x_vector[1], 2), x_vector[1] * x_vector[2] - x_vector[0]], [x_vector[0] * x_vector[2] - x_vector[1], x_vector[1] * x_vector[2] + x_vector[0], pow(x_vector[2], 2)]])
    z_vector = np.array([0.0, 0.0, 0.0])
    z_vector[0] = y_vector[0] * rot_matrix[0][0] + y_vector[1] * rot_matrix[0][1] + y_vector[2] * rot_matrix[0][2]
    z_vector[1] = y_vector[0] * rot_matrix[1][0] + y_vector[1] * rot_matrix[1][1] + y_vector[2] * rot_matrix[1][2]
    z_vector[2] = y_vector[0] * rot_matrix[2][0] + y_vector[1] * rot_matrix[2][1] + y_vector[2] * rot_matrix[2][2]
    #z should already be normalised


    frame_rot_matrix = np.array([[x_vector[0], y_vector[0], z_vector[0]], [x_vector[1], y_vector[1], z_vector[1]], [x_vector[2], y_vector[2], z_vector[2]]])
    return np.array([p1[0], get_euler_angles(frame_rot_matrix)])

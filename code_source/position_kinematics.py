import numpy as np
import math as mt
import random as rd

def get_euler_angles(rot_matrix):
    #gives the euler angles in ZYZ configuration
    euler_angles = np.array([0.0, 0.0, 0.0])

    euler_angles[0] = mt.degrees(mt.atan2(rot_matrix[1][2], rot_matrix[0][2]))

    cos1 = mt.cos(mt.radians(euler_angles[0]))
    sin1 = mt.sin(mt.radians(euler_angles[0]))

    if mt.fabs(rot_matrix[2][2]) < 0.0000001:
        euler_angles[1] = 90
    else :
        euler_angles[1] = mt.degrees(mt.atan2(cos1 * rot_matrix[0][2] + sin1 * rot_matrix[1][2], rot_matrix[2][2]))
    if mt.fabs(euler_angles[1]) < 0.000000001:
        #singularity
        euler_angles[0] = 0
        euler_angles[2] = mt.degrees(mt.acos(rot_matrix[0][0]))
        return euler_angles
    if mt.fabs(euler_angles[1] - 180) < 0.000000001:
        #singularity

        euler_angles[0] = 0
        if mt.fabs(mt.fabs(rot_matrix[0][0]) - 1) < 0.000000001:
            euler_angles[2] = 0
            return euler_angles
        euler_angles[2] = mt.degrees(mt.acos(-rot_matrix[0][0]))
        return euler_angles

    euler_angles[2] = mt.degrees(mt.atan2(-sin1 * rot_matrix[0][0] + cos1 * rot_matrix[1][0], -sin1 * rot_matrix[0][1] + cos1 * rot_matrix[1][1]))
    if euler_angles[1] < -0.000000001:
        euler_angles[1] *= -1
        if euler_angles[0] < 0:
            euler_angles[0] += 180
        else :
            euler_angles[0] -= 180
        if euler_angles[2] < 0:
            euler_angles[2] += 180
        else :
            euler_angles[2] -= 180

    return euler_angles

def get_rot_matrix(euler_angles):
    cos = np.array([0.0, 0.0, 0.0])
    sin = np.array([0.0, 0.0, 0.0])
    for i in range(3):
        cos[i] = mt.cos(mt.radians(euler_angles[i]))
        sin[i] = mt.sin(mt.radians(euler_angles[i]))
    return np.array([[cos[0] * cos[1] * cos[2] - sin[0] * sin[2], -cos[2] * sin[0] - cos[0] * cos[1] * sin[2], cos[0] * sin[1]], [cos[0] * sin[2] + cos[1] * cos[2] * sin[0], cos[0] * cos[2] - cos[1] * sin[0] * sin[2], sin[0] * sin[1]], [-cos[2] * sin[1], sin[1] * sin[2], cos[1]]])

def get_transition_matrix(position):
    rot_matrix = get_rot_matrix(position[1])
    return np.array([np.append(rot_matrix[0], position[0][0]), np.append(rot_matrix[1], position[0][1]), np.append(rot_matrix[2], position[0][2]), [0, 0, 0, 1]])

class position:

    def world_to_frame(self, position):
        # defining the offset between frame origin and position in world
        offset_F_P_in_world = np.array([position[0][0] - self.frame_offset[0][0], position[0][1] - self.frame_offset[0][1], position[0][2] - self.frame_offset[0][2]])
        # defining the offset between frame origin and position in frame
        offset_F_P_in_frame = np.dot(offset_F_P_in_world, self.rot_matrix_W_F)
        #defining rotation matrix from frame to position
        main_matrix = np.dot(self.rot_matrix_F_W, get_rot_matrix(position[1]))

        return np.array([offset_F_P_in_frame, get_euler_angles(main_matrix)])

    def frame_to_world(self, position):
        #defining the rotation matrix from world to position
        main_matrix = np.dot(self.rot_matrix_W_F, get_rot_matrix(position[1]))
        #defining the offset from frame to point in world
        offset_F_P_in_world = np.dot(position[0], self.rot_matrix_F_W)
        #defining the offset from world to position in world
        offset_W_P_in_world = np.array([offset_F_P_in_world[0] + self.frame_offset[0][0], offset_F_P_in_world[1] + self.frame_offset[0][1], offset_F_P_in_world[2] + self.frame_offset[0][2]])

        return np.array([offset_W_P_in_world, get_euler_angles(main_matrix)])

    def world_to_tool(self, position):
        tr1 = get_transition_matrix(position)
        tr2 = np.dot(np.linalg.inv(self.main_matrix), tr1)
        return np.array([[tr2[0][3], tr2[1][3], tr2[2][3]], get_euler_angles(tr2)])

    def tool_to_world(self, position):
        tr1 = get_transition_matrix(position)
        tr2 = np.dot(self.main_matrix, tr1)
        return np.array([[tr2[0][3], tr2[1][3], tr2[2][3]], get_euler_angles(tr2)])

    def crash_check(self):
        #basic collision avoidance function, far from covering all cases but better than nothing
        #must be called at the end of both kinematics functions
        #first checking if the joints angles are in range of motion :
        joint_limits = np.array([[-90, 90], [-20, 110], [10, 130], [-180, 180], [-80, 100], [-180, 180]])
        for i in range(6):
            if self.joint_angles[i] < joint_limits[i][0] or self.joint_angles[i] > joint_limits[i][1]:
                return True
        # now checking if wrist position is to close from robot base
        wrist_base_distance = mt.sqrt(mt.pow(self.wrist_position[0], 2) + mt.pow(self.wrist_position[1], 2))
        if self.wrist_position[2] < 0 and wrist_base_distance < 70:
            return True

        #same thing for the tool position
        tool_base_distance = mt.sqrt(mt.pow(self.tool_frame_in_world[0][0], 2) + mt.pow(self.tool_frame_in_world[0][1], 2))
        if self.tool_frame_in_world[0][2] < 0 and tool_base_distance < 70:
            return True
        if self.tool_frame_in_world[0][2] >= 0 and tool_base_distance < 40:
            return True
        return False


    def forward_kinematics(self):
        #defining cos and sin of joint angles
        angle_cos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        angle_sin = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        for i in range(6):
            angle_cos[i] = mt.cos(mt.radians(self.joint_angles[i]))
            angle_sin[i] = mt.sin(mt.radians(self.joint_angles[i]))
        #defining homogenous transition matrix for every joint
        #the last one has the tool offset built-in
        tr_matrix_0 = np.array([[angle_cos[0], -angle_sin[0], 0.0, 0.0], [angle_sin[0], angle_cos[0], 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        tr_matrix_1 = np.array([[angle_cos[1], 0.0, angle_sin[1], angle_sin[1] * 80], [0.0, 1.0, 0.0, 0.0], [-angle_sin[1], 0.0, angle_cos[1], angle_cos[1] * 80], [0.0, 0.0, 0.0, 1.0]])
        tr_matrix_2 = np.array([[angle_cos[2], 0.0, angle_sin[2], angle_sin[2] * 95.5], [0.0, 1.0, 0.0, 0.0], [-angle_sin[2], 0.0, angle_cos[2], angle_cos[2] * 95.5], [0.0, 0.0, 0.0, 1.0]])
        tr_matrix_3 = np.array([[angle_cos[3], -angle_sin[3], 0.0, 0.0], [angle_sin[3], angle_cos[3], 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        tr_matrix_4 = np.array([[angle_cos[4], 0.0, angle_sin[4], 0.0], [0.0, 1.0, 0.0, 0.0], [-angle_sin[4], 0.0, angle_cos[4], 0.0], [0.0, 0.0, 0.0, 1.0]])
        tr_matrix_5 = np.array([[angle_cos[5], -angle_sin[5], 0.0, self.tool_offset[0] * angle_cos[5] - self.tool_offset[1] * angle_sin[5]], [angle_sin[5], angle_cos[5], 0.0, angle_sin[5] * self.tool_offset[0] + angle_cos[5] * self.tool_offset[1]], [0.0, 0.0, 1.0, self.tool_offset[2]], [0.0, 0.0, 0.0, 1.0]])

        #multiplying transition matrix in order to get wrist and tool position
        self.main_matrix = np.dot(tr_matrix_0, tr_matrix_1)
        self.main_matrix = np.dot(self.main_matrix, tr_matrix_2)
        #at this point main matrix is the transition matrix from world to the wrist
        self.wrist_position = np.array([self.main_matrix[0][3], self.main_matrix[1][3], self.main_matrix[2][3]])
        self.main_matrix = np.dot(self.main_matrix, tr_matrix_3)
        self.main_matrix = np.dot(self.main_matrix, tr_matrix_4)
        self.main_matrix = np.dot(self.main_matrix, tr_matrix_5)
        self.tool_frame_in_world = np.array([[self.main_matrix[0][3], self.main_matrix[1][3], self.main_matrix[2][3]], get_euler_angles(self.main_matrix)])
        #getting from world to frame
        self.tool_frame = self.world_to_frame(self.tool_frame_in_world)
        #forward kinematics is now solved !
        return not self.crash_check()

    def inverse_kinematics(self):
        self.joint_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        #converting tool frame from frame to world
        self.tool_frame_in_world = self.frame_to_world(self.tool_frame)
        #getting wrist position from tool orientation and offset
        self.main_matrix = get_transition_matrix(self.tool_frame_in_world)
        rot_matrix = get_rot_matrix(self.tool_frame_in_world[1])
        self.wrist_position = np.array(self.tool_frame_in_world[0])
        self.wrist_position[0] -= self.tool_offset[0] * self.main_matrix[0][0] + self.tool_offset[1] * self.main_matrix[0][1] + self.tool_offset[2] * self.main_matrix[0][2]
        self.wrist_position[1] -= self.tool_offset[0] * self.main_matrix[1][0] + self.tool_offset[1] * self.main_matrix[1][1] + self.tool_offset[2] * self.main_matrix[1][2]
        self.wrist_position[2] -= self.tool_offset[0] * self.main_matrix[2][0] + self.tool_offset[1] * self.main_matrix[2][1] + self.tool_offset[2] * self.main_matrix[2][2]
        #now that we know the wrist position, we can get the first three joints position from basic trigonometry
        #the numerical values come from the robot's dimentions
        if mt.fabs(self.wrist_position[0]) < 0.00001 and mt.fabs(self.wrist_position[1]) < 0.00001:
            #singularity, just set joint 0 to 0degrees
            self.joint_angles[0] = 0.0
        else:
            self.joint_angles[0] = mt.degrees(mt.atan(self.wrist_position[1] / self.wrist_position[0]))
            #only works if wrist_position[0] > 0 but that's ok for me
        square_sum = mt.pow(self.wrist_position[0], 2) + mt.pow(self.wrist_position[1], 2) + mt.pow(self.wrist_position[2], 2)
        if mt.sqrt(square_sum) > 175.5:
            #position is not possible to reach
            return False
        if self.wrist_position[2] >= 0:
            #formula is a bit different depending if the wrist center is above world origin or not
            self.joint_angles[1] = 90 - mt.degrees(mt.acos(mt.sqrt(square_sum - mt.pow(self.wrist_position[2], 2)) / mt.sqrt(square_sum))) - mt.degrees(mt.acos((square_sum - 2720.25) / (160 * mt.sqrt(square_sum))))
        else :
            self.joint_angles[1] = 90 + mt.degrees(mt.acos(mt.sqrt(square_sum - mt.pow(self.wrist_position[2], 2)) / mt.sqrt(square_sum))) - mt.degrees(mt.acos((square_sum - 2720.25) / (160 * mt.sqrt(square_sum))))
        self.joint_angles[2] = 180 - mt.degrees(mt.acos((15520.25 - square_sum) / 15280))
        #inverse kinematics for positioning of the wrist is now solved.
        #we can get rotation matrix from world to wrist (rot_matrix_W_3)
        angle_cos = np.array([mt.cos(mt.radians(self.joint_angles[0])), mt.cos(mt.radians(self.joint_angles[1])), mt.cos(mt.radians(self.joint_angles[2]))])
        angle_sin = np.array([mt.sin(mt.radians(self.joint_angles[0])), mt.sin(mt.radians(self.joint_angles[1])), mt.sin(mt.radians(self.joint_angles[2]))])
        rot_matrix_0 = np.array([[angle_cos[0], -angle_sin[0], 0.0], [angle_sin[0], angle_cos[0], 0.0], [0.0, 0.0, 1.0]])
        rot_matrix_1 = np.array([[angle_cos[1], 0.0, angle_sin[1]], [0.0, 1.0, 0.0], [-angle_sin[1], 0.0, angle_cos[1]]])
        rot_matrix_2 = np.array([[angle_cos[2], 0.0, angle_sin[2]], [0.0, 1.0, 0.0], [-angle_sin[2], 0.0, angle_cos[2]]])
        rot_matrix_W_3 = np.dot(rot_matrix_0, rot_matrix_1)
        rot_matrix_W_3 = np.dot(rot_matrix_W_3, rot_matrix_2)
        #rot_matrix_world_tool = rot_matrix_World_wrist * rot_matrix_wrist_Tool
        #so rot_matrix_wrist_tool = invert_matrix(rot_matrix_World_wrist) * rot_matrix_world_tool
        #and rotation matrix between wrist and tool is what we want
        rot_matrix_3_T = np.linalg.inv(rot_matrix_W_3)
        rot_matrix_3_T = np.dot(rot_matrix_3_T, rot_matrix)
        #now we can get the euler angles, witch are exactly the joint angles
        wrist_euler_angles = get_euler_angles(rot_matrix_3_T)
        self.joint_angles[3] = wrist_euler_angles[0]
        self.joint_angles[4] = wrist_euler_angles[1]
        self.joint_angles[5] = wrist_euler_angles[2]
        #inverse kinematics is now solved !

        return not self.crash_check()

    def __init__(self, info, TO, FO):
        self.tool_offset = TO #if there is no tool then tool_offset = [0, 0, 50]
        self.frame_offset = FO #if the frame is world then frame offset = [[0, 0, 0], [0, 0, 0]]
        #setting rotation matrix between world and frame
        self.rot_matrix_W_F = get_rot_matrix(self.frame_offset[1])
        self.rot_matrix_F_W = np.linalg.inv(self.rot_matrix_W_F)
        if info.ndim == 1:
            #info is joint angles so forward kinematics needs to be performed
            self.joint_angles = info
            self.position_ok = self.forward_kinematics()

        if info.ndim == 2:
            #info is desired tool frame so inverse kinematics needs to be performed
            self.tool_frame = info
            self.position_ok = self.inverse_kinematics()

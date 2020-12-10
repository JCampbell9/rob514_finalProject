import numpy as np
import csv
import sys
import os
from numpy import sin, cos, pi
import pylab as plt

import arm_calibration
from endEffector_to_world import get_ee_world_location

print("\nERROR CALCULATION...")

directory = os.path.dirname(os.path.realpath(__file__))


class RobotArm:
    def __init__(self):

        self.joint_angles = {}
        self.joint_lengths = {"joint1": 0, "joint2": 0.2755, "joint3": 0.205, "joint4": 0.205, "joint5": 0.2073, "joint6": 0.1038, "joint7": 0.1038}
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        self.joint_min = {"joint1": -1744.444444, "joint2": 2.442222222, "joint3": -17444.44444, "joint4": 0.523333333, "joint5": -1744.444444,
                              "joint6": 1.133888889, "joint7": -174.4444444}
        self.joint_max = {"joint1": 174.4444444, "joint2": 5.460111111, "joint3": 2.878333333, "joint4": 5.756666667, "joint5": 174.4444444,
                              "joint6": 5.146111111, "joint7": 174.4444444}


        self.get_joint_angles_from_physical_data()

        # Get End Effector position of the physical robot
        Palm_in_world, Palm_in_ArUco = get_ee_world_location()

        # Determine the target endpoint based off of the diff in the End effector location and palm
        self.arm_endpoint = Palm_in_world

        # Actual Palm arUco marker location
        self.target_endpoint = Palm_in_ArUco

        self.done_parts = [] # Stores final arm parts for plotting
        self.done_parts_dict = {}
        self.cost_values = [] # Holds each cost value made from changes in gradient descent

    def get_joint_angles_from_physical_data(self):
        """ Gets the joint angles saved within arm_cal/ with the physical robot arm data """
        print("\nGetting the physical arm joint angles...")
        with open(directory + '/test_data/Matrices/Angles_9.0.csv', newline='') as file:
            joint_data = csv.reader(file)
            idx = 0
            for row in joint_data:
                for j in row:
                    key = self.joint_names[idx]
                    self.joint_angles[key] = float(j)
                    print(key, ": ", self.joint_angles[key])
                    idx += 1
                    if idx == len(self.joint_names):
                        break
            file.close()

    def get_ee_transform_matrices(self):
        """ Get the saved End Effector to Palm Translation and Rotation matrices """
        translation_mat = np.zeros((4, 4))
        rotation_mat = np.zeros((4, 4))

        with open(directory + '/EE_to_Palm_Translation_Matrix.csv', newline='') as f:
            reader = csv.reader(f)
            for j, row in enumerate(reader):
                for i, col in enumerate(row):
                    translation_mat[j][i] = float(col)

        with open(directory + '/EE_to_Palm_Rotation_Matrix.csv', newline='') as f:
            reader = csv.reader(f)
            for j, row in enumerate(reader):
                for i, col in enumerate(row):
                    rotation_mat[j][i] = float(col)

        return translation_mat, rotation_mat

    def save_new_joint_angles(self):
        save_file = "/OptimizedAngles.csv"
        print("Saving new joint angles at ",save_file)
        with open(directory + save_file, "w", newline="") as outfile:
            writer = csv.writer(outfile)
            writer.writerow(robot.joint_angles.values())
            outfile.close()

    def plot_arm(self,title):
        """ Plot arm parts (lines) with current transformation """
        x_vals = []
        y_vals = []
        x_endpoints = []
        y_endpoints = []

        for name in self.joint_names:
            part = self.done_parts_dict[name]
            x_vals += [part[0][0], part[1][0]]
            y_vals += [part[0][1], part[1][1]]
            x_endpoints += [part[1][0]]
            y_endpoints += [part[1][1]]

        plt.plot(x_vals, y_vals, '-o')
        plt.title(title)

        for i, txt in enumerate(self.joint_names):
            plt.annotate(txt, (x_endpoints[i], y_endpoints[i]))

        plt.show()

    def plot_cost(self):
        """ Plot cost value after each angle change from gradient descent """
        steps = np.arange(len(self.cost_values))
        plt.plot(steps, self.cost_values, '-o')
        plt.xlabel("Steps")
        plt.ylabel("Cost value")
        plt.title("Cost value per step using Gradient Descent")
        plt.show()

    # Create a rotation matrix
    @staticmethod
    def rotation_matrix(theta):
        """Create a 3x3 rotation matrix that rotates in the x,y plane
        @param: theta - amount to rotate by in radians
        @return: 3x3 matrix, 2D rotation plus identity """
        m_rot = np.identity(3)
        m_rot[0][0] = cos(theta)
        m_rot[0][1] = -sin(theta)
        m_rot[1][0] = sin(theta)
        m_rot[1][1] = cos(theta)
        return m_rot

    # Create a translation matrix
    @staticmethod
    def translation_matrix(dx, dy):
        """Create a 3x3 translation matrix that moves by dx, dy
        @param: dx - translate by that much in x
        @param: dy - translate by that much in y
        @return: 3x3 matrix """
        m_trans = np.identity(3)
        m_trans[0, 2] = dx
        m_trans[1, 2] = dy
        return m_trans

    # Return the matrices that move each of the components. Do this as a dictionary, just to be clean
    def get_matrices(self):
        """ Defines transformation matrices for each robot part and returns dictionary
            of all matrix transformations. """
        # Define dictionary containing all transformation matrices
        # Each of these should be of the form: Translation * rotation
        mat_ret = dict()

        # Each entry defined as <part>_T for translation, <part>_R for rotation
        for joint_key, joint_value in self.joint_angles.items():
            mat_ret[joint_key + "_T"] = self.translation_matrix(self.joint_lengths[joint_key], 0)
            mat_ret[joint_key + "_R"] = self.rotation_matrix(joint_value)

        return mat_ret

    # Apply the matrix m to the points in rect
    def transform_part(self, part, m):
        """Apply the 3x3 transformation matrix to the arm part
        @param: part: Arm line from make_arm_part
        @param: m - 3x3 matrix
        @return: a 1x4 array of x,y values of the transformed arm part"""
        part_t = []
        for p in part:
            p_new = m @ np.transpose(p)
            part_t.append(np.transpose(p_new))
        return part_t

    def make_arm_part(self, in_len):
        """Draw a line of the given length representing the arm part
        @param: in_len desired length
        @return: a 1x2 array of x,y values representing the two ends of the arm part"""
        x_l = 0
        x_r = in_len
        return [[x_l, 0, 1], [x_r, 0, 1]]

    def get_arm_endpoint(self, part_ret="joint7"):
        """ Return the end point of the arm. Determines appropriate transformations of each robot part
            to get the final end point of the arm, even as the arm moves. """

        mat_accum = np.identity(3)
        arm_parts = dict()
        for joint_key, joint_length in self.joint_lengths.items():
            arm_parts[joint_key] = self.make_arm_part(joint_length)

        ''' Get new end effector endpoint coordinate location based on angle changes '''
        mat_ret = self.get_matrices()  # Returns dict of matrix rotation/translation for each part
        self.done_parts = []  # Stores final arm parts (lines) to be plotted

        i = len(self.joint_names) - 1  # Index marker for determining the current part to transform

        # Go through the full list of components
        for j in range(len(self.joint_names)):
            part = self.joint_names[i]
            curr_part = part

            # Do the initial transformation for the current part
            m = np.dot(mat_ret[part + "_T"], mat_ret[part + "_R"])
            part_transform = self.transform_part(arm_parts[part], m)

            # Adjust number of transforms so Finger2 does not apply Finger1's transform matrices
            if part == "joint7":
                num_transforms = i - 1
            else:
                num_transforms = i

            # Apply all transforms to current part (traversing backwards through the parts list)
            for k in range(i):
                if (num_transforms - k - 1) < 0:
                    break
                part = self.joint_names[num_transforms - k - 1]
                m = np.dot(mat_ret[part + "_T"], mat_ret[part + "_R"])
                part_transform = self.transform_part(part_transform, np.dot(mat_ret[part + "_T"], mat_ret[part + "_R"]))

            self.done_parts.append(part_transform)
            self.done_parts_dict[curr_part] = part_transform

            # We want the endpoint of the arm, so we take the endpoint based on the last robot part
            if self.joint_names[i] == part_ret:
                fore_end = part_transform[1]
                mat_accum[0] = fore_end[0]
                mat_accum[1] = fore_end[1]
            i = i - 1

        pt_end = mat_accum[0:2, 2]

        return pt_end

    def cost(self, curr_xy):
        """
        Calculates the cost between the given point and the target point.
        :param curr_xy: Current endpoint of arm
        :return: dist: Distance between current endpoint and target point
        """
        dist = np.linalg.norm(curr_xy - self.target_endpoint[0:1])
        return dist

    def gradient(self, joint_key):
        """
        Calculates the gradient between the current endpoint and the new endpoint from a change in theta.
        :param joint_angle: Current joint component
        :return: Gradient from angle based on the change in endpoint cost
        """
        change = 0.3  # Amount to change theta by
        old_xy = self.get_arm_endpoint()  # Old endpoint value
        old_cost = self.cost(old_xy)  # Old endpoint cost (to target)

        angle_value = self.joint_angles[joint_key] + change  # Change the angle
        self.joint_angles[joint_key] = angle_value

        new_xy = self.get_arm_endpoint()  # New endpoint value
        new_cost = self.cost(new_xy)  # New endpoint cost (to target)

        gradient = (new_cost - old_cost) / change  # Calculate the gradient using the change in cost of x,y

        return gradient

    def reach_gradient(self):
        """Align the robot end point (palm) to the target point using gradient descent"""
        step_size = 0.05
        min_step_size = 0.001
        moved_closer = True
        while_loop_counter = 0
        max_steps = 100
        old_total_cost = 10
        epsilon = 0.05

        # While moved closer and not reached minimum step size
        while moved_closer and step_size > min_step_size:
            while_loop_counter += 1
            # Set a maximum number of steps per change to see progress - used for testing
            if while_loop_counter > max_steps:
                break
            new_total_cost = 0
            text = ""
            i = 0

            # Go through each joint within the arm
            for joint_key, joint_value in self.joint_angles.items():
                # Text to show for each joint change
                text += str(self.joint_names[i]) + " "
                i += 1

                # Old endpoint values
                old_value = joint_value
                old_endpoint = self.get_arm_endpoint()
                old_cost = self.cost(old_endpoint)

                # Gradient of old values
                gradient = self.gradient(joint_key)
                if gradient > 0:  # Determine direction of gradient
                    direction = 1
                else:
                    direction = -1

                # Determine new angle value based on gradient
                self.joint_angles[joint_key] = (old_value - direction * step_size)

                if self.joint_angles[joint_key] < self.joint_min[joint_key]:
                    self.joint_angles[joint_key] = self.joint_min[joint_key]
                elif self.joint_angles[joint_key] > self.joint_max[joint_key]:
                    self.joint_angles[joint_key] = self.joint_max[joint_key]

                # Determine new endpoint from new angle
                new_endpoint = self.get_arm_endpoint()
                new_cost = self.cost(new_endpoint)

                # Determine the cost of
                if new_cost > old_cost:
                    self.joint_angles[joint_key] = old_value
                    new_total_cost += old_cost
                    text += ": No change \n"
                else:
                    text += ": Improved by " + str(direction * step_size) + "\n"
                    new_total_cost += new_cost

            # Display change of each joint through text
            print("Robot part changes: \n", text)
            self.cost_values += [new_total_cost]

            # Check if improved from previous position
            if old_total_cost < new_total_cost:
                step_size -= .01
                moved_closer = False
            else:
                moved_closer = True

            print("abs(old_total_cost - new_total_cost): ",abs(old_total_cost - new_total_cost))
            print("new_total_cost: ",new_total_cost)
            # If changes are less than epsilon, we stop
            if abs(old_total_cost - new_total_cost) < epsilon:
                break
            old_total_cost = new_total_cost
            print("IN function, joint_angles: ",self.joint_angles.items())

        # Save new joint angle values
        self.save_new_joint_angles()

print("GRADIENT DESCENT...")
robot = RobotArm()

before_endpoint = robot.get_arm_endpoint()
print("target ENDPOINT: ", robot.target_endpoint)
print("Before ENDPOINT: ", before_endpoint)
print("Before cost: ", robot.cost(before_endpoint))
robot.plot_arm("Before: Arm part transformation")

robot.reach_gradient()

after_endpoint = robot.get_arm_endpoint()
print("Before ENDPOINT: ", before_endpoint)
print("After ENDPOINT: ", after_endpoint)
print("target ENDPOINT: ", robot.target_endpoint)
print("After cost: ",robot.cost(after_endpoint))
robot.plot_arm("After: Arm part transformation")

# Plot cost over time from Gradient Descent algorithm
robot.plot_cost()

print("After, Joint angles: ",robot.joint_angles.items())

robot.get_joint_angles_from_physical_data()
print("Before, Joint angles: ",robot.joint_angles.items())

print("Done :)")

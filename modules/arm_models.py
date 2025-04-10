from math import sqrt, sin, cos, atan, atan2, degrees, pi
import numpy as np
from matplotlib.figure import Figure
from helper_fcns.utils import EndEffector, rotm_to_euler, euler_to_rotm, check_joint_limits, dh_to_matrix, near_zero, wraptopi

PI = 3.1415926535897932384
# np.set_printoptions(precision=3)

PI = 3.1415926535897932384

class Robot:
    """
    Represents a robot manipulator with various kinematic configurations.
    Provides methods to calculate forward kinematics, inverse kinematics, and velocity kinematics.
    Also includes methods to visualize the robot's motion and state in 3D.

    Attributes:
        num_joints (int): Number of joints in the robot.
        ee_coordinates (list): List of end-effector coordinates.
        robot (object): The robot object (e.g., TwoDOFRobot, ScaraRobot, etc.).
        origin (list): Origin of the coordinate system.
        axes_length (float): Length of the axes for visualization.
        point_x, point_y, point_z (list): Lists to store coordinates of points for visualization.
        show_animation (bool): Whether to show the animation or not.
        plot_limits (list): Limits for the plot view.
        fig (matplotlib.figure.Figure): Matplotlib figure for 3D visualization.
        sub1 (matplotlib.axes._subplots.Axes3DSubplot): Matplotlib 3D subplot.
    """

    def __init__(self, type='2-dof', show_animation: bool=True):
        """
        Initializes a robot with a specific configuration based on the type.

        Args:
            type (str, optional): Type of robot (e.g., '2-dof', 'scara', '5-dof'). Defaults to '2-dof'.
            show_animation (bool, optional): Whether to show animation of robot movement. Defaults to True.
        """
        if type == '2-dof':
            self.num_joints = 2
            self.ee_coordinates = ['X', 'Y']
            self.robot = TwoDOFRobot()
        
        elif type == 'scara':
            self.num_joints = 3
            self.ee_coordinates = ['X', 'Y', 'Z', 'Theta']
            self.robot = ScaraRobot()

        elif type == '5-dof':
            self.num_joints = 5
            self.ee_coordinates = ['X', 'Y', 'Z', 'RotX', 'RotY', 'RotZ']
            self.robot = FiveDOFRobot()
        
        self.origin = [0., 0., 0.]
        self.axes_length = 0.04
        self.point_x, self.point_y, self.point_z = [], [], []
        self.waypoint_x, self.waypoint_y, self.waypoint_z = [], [], []
        self.waypoint_rotx, self.waypoint_roty, self.waypoint_rotz = [], [], []
        self.theta_traj = [] # stored trajectory
        self.show_animation = show_animation
        self.plot_limits = [0.65, 0.65, 0.8]

        if self.show_animation:
            self.fig = Figure(figsize=(12, 10), dpi=100)
            self.sub1 = self.fig.add_subplot(1,1,1, projection='3d') 
            self.fig.suptitle("Manipulator Kinematics Visualization", fontsize=16)

        # initialize figure plot
        self.init_plot()

    
    def init_plot(self):
        """Initializes the plot by calculating the robot's points and calling the plot function."""
        self.robot.calc_robot_points()
        self.plot_3D()

    
    def update_plot(self, pose=None, angles=None, soln=0, numerical=False):
        """
        Updates the robot's state based on new pose or joint angles and updates the visualization.

        Args:
            pose (EndEffector, optional): Desired end-effector pose for inverse kinematics.
            angles (list, optional): Joint angles for forward kinematics.
            soln (int, optional): The inverse kinematics solution to use (0 or 1).
            numerical (bool, optional): Whether to use numerical inverse kinematics.
        """
        if pose is not None: # Inverse kinematics case
            if not numerical:
                self.robot.calc_inverse_kinematics(pose, soln=soln)
            else:
                self.robot.calc_numerical_ik(pose)
        elif angles is not None: # Forward kinematics case
            self.robot.calc_forward_kinematics(angles, radians=False)
        else:
            return
        self.plot_3D()


    def move_velocity(self, vel):
        """
        Moves the robot based on a given velocity input.

        Args:
            vel (list): Velocity input for the robot.
        """
        self.robot.calc_velocity_kinematics(vel)
        self.plot_3D()
        

    def draw_line_3D(self, p1, p2, format_type: str = "k-"):
        """
        Draws a 3D line between two points.

        Args:
            p1 (list): Coordinates of the first point.
            p2 (list): Coordinates of the second point.
            format_type (str, optional): The format of the line. Defaults to "k-".
        """
        self.sub1.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], format_type)


    def draw_ref_line(self, point, axes=None, ref='xyz'):
        """
        Draws reference lines from a given point along specified axes.

        Args:
            point (list): The coordinates of the point to draw from.
            axes (matplotlib.axes, optional): The axes on which to draw the reference lines.
            ref (str, optional): Which reference axes to draw ('xyz', 'xy', or 'xz'). Defaults to 'xyz'.
        """
        line_width = 0.7
        if ref == 'xyz':
            axes.plot([point[0], self.plot_limits[0]],
                      [point[1], point[1]],
                      [point[2], point[2]], 'b--', linewidth=line_width)    # X line
            axes.plot([point[0], point[0]],
                      [point[1], self.plot_limits[1]],
                      [point[2], point[2]], 'b--', linewidth=line_width)    # Y line
            axes.plot([point[0], point[0]],
                      [point[1], point[1]],
                      [point[2], 0.0], 'b--', linewidth=line_width)         # Z line
        elif ref == 'xy':
            axes.plot([point[0], self.plot_limits[0]],
                      [point[1], point[1]], 'b--', linewidth=line_width)    # X line
            axes.plot([point[0], point[0]],
                      [point[1], self.plot_limits[1]], 'b--', linewidth=line_width)    # Y line
        elif ref == 'xz':
            axes.plot([point[0], self.plot_limits[0]],
                      [point[2], point[2]], 'b--', linewidth=line_width)    # X line
            axes.plot([point[0], point[0]],
                      [point[2], 0.0], 'b--', linewidth=line_width)         # Z line


    def plot_waypoints(self):
        """
        Plots the waypoints in the 3D visualization
        """
        # draw the points
        self.sub1.plot(self.waypoint_x, self.waypoint_y, self.waypoint_z, 'or', markersize=8)


    def plot_ee_trajectory(self):
        """TBA
        """
        xlist, ylist, zlist = [], [], []

        for th in self.theta_traj:
            ee_position = self.robot.solve_forward_kinematics(th, radians=True)
            xlist.append(ee_position[0])
            ylist.append(ee_position[1])
            zlist.append(ee_position[2])

        # draw the points
        self.sub1.plot(xlist, ylist, zlist, 'bo', markersize=2)


    def update_ee_trajectory(self):
        self.theta_traj.append(self.robot.theta) # add the latest thetalist


    def reset_ee_trajectory(self):
        self.theta_traj = []

    
    def solve_inverse_kinematics(self, pose: EndEffector, soln=0):
        return self.robot.solve_inverse_kinematics(pose)
    

    def update_waypoints(self, waypoints: list):
        """
        Updates the waypoints into a member variable
        """
        for i in range(len(waypoints)):
            self.waypoint_x.append(waypoints[i][0])
            self.waypoint_y.append(waypoints[i][1])
            self.waypoint_z.append(waypoints[i][2])
            # self.waypoint_rotx.append(waypoints[i][3])
            # self.waypoint_roty.append(waypoints[i][4])
            # self.waypoint_rotz.append(waypoints[i][5])


    def get_waypoints(self):
        return [
            [self.waypoint_x[0], self.waypoint_y[0], self.waypoint_z[0]],
            [self.waypoint_x[1], self.waypoint_y[1], self.waypoint_z[1]]
        ]
    

    def plot_3D(self):
        """
        Plots the 3D visualization of the robot, including the robot's links, end-effector, and reference frames.
        """        
        self.sub1.cla()
        self.point_x.clear()
        self.point_y.clear()
        self.point_z.clear()

        EE = self.robot.ee

        # draw lines to connect the points
        for i in range(len(self.robot.points)-1):
            self.draw_line_3D(self.robot.points[i], self.robot.points[i+1])

        # draw the points
        for i in range(len(self.robot.points)):
            self.point_x.append(float(self.robot.points[i][0]))
            self.point_y.append(float(self.robot.points[i][1]))
            self.point_z.append(float(self.robot.points[i][2]))
        self.sub1.plot(self.point_x, self.point_y, self.point_z, marker='o', markerfacecolor='m', markersize=12)

        # draw the waypoints
        self.plot_waypoints()

        # draw the EE trajectory
        self.plot_ee_trajectory()

        # draw the EE
        self.sub1.plot(EE.x, EE.y, EE.z, 'bo')
        # draw the base reference frame
        self.draw_line_3D(self.origin, [self.origin[0] + self.axes_length, self.origin[1], self.origin[2]], format_type='r-')
        self.draw_line_3D(self.origin, [self.origin[0], self.origin[1] + self.axes_length, self.origin[2]], format_type='g-')
        self.draw_line_3D(self.origin, [self.origin[0], self.origin[1], self.origin[2] + self.axes_length], format_type='b-')
        # draw the EE reference frame
        self.draw_line_3D([EE.x, EE.y, EE.z], self.robot.EE_axes[0], format_type='r-')
        self.draw_line_3D([EE.x, EE.y, EE.z], self.robot.EE_axes[1], format_type='g-')
        self.draw_line_3D([EE.x, EE.y, EE.z], self.robot.EE_axes[2], format_type='b-')
        # draw reference / trace lines
        self.draw_ref_line([EE.x, EE.y, EE.z], self.sub1, ref='xyz')

        # add text at bottom of window
        pose_text = "End-effector Pose:      [ "
        pose_text += f"X: {round(EE.x,4)},  "
        pose_text += f"Y: {round(EE.y,4)},  "
        pose_text += f"Z: {round(EE.z,4)},  "
        pose_text += f"RotX: {round(EE.rotx,4)},  "
        pose_text += f"RotY: {round(EE.roty,4)},  "
        pose_text += f"RotZ: {round(EE.rotz,4)}  "
        pose_text += " ]"

        theta_text = "Joint Positions (deg/m):     ["
        for i in range(self.num_joints):
            theta_text += f" {round(np.rad2deg(self.robot.theta[i]),2)}, "
        theta_text += " ]"
        
        textstr = pose_text + "\n" + theta_text
        self.sub1.text2D(0.2, 0.02, textstr, fontsize=13, transform=self.fig.transFigure)

        self.sub1.set_xlim(-self.plot_limits[0], self.plot_limits[0])
        self.sub1.set_ylim(-self.plot_limits[1], self.plot_limits[1])
        self.sub1.set_zlim(0, self.plot_limits[2])
        self.sub1.set_xlabel('x [m]')
        self.sub1.set_ylabel('y [m]')




class TwoDOFRobot():
    """
    Represents a 2-degree-of-freedom (DOF) robot arm with two joints and one end effector.
    Includes methods for calculating forward kinematics (FPK), inverse kinematics (IPK),
    and velocity kinematics (VK).

    Attributes:
        l1 (float): Length of the first arm segment.
        l2 (float): Length of the second arm segment.
        theta (list): Joint angles.
        theta_limits (list): Joint limits for each joint.
        ee (EndEffector): The end effector object.
        points (list): List of points representing the robot's configuration.
        num_dof (int): The number of degrees of freedom (2 for this robot).
    """

    def __init__(self):
        """
        Initializes a 2-DOF robot with default arm segment lengths and joint angles.
        """
        self.l1 = 0.30  # Length of the first arm segment
        self.l2 = 0.25  # Length of the second arm segment

        self.theta = [0.0, 0.0]  # Joint angles (in radians)
        self.theta_limits = [[-PI, PI], [-PI + 0.261, PI - 0.261]]  # Joint limits

        self.ee = EndEffector()  # The end-effector object
        self.num_dof = 2  # Number of degrees of freedom
        self.points = [None] * (self.num_dof + 1)  # List to store robot points


    def calc_forward_kinematics(self, theta: list, radians=False):
        """
        Calculates the forward kinematics for the robot based on the joint angles.

        Args:
            theta (list): Joint angles.
            radians (bool, optional): Whether the angles are in radians or degrees. Defaults to False.
        """
        
        ########################################

        # insert your code here

        ########################################


        # Update the robot configuration (i.e., the positions of the joints and end effector)
        self.calc_robot_points()


    def calc_inverse_kinematics(self, EE: EndEffector, soln=0):
        """
        Calculates the inverse kinematics (IK) for a given end effector position.

        Args:
            EE (EndEffector): The end effector object containing the target position (x, y).
            soln (int, optional): The solution branch to use. Defaults to 0 (first solution).
        """
        x, y = EE.x, EE.y
        l1, l2 = self.l1, self.l2

        ########################################

        # insert your code here

        ########################################
        
        # Calculate robot points based on the updated joint angles
        self.calc_robot_points()


    def calc_numerical_ik(self, EE: EndEffector, tol=0.01, ilimit=50):
        """
        Calculates numerical inverse kinematics (IK) based on input end effector coordinates.

        Args:
            EE (EndEffector): The end effector object containing the target position (x, y).
            tol (float, optional): The tolerance for the solution. Defaults to 0.01.
            ilimit (int, optional): The maximum number of iterations. Defaults to 50.
        """
        
        x, y = EE.x, EE.y
        
        ########################################

        # insert your code here

        ########################################

        self.calc_robot_points()


    def calc_velocity_kinematics(self, vel: list):
        """
        Calculates the velocity kinematics for the robot based on the given velocity input.

        Args:
            vel (list): The velocity vector for the end effector [vx, vy].
        """
        
        ########################################

        # insert your code here

        ########################################

        # Update robot points based on the new joint angles
        self.calc_robot_points()


    

    def calc_robot_points(self):
        """
        Calculates the positions of the robot's joints and the end effector.

        Updates the `points` list, storing the coordinates of the base, shoulder, elbow, and end effector.
        """

        

        ########################################

        # Replace the placeholder values with your code


        placeholder_value = [0.0, 0.0, 0.0]


        # Base position
        self.points[0] = placeholder_value
        # Shoulder joint
        self.points[1] = placeholder_value
        # Elbow joint
        self.points[2] = placeholder_value

        ########################################





        # Update end effector position
        self.ee.x = self.points[2][0]
        self.ee.y = self.points[2][1]
        self.ee.z = self.points[2][2]
        self.ee.rotz = self.theta[0] + self.theta[1]

        # End effector axes
        self.EE_axes = np.zeros((3, 3))
        self.EE_axes[0] = np.array([cos(self.theta[0] + self.theta[1]), sin(self.theta[0] + self.theta[1]), 0]) * 0.075 + self.points[2]
        self.EE_axes[1] = np.array([-sin(self.theta[0] + self.theta[1]), cos(self.theta[0] + self.theta[1]), 0]) * 0.075 + self.points[2]
        self.EE_axes[2] = np.array([0, 0, 1]) * 0.075 + self.points[2]



class ScaraRobot():
    """
    A class representing a SCARA (Selective Compliance Assembly Robot Arm) robot.
    This class handles the kinematics (forward, inverse, and velocity kinematics) 
    and robot configuration, including joint limits and end-effector calculations.
    """
    
    def __init__(self):
        """
        Initializes the SCARA robot with its geometry, joint variables, and limits.
        Sets up the transformation matrices and robot points.
        """
        # Geometry of the robot (link lengths in meters)
        self.l1 = 0.35  # Base to 1st joint
        self.l2 = 0.18  # 1st joint to 2nd joint
        self.l3 = 0.15  # 2nd joint to 3rd joint
        self.l4 = 0.30  # 3rd joint to 4th joint (tool or end-effector)
        self.l5 = 0.12  # Tool offset

        # Joint variables (angles in radians)
        self.theta = [0.0, 0.0, 0.0]

        # Joint angle limits (min, max) for each joint
        self.theta_limits = [
            [-np.pi, np.pi],
            [-np.pi + 0.261, np.pi - 0.261],
            [0, self.l1 + self.l3 - self.l5]
        ]

        # End-effector (EE) object to store EE position and orientation
        self.ee = EndEffector()

        # Number of degrees of freedom and number of points to store robot configuration
        self.num_dof = 3
        self.num_points = 7
        self.points = [None] * self.num_points

        # Transformation matrices (DH parameters and resulting transformation)
        self.DH = np.zeros((5, 4))  # Denavit-Hartenberg parameters (theta, d, a, alpha)
        self.T = np.zeros((self.num_dof, 4, 4))  # Transformation matrices

        ########################################

        # insert your additional code here

        ########################################

    
    def calc_forward_kinematics(self, theta: list, radians=False):
        """
        Calculate Forward Kinematics (FK) based on the given joint angles.

        Args:
            theta (list): Joint angles (in radians if radians=True, otherwise in degrees).
            radians (bool): Whether the input angles are in radians (default is False).
        """
        ########################################

        # insert your code here

        ########################################

        # Calculate robot points (e.g., end-effector position)
        self.calc_robot_points()


    def calc_inverse_kinematics(self, EE: EndEffector, soln=0):
        """
        Calculate Inverse Kinematics (IK) based on the input end-effector coordinates.

        Args:
            EE (EndEffector): End-effector object containing desired position (x, y, z).
            soln (int): Solution index (0 or 1), for multiple possible IK solutions.
        """
        x, y, z = EE.x, EE.y, EE.z
        l1, l2, l3, l4, l5 = self.l1, self.l2, self.l3, self.l4, self.l5

        ########################################

        # insert your code here

        ########################################

        # Recalculate Forward Kinematics to update the robot's configuration
        self.calc_forward_kinematics(self.theta, radians=True)


    def calc_velocity_kinematics(self, vel: list):
        """
        Calculate velocity kinematics and update joint velocities.

        Args:
            vel (array): Linear velocities (3D) of the end-effector.
        """
        ########################################

        # insert your code here

        ########################################

        # Recalculate robot points based on updated joint angles
        self.calc_robot_points()
  

    def calc_robot_points(self):
        """
        Calculate the main robot points (links and end-effector position) using the current joint angles.
        Updates the robot's points array and end-effector position.
        """

        # Calculate transformation matrices for each joint and end-effector
        self.points[0] = np.array([0, 0, 0, 1])
        self.points[1] = np.array([0, 0, self.l1, 1])
        self.points[2] = self.T[0]@self.points[0]
        self.points[3] = self.points[2] + np.array([0, 0, self.l3, 1])
        self.points[4] = self.T[0]@self.T[1]@self.points[0] + np.array([0, 0, self.l5, 1])
        self.points[5] = self.T[0]@self.T[1]@self.points[0]
        self.points[6] = self.T[0]@self.T[1]@self.T[2]@self.points[0]

        self.EE_axes = self.T[0]@self.T[1]@self.T[2]@np.array([0.075, 0.075, 0.075, 1])
        self.T_ee = self.T[0]@self.T[1]@self.T[2]

        # End-effector (EE) position and axes
        self.ee.x = self.points[-1][0]
        self.ee.y = self.points[-1][1]
        self.ee.z = self.points[-1][2]
        rpy = rotm_to_euler(self.T_ee[:3,:3])
        self.ee.rotx, self.ee.roty, self.ee.rotz = rpy
        
        # EE coordinate axes
        self.EE_axes = np.zeros((3, 3))
        self.EE_axes[0] = self.T_ee[:3,0] * 0.075 + self.points[-1][0:3]
        self.EE_axes[1] = self.T_ee[:3,1] * 0.075 + self.points[-1][0:3]
        self.EE_axes[2] = self.T_ee[:3,2] * 0.075 + self.points[-1][0:3]


class FiveDOFRobot:
    """
    A class to represent a 5-DOF robotic arm with kinematics calculations, including
    forward kinematics, inverse kinematics, velocity kinematics, and Jacobian computation.

    Attributes:
        l1, l2, l3, l4, l5: Link lengths of the robotic arm.
        theta: List of joint angles in radians.
        theta_limits: Joint limits for each joint.
        ee: End-effector object for storing the position and orientation of the end-effector.
        num_dof: Number of degrees of freedom (5 in this case).
        points: List storing the positions of the robot joints.
        DH: Denavit-Hartenberg parameters for each joint.
        T: Transformation matrices for each joint.
    """
    
    def __init__(self):
        """Initialize the robot parameters and joint limits."""
        # Link lengths
        # self.l1, self.l2, self.l3, self.l4, self.l5 = 0.30, 0.15, 0.18, 0.15, 0.12
        self.l1, self.l2, self.l3, self.l4, self.l5 = 0.155, 0.099, 0.095, 0.055, 0.105 # from hardware measurements
        
        # Joint angles (initialized to zero)
        self.theta = [0, 0, 0, 0, 0]
        
        # Joint limits (in radians)
        self.theta_limits = [
            [-np.pi, np.pi], 
            [-np.pi/3, np.pi], 
            [-np.pi+np.pi/12, np.pi-np.pi/4], 
            [-np.pi+np.pi/12, np.pi-np.pi/12], 
            [-np.pi, np.pi]
        ]

        self.thetadot_limits = [
            [-np.pi*2, np.pi*2], 
            [-np.pi*2, np.pi*2], 
            [-np.pi*2, np.pi*2], 
            [-np.pi*2, np.pi*2], 
            [-np.pi*2, np.pi*2]
        ]
        
        # End-effector object
        self.ee = EndEffector()
        
        # Robot's points
        self.num_dof = 5
        self.points = [None] * (self.num_dof + 1)
        
        # Denavit-Hartenberg parameters and transformation matrices
        self.DH = np.zeros((5, 4))
        self.T = np.zeros((self.num_dof, 4, 4))

    
    def calc_forward_kinematics(self, theta: list, radians=False):
        """
        Calculate forward kinematics based on the provided joint angles.
        
        Args:
            theta: List of joint angles (in degrees or radians).
            radians: Boolean flag to indicate if input angles are in radians.
        """
        if not radians:
            # Convert degrees to radians
            self.theta = np.deg2rad(theta)
        else:
            self.theta = theta
        
        # Apply joint limits
        self.theta = [np.clip(th, self.theta_limits[i][0], self.theta_limits[i][1]) 
                      for i, th in enumerate(self.theta)]

        # Set the Denavit-Hartenberg parameters for each joint
        self.DH[0] = [self.theta[0], self.l1, 0, np.pi/2]
        self.DH[1] = [self.theta[1] + np.pi/2, 0, self.l2, np.pi]
        self.DH[2] = [self.theta[2], 0, self.l3, np.pi]
        self.DH[3] = [self.theta[3] - np.pi/2, 0, 0, -np.pi/2]
        self.DH[4] = [self.theta[4], self.l4 + self.l5, 0, 0]

        # Compute the transformation matrices
        for i in range(self.num_dof):
            self.T[i] = dh_to_matrix(self.DH[i])
        
        # Calculate robot points (positions of joints)
        self.calc_robot_points()


    def calc_inverse_kinematics(self, EE: EndEffector, soln=0):
        """
        Calculate inverse kinematics to determine the joint angles based on end-effector position.
        
        Args:
            EE: EndEffector object containing desired position and orientation.
            soln: Optional parameter for multiple solutions (not implemented).
        """
        # Extract position and orientation of the end-effector
        l1, l2, l3, l4, l5 = self.l1, self.l2, self.l3, self.l4, self.l5
        
        # Desired position and rotation matrix
        p = np.array([EE.x, EE.y, EE.z])
        rpy = np.array([EE.rotx, EE.roty, EE.rotz])
        R = euler_to_rotm(rpy)
        
        # Calculate wrist position
        wrist_pos = p - R @ np.array([0, 0, 1]) * (l4 + l5)
        # print(f"Wrist pose: \n {wrist_pos} \n")
        # print(f"Rotation matrix, R: {R}, \n Determinant: {np.linalg.det(R)}")

        try: 
            # Solve for theta_1 using trigonometry
            x_wrist, y_wrist, z_wrist = wrist_pos
            theta1 = [atan2(y_wrist, x_wrist), wraptopi(atan2(y_wrist, x_wrist) + PI)]
            # TIP #1: the wraptopi ensures that the values stays within -pi to pi

            # using cosine rule, find theta_3
            s = z_wrist - l1
            r = [-sqrt(x_wrist**2 + y_wrist**2), sqrt(x_wrist**2 + y_wrist**2)]
            # TIP #2: consider two cases for r (+ve and -ve)
            L = sqrt(r[0]**2 + s**2)
            ctheta3 = (L**2 - l2**2 - l3**2) / (2 * l2 * l3)
            theta3 = [atan2(sqrt(1 - ctheta3**2), ctheta3), atan2(-sqrt(1 - ctheta3**2), ctheta3)]
            # TIP #3: Alternative approach to finding theta3
            # beta = np.arccos((-L**2 + l2**2 + l3**2) / (2 * l2 * l3))
            # theta3 = [pi - beta, beta - pi]   


            possible_solns, valid_solns= [], []

            for th3 in theta3:
                # calculate theta2
                theta2 = [atan2(r[0], s) - atan2(l3*sin(-th3), l2 + l3*cos(-th3)), \
                          atan2(r[1], s) - atan2(l3*sin(-th3), l2 + l3*cos(-th3))]
                # TIP #4: We set theta3 to -ve because it's moving in the opposite direction to theta2
                
                # calculate theta4 and theta5
                for th2 in theta2:
                    c23_ = cos(th2)*cos(th3) + sin(th2)*sin(th3)
                    s23_ = sin(th2)*cos(th3) - cos(th2)*sin(th3)

                    for th1 in theta1:
                        c1, s1 = cos(th1), sin(th1)
                        R0_3 = np.array([[-c1*s23_, -c1*c23_, s1],
                                        [-s1*s23_, -s1*c23_, -c1],
                                        [c23_, -s23_, 0]])
                        # TIP #5 This is derived from calculating R3_4*R4_5 from the DH table
                        R3_5 = R0_3.T @ R

                        th4 = atan2(R3_5[1,2], R3_5[0,2])
                        th5 = atan2(-R3_5[2,0], -R3_5[2,1])

                        solution = [th1, th2, th3, th4, th5]
                        possible_solns.append(solution)
                        if self.check_valid_ik_soln(solution, EE):
                            valid_solns.append(solution)

            if len(valid_solns) > 0:
                if soln == 0:
                    self.theta = valid_solns[0]
                elif soln == 1:
                    self.theta = valid_solns[1]
                
                # Calculate forward kinematics with the new joint angles
                self.calc_forward_kinematics(self.theta, radians=True)

                return True
            
            else:
                print(f"\n[Analytical IK solver] ERROR: No valid solution found! Joint limits may be exceeded or position may be unreachable.")
                # raise ValueError
                return False
        
        except RuntimeWarning:
            print("\n [ERROR] (Runtime) Joint limits exceeded! \n")
        
   
    def calc_numerical_ik(self, EE: EndEffector, tol=1e-3, ilimit=500):
        """ Calculate numerical inverse kinematics based on input coordinates. """
        
        Te_d = [EE.x, EE.y, EE.z]

        # move robot slightly out of zeros singularity
        if all(th == 0.0 for th in self.theta):
            self.theta = [self.theta[i] + np.random.rand()*0.05 for i in range(self.num_dof)]
        
        # Iteration count
        i = 0
        q = self.theta.copy()
        
        while i < ilimit:
            i += 1

            # compute current EE position based on q
            Te = self.solve_forward_kinematics(q, radians=True)

            # calculate the EE position error
            e = [0, 0, 0]
            e[0] = Te_d[0] - Te[0]
            e[1] = Te_d[1] - Te[1]
            e[2] = Te_d[2] - Te[2]

            # update q
            J = self.jacobian(q)
            q += self.damped_inverse_jacobian(q) @ e

            # check for joint limits
            for j, th in enumerate(q):
                q[j] = np.clip(th, self.theta_limits[j][0], self.theta_limits[j][1])

            # Check if we have arrived
            if abs(max(e, key=abs)) < tol:
                break 
        
        if abs(max(e, key=abs)) > tol:
            print("\n [ERROR] Numerical IK solution failed to converge... \n \
                  Possible causes: \n \
                  1. cartesian position is not reachable by the robot, given the joint limits \n \
                  2. desired joint configuration is very close to OR at a singularity \n \
                  3. iterative algorithm is stuck at a local minima \n \
                  4. solver is taking too long to converge  \n")
            print(f"Max position error: {max(e, key=abs)} | # iterations: {i}/{ilimit} ")
            # raise ValueError
            return False

        self.theta = q

        print(f"Solution found = {q} | Max pos error = {max(e, key=abs)} | # iterations: {i}/{ilimit}  \n")

        # update forward kinematics
        self.calc_forward_kinematics(self.theta, radians=True)

        return True

        

    def calc_velocity_kinematics(self, vel: list):
        """
        Calculate the joint velocities required to achieve the given end-effector velocity.
        
        Args:
            vel: Desired end-effector velocity (3x1 vector).
        """
        # Avoid singularity by perturbing joint angles slightly
        if all(th == 0.0 for th in self.theta):
            self.theta = [th + np.random.rand() * 0.01 for th in self.theta]

        # Calculate the joint velocity using the inverse Jacobian
        # thetadot = self.inverse_jacobian(pseudo=True) @ vel
        thetadot = self.damped_inverse_jacobian() @ vel

        # print(f'normal case: {self.inverse_jacobian(pseudo=True)}')
        # print(f'damped inverse case: {self.damped_inverse_jacobian()}')

        # (Corrective measure) Ensure joint velocities stay within limits
        thetadot = np.clip(thetadot, [limit[0] for limit in self.thetadot_limits], [limit[1] for limit in self.thetadot_limits])

        # Update joint angles
        self.theta[0] += 0.02 * thetadot[0]
        self.theta[1] += 0.02 * thetadot[1]
        self.theta[2] += 0.02 * thetadot[2]
        self.theta[3] += 0.02 * thetadot[3]
        self.theta[4] += 0.02 * thetadot[4]

        # print(f'linear vel: {[round(vel[0], 3), round(vel[1], 3), round(vel[2], 3)]}')
        # print(f'thetadot (deg/s) = {[round(td,2) for td in thetadot]}')
        # print(f'Commanded theta (deg) = {[round(th,2) for th in self.theta]}')  

        # Recompute robot points based on updated joint angles
        self.calc_forward_kinematics(self.theta, radians=True)


    def jacobian(self, theta: list = None):
        """
        Compute the Jacobian matrix for the current robot configuration.

        Args:
            theta (list, optional): The joint angles for the robot. Defaults to self.theta.
        
        Returns:
            Jacobian matrix (3x5).
        """
        # Use default values if arguments are not provided
        if theta is None:
            theta = self.theta

        # Define DH parameters
        DH = np.zeros((5, 4))
        DH[0] = [theta[0], self.l1, 0, np.pi/2]
        DH[1] = [theta[1] + np.pi/2, 0, self.l2, np.pi]
        DH[2] = [theta[2], 0, self.l3, np.pi]
        DH[3] = [theta[3] - np.pi/2, 0, 0, -np.pi/2]
        DH[4] = [theta[4], self.l4 + self.l5, 0, 0]

        # Compute transformation matrices
        T = np.zeros((self.num_dof,4,4))
        for i in range(self.num_dof):
            T[i] = dh_to_matrix(DH[i])

        # Precompute transformation matrices for efficiency
        T_cumulative = [np.eye(4)]
        for i in range(self.num_dof):
            T_cumulative.append(T_cumulative[-1] @ T[i])

        # Define O0 for calculations
        O0 = np.array([0, 0, 0, 1])
        
        # Initialize the Jacobian matrix
        jacobian = np.zeros((3, self.num_dof))

        # Calculate the Jacobian columns
        for i in range(self.num_dof):
            T_curr = T_cumulative[i]
            T_final = T_cumulative[-1]
            
            # Calculate position vector r
            r = (T_final @ O0 - T_curr @ O0)[:3]

            # Compute the rotation axis z
            z = T_curr[:3, :3] @ np.array([0, 0, 1])

            # Compute linear velocity part of the Jacobian
            jacobian[:, i] = np.cross(z, r)

        # Replace near-zero values with zero, primarily for debugging purposes
        return near_zero(jacobian)
    
    def inverse_jacobian(self, pseudo=False):
        """
        Compute the inverse of the Jacobian matrix using either pseudo-inverse or regular inverse.
        
        Args:
            pseudo: Boolean flag to use pseudo-inverse (default is False).
        
        Returns:
            The inverse (or pseudo-inverse) of the Jacobian matrix.
        """

        J = self.jacobian()
        JT = np.transpose(J)
        manipulability_idx = np.sqrt(np.linalg.det(J @ JT))

        if pseudo:
            return np.linalg.pinv(self.jacobian())
        else:
            return np.linalg.inv(self.jacobian())
        
        
    def damped_inverse_jacobian(self, q = None, damping_factor=0.025):
        if q is not None:
            J = self.jacobian(q)
        else:
            J = self.jacobian()

        JT = np.transpose(J)
        I = np.eye(3)
        return JT @ np.linalg.inv(J @ JT + (damping_factor**2)*I)


    def dh_to_matrix(self, dh_params: list) -> np.ndarray:
        """Converts Denavit-Hartenberg parameters to a transformation matrix.

        Args:
            dh_params (list): Denavit-Hartenberg parameters [theta, d, a, alpha].

        Returns:
            np.ndarray: A 4x4 transformation matrix.
        """
        theta, d, a, alpha = dh_params
        return np.array([
            [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
            [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
            [0, sin(alpha), cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def solve_forward_kinematics(self, theta: list, radians=False):

        # Convert degrees to radians
        if not radians:
            for i in range(len(theta)):
                theta[i] = np.deg2rad(theta[i])

        # DH parameters = [theta, d, a, alpha]
        DH = np.zeros((5, 4))
        DH[0] = [theta[0],   self.l1,    0,       np.pi/2]
        DH[1] = [theta[1]+np.pi/2,   0,          self.l2, np.pi]
        DH[2] = [theta[2],   0,          self.l3, np.pi]
        DH[3] = [theta[3]-np.pi/2,   0,          0,       -np.pi/2]
        DH[4] = [theta[4],   self.l4+self.l5, 0, 0]

        T = np.zeros((self.num_dof,4,4))
        for i in range(self.num_dof):
            T[i] = dh_to_matrix(DH[i])

        return T[0] @ T[1] @ T[2] @ T[3] @ T[4] @ np.array([0, 0, 0, 1])


    def solve_inverse_kinematics(self, EE: EndEffector, tol=1e-3, ilimit=500):

        Te_d = [EE.x, EE.y, EE.z]
        
        # Iteration count
        i = 0
        q = self.theta.copy()

        # move robot slightly out of zeros singularity
        q = [q[i] + np.random.rand()*0.05 for i in range(self.num_dof)]
        
        while i < ilimit:
            i += 1

            # compute current EE position based on q
            Te = self.solve_forward_kinematics(q, radians=True)

            # print(f"Current position: {Te}")
            # calculate the EE position error
            e = [0, 0, 0]
            e[0] = Te_d[0] - Te[0]
            e[1] = Te_d[1] - Te[1]
            e[2] = Te_d[2] - Te[2]

            # update q
            J = self.jacobian(q)
            # q += np.linalg.pinv(J) @ e
            q += self.damped_inverse_jacobian(q) @ e

            # check for joint limits
            for j, th in enumerate(q):
                q[j] = np.clip(th, self.theta_limits[j][0], self.theta_limits[j][1])

            # Check if we have arrived
            if abs(max(e, key=abs)) < tol:
                break 
        
        if abs(max(e, key=abs)) > tol:
            print("\n [ERROR] Numerical IK solution failed to converge... \n \
                  Possible causes: \n \
                  1. cartesian position is not reachable by the robot, given the joint limits \n \
                  2. desired joint configuration is very close to OR at a singularity \n \
                  3. iterative algorithm is stuck at a local minima \n \
                  4. solver is taking too long to converge  \n")
            print(f"Max position error: {max(e, key=abs)} | # iterations: {i}/{ilimit} ")
            # raise ValueError
            return False

        return q


    def calc_robot_points(self):
        """ Calculates the main arm points using the current joint angles """

        # Initialize points[0] to the base (origin)
        self.points[0] = np.array([0, 0, 0, 1])

        # Precompute cumulative transformations to avoid redundant calculations
        T_cumulative = [np.eye(4)]
        for i in range(self.num_dof):
            T_cumulative.append(T_cumulative[-1] @ self.T[i])

        # Calculate the robot points by applying the cumulative transformations
        for i in range(1, 6):
            self.points[i] = T_cumulative[i] @ self.points[0]

        # Calculate EE position and rotation
        self.EE_axes = T_cumulative[-1] @ np.array([0.075, 0.075, 0.075, 1])  # End-effector axes
        self.T_ee = T_cumulative[-1]  # Final transformation matrix for EE

        # Set the end effector (EE) position
        self.ee.x, self.ee.y, self.ee.z = self.points[-1][:3]
        
        # Extract and assign the RPY (roll, pitch, yaw) from the rotation matrix
        rpy = rotm_to_euler(self.T_ee[:3, :3])
        self.ee.rotx, self.ee.roty, self.ee.rotz = rpy[0], rpy[1], rpy[2]

        # Calculate the EE axes in space (in the base frame)
        self.EE = [self.ee.x, self.ee.y, self.ee.z]
        self.EE_axes = np.array([self.T_ee[:3, i] * 0.075 + self.points[-1][:3] for i in range(3)])


    def check_valid_ik_soln(self, thetalist: list, ee_pose: EndEffector, tol: float = 0.002):

        if not check_joint_limits(thetalist, self.theta_limits):
            return False
        
        ee_position_calc = self.solve_forward_kinematics(thetalist, radians=True)
        ee_position_actual = np.array([ee_pose.x, ee_pose.y, ee_pose.z])

        # calculate the EE position error
        e = [0, 0, 0]
        e[0] = ee_position_calc[0] - ee_position_actual[0]
        e[1] = ee_position_calc[1] - ee_position_actual[1]
        e[2] = ee_position_calc[2] - ee_position_actual[2]

        # print(f'distance {np.linalg.norm(e)}')
        # print(f'calc {ee_position_calc} | act {ee_position_actual}\n')
        return True if np.linalg.norm(e) < tol else False


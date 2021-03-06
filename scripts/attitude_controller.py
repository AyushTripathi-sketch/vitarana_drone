#!/usr/bin/env python

'''
This python file runs a ROS-node of name attitude_control which controls the roll pitch and yaw angles of the eDrone.
This node publishes and subsribes the following topics:
		PUBLICATIONS            SUBSCRIPTIONS
		/roll_error             /pid_tuning_yaw
		/pitch_error            /pid_tuning_pitch
		/yaw_error              /pid_tuning_roll
		/edrone/pwm             /edrone/imu/data
								/edrone/drone_command

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
'''

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf


class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		rospy.init_node('dummy_controller',anonymous = True)  # initializing ros node with name drone_control

		# This corresponds to your current orientation of eDrone in quaternion format. This value must be updated each time in your imu callback
		# [x,y,z,w]
		self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

		# This corresponds to your current orientation of eDrone converted in euler angles form.
		# [r,p,y]
		self.drone_orientation_euler = [0.0, 0.0, 0.0]

		# This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
		# [r_setpoint, p_setpoint, y_setpoint]
		self.setpoint_cmd = [1500,1500,1500,1000]

		# The setpoint of orientation in euler angles at which you want to stabilize the drone
		# [r_setpoint, p_psetpoint, y_setpoint]
		self.setpoint_euler = [0.0,0.0,0.0]

		# Declaring pwm_cmd of message type prop_speed and initializing values
		# Hint: To see the message structure of prop_speed type the following command in the terminal
		# rosmsg show vitarana_drone/prop_speed

		self.pwm_cmd = prop_speed()
		self.pwm_cmd.prop1 = 0.0
		self.pwm_cmd.prop2 = 0.0
		self.pwm_cmd.prop3 = 0.0
		self.pwm_cmd.prop4 = 0.0

		# initial setting of Kp, Kd and ki for [roll, pitch, yaw]. eg: self.Kp[2] corresponds to Kp value in yaw axis
		# after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [28.8,28.8,0.6]
		self.Ki = [0.0,0.0,0.0]
		self.Kd = [281.7,281.7,0]
		# -----------------------Add other required variables for pid here ----------------------------------------------
		self.min_values = [0, 0, 0, 0]
		self.max_values = [1024, 1024, 1024, 1024]

		self.prev_error = [0,0,0]
		self.error = [0,0,0]
		self.differential_error = [0,0,0]
		self.sum_Iterm = [0,0,0]
		self.has_reached = 0
		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [roll, pitch, yaw]
		#        Add variables for limiting the values like self.max_values = [1024, 1024, 1024, 1024] corresponding to [prop1, prop2, prop3, prop4]
		#                                                   self.min_values = [0, 0, 0, 0] corresponding to [prop1, prop2, prop3, prop4]
		#
		# ----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.060  # in seconds

		# Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
		self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=10)

		# ------------------------Add other ROS Publishers here-----------------------------------------------------
		self.roll_pub = rospy.Publisher('/roll_error', Float32, queue_size=10)
		self.pitch_pub = rospy.Publisher('/pitch_error', Float32, queue_size=10)
		self.yaw_pub = rospy.Publisher('/yaw_error', Float32, queue_size=10)
		self.zero_pub = rospy.Publisher('/zero_error', Float32, queue_size=10)
		# -----------------------------------------------------------------------------------------------------------

		# Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
		rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
		rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
		#rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
		#rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
		#rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)
		rospy.Subscriber('/reached', Float32, self.has_reached_callback)

	# Imu callback function
	# The function gets executed each time when imu publishes /edrone/imu/data

	# Note: The imu publishes various kind of data viz angular velocity, linear acceleration, magnetometer reading (if present),
	# but here we are interested in the orientation which can be calculated by a complex algorithm called filtering which is not in the scope of this task,
	# so for your ease, we have the orientation published directly BUT in quaternion format and not in euler angles.
	# We need to convert the quaternion format to euler angles format to understand the orienataion of the edrone in an easy manner.
	# Hint: To know the message structure of sensor_msgs/Imu, execute the following command in the terminal
	# rosmsg show sensor_msgs/Imu

	def imu_callback(self, msg):

		self.drone_orientation_quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

		# --------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

	def drone_command_callback(self, msg):
		self.setpoint_cmd = [msg.rcRoll, msg.rcPitch, msg.rcYaw, msg.rcThrottle]
		print(msg)

		# ---------------------------------------------------------------------------------------------------------------

	# Callback function for /pid_tuning_roll
	# This function gets executed each time when /tune_pid publishes /pid_tuning_roll
	def roll_set_pid(self,msg):
		self.Kp[0] = msg.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[0] = msg.Ki * 0.008
		self.Kd[0] = msg.Kd * 0.3
 
	# ----------------------------Define callback function like roll_set_pid to tune pitch, yaw--------------
	def pitch_set_pid(self,msg):
		self.Kp[1] = msg.Kp * 0.06
		self.Ki[1] = msg.Ki * 0.008
		self.Kd[1] = msg.Kd * 0.3

	def yaw_set_pid(self,msg):
		self.Kp[2] = msg.Kp * 0.06
		self.Ki[2] = msg.Ki * 0.008
		self.Kd[2] = msg.Kd * 0.3

	def has_reached_callback(self,msg):
		self.has_reached = msg.data

	# ----------------------------------------------------------------------------------------------------------------------

	def pid(self):
		# -----------------------------Write the PID algorithm here--------------------------------------------------------------

		# Steps:
		#   1. Convert the quaternion format of orientation to euler angles
		(self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

		#   2. Convert the setpoint that is in the range of 1000 to 2000 into angles with the limit from -10 degree to 10 degree in euler angles
		self.setpoint_euler = [i * 0.02 - 30 for i in self.setpoint_cmd[0:3]]

		#   3. Compute error in each axis. eg: error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0], where error[0] corresponds to error in roll...
		self.error = [x1-x2 for (x1,x2) in zip(self.setpoint_euler, self.drone_orientation_euler)]
		self.error[2]=self.error[2]*10000
		#   4. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
		self.differential_error = [x1-x2 for (x1,x2) in zip(self.error,self.prev_error)]

		#   5. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
		self.out_roll = float(self.error[0] * self.Kp[0] + self.sum_Iterm[0] * self.Ki[0] + self.differential_error[0] * self.Kd[0])
		self.out_pitch = float(self.error[1] * self.Kp[1] + self.sum_Iterm[1] * self.Ki[1] + self.differential_error[1] * self.Kd[1])
		self.out_yaw = float(self.error[2] * self.Kp[2] + self.sum_Iterm[2] * self.Ki[2] + self.differential_error[2] * self.Kd[2])
		print("roll ",self.out_roll)
		print("pitch ",self.out_pitch)
		print("yaw",self.out_yaw)
		
		# Also convert the range of 1000 to 2000 to 0 to 1024 for throttle here itslef
		throttle_val =self.setpoint_cmd[3] * 1.024 - 1024
		print("throttle ",throttle_val)

		#   6. Use this computed output value in the equations to compute the pwm for each propeller. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
		self.pwm_cmd.prop1 = throttle_val + self.out_roll - self.out_pitch - self.out_yaw 
		self.pwm_cmd.prop2 = throttle_val - self.out_roll - self.out_pitch + self.out_yaw
		self.pwm_cmd.prop3 = throttle_val - self.out_roll + self.out_pitch - self.out_yaw
		self.pwm_cmd.prop4 = throttle_val + self.out_roll + self.out_pitch + self.out_yaw
		
		#   7. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
		
		#   8. Limit the output value and the final command value between the maximum(0) and minimum(1024)range before publishing. For eg : if self.pwm_cmd.prop1 > self.max_values[1]:
		if self.pwm_cmd.prop1 > self.max_values[0]:
			self.pwm_cmd.prop1 = self.max_values[0]

		if self.pwm_cmd.prop1 < self.min_values[0]:
			self.pwm_cmd.prop1 = self.min_values[0]

		if self.pwm_cmd.prop2 > self.max_values[1]:
			self.pwm_cmd.prop2 = self.max_values[1]

		if self.pwm_cmd.prop2 < self.min_values[1]:
			self.pwm_cmd.prop2 = self.min_values[1]

		if self.pwm_cmd.prop3 > self.max_values[2]:
			self.pwm_cmd.prop3 = self.max_values[2]

		if self.pwm_cmd.prop3 < self.min_values[2]:
			self.pwm_cmd.prop3 = self.min_values[2]

		if self.pwm_cmd.prop4 > self.max_values[3]:
			self.pwm_cmd.prop4 = self.max_values[3]

		if self.pwm_cmd.prop4 < self.min_values[3]:
			self.pwm_cmd.prop4 = self.min_values[3]                                                                                                                                
		
		#   8. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
		self.prev_error = self.error
		#   9. Add error_sum to use for integral component
		self.sum_Iterm = [x1+x2 for (x1,x2) in zip(self.sum_Iterm,self.error)]

		# publish topics
		self.pwm_pub.publish(self.pwm_cmd)
		self.roll_pub.publish(self.error[0])
		self.pitch_pub.publish(self.error[1])
		self.yaw_pub.publish(self.error[2])
		self.zero_pub.publish(self.error[0]-self.error[0])

if __name__ == '__main__':

	e_drone = Edrone()
	r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		if(e_drone.has_reached):
			break
		e_drone.pid()
		r.sleep()

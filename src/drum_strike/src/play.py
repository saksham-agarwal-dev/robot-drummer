#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_commander import MoveGroupCommander
from cv.msg import DetectedNote

class Player:
	def __init__(self):
		# Wait for the IK service to become available
		rospy.wait_for_service('compute_ik')
		rospy.init_node('player_controller', anonymous=True)

		# Create the function used to call the service
		self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

		# Home pose is the default; we should always return here after a hit.
		# Red pose hits the center of the drum.
		# Blue pose hits the rim of the drum.

		# Azula/Ada
		# self.home_pose = Pose(position = Point(x=0.829, y=0.172, z=0.065), 
		# 						orientation = Quaternion(x=0, y=1, z=0, w=0))
		# self.red_pose = Pose(position = Point(x=0.817, y=0.167, z=-0.074), 
		# 						orientation = Quaternion(x=0, y=1, z=0, w=0))
		# self.blue_pose = Pose(position = Point(x=0.897, y=0.14, z=-0.03), 
		# 						orientation = Quaternion(x=0, y=1, z=0, w=0))

		# Alan
		self.home_pose = Pose(position = Point(x=0.484, y=0.180, z=0.073), 
								orientation = Quaternion(x=0, y=1, z=0, w=0))
		self.red_pose = Pose(position = Point(x=0.521, y=0.143, z=-0.057), 
								orientation = Quaternion(x=0, y=1, z=0, w=0))
		self.blue_pose = Pose(position = Point(x=0.620, y=0.168, z=-0.038), 
								orientation = Quaternion(x=0, y=1, z=0, w=0))
		
		# move the robot home first
		input('Press [ Enter ] to move to home position: ')
		self.move(self.home_pose)
		input('Press [ Enter ] to start playing! ')

		# Queue size is 0, since when the arm is moving to hit we don't want to detect any notes.
		# If we miss any points that's alright, since it would mean the robot physically wouldn't be able to hit it fast enough anyways.
		self.note_sub = rospy.Subscriber('detected_note', DetectedNote, self.play_callback, queue_size=1) 

		# tracks double counting, so we only hit when this is even
		self.double_count_flag = 0
		
		rospy.spin()

	'''
		Called when a note is detected. 
	'''
	def play_callback(self, note):
		# each note is detected twice, so ignore the second reading
		# the subscriber's queue will handle keeping the most recently read note in the queue
		if self.double_count_flag % 2 == 1:
			self.double_count_flag += 1
			return
		
		self.play_note(note)
		self.double_count_flag += 1

	def play_note(self, note):

		if note.is_red:
			print("AKA")
			self.move(self.red_pose)
			self.move(self.home_pose)

		elif note.is_blue:
			print("AO")
			self.move(self.blue_pose)
			self.move(self.home_pose)
			
		return

	'''
		Moves the arm to the given position. 
	'''
	def move(self, target_pose):
		# Construct the request
		request = GetPositionIKRequest()
		request.ik_request.group_name = "right_arm"

		link = "right_gripper_tip"

		request.ik_request.ik_link_name = link
		request.ik_request.pose_stamped.header.frame_id = "base"
		request.ik_request.pose_stamped.pose = target_pose
		
		try:
			# Send the request to the service
			response = self.compute_ik(request)
			
			# Print the response
			# print(response)
			group = MoveGroupCommander("right_arm")

			# Set the position and orientation target
			group.set_pose_target(request.ik_request.pose_stamped)

			# Increase velocity and acceleration factors, and set planning time to max 1 second
			group.set_max_velocity_scaling_factor(1)
			group.set_max_acceleration_scaling_factor(1)
			group.set_planning_time(1)

			# Plan inverse kinematics
			plan = group.plan()
			# user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
			
			# Execute inverse kinematics if it's safe
			# if user_input == 'y':
			group.execute(plan[1])
			
		except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

if __name__ == '__main__':
	Player()

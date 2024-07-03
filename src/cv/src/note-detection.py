#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from cv.msg import DetectedNote

# Detects notes based on color (red and blue for now), from the input camera
class ObjectDetector:
	def __init__(self):
		rospy.init_node("note_detection", anonymous=True)

		self.bridge = CvBridge()
		self.cv_color_image = None
		self.color_image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.color_image_callback)
		self.note_pub = rospy.Publisher('detected_note', DetectedNote, queue_size=1)

		rospy.spin()

	def color_image_callback(self, msg):
		try:
			# Convert the ROS Image message to an OpenCV image (BGR8 format)
			self.cv_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

			# Trim the image to only include the bounding box
			trimmed_img = self.image_trim(self.cv_color_image)

			# Run cv detection
			self.detect_note(trimmed_img)

		except Exception as e:
			print("Error: ", e)

	'''
		Cuts out a bounding box from the camera image, to focus on a single location for detection.
		Input: CV color image, BGR8 format.
		Output: Cropped image; the box is located such that after a note is detected, 
				the time for the robot to move to the position matches with the time the note has left to the hit point.
	'''
	def image_trim(self, img):
		# x0 = 395
		# y0 = 245
		# x1 = 455
		# y1 = 295
		x0 = 845
		y0 = 245
		x1 = 905
		y1 = 295
		cropped_img = img[y0:y1, x0:x1]
		cv2.rectangle(img, (x0, y0), (x1, y1), color=(0,255,0), thickness=2)

		# cv2.imshow('frame', cropped_img)
		cv2.imshow('frame', img)
		cv2.waitKey(1) 
		return cropped_img
	
	'''
		Given the cropped image, detects red and blue notes. 
		Red corresponds to a center hit on the drum. Blue is a hit on the rim/edge. 
		Output: string of the note color detected. 
	'''
	def detect_note(self, image):
		# Convert the color image to HSV color space
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

		# mean_center_row_hsv_val = np.mean(hsv[len(hsv)//2], axis=0)
		# print("Current mean values at center row of image: ", mean_center_row_hsv_val)

		# Extract red notes; red wraps around the color spectrum so we need 2 ranges
		lower_red1 = np.array([0, 180, 122.191], np.uint8)
		upper_red1 = np.array([80, 282.583, 212.1916], np.uint8)
		lowerMaskRed = cv2.inRange(hsv, lower_red1, upper_red1)

		# Probably need to fine tune this, for when the background turns fire
		lower_red2 = np.array([170, 120, 70], np.uint8)
		upper_red2 = np.array([180, 255, 255], np.uint8)
		upperMaskRed = cv2.inRange(hsv, lower_red2, upper_red2)
								
		red_y_coords, red_x_coords = np.nonzero(lowerMaskRed + upperMaskRed)

		# Extract blue note
		lower_blue = np.array([85.95, 103.9, 232.45], np.uint8)
		upper_blue = np.array([105.95, 123.9, 252.45], np.uint8)
		maskBlue = cv2.inRange(hsv, lower_blue, upper_blue)
		blue_y_coords, blue_x_coords = np.nonzero(maskBlue)

		# Check if we didn't find any notes. Otherwise: 
		# Choose the mask/color with the most identified pixels/coordinates, if they pass the threshold. 
		threshold = 5
		num_red = len(red_x_coords) + len(red_y_coords)
		num_blue = len(blue_x_coords) + len(blue_y_coords)

		if num_red > threshold:
			print("Detected red note.")
			self.note_pub.publish(DetectedNote(is_red=True, is_blue=False))

		elif num_blue > threshold:
			print("Detected blue note.")
			self.note_pub.publish(DetectedNote(is_red=False, is_blue=True))
			
		# If neither meet the threshold, do nothing
		return 

if __name__ == "__main__":
    ObjectDetector()
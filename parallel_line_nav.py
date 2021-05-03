#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from numpy import pi
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Int32MultiArray
import geometry_msgs.msg
from geometry_msgs.msg import Polygon, PolygonStamped, Point32
import tf2_ros
import time
import json

# Physical params
robot_length = 0.42
robot_width = 0.35
from_back_to_lidar = 0.39

# Behavior params
stop_dist = 0.5
avoid_length = 0.50 # 1.0
avoid_width = 0.75

#									Y
#									|					             								
#									|								 |<-avoid_length->|
#								|	|                   |<-stop_dist>|                .
#								|<-----robot_length---->|            .                .
#								|	|                   |            .                .
#	P---------------------------M	|					G-----------------------------I		<-
#	.			   .			|	|					|			 .				  .	     |
#	.			   L------------A___|___________________B------------E				  .      |
#	.			   .        	|	|		^			|  	 		 .				  .      |
#	.			   .	    	|   |		|			|            .                .      |
#	.			   .      		|   |		|			|      	     .				  .      |
#	.			   .    		|	|		|			|            .	              .      | avoid_width
#	.			   .  			|	|	robot_width		|	         .                .      |
#	S--------------W------------|---O-------------------|------------T----------------K------|------- X
#	.			   . 			|	|		|			|   	     .                .      |
#	.			   .    		|	|		|			|		     .                .      |
#	.			   .      		|	|		|			| 	         .                .      |
#	.			   .        	|	|		|			|            .                .      | 
#	.			   .		 	|	|		|			|            .                .      |
#	.			   R------------C___|_______V___________D------------F                .      |
#	.			   .			|	|					|            .                .      |
#	Q---------------------------N	|					H-----------------------------J     <-
#
# Robot's footprint is ABCD
# O is where the lidar is placing
# robot is facing forward on X-direction
# BDT and ACW are the STOP_ZONE

# AB is robot length
# BD is robot width
# AC_O is lidar offset from back side

AB = robot_length
BD = robot_width
AC_O = from_back_to_lidar

# Footprint coords
A = (-AC_O, robot_width/2)
B = (robot_length-AC_O, robot_width/2)
C = (-AC_O, -robot_width/2)
D = (robot_length-AC_O, -robot_width/2)
# Front Stop and Avoid zone coords
G = ((robot_length-AC_O), avoid_width/2)
H = ((robot_length-AC_O), -avoid_width/2)
E = ((robot_length-AC_O+stop_dist), robot_width/2)
F = ((robot_length-AC_O+stop_dist), -robot_width/2)
I = ((robot_length-AC_O+stop_dist+avoid_length), avoid_width/2)
J = ((robot_length-AC_O+stop_dist+avoid_length), -avoid_width/2)
T = ((robot_length-AC_O+stop_dist), 0)
K = ((robot_length-AC_O+stop_dist+avoid_length), 0)
# Back Stop and Avoid zone coords
M = (-AC_O, avoid_width/2)
N = (-AC_O, -avoid_width/2)
L = ((-AC_O-stop_dist), robot_width/2)
R = ((-AC_O-stop_dist), -robot_width/2)
P = ((-AC_O-stop_dist-avoid_length), avoid_width/2)
Q = ((-AC_O-stop_dist-avoid_length), -avoid_width/2)
W = ((-AC_O-stop_dist), 0)
S = ((-AC_O-stop_dist-avoid_length), 0)


## Stop zone of triangle shape
## Front
# slope and y-intercept of first quadant
m1 = (T[1] - B[1])/(T[0] - B[0])
b1 = B[1] - m1*(B[0])
# slope and y-intercept of second quadant
m2 = (T[1] - D[1])/(T[0] - D[0])
b2 = D[1] - m2*(D[0])

## Back
# slope and y-intercept of thrid quadant
m3 = (A[1] - W[1])/(A[0] - W[0])
b3 = A[1] - m3*(A[0])
# slope and y-intercept of fourth quadant
m4 = (C[1] - W[1])/(C[0] - W[0])
b4 = C[1] - m4*(C[0])



print("A", A)
print("B", B)
print("C", C)
print("D", D)
print("E", E)
print("F", F)
print("G", G)
print("H", H)
print("I", I)
print("J", J)
print("K", K)
print("L", L)
print("M", M)
print("N", N)
print("P", P)
print("Q", Q)
print("R", R)
print("S", S)
print("T", T)
print("W", W)

# quit()


class LineDetector:

	def __init__(self):

		rospy.init_node('parallel_nav_node', anonymous=True)
		rospy.Subscriber("/scan", LaserScan, self.scan_callback)
		rospy.Subscriber("/console_drive_msg", String, self.console_callback)

		self.image_pub = rospy.Publisher("image_topic", Image)
		# self.bw_image_pub = rospy.Publisher("bw_image_topic", Image)
		self.bridge = CvBridge()

		self.sbus_cmd_pub = rospy.Publisher("/sbus_cmd", Int32MultiArray, queue_size=10)
		self.sbus_cmd = Int32MultiArray()

		self.foot_pub = rospy.Publisher("/footprint", PolygonStamped)
		self.front_stop_pub = rospy.Publisher("/front_stop_zone", PolygonStamped)
		self.back_stop_pub = rospy.Publisher("/back_stop_zone", PolygonStamped)

		self.pg = PolygonStamped()
		self.pg_front_stop = PolygonStamped()
		self.pg_back_stop = PolygonStamped()

		self.br = tf2_ros.TransformBroadcaster()
		self.t = geometry_msgs.msg.TransformStamped()	

		########################### Image Parameters ##########################
		##### Frame size #####
		self.frame_width = 300
		self.frame_height = 600 
		# self.frame_width_spacing = 140	#250
		# self.frame_height_spacing = 140	#150

		self.frame_real_width = 2.0	#float(self.frame_width/self.frame_width_spacing)
		self.frame_real_height = 4.0	#float(self.frame_height/self.frame_height_spacing)

		##### Blank map #####
		self._map = np.ones((self.frame_height, self.frame_width, 3), np.uint8) * 255
		self.amap = np.copy(self._map)

		##### Line detection #####
		# Canny parameters don't affect much on this bitmap image
		self.cannyMinVal = 50	# 
		self.cannyMaxVal = 200   # 
		# houghThresh is important in case of non-solid wall (cage, poles)
		self.houghThresh = 25	#10	#20	# 50
		self.houghMinLength = 10 # 5
		self.houghMaxGap = 150	# 150

		##### Steering-adjusting zone on image #####
		## TODO: change lane_width_meter according to real lane width (estimate)
		## if one line is disappear we will keep the bot far away from half of this width
		self.lane_width_meter = 0.7
		self.half_lane_width_meter =  self.lane_width_meter/2.0
		self.half_lane_width_pixel = self.meter2pixel(self.half_lane_width_meter)

		self.frame_width_half = int(self.frame_width/2)
		self.frame_height_half = int(self.frame_height/2)

		self.x_mid_lane_MAX = int(self.frame_width_half) + self.half_lane_width_pixel/1.2#70
		self.x_mid_lane_MIN = int(self.frame_width_half) - self.half_lane_width_pixel/1.2#70
		self.center_offset_left = 3
		self.center_offset_right = 3
		self.center_min_DB = int(self.frame_width_half - self.center_offset_left)
		self.center_max_DB = int(self.frame_width_half + self.center_offset_right)

		self.center_thr_adj_min = int((self.x_mid_lane_MIN + self.center_min_DB)/2) 
		self.center_thr_adj_max = int((self.x_mid_lane_MAX + self.center_max_DB)/2) 

		##### Rearrange and plotting #####
		self.left_line_list = []
		self.right_line_list = []
		# it would be this sequence, 
		# self.left_line_list.append([y_top, y_buttom, x_of_y_top, x_of_y_buttom])
		# so left_line_list[*][0] is always the y min value
		#    left_line_list[*][1] is always the y max value
		#    left_line_list[*][2] is the correct pair of y min
		#    left_line_list[*][3] is the correct pair of y max
		self.y_L_dist = np.array([])
		self.y_R_dist = np.array([])

		##### Mask #####
		## Focus on front half frame
		focus_offset = 50
		self.front_vertices = np.array([[int(self.x_mid_lane_MIN - focus_offset), 0],
					[int(self.x_mid_lane_MIN - focus_offset), self.frame_height_half],
					[int(self.x_mid_lane_MAX + focus_offset),self.frame_height_half],
					[int(self.x_mid_lane_MAX + focus_offset),0]], 
					np.int32)

		self.back_vertices = np.array([[int(self.x_mid_lane_MIN - focus_offset), self.frame_height_half],
					[int(self.x_mid_lane_MIN - focus_offset), self.frame_height],
					[int(self.x_mid_lane_MAX + focus_offset),self.frame_height],
					[int(self.x_mid_lane_MAX + focus_offset),self.frame_height_half]], 
					np.int32)

		self.sides_vertices = np.array([[int(self.x_mid_lane_MIN - focus_offset), 0],
					[int(self.x_mid_lane_MIN - focus_offset), self.frame_height],
					[int(self.x_mid_lane_MAX + focus_offset),self.frame_height],
					[int(self.x_mid_lane_MAX + focus_offset),0]], 
					np.int32)



		
		########################### Laser Scan Parameters ##########################
		self.angle_list = None
		self.range_list = None

		self.X = np.array([])
		self.Y = np.array([])

		self.PX = np.array([])
		self.PY = np.array([])


		########################### Robot Parameters ##########################
		##### Steering adjust #####
		self.sbus_steering_mid = 1024
		self.sbus_throttle_mid = 1031	#1024

		self.sbus_throttle_fwd_const = self.sbus_throttle_mid + 59	# =1090
		self.sbus_throttle_bwd_const = self.sbus_throttle_mid - 59	# =972

		self.sbus_throttle_adj = 12
		self.sbus_throttle_fwd_max = self.sbus_throttle_fwd_const
		self.sbus_throttle_fwd_min = self.sbus_throttle_fwd_const - self.sbus_throttle_adj
		self.sbus_throttle_bwd_max = self.sbus_throttle_bwd_const
		self.sbus_throttle_bwd_min = self.sbus_throttle_bwd_const + self.sbus_throttle_adj

		self.sbus_steering_adj = 12	#31			# adjusted range
		self.sbus_steering_max_DB = 1044	# 1024+24  the lowest value of steering to the left
		self.sbus_steering_min_DB = 996	# 1024-24  the lowest value of steering to the right
		self.sbus_steering_max = self.sbus_steering_max_DB + self.sbus_steering_adj
		self.sbus_steering_min = self.sbus_steering_min_DB - self.sbus_steering_adj

		self.sbus_steering_BWD_max_DB = 1052	# 1024+24  the lowest value of steering to the left
		self.sbus_steering_BWD_min_DB = 1004	# 1024-24  the lowest value of steering to the right
		self.sbus_steering_BWD_max = self.sbus_steering_BWD_max_DB + self.sbus_steering_adj
		self.sbus_steering_BWD_min = self.sbus_steering_BWD_min_DB - self.sbus_steering_adj

		########################### Filter Parameters ##########################
		self.line_list_length = 10	# use on moving average filter

		self.steering_list_length = 10#100	# for moving average filter

		##################### Robot Stop zone ########################
		self.FRONT_STOP = False
		self.BACK_STOP = False

		self.ROBOT_MODE = 'STOP'
		self.prev_ROBOT_MODE = self.ROBOT_MODE

		##################### Linear regression ########################
		# self.x_left_array = np.array([])
		# self.y_left_array = np.array([])

		##################### Console variables ########################
		self.CONSOLE_ARM = 0
		self.CONSOLE_MODE = 'STOP'

		########################### Start ##########################
		self.run()

		rospy.spin()

	def console_callback(self, msg):
		parse_data = json.loads(msg.data)

		self.CONSOLE_ARM = parse_data["ARM"]
		self.CONSOLE_MODE = parse_data["MODE"].encode("ascii", "ignore")

		if self.CONSOLE_MODE == 'FRONT':
			self.ROBOT_MODE = 'FRONT'
		elif self.CONSOLE_MODE == 'BACK':
			self.ROBOT_MODE = 'BACK'
		elif self.CONSOLE_MODE == 'STOP':
			self.ROBOT_MODE = 'STOP'
		else:
			self.ROBOT_MODE = 'STOP'


	def inFrontStopZone(self, x,y):
		if (B[0] < x) and (x < T[0]):
			if (D[1] < y) and (y < B[1]):
				y_cal1 = m1*x + b1
				y_cal2 = m2*x + b2
				# print("y_cal1", y_cal1)
				# print("y_cal2", y_cal2)
				if (y < y_cal1) and (y > y_cal2):
					return True
				else:
					return False

			else:
				return False

		else:
			return False

	def inBackStopZone(self, x, y):

		if (W[0] < x) and (x < A[0]):
			if (C[1] < y) and (y < A[1]):

				y_cal3 = m3*x + b3
				y_cal4 = m4*x + b4

				if (y < y_cal3) and (y > y_cal4):
					return True
				else:
					return False
			else:
				return False
		else:
			return False



	def send_throttle_steering(self, sbus_throttle, sbus_steering):
		udpPacket = struct.pack('HH', sbus_throttle, sbus_steering)
		self.s.sendto(udpPacket, (self.MOAB_COMPUTER, self.MOAB_PORT))

	def width_pixel2meter(self, width_px):
		width_meter = (self.frame_real_width*width_px)/self.frame_width
		return width_meter

	def meter2pixel(self,input_meter):
		output_pixel = (self.frame_width/self.frame_real_width)*input_meter
		return output_pixel

	def roi(self, img, vertices):
		#blank mask:
		mask = np.zeros_like(img)
		# fill the mask
		cv2.fillPoly(mask, [vertices], (255,255,255))
		# now only show the area that is the mask
		masked = cv2.bitwise_and(img, mask)
		return masked

	def map_with_limit(self, val, in_min, in_max, out_min, out_max):

		# out = ((val - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min
		## in_min must be the minimum input 
		## in_max must be the maximum input

		## out_min is supposed to map with in_min value
		## out_max is sipposed to map with in_max value
		## out_min can be less/more than out_max, doesn't matter


		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min

		if out_min > out_max:
			if out > out_min:
				out = out_min
			elif out < out_max:
				out = out_max
			else:
				pass
		elif out_max > out_min:
			if out > out_max:
				out = out_max
			elif out < out_min:
				out = out_min
			else:
				pass
		else:
			pass

		# print(m, val, in_min, in_max, out_min, out_max)

		return out

	def map(self, val, in_min, in_max, out_min, out_max):

		# out = ((val - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min
		## in_min must be the minimum input 
		## in_max must be the maximum input

		## out_min is supposed to map with in_min value
		## out_max is sipposed to map with in_max value
		## out_min can be less/more than out_max, doesn't matter

		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min

		return out

	def scan_callback(self, msg):

		self.range_list = msg.ranges
		self.angle_list = np.arange((msg.angle_min), msg.angle_max, msg.angle_increment)

		## using numpy array multiplication is much efficient than for loop each element
		self.X = np.multiply(self.range_list, np.cos(self.angle_list))
		self.Y = np.multiply(self.range_list, np.sin(self.angle_list))

		## using numpy broadcasting to map X -> PY and to map Y -> PX
		self.PY = self.map(self.X, -(self.frame_real_height/2), (self.frame_real_height/2), self.frame_height, 0)
		self.PX = self.map(self.Y, -(self.frame_real_width/2), (self.frame_real_width/2), self.frame_width, 0)


		## Use X and Y to check if there is point in front-stop-zone
		for i, (x,y) in enumerate(zip(self.X, self.Y)):
			if self.inFrontStopZone(x,y) and self.ROBOT_MODE == 'FRONT':
				# print("{:d}  stop at x: {:.2f}  y: {:.2f}".format(i, x, y))
				self.FRONT_STOP = True
				self.ROBOT_MODE = 'BACK'
				break
			else:
				self.FRONT_STOP = False

			if self.inBackStopZone(x,y) and self.ROBOT_MODE == 'BACK':
				self.BACK_STOP = True
				break
			else:
				self.BACK_STOP = False


	def MovingAverage(self, data_raw, data_array, array_size):

		if len(data_array) < array_size:
			data_array = np.append(data_array, data_raw)
		else:
			data_array = np.delete(data_array, 0)
			data_array = np.append(data_array, data_raw)
		data_ave = np.average(data_array)
		return data_ave, data_array

	def image_bitmap(self, px, py):
		if px > 0 and px < self.frame_width:
			if py > 0 and py < self.frame_height:
				self.amap[py,px] = 0,0,0

	def run(self):

		rate = rospy.Rate(20) # 10hz

		steering_list = np.array([])
		x_min_list = np.array([])
		x_max_list = np.array([])
		y_min_list = np.array([])
		y_max_list = np.array([])

		m = 0.0
		prev_sbus_steering = 1024
		last_front_stop_time = time.time()
		last_back_stop_time = time.time()
		while not rospy.is_shutdown():
			# startTime = time.time()
			## reset the list if the direction change
			if self.ROBOT_MODE != self.prev_ROBOT_MODE:
				x_min_list = np.array([])
				x_max_list = np.array([])
				y_min_list = np.array([])
				y_max_list = np.array([])
				steering_list = np.array([])

			if (self.CONSOLE_ARM == 1):

				## Use PX and PY to bitmap image
				for px,py in zip(self.PX, self.PY):
					self.image_bitmap(int(px),int(py))
			
				## Line detection
				processed_img = cv2.cvtColor(self.amap, cv2.COLOR_BGR2GRAY)
				processed_img = cv2.Canny(processed_img, self.cannyMinVal, self.cannyMaxVal)
				
				## In case of pigfarm, have a long-range view is better than crop fron-back
				# if (self.ROBOT_MODE == "FRONT"):
				# 	## roi on processed image to filter undesired area out
				# 	processed_img = self.roi(processed_img, self.front_vertices)
				# 	## this roi is for visualizing, how much we filter the side area out
				# 	## can be commented out
				# 	self.amap = self.roi(self.amap, self.front_vertices)
				# elif (self.ROBOT_MODE == "BACK"):
				# 	## roi on processed image to filter undesired area out
				# 	processed_img = self.roi(processed_img, self.back_vertices)
				# 	## this roi is for visualizing, how much we filter the side area out
				# 	## can be commented out
				# 	self.amap = self.roi(self.amap, self.back_vertices)

				if (self.ROBOT_MODE == "FRONT") or (self.ROBOT_MODE == "BACK"):
					processed_img = self.roi(processed_img, self.sides_vertices)
					self.amap = self.roi(self.amap, self.sides_vertices)
	

				## If HoughLineP can find line features on the image
				linesP = cv2.HoughLinesP(processed_img, 1, pi/180, self.houghThresh, minLineLength=self.houghMinLength, maxLineGap=self.houghMaxGap)
				if linesP is not None:
					slope = np.array([])

					for i in range(0, len(linesP)):
						l = linesP[i][0]
						x1 = l[0]
						y1 = l[1]
						x2 = l[2]
						y2 = l[3]

						## Perfect vertical line
						if (x1==x2):
							m = None
							x = x1

							## We are trying to rearrange the order of x1,y1 as min pixel and x2,y2 as max pixel
							if x < (self.frame_width/2):
								if y2 < y1:
									self.left_line_list.append([y2,y1,x2,x1])
								else:
									# flip the lowset value of y to first element, same with x pair of it
									self.left_line_list.append([y1,y2,x1,x2])
							else:
								if y2 < y1:
									self.right_line_list.append([y2,y1,x2,x1])
								else:
									# flip the lowset value of y to first element, same with x pair of it
									self.right_line_list.append([y1,y2,x1,x2])

						else:
							m = (float(y2)-float(y1))/(float(x2)-float(x1))
							x_ave = (x2+x1)/2.0

							x = x_ave

							## Horizontal line with slope
							## if slope is pretty less (mostly horizontal flat) or if slope that not much vertical
							if (abs(m) < 1) or (abs(m) > 15): 
								pass
								# msg = "NO_LINE"
								# cv2.line(self.amap, (x1, y1), (x2, y2), (150,150,150), 1, cv2.LINE_AA)

							## Vertical line with slope
							else:
								## We are trying to rearrange the order of x1,y1 as min pixel and x2,y2 as max pixel
								## same as perfect vertical line above case
								if x < (self.frame_width/2):
									if y1 > y2:
										self.left_line_list.append([y2,y1,x2,x1])
									else:
										# flip the lowset value of y to first element, same with x pair of it
										self.left_line_list.append([y1,y2,x1,x2])
								else:
									if y1 > y2:
										self.right_line_list.append([y2,y1,x2,x1])
									else:
										self.right_line_list.append([y1,y2,x1,x2])

						# print("i: %d, m: %s, %d %d %d %d" %(i,str(m),y2,y1,x2,x1))

				## HoughLineP cannot find any line
				else:
					msg = "NO_LINE"
					self.left_line_list = []
					self.right_line_list = []
					self.y_L_dist = np.array([])
					self.y_R_dist = np.array([])


				

				## Draw a circle do at middle of image
				cv2.circle(self.amap, (self.frame_width_half,self.frame_height_half), 4, (0,0,0), -1)

				## Find the best candidate line on each left and right side and plot
				## Left line
				if len(self.left_line_list) > 0:
					## Calculate the height distance of line, it should simply be ymax - ymin
					for left_line_elem in self.left_line_list:
						self.y_L_dist = np.append(self.y_L_dist, (left_line_elem[1] - left_line_elem[0]))

					## Using argmax to find the longest line in the y_L_dist list
					idx_L = np.argmax(self.y_L_dist)

					## Use that longest line as a candidate of left side line
					y_min_plot_L = self.left_line_list[idx_L][0]
					y_max_plot_L = self.left_line_list[idx_L][1]
					x_min_plot_L = self.left_line_list[idx_L][2]
					x_max_plot_L = self.left_line_list[idx_L][3]

					## Draw candidate left side line
					cv2.line(self.amap, (x_min_plot_L, y_min_plot_L), (x_max_plot_L, y_max_plot_L), (255,0,0), 1, cv2.LINE_AA)
					x_mid_L = (x_min_plot_L + x_max_plot_L)/2
					y_mid_L = (y_min_plot_L + y_max_plot_L)/2
					# clear array out
					self.y_L_dist = np.array([])
				else:
					x_mid_L = 0

				## Right line
				if len(self.right_line_list) > 0:
					## Calculate the height distance of line, it should simply be ymax - ymin		
					for right_line_elem in self.right_line_list:
						self.y_R_dist = np.append(self.y_R_dist, (right_line_elem[1] - right_line_elem[0]))

					## Using argmax to find the longest line in the y_R_dist list
					idx_R = np.argmax(self.y_R_dist)

					## Use that longest line as a candidate of right side line
					y_min_plot_R = self.right_line_list[idx_R][0]
					y_max_plot_R = self.right_line_list[idx_R][1]
					x_min_plot_R = self.right_line_list[idx_R][2]
					x_max_plot_R = self.right_line_list[idx_R][3]
					cv2.line(self.amap, (x_min_plot_R, y_min_plot_R), (x_max_plot_R, y_max_plot_R), (0,255,0), 1, cv2.LINE_AA)
					x_mid_R = (x_min_plot_R + x_max_plot_R)/2
					y_mid_R = (y_min_plot_R + y_max_plot_R)/2
					# clear array out
					self.y_R_dist = np.array([])
				else:
					x_mid_R = 0

				## If two lines of left and right sides are existing
				if x_mid_L != 0 and x_mid_R != 0:

					## We calculate the average of start and end point between left and right lines
					x_min_plot_MID = (x_min_plot_L + x_min_plot_R)/2
					y_min_plot_MID = ((y_min_plot_L + y_min_plot_R)/2) 
					x_max_plot_MID = (x_max_plot_L + x_max_plot_R)/2
					y_max_plot_MID = ((y_max_plot_L + y_max_plot_R)/2) 
					case = "LR_"

				elif x_mid_L != 0 and x_mid_R == 0:
					x_min_plot_MID = x_min_plot_L + self.half_lane_width_pixel
					y_min_plot_MID = y_min_plot_L
					x_max_plot_MID = x_max_plot_L + self.half_lane_width_pixel
					y_max_plot_MID = y_max_plot_L
					case = "L_"

				elif x_mid_L == 0 and x_mid_R != 0:
					x_min_plot_MID = x_min_plot_R - self.half_lane_width_pixel
					y_min_plot_MID = y_min_plot_R
					x_max_plot_MID = x_max_plot_R - self.half_lane_width_pixel
					y_max_plot_MID = y_max_plot_R
					case = "R_"

				else:
					# print("Some lines are missing...")
					sbus_steering = 1024	#prev_sbus_steering#
					if self.ROBOT_MODE == "FRONT":
						sbus_throttle = self.sbus_throttle_fwd_const
					elif self.ROBOT_MODE == "BACK":
						sbus_throttle = self.sbus_throttle_bwd_const
					else:
						sbus_throttle = self.sbus_throttle_mid				

					x_min_plot_MID = 0
					x_max_plot_MID = 0
					case = "NO_"


				## Care only if it found a line
				if case != "NO_":
					## Fliter noisy points
					x_min_plot_MID, x_min_list = self.MovingAverage(x_min_plot_MID, x_min_list, self.line_list_length)
					x_max_plot_MID, x_max_list = self.MovingAverage(x_max_plot_MID, x_max_list, self.line_list_length)
					y_min_plot_MID, y_min_list = self.MovingAverage(y_min_plot_MID, y_min_list, self.line_list_length)
					y_max_plot_MID, y_max_list = self.MovingAverage(y_max_plot_MID, y_max_list, self.line_list_length)

					cv2.line(self.amap, (int(x_min_plot_MID), int(y_min_plot_MID)), (int(x_max_plot_MID), int(y_max_plot_MID)), (0,0,255), 1, cv2.LINE_AA)
					cv2.line(self.amap, (self.frame_width_half, int(y_min_plot_MID)), (self.frame_width_half, int(y_max_plot_MID)), (0,0,0), 1, cv2.LINE_AA)

					if self.ROBOT_MODE == 'FRONT':
						if (x_min_plot_MID < self.center_min_DB):
							sbus_steering = int(self.map_with_limit(x_min_plot_MID, self.x_mid_lane_MIN, self.center_min_DB , self.sbus_steering_max, self.sbus_steering_max_DB))
							# sbus_throttle = self.sbus_throttle_const
							sbus_throttle = int(self.map_with_limit(x_min_plot_MID, self.x_mid_lane_MIN, self.center_thr_adj_min, self.sbus_throttle_fwd_min, self.sbus_throttle_fwd_const))
							case = case + "FWD_STR_L"
						elif (x_min_plot_MID > self.center_max_DB):
							sbus_steering = int(self.map_with_limit(x_min_plot_MID, self.center_max_DB, self.x_mid_lane_MAX, self.sbus_steering_min_DB, self.sbus_steering_min))
							# sbus_throttle = self.sbus_throttle_const
							sbus_throttle = int(self.map_with_limit(x_min_plot_MID, self.center_thr_adj_max, self.x_mid_lane_MAX, self.sbus_throttle_fwd_const, self.sbus_throttle_fwd_min))
							case = case + "FWD_STR_R"
						else:
							# sbus_steering = 1024
							sbus_steering = int(self.map_with_limit(x_min_plot_MID, self.center_min_DB, self.center_max_DB, (self.sbus_steering_max_DB - 1), (self.sbus_steering_min_DB + 1)))
							sbus_throttle = self.sbus_throttle_fwd_const
							case = case + "FWD_THR"

							
					elif self.ROBOT_MODE == 'BACK':
						if (x_max_plot_MID < self.center_min_DB):
							sbus_steering = int(self.map_with_limit(x_max_plot_MID, self.x_mid_lane_MIN, self.center_min_DB , self.sbus_steering_BWD_max, self.sbus_steering_BWD_max_DB))
							# sbus_throttle = self.sbus_throttle_const
							sbus_throttle = int(self.map_with_limit(x_max_plot_MID, self.x_mid_lane_MIN, self.center_thr_adj_min, self.sbus_throttle_bwd_min, self.sbus_throttle_bwd_const))
							case = case + "BWD_STR_L"
						elif (x_max_plot_MID > self.center_max_DB):
							sbus_steering = int(self.map_with_limit(x_max_plot_MID, self.center_max_DB, self.x_mid_lane_MAX, self.sbus_steering_BWD_min_DB, self.sbus_steering_BWD_min))
							# sbus_throttle = self.sbus_throttle_const
							sbus_throttle = int(self.map_with_limit(x_max_plot_MID, self.center_thr_adj_max, self.x_mid_lane_MAX, self.sbus_throttle_bwd_const, self.sbus_throttle_bwd_min))
							case = case + "BWD_STR_R"
						else:
							# sbus_steering = 1024
							sbus_steering = int(self.map_with_limit(x_max_plot_MID, self.center_min_DB, self.center_max_DB, (self.sbus_steering_BWD_max_DB - 1), (self.sbus_steering_BWD_min_DB + 1)))
							sbus_throttle = self.sbus_throttle_bwd_const
							case = case + "BWD_THR"

							
					else:
						sbus_steering = self.sbus_steering_mid
						sbus_throttle = self.sbus_throttle_mid

				cv2.circle(self.amap, (int(self.x_mid_lane_MIN), 0), 3, (255,0,0), -1)
				cv2.circle(self.amap, (int(self.x_mid_lane_MAX), 0), 3, (0,255,0), -1)
				cv2.circle(self.amap, (int(self.center_min_DB), 0), 3, (255,255,0), -1)
				cv2.circle(self.amap, (int(self.center_max_DB), 0), 3, (0,255,255), -1)
				cv2.circle(self.amap, (int(self.center_thr_adj_min), 0), 3, (0,100,255), -1)
				cv2.circle(self.amap, (int(self.center_thr_adj_max), 0), 3, (100,0,255), -1)

				cv2.circle(self.amap, (int(self.x_mid_lane_MIN), self.frame_height), 3, (255,0,0), -1)
				cv2.circle(self.amap, (int(self.x_mid_lane_MAX), self.frame_height), 3, (0,255,0), -1)
				cv2.circle(self.amap, (int(self.center_min_DB), self.frame_height), 3, (255,255,0), -1)
				cv2.circle(self.amap, (int(self.center_max_DB), self.frame_height), 3, (0,255,255), -1)
				cv2.circle(self.amap, (int(self.center_thr_adj_min), self.frame_height), 3, (0,100,255), -1)
				cv2.circle(self.amap, (int(self.center_thr_adj_max), self.frame_height), 3, (100,0,255), -1)

				

				## Filter steering data
				steering_ave, steering_list = self.MovingAverage(sbus_steering, steering_list, self.steering_list_length) #10
				steering_ave = int(steering_ave)

				if self.FRONT_STOP or self.BACK_STOP:
					steering_ave = self.sbus_steering_mid
					sbus_throttle = self.sbus_throttle_mid


				self.sbus_cmd.data = [steering_ave, sbus_throttle]
				
				print("case: %s x_min_plot_MID: %d , str: %d, ave: %d, thr: %d" %(case, x_min_plot_MID, sbus_steering, steering_ave, sbus_throttle))

				# clear list out
				self.left_line_list = []
				self.right_line_list = []


				## publish image topic
				try:
					image_message = self.bridge.cv2_to_imgmsg(self.amap, encoding="bgr8") #bgr8 #8UC1
					# bw_image_message = self.bridge.cv2_to_imgmsg(processed_img, encoding="8UC1")
					self.image_pub.publish(image_message)
					# self.bw_image_pub.publish(bw_image_message)
				except CvBridgeError as e:
					print(e)

				self.amap = np.copy(self._map)
				processed_img = np.copy(self._map)

				## this takes 0.06 ms
				prev_sbus_steering = sbus_steering

			else:
				self.sbus_cmd.data = [self.sbus_steering_mid, self.sbus_throttle_mid]

			## Drawing Polygons ##
			# footprint
			self.pg.header.stamp = rospy.Time.now()
			self.pg.header.frame_id = "base_footprint"
			self.pg.polygon.points = [
								Point32(x=A[0], y=A[1]),
								Point32(x=B[0], y=B[1]),
								Point32(x=D[0], y=D[1]),
								Point32(x=C[0], y=C[1])]

			# front stop zone
			self.pg_front_stop.header.stamp = rospy.Time.now()
			self.pg_front_stop.header.frame_id = "base_footprint"
			self.pg_front_stop.polygon.points = [
								Point32(x=B[0], y=B[1]),
								Point32(x=T[0], y=T[1]),
								Point32(x=D[0], y=D[1]),
								Point32(x=B[0], y=B[1])]

			# back stop zone
			self.pg_back_stop.header.stamp = rospy.Time.now()
			self.pg_back_stop.header.frame_id = "base_footprint"
			self.pg_back_stop.polygon.points = [
									Point32(x=A[0], y=A[1]),
									Point32(x=C[0], y=C[1]),
									Point32(x=W[0], y=W[1]),
									Point32(x=A[0], y=A[1])]

			# construct tf
			self.t.header.frame_id = "base_footprint" 
			self.t.header.stamp = rospy.Time.now()
			self.t.child_frame_id = "base_link"
			self.t.transform.translation.x = 0.0
			self.t.transform.translation.y = 0.0
			self.t.transform.translation.z = 0.0

			self.t.transform.rotation.x = 0.0
			self.t.transform.rotation.y = 0.0
			self.t.transform.rotation.z = 0.0
			self.t.transform.rotation.w = 1.0
			self.br.sendTransform(self.t)

			# laser_pub.publish(new_scan)
			self.foot_pub.publish(self.pg)
			self.front_stop_pub.publish(self.pg_front_stop)
			self.back_stop_pub.publish(self.pg_back_stop)


			# self.x_left_array = np.array([])
			# self.y_left_array = np.array([])

			self.prev_ROBOT_MODE = self.ROBOT_MODE


			self.sbus_cmd_pub.publish(self.sbus_cmd)
			# period = time.time() - startTime
			# print(period)
			rate.sleep()



if __name__ == '__main__':
	line_detector = LineDetector()

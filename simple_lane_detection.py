'''
Simple Lane detection using openCV
The algorithm is simple detection of lines using probabilistic hough line, calculating the slope of each line
(assuming left and right lane)
then calculating the furthest and lowest points on both lanes.
then draw lines and polygon between them to indicate the lane.

Made by Karim Hamdy, Project of Self Driving car Engineer Nano-degree

'''
#doing all the relevant imports
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2

#----------------------------------------------------------------------------------------------------------------------
#Function Prototypes
def canny_detection(image,low_threhold_input,high_threshold_input):
	return cv2.Canny(image, low_threhold_input, high_threshold_input)
def ROI(image_ref,ignore_mask_color):
	mask_temp = np.zeros_like(image_ref)   
	ignore_mask_color = 255
	vertices = np.array([[(0,imshape[0]),(525, 300), (500, 300), (imshape[1],imshape[0])]], dtype=np.int32)
	cv2.fillPoly(mask_temp, vertices, ignore_mask_color)
	return cv2.bitwise_and(edges, mask_temp)

def hough_line_p(rho,theta,threhold,min_line_len,max_line_gap):
	#these values depend highly on the input video, they may differ on using other input data.
	rho = 1	
	theta = np.pi/180
	threshold = 45 #12
	min_line_length = 7
	max_line_gap = 1
	return cv2.HoughLinesP(masked_edges, rho, theta, threshold, np.array([]),
	                            min_line_length, max_line_gap)

def line_calculation(lines_input):
	#calculation of the slope of the left line of lane, and the right line of lane.
	slope_total_left = []
	slope_avg_left = 0
	highest_x_left = 0;
	highest_y_left = 0;
	highest_slope_left = 0;
	highest_x_right = imshape[1];
	highest_y_right = 0;
	highest_slope_right = 0;


	for line in lines_input:
	    for x1,y1,x2,y2 in line:
	    	if x1 != x2:
	    		slope_value = (float(y2)-float(y1))/(x2-x1)
		    	if slope_value >0: 						#calculating the slope of the line in the right half of the image
		        	if x1<highest_x_right :
		        		highest_x_right = x1
		        		highest_y_right = y1
		        		highest_slope_right = slope_value
		        	if x2<highest_x_right :
		        		highest_x_right = x2 
		        		highest_y_right = y2
		        		highest_slope_right = slope_value

		    	elif slope_value <0 : 						#calculating the slope of the line in the right half of the image
		    		slope_total_left.append (slope_value)
		        	if x1 <imshape[1]/2 and x2<imshape[1]/2:
			        	if y1>330 or y2 > 330:
				        	if x1>highest_x_left :
				        		highest_x_left = x1
				        		highest_y_left = y1
				        		highest_slope_left = slope_value
				        	if x2>highest_x_left :
				        		highest_x_left = x2 
				        		highest_y_left = y2
				        		highest_slope_left = slope_value
	for s in slope_total_left:
		slope_avg_left = slope_avg_left + s
	slope_avg_left = slope_avg_left/ len(slope_total_left) 		#calculating the average slope of these lines
	x_coo_edge_down_left = int(float((imshape[0]-highest_y_left+highest_x_left*slope_avg_left))/slope_avg_left ) 				#using the formula (y2-y1/x2-x1) = slope, while adding 25 to compensate for the error
	x_coo_edge_down_right = int(float((imshape[0]-highest_y_right+highest_x_right*highest_slope_right))/highest_slope_right ) 				#using the formula (y2-y1/x2-x1) = slope, while adding 25 to compensate for the error
	return highest_x_left,highest_y_left,highest_x_right,highest_y_right,slope_avg_left,highest_slope_right,x_coo_edge_down_left,x_coo_edge_down_right

def draw_lane_lines(image_input,edges_input):
	cv2.line(line_image,(x_coo_edge_down_right,imshape[0]),(highest_x_right, highest_y_right),(0,255,0),7)	
	cv2.line(line_image,(x_coo_edge_down_left,imshape[0]),(highest_x_left, highest_y_left),(0,0,255),7)	
	pts = np.array([[x_coo_edge_down_right, imshape[0]],[highest_x_right, highest_y_right],[highest_x_left, highest_y_left],[x_coo_edge_down_left, imshape[0]]], np.int32)
	pts = pts.reshape((-1,1,2))
	cv2.fillPoly(line_image,[pts],(255,0,0))
	color_edges = np.dstack((edges_input, edges_input, edges_input)) 
	return cv2.addWeighted(image_input, 0.8, line_image, layer_weight, 0)




#------------------------------------------------------------------------------------------------------------------------------

layer_weight = 0

cap = cv2.VideoCapture('solidYellowLeft.avi')
low_threshold_canny = 50
high_threshold_canny = 150	

rho_hough = 1
theta_hough = np.pi/180
threshold_hough = 45 #12
min_line_length_hough = 7
max_line_gap_hough = 1

while cap.isOpened():
	ret, image = cap.read()	
	if ret == False:								#Making sure there is a valid frame to process on.
		break
	gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
	imshape = image.shape
	line_image = np.copy(image)*0 #creating a blank to draw lines on
	# Define a kernel size for Gaussian smoothing / blurring
	# Note: this step is optional as cv2.Canny() applies a 5x5 Gaussian internally
	kernel_size = 5
	blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

	# Define parameters for Canny and run it
	# NOTE: if you try running this code you might want to change these!

	edges = canny_detection(blur_gray,low_threshold_canny,high_threshold_canny)
	cv2.imshow('canny',edges)
	
	
	masked_edges = ROI(edges,255) 					#defining region of interest based on the input feed video
	cv2.imshow('masked_edges',masked_edges)
	
	lines = hough_line_p(rho_hough,theta_hough,threshold_hough,min_line_length_hough,max_line_gap_hough)

	if lines is None:
		continue
	calculations = line_calculation(lines)			#parsing the data to be used in drawing the lane lines
	highest_x_left = calculations[0]
	highest_y_left = calculations[1]
	highest_x_right = calculations[2]
	highest_y_right = calculations[3]
	slope_avg_left = calculations[4]
	highest_slope_right = calculations[5]
	x_coo_edge_down_left = calculations[6]
	x_coo_edge_down_right = calculations[7]	



	
	combo = draw_lane_lines(image,edges)		
	cv2.imshow('combo',combo)
	
	if cv2.waitKey(10) == ord('q'):				#pressing 'd' to shows the lane detection 
		break									#pressing 'q' quits
	if cv2.waitKey(1) == ord('p'):				#pressing 'p' pauses
		while(cv2.waitKey(10) != ord('q')):
			continue
	if cv2.waitKey(1) == ord('d'):
		if layer_weight<1:
			layer_weight = layer_weight +0.1
	
cap.release()

cv2.destroyAllWindows()
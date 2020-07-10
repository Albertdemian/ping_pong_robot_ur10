#!/usr/bin/env python3
# license removed for brevity
import pyrealsense2 as rs
import numpy as np
import cv2
import time 
import imutils
import os 
import  matplotlib.pyplot as plt
from aruco_marker_find import Aruco_Marker_Find
import glob

import rospy
from geometry_msgs.msg import Point

Aruco_Marker = Aruco_Marker_Find(id_to_find = 7, marker_size = 10) # init the object for aruco marker

class FPS():
    def __init__(self):
        self.fps = 0
        self.i = 1
        self.start_time = time.time()
    def update(self):
        stop_time = time.time()
        fps_frame = 1/(stop_time-self.start_time)
        self.fps = (self.i-1)*self.fps/self.i + fps_frame/self.i
        self.i +=1
        self.start_time = stop_time
    def get(self):
        return self.fps

class RealSense():
    def __init__(self):

        self.ball_contour_color  = (0,255,0) # boundary colour of the ball being detected
        self.marker_contour_color  = (0,0,255) # # boundary colour of the ball being detected (changed to Aruco marker) 
        self.center_color = (0,0,255) # center point colour of the ball being detected
        self.center_line_thickness = 4 # 

        self.resize_ratio = 1 # at what rate the frames are resized. It could inrease FPS but decrease the accuracy.

        self.camera_matrix = np.loadtxt('cameraMatrix.txt', delimiter=',') # the file where camera matrix is located
        self.distortion_coefficients = np.loadtxt('cameraDistortion.txt', delimiter=',')  # # the file where dist of the camera is located
        # You can define the arrays above by calibrating the camera. You can find the method in real_sense.py.
        fps = 60 # set desired FPS
        
        # available resolutions = (640,360),(640,400),(640,480),(848.100),(848,480),(1280,720),(1280,800)
        image_width = 640
        image_height = 360

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        config.enable_stream(rs.stream.depth, image_width, image_height, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, image_width, image_height, rs.format.bgr8, fps)

        # Start streaming
        self.pipeline.start(config)

        self.script_path = os.path.dirname(os.path.abspath(__file__))
        os.chdir(self.script_path)

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        # Some parameters for tracking

        #define color boundary of the ball in HSV space
        self.ball_boundary = ([0, 185, 184], [18, 255, 255]) # you can use range-detector for extracting this range. 
    
        # Tag  ( Aruco Marker) offset wrt base
        self.tag_offset_r = np.array([0.298, -0.294, -0.11], dtype = float) #[cm]


        self.tag_check = False # flag is True if marker is found. We use it just to remember the transformation from robot to the marker and not to calculate it every time because it's time consuming.

        self.online_tracker_init_flag = False

    def track_ball(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            pass
        # Intrinsics & Extrinsics
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(
            color_frame.profile)
        
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        #isolate colors based on boundaries 

        # color_image, ball_coords = self.online_ball_tracker(color_image, depth_frame, depth_intrin, tracker_type = 'CSRT')
        color_image, ball_coords = self.color_tracker(self.ball_boundary, color_image, depth_frame, depth_intrin)



        if self.tag_check == False:
            self.tag_result = Aruco_Marker.track(color_image)
        # Check if the camera catched the tag
        if self.tag_result != None and self.tag_check == False:
            self.tag_result_updated = self.tag_result
            self.tag_check = True

        # start to determing the position of the ball in the frame
        if ball_coords != None and self.tag_check ==True:
            _, R_ct, (tag_x, tag_y, tag_z) = self.tag_result_updated
            H_ct = np.array([[R_ct[0,0], R_ct[0,1], R_ct[0,2], tag_x],
                            [R_ct[1,0], R_ct[1,1], R_ct[1,2], tag_y],
                            [R_ct[2,0], R_ct[2,1], R_ct[2,2], tag_z],
                            [0,0,0,1]], dtype = float)

            H_tr = np.array([[1,0,0, self.tag_offset_r[0]],
                            [0,1,0, self.tag_offset_r[1]],
                            [0,0,1, self.tag_offset_r[2]],
                            [0,0,0,1]], dtype = float)

            H_cr = np.dot(H_tr, H_ct)
            

            ball_pos_wrt_robot_frame = np.dot(H_cr, np.array([[ball_coords[0]],
            [ball_coords[1]],
            [ball_coords[2]],
            [1]]))
            return ball_pos_wrt_robot_frame, depth_image, color_image   
        else:
            return [None, None, None], depth_image, color_image

    def online_ball_tracker(self, image, depth_frame, depth_intrin, tracker_type = 'CSRT'):
        '''
        # Set up tracker.
        tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
        '''
        if self.online_tracker_init_flag == False:
            if tracker_type == 'BOOSTING':
                self.online_tracker = cv2.TrackerBoosting_create()
            if tracker_type == 'MIL':
                self.online_tracker = cv2.TrackerMIL_create()
            if tracker_type == 'KCF':
                self.online_tracker = cv2.TrackerKCF_create()
            if tracker_type == 'TLD':
                self.online_tracker = cv2.TrackerTLD_create()
            if tracker_type == 'MEDIANFLOW':
                self.online_tracker = cv2.TrackerMedianFlow_create()
            if tracker_type == 'GOTURN':
                self.online_tracker = cv2.TrackerGOTURN_create()
            if tracker_type == 'MOSSE':
                self.online_tracker = cv2.TrackerMOSSE_create()
            if tracker_type == "CSRT":
                self.online_tracker = cv2.TrackerCSRT_create()
            self.online_tracker_init_flag = True
            print('initialize the ball bounding box, bbox = [x,y,w,h]')


            # Uncomment the line below to select a different bounding box
            bbox = cv2.selectROI(image, False)
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(image, p1, p2, (255,0,0), 2, 1)
            # Initialize tracker with first frame and bounding box
            self.ok = self.online_tracker.init(image, bbox)
            ball_coords = (int(bbox[0]+bbox[2]/2), int(bbox[1]+bbox[3]/2))
            depth = depth_frame.get_distance(ball_coords[0],ball_coords[1])
            depth_point = rs.rs2_deproject_pixel_to_point(
                                depth_intrin, [ball_coords[0],ball_coords[1]], depth)
            
        else:
           
            # Update tracker
            self.ok, bbox = self.online_tracker.update(image)

            # Draw bounding box
            if self.ok:
                # Tracking success
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv2.rectangle(image, p1, p2, (255,0,0), 2, 1)
                ball_coords = (int(bbox[0]+bbox[2]/2), int(bbox[1]+bbox[3]/2))
                depth = depth_frame.get_distance(ball_coords[0],ball_coords[1])
                depth_point = rs.rs2_deproject_pixel_to_point(
                                    depth_intrin, [ball_coords[0],ball_coords[1]], depth)
                # Display tracker type on frame
                cv2.putText(image, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)
            else :
                # Tracking failure
                cv2.putText(image, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
                depth_point = None
        return image, depth_point

    def color_tracker(self, boundary, image, depth_frame, depth_intrin):
        # create NumPy arrays from the boundaries
        lower = np.array(boundary[0], dtype = "uint8")
        upper = np.array(boundary[1], dtype = "uint8")

        # find the colors within the specified boundaries and apply
        # the mask
        # image = cv2.GaussianBlur(image, (11, 11), 0) # blur
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # only proceed if the radius meets a minimum size
            if radius > 0:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image, (int(x), int(y)), int(radius),
                    self.ball_contour_color, 2)
                ball_coords = (int(x), int(y))

                depth = depth_frame.get_distance(ball_coords[0],ball_coords[1])
                depth_point = rs.rs2_deproject_pixel_to_point(
                                    depth_intrin, [ball_coords[0],ball_coords[1]], depth)
                cv2.putText(image,'Ball pos: '+str(round(depth_point[0],2)) +' ' + str(round(depth_point[1],2)) + ' ' + str(round(depth_point[2],2)),(ball_coords[0]-int(2*radius),ball_coords[1]+int(1.2*radius)),cv2.FONT_HERSHEY_SIMPLEX ,0.5,(0,0,0))
                cv2.putText(image,'Ball distance: '+str(round(np.sqrt(depth_point[0]**2+ depth_point[1]**2+depth_point[2]**2),2)) + ' m',(ball_coords[0]-int(2*radius),ball_coords[1]+int(2.2*radius)),cv2.FONT_HERSHEY_SIMPLEX ,0.5,(0,0,0))
                return image, depth_point
        return image, None


    def collect_images(self, pause = 3, number_of_images = 15, folder='Pictures'):
        '''
        This is the code for collecting and shoting the images. It might be needed for calibration or for HSV color range finding (to extract the ball) 
        '''
        frame = 0
        start_time = time.time()
        if not os.path.exists(folder):
            os.makedirs(folder)
        while frame < number_of_images:
            # Wait for a coherent pair of frames: depth and color
            frames = self.pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = self.align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            #calculate pointcloud from frames
            #point_cloud = rs.pointcloud.calculate(aligned_frames)
            
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
                
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            color_image = cv2.resize(color_image,None, fx = self.resize_ratio,fy= self.resize_ratio,interpolation = cv2.INTER_CUBIC)
            # images = np.hstack((color_image, depth_colormap))
            # Show images
            time_left = pause - (time.time() - start_time)
            cv2.putText(color_image,'grab in '+str(round(time_left,2)) + 'sec ...',(30,30),cv2.FONT_HERSHEY_SIMPLEX ,1,(0,0,0))
            # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_image)
            if time_left < 0:
                cv2.imwrite(os.path.join(folder,'img_'+str(frame)+'.jpg'), color_image, [cv2.IMWRITE_JPEG_QUALITY, 100])
                frame += 1
                start_time = time.time()
            if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
        self.pipeline.stop()  

    def camera_calibrate(self, nx = 12, ny = 9, l = 100, path='Pictures'):
        '''
        Chessboard should be used.

        nx - number of inside corners in x;
        ny - number of inside corners in y;
        l - width of the square (in mm);

        reference: https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html#calibration
        '''
        # prepare object points
        if not os.path.exists(path):
            os.makedirs(path)
        #CALIBRATING THE CAMERA
        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(nx,ny,0)
        objp = np.zeros((ny*nx, 3), np.float32)
        objp[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        #Finding all the images .jpg
        os.chdir(path)
        images = glob.glob('*.jpg')
        print(images)
        print('Unrecognized images (if any):')
        #Loop for finding corners, calculating image points and object points
        examples = []
        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (nx, ny), None)

            # If found, add object points, image points (after refining them)
            if ret:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (9, 9), (-1, -1), criteria)
                imgpoints.append(corners2)
                examples.append(gray)
                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (nx, ny), corners2, ret)
                
                cv2.imshow('ImageWindow', img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            else:
                #if some images are uncorrect
                print(fname) #to may be delete them manually after that
        #Calibration
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        #dist – Output vector of distortion coefficients
        # (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6]]) of 4, 5, or 8 elements.
        #rvecs – Output vector of rotation vectors 
        #ret-
        #mtx- Output 3x3 floating-point camera matrix
        #tvecs – Output vector of translation vectors estimated for each pattern view.
        print(mtx)
        print(dist)
        np.save('calibration.npy', np.array([mtx, dist, rvecs, tvecs], dtype = object))
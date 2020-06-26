import pyrealsense2 as rs
import numpy as np
import cv2
import time 
import imutils
from collections import deque
import os 

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

        self.ball_contour_color  = (0,255,0)
        self.marker_contour_color  = (0,0,255)
        self.center_color = (0,0,255)
        self.center_line_thickness = 4

        self.resize_ratio = 1

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

    def track_ball(self, buffer_size = 64):

        #define color boundaries in HSV space
        # self.boundary = ([0, 0, 171], [36, 94, 255])       #BALL
        self.ball_boundary = ([0, 0, 171], [75, 42, 255]) 
        self.marker_boundary =  ([0, 211, 65], [3, 255, 255]) 
        fps = FPS()
        pts = deque(maxlen=buffer_size)
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = self.pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = self.align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Intrinsics & Extrinsics
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
            color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
            depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(
                color_frame.profile)
            #calculate pointcloud from frames
            #point_cloud = rs.pointcloud.calculate(aligned_frames)
            
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            #isolate colors based on boundaries 
            color_image, ball_coords = self.color_tracker(self.ball_boundary, pts, color_image, depth_frame, depth_intrin, object = 'ball')
            #find marker
            color_image, marker_coords = self.color_tracker(self.marker_boundary, pts, color_image, depth_frame, depth_intrin, object='marker')
            
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.2), cv2.COLORMAP_JET)
            color_image = cv2.resize(color_image,None, fx = self.resize_ratio,fy= self.resize_ratio,interpolation = cv2.INTER_CUBIC)
            depth_colormap = cv2.resize(depth_colormap,None, fx=self.resize_ratio,fy=self.resize_ratio, interpolation=cv2.INTER_CUBIC)
            cv2.putText(color_image,'fps = '+str(int(fps.get())),(40,30),cv2.FONT_HERSHEY_SIMPLEX,2,(0,0,0))
            images = np.hstack((color_image, depth_colormap))
            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            fps.update()
            if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
        # Stop streaming
        self.pipeline.stop()   
             
    def color_tracker(self, boundary, pts, image, depth_frame, depth_intrin, object = 'ball'):
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
            if object == 'ball':
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                # only proceed if the radius meets a minimum size
                if radius > 10:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image, (int(x), int(y)), int(radius),
                        self.ball_contour_color, 2)
                    ball_coords = (int(x), int(y))

                    depth = depth_frame.get_distance(ball_coords[0],ball_coords[1])
                    depth_point = rs.rs2_deproject_pixel_to_point(
                                        depth_intrin, [ball_coords[0],ball_coords[1]], depth)
                    cv2.putText(image,'Ball pos: '+str(round(depth_point[0],2)) +' ' + str(round(depth_point[1],2)) + ' ' + str(round(depth_point[2],2)),(ball_coords[0]-int(radius),ball_coords[1]+int(1.2*radius)),cv2.FONT_HERSHEY_SIMPLEX ,0.5,(0,0,0))
                    cv2.putText(image,'Ball distance: '+str(round(np.sqrt(depth_point[0]**2+ depth_point[1]**2+depth_point[2]**2),2)) + ' m',(ball_coords[0]-int(radius),ball_coords[1]+int(2.2*radius)),cv2.FONT_HERSHEY_SIMPLEX ,0.5,(0,0,0))
                    return image, ball_coords
            elif object == 'marker':
                marker_coords = []
                for c in cnts:
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    M = cv2.moments(c)
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    # only proceed if the radius meets a minimum size
                    if radius > 0:
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(image, (int(x), int(y)), int(radius),
                            self.marker_contour_color, 2)
                        marker_coord = (int(x), int(y))

                        depth = depth_frame.get_distance(marker_coord[0],marker_coord[1])
                        depth_point = rs.rs2_deproject_pixel_to_point(
                                            depth_intrin, [marker_coord[0],marker_coord[1]], depth)
                        cv2.putText(image,'marker pos: '+str(round(depth_point[0],2)) +' ' + str(round(depth_point[1],2)) + ' ' + str(round(depth_point[2],2)),(marker_coord[0]-int(radius),marker_coord[1]+int(1.2*radius)),cv2.FONT_HERSHEY_SIMPLEX ,0.5,(0,0,0))
                        cv2.putText(image,'marker distance: '+str(round(np.sqrt(depth_point[0]**2+ depth_point[1]**2+depth_point[2]**2),2)) + ' m',(marker_coord[0]-int(radius),marker_coord[1]+int(2.2*radius)),cv2.FONT_HERSHEY_SIMPLEX ,0.5,(0,0,0))
                        marker_coords.append(marker_coord)
                return image, marker_coords
        # update the points queue
        # pts.appendleft(center)
        return image, None


    def collect_images(self, pause = 3, number_of_images = 15, folder='Pictures'):
        frame = 0
        start_time = time.time()
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
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_image)
            if time_left < 0:
                cv2.imwrite(os.path.join(folder,'img_'+str(frame)+'.jpg'), color_image, [cv2.IMWRITE_JPEG_QUALITY, 100])
                frame += 1
                start_time = time.time()
            if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
        self.pipeline.stop()  
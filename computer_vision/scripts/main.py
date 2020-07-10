#!/usr/bin/env python3
# license removed for brevity

from real_sense import RealSense, FPS
import numpy as np
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg       import Float64
import cv2

D435 = RealSense() # Init the object for working with the camera
# D435.collect_images(number_of_images = 3, folder = 'HSV_get') # THis may be needed for HSV adjusting

# D435.collect_images(number_of_images = 15,folder = 'Pictures') # For calibration
# D435.camera_calibrate()

# Tracking the ball


def talker():
    pub_ball = rospy.Publisher('ball_position_pub', Point, queue_size=1)
    pub_ball_flag = rospy.Publisher('ball_in_a_scene_flag', Float64, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(60) # 10hz

    fps = FPS()
    render_flag = 0
    max_flag_no = 120
    while not rospy.is_shutdown():
        ball_pos, depth_image, color_image = D435.track_ball()
        if ball_pos[0]!= None:
            # print(ball_pos_wrt_robot_frame)
            # print('distance from the robot to the ball:', np.sqrt(ball_pos_wrt_robot_frame[0]**2+ball_pos_wrt_robot_frame[1]**2+ball_pos_wrt_robot_frame[2]**2))
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            ball_in_a_scene_flag = 1
            pub_ball_flag.publish(ball_in_a_scene_flag)
            # rospy.loginfo([float(ball_pos[0]), float(ball_pos[1,0]), float(ball_pos[2,0])])
            pub_ball.publish(float(ball_pos[0]), float(ball_pos[1,0]), float(ball_pos[2,0]))
        else:
            ball_in_a_scene_flag = 0
            pub_ball_flag.publish(ball_in_a_scene_flag)
        
        if render_flag <= max_flag_no:
            # rospy.loginfo([ball_in_a_scene_flag])
            # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.2), cv2.COLORMAP_JET)
            color_image = cv2.resize(color_image,None, fx = D435.resize_ratio,fy= D435.resize_ratio,interpolation = cv2.INTER_CUBIC)
            # depth_colormap = cv2.resize(depth_colormap,None, fx=D435.resize_ratio,fy=D435.resize_ratio, interpolation=cv2.INTER_CUBIC)
            cv2.putText(color_image,'fps = '+str(int(fps.get())),(40,30),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0))
            # images = np.hstack((color_image, depth_colormap))
            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_image)

        render_flag +=1
            
        fps.update()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        # Stop streaming
        D435.pipeline.stop()
        pass



#-----------BASLERS---------------#
# from basler_ace import Basler_Ace_Camera

# cameras = Basler_Ace_Camera()

# # cameras.record(camera_ID=0)q
# # cameras.stereo_record()

# # cameras.collect_images(number_of_images=3, camera_ID = 1, folder = 'HSV_get')

# # cameras.camera_calibrate(camera_ID = 1)
# # 
# cameras.track_the_ball(set_type = 'stereo', k = 2)
# # cameras.track_the_ball(set_type = 'single', camera_ID = 1,k = 2)
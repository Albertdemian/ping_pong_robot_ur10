from real_sense import RealSense

D435 = RealSense()
# D435.collect_images(number_of_images = 3, folder = 'HSV_get')
D435.track_ball()






#-----------BASLERS---------------#
# from basler_ace import Basler_Ace_Camera

# cameras = Basler_Ace_Camera()

# # cameras.record(camera_ID=0)
# # cameras.stereo_record()

# # cameras.collect_images(number_of_images=3, camera_ID = 1, folder = 'HSV_get')

# # cameras.camera_calibrate(camera_ID = 1)
# # 
# cameras.track_the_ball(set_type = 'stereo', k = 2)
# # cameras.track_the_ball(set_type = 'single', camera_ID = 1,k = 2)
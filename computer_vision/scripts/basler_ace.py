from pypylon import pylon
import cv2
import matplotlib.pyplot as plt
import time
import os
import numpy as np
import glob
import imutils
from collections import deque
# list-like data structure with super fast appends and pops to maintain a list of the past N (x, y)-locations of the ball in our video stream.
#  Maintaining such a queue allows us to draw the “contrail” of the ball as its being tracked.

# This is the script for working with Basler Ace cameras, but we stopped improving it, because we didn't have 2 similar cameras.

# Created function in order to calculate FPS
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

class Basler_Ace_Camera():
    def __init__(self):
        # converting to opencv bgr format (it's needed for displaying the image)
        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        tlFactory = pylon.TlFactory.GetInstance()
        devices = tlFactory.EnumerateDevices()
        if len(devices) == 0:
            raise pylon.RUNTIME_EXCEPTION("No camera present.")

        cameras = pylon.InstantCameraArray(2)

        for i, camera in enumerate(cameras):
            camera.Attach(tlFactory.CreateDevice(devices[i]))

        self.cameras = cameras
        self.cameras.Open()

        self.script_path = os.path.dirname(os.path.abspath(__file__))
        os.chdir(self.script_path)


        # cv2.namedWindow('Acquisition', cv2.WINDOW_NORMAL)
        # cv2.resizeWindow('Acquisition', 1280, 1024)

    def record(self, camera_ID = 0):
        ''' 
        the method for just camera record...
        '''
        # # demonstrate some feature access
        # new_width = self.camera.Width.GetValue() - self.camera.Width.GetInc()
        # if new_width >= self.camera.Width.GetMin():
        #     self.camera.Width.SetValue(new_width)

        self.cameras[camera_ID].StartGrabbing() # Starts recording

        fps = FPS()

        while self.cameras[camera_ID].IsGrabbing():
            grabResult = self.cameras[camera_ID].RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

            if grabResult.GrabSucceeded():
                # Access the image data
                img = grabResult.Array
                cv2.putText(img,'fps = '+str(int(fps.get())),(20,20),cv2.FONT_HERSHEY_SIMPLEX ,1,(0,0,255))
                cv2.imshow('ImageWindow', img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            grabResult.Release()
            fps.update()
        self.cameras[camera_ID].Close()

    def stereo_record(self, ):
        
        # Starts grabbing for all cameras
        self.cameras.StartGrabbing(pylon.GrabStrategy_LatestImageOnly, 
                      pylon.GrabLoop_ProvidedByUser)
        fps = FPS()
        while self.cameras.IsGrabbing():
            grabResult1 = self.cameras[0].RetrieveResult(5000, 
                                pylon.TimeoutHandling_ThrowException)
            
            grabResult2 = self.cameras[1].RetrieveResult(5000, 
                                pylon.TimeoutHandling_ThrowException)
            
            if grabResult1.GrabSucceeded() & grabResult2.GrabSucceeded():
                # grabResult1 = self.converter.Convert(grabResult1)
                # grabResult2 = self.converter.Convert(grabResult2)
                frame_0 = grabResult1.GetArray()
                frame_1 = grabResult2.GetArray()
                frame_1 = imutils.resize(frame_1, height=1024)
                # cv2.putText(frame_0,'fps = '+str(int(fps.get())),(40,30),cv2.FONT_HERSHEY_DUPLEX,2,(0,0,255))
                print(fps.get())
                # If q is pressed exit and destroy window
                cv2.imshow('Acquisition', np.hstack([frame_0,frame_1]))
                fps.update()
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        cv2.destroyAllWindows()
            
    def collect_images(self, pause = 3, number_of_images = 15, camera_ID = 0, folder='Pictures'):
        '''
        The method that allows you to collect images in the needed folder.

        pause - the time interval camera grab an image (in sec, integer));
        number_of_images - the needed number of images;
        path-the folder in which the images will be collected
        '''
        self.cameras[camera_ID].StartGrabbing() # Starts recording

        fps = FPS()
        frame = 0
        path = os.path.join(self.script_path,folder)
        if not os.path.exists(folder):
            os.makedirs(folder)
        shot_time_start = time.time()
        while self.cameras[camera_ID].IsGrabbing():
            grabResult = self.cameras[camera_ID].RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

            if grabResult.GrabSucceeded():
                # Access the image data
                shot_time_stop = time.time()
                img = grabResult.Array
                cv2.putText(img,'fps = '+str(int(fps.get())),(20,30),cv2.FONT_HERSHEY_SIMPLEX ,1,(0,0,255))
                cv2.putText(img,'grab in '+str(round(pause-(shot_time_stop-shot_time_start),2)) + 'sec ...',(900,30),cv2.FONT_HERSHEY_SIMPLEX ,1,(0,0,255))
                cv2.imshow('ImageWindow', img)
                if shot_time_stop-shot_time_start > pause:
                    cv2.imwrite(os.path.join(folder,'img_'+str(frame)+'.jpg'), img, [cv2.IMWRITE_JPEG_QUALITY, 100])
                    frame+=1
                    shot_time_start = time.time()
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                if frame == number_of_images:
                    break

            grabResult.Release()
            fps.update()
        self.cameras[camera_ID].Close() 

    def camera_calibrate(self, nx = 6, ny = 9, l = 33, path='Pictures'):
        '''
        Chessboard should be used.

        nx - number of inside corners in x;
        ny - number of inside corners in y;
        l - width of the square (in mm);

        reference: https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html#calibration
        '''
        # prepare object points

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
        os.chdir(self.script_path+'/'+path)
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

        #Store the parameters in the folder
        #Starting undistortion

        # Read in an image
        gray = examples[int(np.random.randint(0,len(examples),1))] #you may put here any photo
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h,  w = img.shape[:2]


        #Returns the new (optimal) camera matrix based on the free scaling parameter.
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0, (w, h))
        #undistort
        img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
        cv2.imwrite('object_size_undistorted.png', dst)
        np.savez('Parameters', ret=ret, matrix=mtx, distance=dist, rotation=rvecs, translation=tvecs, newmatrix=newcameramtx, roi = roi)
        # undistort
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
        #cv2.imwrite('object_measure.png', dst)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]

        #Plotting
        f, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 9))
        f.tight_layout()
        ax1.imshow(img)
        ax1.set_title('Original Image', fontsize=20)
        ax2.imshow(dst)
        ax2.set_title('Undistorted Image', fontsize=20)
        plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)
        plt.suptitle('CALIBRATION OF THE CAMERA')
        plt.show()
        #calculating the reprojection error
        tot_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            tot_error += error
        print("reprojection error = %.2f \n" % (tot_error/len(objpoints)))

    def track_the_ball(self, buffer_size = 64, set_type = 'single', path = 'Pictures', camera_ID = 0, ball_diameter = 40, k = 2):
        '''
        path - the path where camera parameters npz file saved;
        buffer_size - is the maximum size of our deque , 
        which maintains a list of the previous (x, y)-coordinates 
        of the ball we are tracking;
        ball_diameter - the diameter of the ball to track (inch)
        k - reduce image ratio

        reference: https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
        '''
        # Export the parameters of the camera from the files
        data = np.load(self.script_path+'/'+path+'/Parameters.npz')
        # define the lower and upper boundaries of the ball
        # ball in the HSV color space, then initialize the
        # list of tracked points
        # HSVLower = (0, 0, 0)
        # HSVUpper = (255, 102, 80)
        HSVLower = (0, 0, 54)
        HSVUpper = (255, 255, 255)
        pts = deque(maxlen=buffer_size)
        # grab the reference to the camera

        # keep looping
        fps = FPS()
        x_center_points = []
        y_center_points = []
        if set_type == 'single':
            # grab the reference to the camera
            self.cameras[camera_ID].StartGrabbing() # Starts recording
            # allow the camera or video file to warm up
            time.sleep(0.5)
            while self.cameras[camera_ID].IsGrabbing():
                grabResult = self.cameras[camera_ID].RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

                if grabResult.GrabSucceeded():
                    # Access the image data
                    frame = grabResult.Array
                    
                    # frame = cv2.undistort(frame, data['matrix'], data['distance'], None, data['newmatrix']) # undistorted
                    # x, y, w, h = data['roi']
                    # frame = frame[y:y+h, x:x+w]

                    # resize the frame, blur it, and convert it to the HSV
                    # color space
                    frame = imutils.resize(frame, width=frame.shape[1]//k) # Downsizing the frame  allows us to process the frame faster, leading to an increase in FPS
                    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
                    bgr = cv2.cvtColor(blurred, cv2.COLOR_GRAY2BGR)
                    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
                    # construct a mask for the ball color, then perform
                    # a series of dilations and erosions to remove any small
                    # blobs left in the mask
                    mask = cv2.inRange(hsv, HSVLower, HSVUpper)
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
                        if radius > 10:
                            # draw the circle and centroid on the frame,
                            # then update the list of tracked points
                            cv2.circle(frame, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
                            cv2.putText(frame,'fps = '+str(int(fps.get())),(20,30),cv2.FONT_HERSHEY_SIMPLEX ,1,(255,255,255))
                            cv2.putText(frame,'distance = '+str(round(ball_diameter*(0.5*(data['newmatrix'][0][0]+data['newmatrix'][1][1]))/(2*radius*k)/10,2))+' cm',(20,60),cv2.FONT_HERSHEY_PLAIN ,1.5,(255,255,255))
                            x_center_points.append(x)
                            y_center_points.append(y)

                    # update the points queue
                    pts.appendleft(center)

                    # loop over the set of tracked points
                    for i in range(1, len(pts)):
                        # if either of the tracked points are None, ignore
                        # them
                        if pts[i - 1] is None or pts[i] is None:
                            continue
                        # otherwise, compute the thickness of the line and
                        # draw the connecting lines
                        thickness = int(np.sqrt(buffer_size / float(i + 1)) * 1.5)
                        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
                    # show the frame to our screen
                    cv2.imshow("Frame", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                # grabResult.Release()
                fps.update()
            plt.plot(x_center_points,y_center_points)
            plt.show()
        elif set_type == 'stereo':
            stereo_pts = [pts, pts]
            self.cameras.StartGrabbing(pylon.GrabStrategy_LatestImageOnly, 
                      pylon.GrabLoop_ProvidedByUser)
            # allow the cameras or video file to warm up
            time.sleep(0.5)
            while self.cameras.IsGrabbing():
                grabResult1 = self.cameras[0].RetrieveResult(5000, 
                                    pylon.TimeoutHandling_ThrowException)
                
                grabResult2 = self.cameras[1].RetrieveResult(5000, 
                                    pylon.TimeoutHandling_ThrowException)
                
                if grabResult1.GrabSucceeded() & grabResult2.GrabSucceeded():
                    # Access the image data
                    frame_0 = grabResult1.Array
                    frame_1 = grabResult2.Array
                    frame_1 = imutils.resize(frame_1, height=1024)
                    frame_0 = imutils.resize(frame_0, height=1024)

                    # resize the frame, blur it, and convert it to the HSV
                    # color space
                    frame_0 = imutils.resize(frame_0, width=frame_0.shape[1]//k)
                    frame_1 = imutils.resize(frame_1, width=frame_1.shape[1]//k) # Downsizing the frame  allows us to process the frame faster, leading to an increase in FPS
                    frames = [frame_1, frame_0]
                    for frame in frames:
                        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
                        bgr = cv2.cvtColor(blurred, cv2.COLOR_GRAY2BGR)
                        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
                        # construct a mask for the ball color, then perform
                        # a series of dilations and erosions to remove any small
                        # blobs left in the mask
                        mask = cv2.inRange(hsv, HSVLower, HSVUpper)
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
                            if radius > 10:
                                # draw the circle and centroid on the frame,
                                # then update the list of tracked points
                                cv2.circle(frame, (int(x), int(y)), int(radius),
                                    (0, 255, 255), 2)
                                cv2.putText(frame,'fps = '+str(int(fps.get())),(20,30),cv2.FONT_HERSHEY_SIMPLEX ,1,(255,255,255))
                                cv2.putText(frame,'distance = '+str(round(ball_diameter*(0.5*(data['newmatrix'][0][0]+data['newmatrix'][1][1]))/(2*radius*k)/10,2))+' cm',(20,60),cv2.FONT_HERSHEY_PLAIN ,1.5,(255,255,255))
                        # update the points queue
                        stereo_pts[frames.index(frame)].appendleft(center)

                        frame_pts = stereo_pts[frames.index(frame)]
                        
                        # loop over the set of tracked points
                        for m in range(1, len(frame_pts)):
                            # if either of the tracked points are None, ignore
                            # them
                            if frame_pts[m - 1] is None or frame_pts[m] is None:
                                continue
                            # otherwise, compute the thickness of the line and
                            # draw the connecting lines
                            thickness = int(np.sqrt(buffer_size / float(m + 1)) * 1.5)
                            cv2.line(frame, pts[m - 1], frame_pts[m], (0, 0, 255), thickness)
                        # show the frame to our screen
                        # If q is pressed exit and destroy window
                    cv2.imshow('Acquisition', np.hstack(frames))
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break 
                fps.update()
                        

        # close all windows
        cv2.destroyAllWindows()
        self.cameras[camera_ID].Close()



#!/usr/bin/python

import cv2
import numpy as np
import rospy
from os import system
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import*
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import (Bool, String, Int8)

onB_StateSub = '/onboard/substate'

class fence_breach_detection:
    
    def __init__(self):
        
        #Init ROS
        rospy.init_node('fence_breach_detection')
        rospy.Timer(rospy.Duration(2), self.timer_callback)
        
        #Variables
        self.ros_img = Image()
        self.plot_vision_images = True
        self.publish_detected_fence_image = False
        self.image_index = 0
        self.enable = False
        
        #Subscribers
        rospy.Subscriber("/mono_cam/image_raw", Image, self.get_ros_image)
        # Onboard state
        rospy.Subscriber(onB_StateSub, String, self._cb_onStateChange)

        #Plublishers
        self.detectec_fence_image = rospy.Publisher("/onboard/detected_fence_image",Image,queue_size=5)

        rospy.spin()

    def get_ros_image(self,img):
        self.ros_img = img

    def fence_detection_vision(self, ros_img):
        
        #Load in the marker image
        try:
            bridge = CvBridge()
            img = bridge.imgmsg_to_cv2(ros_img, "mono8")
            img_rgb = bridge.imgmsg_to_cv2(ros_img, "rgb8")
        except CvBridgeError as e:
            print(e)
            
        # 2020-09-29 Developed by Henrik Skov Midtiby

        # Convert to floating point representation and calculate
        # the Discrete Fourier transform (DFT) of the image.
        img_float32 = np.float32(img)
        dft = cv2.dft(img_float32, flags = cv2.DFT_COMPLEX_OUTPUT)
        dft_shift = np.fft.fftshift(dft)

        # Determine coordinates to the center of the image.
        rows, cols = img.shape
        crow, ccol = rows//2 , cols//2     # center

        # Locate peaks in the FFT magnitude image.
        shifted_fft = cv2.magnitude(dft_shift[:, :, 0], dft_shift[:, :, 1])
        # Black out the center of the FFT
        cv2.circle(shifted_fft, (ccol, crow), 10, 0, -1)
        
        # Threshold values
        # I get good performance when approx 10 peaks are detected.
        # 200000 - very fine reconstruction of the fence
        peak_threshold = 200000 # Suitable for E_3_17
        #peak_threshold = 800000 # Suitable for E_3_16
        shifted_fft_peaks = np.greater(shifted_fft, peak_threshold) * 255.

        # Create a mask by dilating the detected peaks.
        kernel_size = 11
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.dilate(shifted_fft_peaks, kernel)
        mask = cv2.merge((mask[:, :], mask[:, :]))

        # Apply mask to the DFT and then return back to a
        # normal image through the inverse DFT.
        fshift = dft_shift*mask
        f_ishift = np.fft.ifftshift(fshift)
        img_back = cv2.idft(f_ishift)

        if self.plot_vision_images:
           
            # Save input image.
            path = '../../data/fence_detection_vision/' + 'input_image' + str(self.image_index) + '.png'
            cv2.imwrite(path, img_rgb)
            
            # Save the shifted fft spectrum.
            
            path = "../../data/fence_detection_vision/" + "shifted_fft_image" + str(self.image_index) + ".png"
            minval, maxval, a, b = cv2.minMaxLoc(shifted_fft)
            cv2.imwrite(path, shifted_fft * 255 / maxval)


            # Save the fft peak mask.
            path = "../../data/fence_detection_vision/" + "shifted_fft_peak_mask" + str(self.image_index) + ".png"
            cv2.imwrite(path, mask[:, :, 0] * 1.)

            # Save the filtered image image.
            path = "../../data/fence_detection_vision/" + "low_pass_filtered_image" + str(self.image_index) + ".png"
            minval, maxval, a, b = cv2.minMaxLoc(img_back[:, :, 0])
            cv2.imwrite(path, img_back[:, :, 0] * 255. / maxval)

            # Save the filtered image multiplied with the input image.
            path = "../../data/fence_detection_vision/" + "low_pass_filtered_image_mul_with_input" + str(self.image_index) + ".png"
            minval, maxval, a, b = cv2.minMaxLoc(img * img_back[:, :, 0])
            cv2.imwrite(path, img * img_back[:, :, 0] * 255 / maxval)

            self.image_index += 1

        if self.publish_detected_fence_image:
            self.detectec_fence_image.publish(bridge.cv2_to_imgmsg(img_back, "mono8"))
            
    def _cb_onStateChange(self, msg):
        if msg.data == 'fence_breach_detection':
            print('Fence breach detection enabled')
            self.enable = True
        else:
            if self.enable:
                print('Fence breach detection disabled')
                self.enable = False
    
    def timer_callback(self, event):
        if self.enable:
            self.fence_detection_vision(self.ros_img)

if __name__ == "__main__":
    node = fence_breach_detection()


#!/usr/bin/python

import numpy as np
import cv2
from scipy import ndimage


class fence_detection:

    def __init__(self):
        self.shifted_fft_peak = None
        self.shiftet_fft = None
        self.ten_low_pass_filtered = None
        self.twenty_low_pass_filtered = None

        self.fourier_peak_angle = 0. 

    def fence_extraction(self, img):
        # 2020-09-29 Developed by Henrik Skov Midtiby
        img_2 = img
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
        peak_threshold = 200000 # 200000 Suitable for E_3_17
        peak_threshold = 900000 # 800000 Suitable for E_3_16
        shifted_fft_peaks = np.greater(shifted_fft, peak_threshold) * 255.
        
        # Create a mask by dilating the detected peaks.
        kernel_size = 50 #11
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.dilate(shifted_fft_peaks, kernel)
        mask = cv2.merge((mask[:, :], mask[:, :]))
        
        # Apply mask to the DFT and then return back to a
        # normal image through the inverse DFT.
        fshift = dft_shift*mask
        f_ishift = np.fft.ifftshift(fshift)
        img_back = cv2.idft(f_ishift)
        
        # Save input image.
        cv2.imwrite("output/00_input_image.png", img)
        
        # Save the shifted fft spectrum.
        minval, maxval, a, b = cv2.minMaxLoc(shifted_fft)
        self.shiftet_fft = shifted_fft * 255 / maxval
        
        # Save the fft peak mask.
        self.shifted_fft_peak = mask[:, :, 0] * 1.
        cv2.imwrite("output/07_shifted_fft_peak_mask.png", self.shifted_fft_peak)
        
        # Save the filtered image image.
        minval, maxval, a, b = cv2.minMaxLoc(img_back[:, :, 0])
        self.ten_low_pass_filtered = img_back[:, :, 0] * 255. / maxval
        cv2.imwrite("output/10_low_pass_filtered_image.png", self.ten_low_pass_filtered)
        
        # Save the filtered image multiplied with the input image.
        minval, maxval, a, b = cv2.minMaxLoc(img * img_back[:, :, 0])
        self.twenty_low_pass_filtered = img * img_back[:, :, 0] * 255 / maxval
        cv2.imwrite("output/20_low_pass_filtered_image.png", self.twenty_low_pass_filtered)

    def increse_brightness_roi(self, intensity):

        img = self.twenty_low_pass_filtered
        rows,cols = img.shape
        for i in range(rows):
            for j in range(cols):
                k = img[i,j]
                if k < 150:
                    #k += intensity
                    img[i,j] = 0 

        cv2.imwrite("output/increased_low_pass_filtered_image.png", img)

    def find_angle_peak(self):

        img = self.shiftet_fft.copy()
        rows,cols = img.shape
        
        breaker = False
        y = 0
        x = 0

        for i in range(rows):
            if not breaker:
                for j in range(cols):
                    k = img[i,j]
                    if k > 150:
                        y = i
                        x = j
                        cv2.circle(img,(j,i),5,(255,255,0))
                        breaker = True
                        break

        #Center of image 
        center_x = cols/2
        center_y = rows/2
        
        #Find angle 
        hyp = np.sqrt((center_x-x)**2 + (center_y-y)**2)
        kat = center_y-y
        self.fourier_peak_angle = np.arctan((center_x-x)/kat)
        print(np.rad2deg(self.fourier_peak_angle))
        
        #Rotate image 
        r = ndimage.rotate(img_2, np.rad2deg(angle))
        """
        cv2.circle(img,(cols/2,rows/2),5,(255,255,0))
        cv2.circle(img,(x_new,y_new),5,(255,255,0))

        cv2.imwrite("output/05_shifted_fft_image.png", self.shiftet_fft)
        cv2.imwrite("output/test.png", r)

        print("New: " + str(x_new) + " "+ str(y_new))
        print("Center: " + str(center_x) + " "+ str(center_y))
        print("Blob: " + str(x) + " "+ str(y))
        print("Hyp: " + str(hyp))
        print("Kat: " + str(kat))
        """

    def find_fourier_peaks(self):

        img = self.shiftet_fft.copy()
        rows,cols = img.shape

        breaker = False
        y = 0
        x = 0

        for i in range(rows):
            if not breaker:
                for j in range(cols):
                    k = img[i,j]
                    if k > 150:
                        y = i
                        x = j
                        cv2.circle(img,(j,i),5,(255,255,0))
                        breaker = True
                        break



if __name__ == "__main__":

    fd = fence_detection()
    
    img = cv2.imread('input/Eskild_fig_3_17.jpg', 0)
    fd.fence_extraction(img)
    #fd.increse_brightness_roi(20)


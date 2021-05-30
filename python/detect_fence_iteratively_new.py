#!/usr/bin/python3

import numpy as np
import cv2
from scipy import ndimage
import random
import sys
from sklearn.cluster import AgglomerativeClustering
import matplotlib.pyplot as plt
class fence_detection:

    def __init__(self):

        self.original_img = None
        self.shifted_fft_peak = None
        self.shiftet_fft = None
        self.ten_low_pass_filtered = None
        self.twenty_low_pass_filtered = None

        self.fourier_peak_angle = 0.
        
        self.init_pointer = True
        self.img_height = 0
        self.img_width = 0

        self.roi_top_left = 0
        self.roi_bottom_right = 0

        #Next move on line
        self.pointer = [0, 0]
        self.new_pointers = []
        self.new_moves = []
        self.peaks = []
        self.end_points = []
        self.way_to_fence = []
        self.line_dis_threshold = 1
        
        #Varibles for line follower
        self.line_threshold = 2
        self.step_size_line = [1,2,3]
        self.line_move_direction = [1,2,3,0]
        
        #Valid movements [up_right, down_right, down_left, up_left]
        self.valid_moves = [[3,1],[0,2],[1,3],[2,0]]

        #Variables for peak search
        self.peak_threshold = 5
        self.peak_dis_threshold = 8
        self.step_size_peak = [4,5,6,7]
        self.peak_move_direction = [[0,1,0],[1,2,1],[2,3,2],[3,0,3]]
        self.peak_search_direction = [[3,0,1],[0,1,2],[1,2,3],[2,3,0]]
        self.peak_start_end_index = [[0,1],[-1,1],[0,1]]
        
        #Init fence serach. This only happens first time to find the fence structure
        self.init_fence_follower = True

        #See how the recursive algorithm propagates from number of branches 
        self.number_of_branches = 1
        self.max_number_of_recursions = 950

    def fence_extraction(self, img):
        # 2020-09-29 Developed by Henrik Skov Midtiby
        self.original_img = img
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
        #peak_threshold = 200000 # 200000 Suitable for E_3_17
        peak_threshold = 800000 # 800000 Suitable for E_3_16
        shifted_fft_peaks = np.greater(shifted_fft, peak_threshold) * 255.
        
        # Create a mask by dilating the detected peaks.
        kernel_size = 11 #11
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

    def peak_search(self, pointer, pre_move):

        self.new_pointers.append(pointer)
        self.new_moves.append(pre_move)

        while len(self.new_pointers):

            print("Number of branches: " + str(self.number_of_branches))
            
            pointer = self.new_pointers[0]
            self.new_pointers.pop(0)
            pre_move = self.new_moves[0]
            self.new_moves.pop(0)
            
            while True:
                #self.way_to_fence.append(pointer)
                
                #Return if images dimentions exceeded
                if self.image_dimentions_exceeded(self.img_height, self.img_width, 300, 20, pointer):
                    break
                
                #See if pointer has reached a junction. Here the neighboors will be checked. If the 
                #neighboor has a pixel intensity above a threshold a junction has proberly been found. Now move 
                #the pointer in the other two direction (exept for pre move) to find the lines in which 
                #the pointer has to branch from. 
                peak_found = True
                new_branches = []
                new_moves = []
                for (move_dir,search_dir,index) in zip(self.peak_move_direction[pre_move],self.peak_search_direction[pre_move],self.peak_start_end_index):
                    found, branch_pointer = self.fence_connection_search(pointer, move_dir, search_dir, self.peak_threshold, index[0], index[1], self.step_size_peak)
                    if not found:
                        peak_found = False
                        break
                    else:
                        new_branches.append(branch_pointer)
                        new_moves.append(search_dir)

                #New peak detected. See if the peak has already been found
                if peak_found:
                    new_peak_found = True
                    for peak in self.peaks:
                        delta_x = pointer[0] - peak[0]
                        delta_y = pointer[1] - peak[1]
                        dis = np.sqrt(delta_x*delta_x + delta_y*delta_y)
                        if dis < self.peak_dis_threshold:
                            new_peak_found = False
                            break
                    if new_peak_found:
                        peak_x = sum([pixel[0] for pixel in new_branches])/len(new_branches)
                        peak_y = sum([pixel[1] for pixel in new_branches])/len(new_branches)
                        pointer = [peak_x,peak_y]
                        self.peaks.append(pointer)
                        for (new_branch, new_move) in zip(new_branches, new_moves):
                            self.number_of_branches += 1
                            self.new_pointers.append(new_branch)
                            self.new_moves.append(new_move)
                    break
                
                #Search in pre move direction. If only black pixels values are found, move one 
                #pixel location to the side and try again. This solution takes into account skews in the 
                #fence structure. This continoues to end is reached (could be end of fence or hole in the fence).
                end_reached = True
                found, pointer = self.fence_connection_search(pointer, self.line_move_direction[pre_move], pre_move, self.line_threshold, -1, 2, self.step_size_line)
                if found:
                    end_reached = False
                
                #See if endpoint has aleady been detected
                if end_reached:
                    self.end_points.append(pointer)
                    break
            
            self.number_of_branches -= 1
    
    #Helper functions
    def next_moves(self, pointer, step_size):
        
        #Update possible directions to move
        up_left = [pointer[0] - step_size, pointer[1] - step_size]
        up_right = [pointer[0] + step_size, pointer[1] - step_size]
        down_left = [pointer[0] - step_size, pointer[1] + step_size]
        down_right = [pointer[0] + step_size, pointer[1] + step_size]

        #Place directions in config vector
        config = [up_right, down_right, down_left, up_left]
        
        return config
    
    def fence_connection_search(self, pointer, move_direction, search_direction, threshold, start_index, end_index, step_size):
        
        iteration = range(start_index, end_index)

        best_move = self.next_moves(pointer, 0)
        best_threshold = 0
        
        for index in iteration:
            for step in step_size:
                moves = self.next_moves(pointer, index)
                moves = self.next_moves(moves[move_direction], step)
                x = moves[search_direction][0]
                y = moves[search_direction][1]
                #print(self.twenty_low_pass_filtered.shape)
                if abs(self.twenty_low_pass_filtered[y,x]) >= best_threshold:
                    best_threshold = self.twenty_low_pass_filtered[y,x]
                    best_move = moves[search_direction]
        
        if best_threshold >= threshold:
            return True, best_move

        #No connection found
        return False, pointer

    def move_pointer_to_fence(self, pointer, i):
        
        #Direction to move
        pre_move = i
        while self.twenty_low_pass_filtered[pointer[1], pointer[0]] < self.line_threshold:
            config = self.next_moves(pointer, 1)
            pointer = config[pre_move]
            self.way_to_fence.append(pointer)
            #if self.image_dimentions_exceeded(self.img_height,self.img_width,50,pointer):
            #    return
        for move in self.valid_moves[pre_move]:
            found, pointer = self.fence_connection_search(pointer, pre_move, move, self.line_threshold, 0, 1, self.step_size_line)
            if found:
                return pointer, move

        return pointer, move

    def image_dimentions_exceeded(self, img_height, img_width, max_dis_width, max_dis_height, pointer):

        if (pointer[0] > self.roi_bottom_right[0]) or (pointer[0] < self.roi_top_left[0]):
            return True
        if (pointer[1] > self.roi_bottom_right[1]) or (pointer[1] < self.roi_top_left[1]):
            return True

        #Image dimentions no exceeded
        return False

    def find_roi(self, end_points, rescale_x, rescale_y):

        #Sort end points from lowest to biggest
        sorted_lowest = sorted(end_points,key=lambda x: x[1])
        
        for _ in range(20):
            if len(sorted_lowest) > 0:
                sorted_lowest.pop(0)
        for _ in range(30):
            if len(sorted_lowest):
                sorted_lowest.pop(-1)
        
        #Define ROI
        top_left_roi = self.roi_top_left
        bottom_right_roi = self.roi_bottom_right
        roi = [top_left_roi, bottom_right_roi]
        
        #Find end points inside ROI
        points_in_roi = []
        for point in sorted_lowest:
            if (point[0] > roi[0][0]) and (point[1] > roi[0][1]) and (point[0] < roi[1][0]) and (point[1] < roi[1][1]):
                points_in_roi.append(point)
        return roi, points_in_roi


    def find_breach(self, points_in_roi, dist):
        X = np.asarray(points_in_roi)
        clustering = AgglomerativeClustering(distance_threshold=dist, n_clusters=None, linkage='average')
        clustering = clustering.fit(X)

        clusters = []
        for (point,label) in zip(points_in_roi,clustering.labels_):
            cluster = []
            cluster.append(label)
            cluster.append(point)
            clusters.append(cluster)

        sorted_clusters = sorted(clusters,key=lambda x: x[0])
        n_cluster = sorted_clusters[len(sorted_clusters)-1][0]
        clusters = []
        means = []
        for index in range(0,n_cluster+1):
            cluster = []
            mean_x = 0
            mean_y = 0
            for point in sorted_clusters:
                if point[0] == index:
                    mean_x += point[1][0]
                    mean_y += point[1][1]
                    cluster.append(point[1])
            means.append([(mean_x/len(cluster)),(mean_y/len(cluster))])
            clusters.append(cluster)

        r = []
        dist = 0.0
        for (cluster, mean) in zip(clusters,means):
            counter = 1
            dist = 0.0
            for point in cluster:
                print(point)
                dist += np.sqrt((point[0]-mean[0])**2 + (point[1]-mean[1])**2)
                counter += 1
            
            r.append(dist/counter)

        return means, r, clusters

if __name__ == "__main__":

    fd = fence_detection()

    img = cv2.imread('input/hole1.jpg', 0)
    h, w = img.shape
    img = cv2.resize(img, (int(w/3),int(h/3)))
    
    fd.img_height, fd.img_width = img.shape
    
    fd.roi_top_left = [(fd.img_width/2-100), 20]
    fd.roi_bottom_right = [(fd.img_width/2+100),fd.img_height-20]

    fd.fence_extraction(img)
    for i in range(4):
        
        print(i)
        pointer, pre_move = fd.move_pointer_to_fence([fd.img_width//2,fd.img_height//2],i)
        if pointer is not None:
            fd.peak_search(pointer, pre_move)
        
        if len(fd.peaks) > 20:
            break
        fd.peaks = []
        fd.end_points = []
   
    roi, points_in_roi = fd.find_roi(fd.end_points, 0.35, 0.0) #0.35 0.0
    means, std, clusters = [], [], []
    if len(points_in_roi):
        means, std, clusters = fd.find_breach(points_in_roi,100)
    
    img = fd.original_img.copy()
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

    #Draw init way to fence
    print('Number of points to fence (white): ' + str(len(fd.way_to_fence)))
    for point in fd.way_to_fence:
        cv2.circle(img,(int(point[0]),int(point[1])),1,(255,255,255),2)
    
    #Draw peaks
    print('Found numbers of peaks (blue): ' + str(len(fd.peaks)))
    for point in fd.peaks:
        cv2.circle(img,(int(point[0]),int(point[1])),1,(255,0,0),2)

    #Draw ROI
    img = cv2.rectangle(img,(int(roi[0][0]),int(roi[0][1])),(int(roi[1][0]),int(roi[1][1])),(255,255,255),1)

    #Draw points inside ROI
    #print('Found number of points in ROI (green): ' + str(len(points_in_roi)))
    for point in points_in_roi:
        cv2.circle(img,(int(point[0]),int(point[1])),1,(0,255,0),2)

    for (r, mean, cluster) in zip(std, means, clusters):
        if r*2 > 50:
            for point in cluster:
                cv2.circle(img,(int(point[0]),int(point[1])),1,(0,0,255),2)
            cv2.circle(img,(int(mean[0]),int(mean[1])),int(r),(0,0,255),1)
            cv2.circle(img,(int(mean[0]),int(mean[1])),5,(255,255,255),5)
        print('Diameter: ' + str(r*2))
    
    
    print(len(points_in_roi))
    cv2.imwrite("output/peaks_endpoints.png", img)


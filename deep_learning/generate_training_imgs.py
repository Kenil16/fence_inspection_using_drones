import sys
import argparse
import cv2
import numpy as np
import os
import random
#from yattag import Doc, indent
import matplotlib.pyplot as plt
from xml.etree import ElementTree

class video_conversion:

    def __init__(self):
        self.record_img_delay = 500 #ms
        self.alpha = 1.0
        self.beta = 40
        self.augment = False
        self.count = 0
        self.category_id_to_name = {0: 'fence_clean', 1: 'fence_breach', 2:'sign'}
        pass

    def filelist(self, root, file_type):
        return [os.path.join(directory_path, f) for directory_path, directory_name, 
                files in os.walk(root) for f in files if f.endswith(file_type)]

    def write_xml(self, file_out_path, file_name, id_, img_name, class_label, bb_xmin = 0, bb_ymin = 0, bb_xmax = 0, bb_ymax = 0, width = 0, height = 0, depth = 0):

        #Build xml files with annotations
        doc, tag, value = Doc().tagtext()
        with tag('annotation'):
            with tag('filename'):
                value(img_name)
            with tag('id'):
                value(id_)
            with tag('size'):
                with tag('width'):
                    value(width)
                with tag('height'):
                    value(height)
                with tag('depth'):
                    value(depth)
            with tag('object'):
                with tag('name'):
                    value(class_label)
                with tag('bndbox'):
                    with tag('xmin'):
                        value(bb_xmin)
                    with tag('ymin'):
                        value(bb_ymin)
                    with tag('xmax'):
                        value(bb_xmax)
                    with tag('ymax'):
                        value(bb_ymax)
        
        result = indent(
            doc.getvalue(),
            indentation = ' '*4,
            newline = '\r\n'
        )

        #Write annotations to file
        path = file_out_path + file_name
        anno = open(path,'w')
        anno.write(result)
        anno.close()

    def video_to_images(self, path_in, path_out):
        count = 0
        vidcap = cv2.VideoCapture(path_in)
        success, image = vidcap.read()
        success = True
        while success:
            vidcap.set(cv2.CAP_PROP_POS_MSEC,(count*self.record_img_delay)) 
            success, image = vidcap.read()
            if success:
                h,w,c = image.shape
                image_resized = cv2.resize(image,(int(w/4),int(h/4)))
                if self.augment:
                    image_resized = self.change_contrast_brightness(image_resized,self.alpha,self.beta)
                
                cv2.imwrite(path_out + "image_original%d.png" % count, image_resized)
                count = count + 1

    def change_contrast_brightness(self,img,alpha,beta):
        
        for y in range(img.shape[0]):
            for x in range(img.shape[1]):
                for c in range(img.shape[2]):
                    img[y,x,c] = np.clip(alpha*img[y,x,c] + beta, 0, 255)
        return img

    def make_mask(self, x, y, r, img, img_mask = None):
        x = range(int((x-r/2)), int((x+r/2)))
        y = range(int((y-r/2)), int((y+r/2)))
        mask = []
        mask_weights = self.mask_weights(len(x),len(y))

        index_x, index_y = 0, 0
        for point_y in y:
            for point_x in x:
                try: 
                    if img_mask == None:
                        color = [0,0,0] #Make background black by default
                except:
                    color = [img_mask[point_y, point_x, 0], img_mask[point_y, point_x, 1], img_mask[point_y, point_x, 2]]
                point = [point_x, point_y, mask_weights[index_y,index_x], color]
                index_x += 1
                mask.append(point)
            index_x = 0
            index_y += 1
        return mask

    def mask_weights(self, rows, cols):

        #Initite kernel 
        m = np.zeros((rows,cols),dtype=float)
        
        i_start, i_end, j_start, j_end = 0, cols-1, 0, rows-1
        scale = .1
        
        #Give weights to each pixel to get a smooth transition between mask and original image 
        for _ in range(rows//2):
            m[j_start:j_end, i_start] = scale
            m[j_end, i_start:i_end] = scale
            m[j_end:j_start:-1, i_end] = scale
            m[j_start, i_end:i_start:-1] = scale
            i_start += 1
            i_end -= 1
            j_start += 1
            j_end -= 1
            if scale < 1.0:
                scale += 0.1

        #If rows or cols are a odd number give center image last weight
        if not rows % 2 == 0:
            m[rows//2,cols//2] = scale

        return m

    def generate_random_breaches(self, path_in, path_in_mask, path_out, path_out_anno, max_region_width, max_region_height, min_breach_size, max_breach_size):
        
        #Load files to be used in random breach generation and shuffle
        files = self.filelist(path_in,'png')
        files_mask = self.filelist(path_in_mask,'jpg')
        files_shuffle = random.sample(files,len(files))
        
        #Define region from where random breaches can be generated
        img = cv2.imread(files[0])
        h,w,c = img.shape
        region = [[(w/2-max_region_width),(h/2-max_region_height)],[(w/2+max_region_width),(h/2+max_region_height)]]
        
        #Define mask used as background for possible breach
        img_mask = cv2.imread(random.choice(files_mask))
        h,w,c = img_mask.shape
        mask_resized = cv2.resize(img_mask,(int(w/8),int(h/8)))
        
        for file_ in files_shuffle:
            img = cv2.imread(file_)
            h,w,c = img.shape
            #Choice to make custom breach (>=1). Else save data to folders with fence_clean class label
            make_breach = 1#random.randint(0,1)
            
            if not make_breach:
                #Write images and annotations to folder
                self.write_xml(path_out_anno, "file%d.xml" % self.count, self.count, "image%d.png" % self.count, 'fence_clean', width=w, height=h, depth=c)
                cv2.imwrite(path_out + "image%d.png" % self.count, img)
                self.count += 1
            else:
                #Define kernel size (size of breach)
                x = random.randint(region[0][0], region[1][0])
                y = random.randint(region[0][1], region[1][1])
                r = random.randint(min_breach_size, max_breach_size)
                
                #Apply mask to the original image
                mask = self.make_mask(x, y, r, img)#, mask_resized)
                for point in mask:
                    for c_ in range(c):
                        img[point[1], point[0], c_] = np.clip(((1-point[2])*img[point[1], point[0], c_] + point[2]*point[3][c_]), 0, 255)

                #Define points for boundary box
                upper_left = (int(mask[0][0]-mask[0][2]/2-15),(int(mask[0][1]-mask[0][2]/2-15)))
                bottom_right = (int(mask[-1][0]-mask[-1][2]/2+15),(int(mask[-1][1]-mask[-1][2]/2+15)))
        
                #Write images and annotations to folder 
                self.write_xml(path_out_anno,"file%d.xml" % self.count, self.count, "image%d.png" % self.count, 'fence_breach', mask[0][0], mask[0][1], mask[-1][0], mask[-1][1], w, h, c)
                self.visualize(img, path_out + "image%d.png" % self.count, [[upper_left, bottom_right]], [1], self.category_id_to_name,False)
                self.count += 1

    #Inspirations from https://github.com/albumentations-team/albumentations_examples/blob/master/notebooks/example_bboxes.ipynb
    def visualize_bbox(self, img, bbox): #, class_name):
        print(bbox)
        x_min, y_min, x_max, y_max = map(int, bbox)
        cv2.rectangle(img, (x_min,y_min),(x_max,y_max), color=(0,0,255), thickness=2)
        
        #((text_width, text_height), _) = cv2.getTextSize(class_name, cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)    
        #cv2.rectangle(img, (bbox[0][0], bbox[0][1] - int(1.3 * text_height)), (bbox[0][0] + text_width, bbox[0][1]), (0,0,255), -1)
        """
        cv2.putText(
            img,
            text=class_name,
            org=(bbox[0][0], bbox[0][1] - int(0.3 * text_height)),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.35, 
            color=(255,255,255), 
            lineType=cv2.LINE_AA,
        )
        """
        return img


    def visualize(self, image, path_out, bboxes, visualize=False): # category_ids, category_id_to_name, visualize = False):
        img = image.copy()
        if visualize:
            for bbox in bboxes[0]: # category_id in zip(bboxes, category_ids):
                #class_name = category_id_to_name[category_id]
                img = self.visualize_bbox(img, bbox)#, class_name)
            plt.figure(figsize=(12, 12))
            plt.axis('off')
        cv2.imwrite(path_out, img)
        
        
    def extract_boxes(self, filename):
        tree = ElementTree.parse(filename)
        root = tree.getroot()
        boxes = list()
        
        for box in root.findall('.//bndbox'):
            xmin = int(box.find('xmin').text)
            ymin = int(box.find('ymin').text)
            xmax = int(box.find('xmax').text)
            ymax = int(box.find('ymax').text)
            coors = [xmin, ymin, xmax, ymax]
            boxes.append(coors)
            width = int(root.find('.//size/width').text)
            height = int(root.find('.//size/height').text)
        
        return boxes, width, height


if __name__=="__main__":
    
    vd = video_conversion()
    #vd.video_to_images("input/grass/grass.mp4","input/grass/images/")
    #vd.generate_random_breaches('input/grass/images','input/grass/mask/','output/fence_grass/images/', 'output/fence_grass/annotations/',50,50,30,50)

    img = cv2.imread('input/fence_train/big_dataset_fences/images/image161.png')
    bb = vd.extract_boxes('input/fence_train/big_dataset_fences/annotations/file161.xml')
    vd.visualize(img, 'img_bb161.png', bb, True)

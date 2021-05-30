import sys
import argparse
import cv2
import numpy as np
import os
import random
#from yattag import Doc, indent
import matplotlib.pyplot as plt
import json as j
import xml.etree.cElementTree as e
from xml.etree import ElementTree
from image_augmentation import augmentation 
from imgaug.augmentables.bbs import BoundingBox, BoundingBoxesOnImage

class json_to_xml:

    def __init__(self):
        
        self.aug = augmentation()
        
        self.xmin = 0
        self.ymin = 0
        self.xmax = 0
        self.ymax = 0
        self.bb = []

        self.w = 0
        self.h = 0
        self.c = 0

        self.img = None
        self.bbs_aug = None
        self.aug_img = None

        self.use_aug = False
    
    def filelist(self, root, file_type):
        return [os.path.join(directory_path, f) for directory_path, directory_name,
                files in os.walk(root) for f in files if f.endswith(file_type)]

    def json_to_xml(self, file_in_path, file_out_path_anno, file_out_path_images):
        
        files = self.filelist(file_in_path,'json')
        counter = 0
        for file_ in files:


            name = file_[42:-5]
            path_in = 'input/fence_train/object_detection/images/' + name
            path_out = file_out_path_images
            
            self.img = cv2.imread(path_in)
            self.h, self.w, self.c = self.img.shape

            if self.h < 3000 and self.w < 3000:
                
                with open(file_) as json_format_file:
                    d = j.load(json_format_file)
                
                for _ in range(1):

                    if self.use_aug:
                        bbox = []
                        for box in self.bb:
                            bbox.append(BoundingBox(x1=box[0], y1=box[1], x2=box[2], y2=box[3]))
                        bbs = BoundingBoxesOnImage(bbox, shape=self.img.shape)
                        self.aug_img, self.bbs_aug = self.aug.seq(image=self.img, bounding_boxes=bbs)
                        self.h, self.w, self.c = self.img.shape

                    r = e.Element("annotations")
                    e.SubElement(r,"filename").text = "image%d.png" % counter
                    e.SubElement(r,"id").text = str(counter)
                    
                    size = e.SubElement(r,"size")
                    e.SubElement(size,"width").text = str(self.w)
                    e.SubElement(size,"height").text = str(self.h)
                    e.SubElement(size,"depth").text = str(self.c)

                    if not self.use_aug:
                        for i in d["objects"]:
                            box = []
                            for k in i["points"]["exterior"]:
                                box.append(k[0])
                                box.append(k[1])
                                
                                if len(box) == 4:
                                    self.bb.append(box)
                                    objects = e.SubElement(r,"object")
                                    e.SubElement(objects,"name").text = 'fence_breach'
                                    bb = e.SubElement(objects,"bndbox")
                                    
                                    e.SubElement(bb,"xmin").text = str(box[0])
                                    e.SubElement(bb,"ymin").text = str(box[1])
                                    e.SubElement(bb,"xmax").text = str(box[2])
                                    e.SubElement(bb,"ymax").text = str(box[3])
                                    box = []

                    else:
                        for item in self.bbs_aug:
                            objects = e.SubElement(r,"object")
                            e.SubElement(objects,"name").text = 'fence_breach'
                            bb = e.SubElement(objects,"bndbox")
                            e.SubElement(bb,"xmin").text = str(item[0][0]).split('.')[0]
                            e.SubElement(bb,"ymin").text = str(item[0][1]).split('.')[0]
                            e.SubElement(bb,"xmax").text = str(item[1][0]).split('.')[0]
                            e.SubElement(bb,"ymax").text = str(item[1][1]).split('.')[0]

                    a = e.ElementTree(r)
                    
                    path = file_out_path_anno +  "file%d.xml" % counter
                    if self.use_aug:
                        cv2.imwrite( path_out + "image%d.png" % counter, self.aug_img)
                    else:
                        cv2.imwrite( path_out + "image%d.png" % counter, self.img)
                        self.use_aug = True

                    a.write(path)
                    counter += 1

            self.use_aug = False
            self.bb = []

if __name__=="__main__":

    jtx = json_to_xml()
    jtx.json_to_xml('input/fence_train/object_detection/labels/','input/fence_train/small_dataset_fences/annotations/','input/fence_train/small_dataset_fences/images/')
    #jtx.json_to_xml('input/fence_train/object_detection/labels/','input/fence_train/big_dataset_fences/annotations/','input/fence_train/big_dataset_fences/images/')


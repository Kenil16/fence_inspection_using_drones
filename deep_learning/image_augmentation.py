import imgaug as ia
import imgaug.augmenters as iaa
from imgaug.augmentables.bbs import BoundingBox, BoundingBoxesOnImage
import cv2 

#Inspriration taking from https://github.com/aleju/imgaug
class augmentation:

    def __init__(self):

        self.sed = ia.seed(1)
        self.sometimes = lambda aug: iaa.Sometimes(1.0, aug)
        self.seq = iaa.Sequential(
                [
                
                #Execute 0 to 5 of the following  augmenters per image. 
                #Don't execute all of them, as that would often be way too strong.
                iaa.Affine(scale=(0.75, 1.25)),
                iaa.MotionBlur(k=5),

                iaa.SomeOf((0, 5),
                    [
                        #Blur each image with varying strength using
                        #Gaussian blur (sigma between 0 and 3.0),
                        #Average/uniform blur (kernel size between 2x2 and 7x7)
                        #Median blur (kernel size between 3x3 and 11x11).
                        iaa.OneOf([
                            iaa.GaussianBlur((0, 1.0)),
                            iaa.AverageBlur(k=(1, 5)),
                            iaa.MedianBlur(k=(1, 5)),
                        ]),

                        #Sharpen each image, overlay the result with the original
                        #image using an alpha between 0 (no sharpening) and 1
                        #(full sharpening effect).
                        iaa.Sharpen(alpha=(0, 1.0), lightness=(0.8, 1.2)),

                        #Same as sharpen, but for an embossing effect.
                        iaa.Emboss(alpha=(0, 1.0), strength=(0, 1.0)),
                        iaa.PerspectiveTransform(scale=(0.01, 0.15)),


                        #Add gaussian noise to some images.
                        #In 50% of these cases, the noise is randomly sampled per
                        #channel and pixel.
                        #In the other 50% of all cases it is sampled once per
                        #pixel (i.e. brightness change).
                        iaa.AdditiveGaussianNoise(
                            loc=0, scale=(0.0, 0.05*255), per_channel=0.5
                        ),

                        #Add a value of -10 to 10 to each pixel.
                        iaa.Add((-5, 5), per_channel=0.5),

                        iaa.imgcorruptlike.Snow(severity=1),
                        iaa.Rain(drop_size=(0.10, 0.20)),
                        iaa.Snowflakes(flake_size=(0.1, 0.4), speed=(0.01, 0.05)),
                        iaa.Fog(),

                        #Change brightness of images (50-150% of original value).
                        iaa.Multiply((0.8, 1.1), per_channel=0.5),

                        #Improve or worsen the contrast of images.
                        iaa.LinearContrast((0.8, 1.0), per_channel=0.5),
                        
                        #In some images distort local areas with varying strength.
                        self.sometimes(iaa.PiecewiseAffine(scale=(0.01, 0.03)))
                    ],
                    #Do all of the above augmentations in random order
                    random_order=True
                )
            ],
            random_order=True
        )

if __name__=="__main__":
 
    aug = augmentation()

    image = cv2.imread('input/fence_train/fences/images/image4.png')
    
    bbs = BoundingBoxesOnImage([
        BoundingBox(x1=572, y1=165, x2=717, y2=309),
        BoundingBox(x1=479, y1=516, x2=695, y2=606),
        BoundingBox(x1=20, y1=143, x2=330, y2=469)
    ], shape=image.shape)

    # Augment BBs and images.
    image_aug, bbs_aug = aug.seq(image=image, bounding_boxes=bbs)
    # print coordinates before/after augmentation (see below)
    # use .x1_int, .y_int, ... to get integer coordinates
    for i in range(len(bbs.bounding_boxes)):
        before = bbs.bounding_boxes[i]
        after = bbs_aug.bounding_boxes[i]
        print("BB %d: (%.4f, %.4f, %.4f, %.4f) -> (%.4f, %.4f, %.4f, %.4f)" % (
            i,
            before.x1, before.y1, before.x2, before.y2,
            after.x1, after.y1, after.x2, after.y2)
        )

    # image with BBs before/after augmentation (shown below)
    image_before = bbs.draw_on_image(image, size=2)
    image_after = bbs_aug.draw_on_image(image_aug, size=2, color=[0, 0, 255])

    cv2.imwrite('test1.png',image_before)

#!/usr/bin/env python

import cv2
import numpy as np
from utils import mouse_handler
from utils import get_four_points
import sys

# add ROS
import rospy
from std_msgs.msg import Int64
from std_msgs.msg import String
from people_msgs.msg import PositionMeasurementArray

img_name = ['black', 'circle_image', 'left_arrow', 'right_arrow']
path = ''


def callback_project(msg):
    global path, img_name
    path = '/home/abhishek/Pictures/warp_' + msg.data + '.png'

def homography(projector_img, img_name):

    # Read destination image (image of the projector)
    im_dst = cv2.imread(projector_img);
    print(im_dst.shape)

    # Get four corners of the billboard
    print 'Click on four corners of a billboard and then press ENTER'
    pts_dst = get_four_points(im_dst)

    # generate layered image

    # crop generated image

    for i in range(len(img_name)):
        # Read source image.
        src_path = '/home/abhishek/Pictures/' + img_name[i] + '.png'
        im_src = cv2.imread(src_path);
        size = im_src.shape
       
        # Create a vector of source points.
        pts_src = np.array(
                           [
                            [0,0],
                            [size[1] - 1, 0],
                            [size[1] - 1, size[0] -1],
                            [0, size[0] - 1 ]
                            ],dtype=float
                           );

    
        # Calculate Homography between source and destination points
        h, status = cv2.findHomography(pts_src, pts_dst);
        
        # Warp source image
        im_temp = cv2.warpPerspective(im_src, h, (im_dst.shape[1],im_dst.shape[0]))

        # Black out polygonal area in destination image.
        cv2.fillConvexPoly(im_dst, pts_dst.astype(int), 0, 16);
        
        # Add warped source image to destination image.
        im_dst = im_dst + im_temp;

        path = '/home/abhishek/Pictures/warp_' + img_name[i] + '.png'
        cv2.imwrite(path,im_temp)

def project():
    rospy.init_node('projector')
    rospy.Subscriber('project', String, callback_project)

    global path
    while not rospy.is_shutdown():
        img = cv2.imread(path)
        print(path)
        cv2.imshow("Image", img);
        cv2.waitKey(1)

if __name__ == '__main__' :
    proj_width = 1154 # 1024
    proj_hight = 768
    global path, img_name

    # image processing
    projector_img = '/home/abhishek/Pictures/IMG_2252.png'
    homography(projector_img, img_name)
    print('Finish homography transitioning')

    # Display image.
    cv2.namedWindow("Image", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("Image", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    path = '/home/abhishek/Pictures/warp_' + 'black' + '.png'
    project()
    

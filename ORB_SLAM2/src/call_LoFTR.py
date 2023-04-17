import matplotlib.pyplot as plt
import cv2
import kornia as K
import kornia.feature as KF
import numpy as np
import torch
from kornia_moons.feature import *
## needed to be called by c
import sys

if not hasattr(sys, 'argv'):
    sys.argv  = ['']
##

def load_torch_image(fname):
    img = K.image_to_tensor(cv2.imread(fname), False).float() /255.
    img = K.color.bgr_to_rgb(img)
    return img

def compare(img1_np, img2_np):
    #fname1 = 'kn_church-2.jpg'
    #fname2 = 'kn_church-8.jpg'
    #fname1 = 'Rosario_4/frame0002.jpg'
    #fname2 = 'Rosario_4/frame0003.jpg'
    # 
    # 
    #img1 = K.geometry.resize(load_torch_image(img1), (600, 375), antialias=True)
    #img2 = K.geometry.resize(load_torch_image(img2), (600, 375), antialias=True)
    img1 = torch.from_numpy(img1_np)
    img2 = torch.from_numpy(img2_np)
    # 
    # 
    matcher = KF.LoFTR(pretrained='outdoor')
    # 
    input_dict = {"image0": K.color.rgb_to_grayscale(img1), # LofTR works on grayscale images only 
                  "image1": K.color.rgb_to_grayscale(img2)}
    # 
    with torch.inference_mode():
        correspondences = matcher(input_dict)

    """Now let's clean-up the correspondences with modern RANSAC and estimate fundamental matrix between two images"""

    mkpts0 = correspondences['keypoints0'].cpu().numpy()
    mkpts1 = correspondences['keypoints1'].cpu().numpy()
    Fm, inliers = cv2.findFundamentalMat(mkpts0, mkpts1, cv2.USAC_MAGSAC, 0.5, 0.999, 100000)
    inliers = inliers > 0

    """Finally, let's draw the matches with a function from [kornia_moons](https://ducha-aiki.github.io/kornia_moons/feature.html#draw_LAF_matches). The correct matches are in green and imprecise matches - in blue"""

    draw_LAF_matches(
        KF.laf_from_center_scale_ori(torch.from_numpy(mkpts0).view(1,-1, 2),
                                    torch.ones(mkpts0.shape[0]).view(1,-1, 1, 1),
                                    torch.ones(mkpts0.shape[0]).view(1,-1, 1)),

        KF.laf_from_center_scale_ori(torch.from_numpy(mkpts1).view(1,-1, 2),
                                    torch.ones(mkpts1.shape[0]).view(1,-1, 1, 1),
                                    torch.ones(mkpts1.shape[0]).view(1,-1, 1)),
        torch.arange(mkpts0.shape[0]).view(-1,1).repeat(1,2),
        K.tensor_to_image(img1),
        K.tensor_to_image(img2),
        inliers,
        draw_dict={'inlier_color': (0.2, 1, 0.2),
                'tentative_color': None, 
                'feature_color': (0.2, 0.5, 1), 'vertical': False})
    plt.show()
    return 0
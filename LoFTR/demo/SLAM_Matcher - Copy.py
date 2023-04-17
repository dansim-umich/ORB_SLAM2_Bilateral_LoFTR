#!/usr/bin/env python
# coding: utf-8

# # Demo LoFTR-DS on a single pair of images in an outdoor environment
# ### (This is a copy of the code in demo_single_pair.ipynb)
# 
# 
# This notebook shows how to use the loftr matcher with default config(dual-softmax) and the pretrained weights.

# In[15]:


import os
# os.chdir("..")
from copy import deepcopy
import sys
sys.path.append("/home/dansim/LoFTR")
if not hasattr(sys, 'argv'):
    sys.argv  = ['']

import torch
import cv2
import numpy as np
import matplotlib.cm as cm
#from matplotlib.pyplot import figure, show, imshow, subplots
import matplotlib.pyplot as plt
from src.utils.plotting import make_matching_figure

from masking import build_mask, apply_mask

# ## Outdoor Example

# In[16]:


from src.loftr import LoFTR, default_cfg

def match_images(img0_raw, img1_raw):
    print("\ngot to python")
    os.chdir("/home/dansim/LoFTR")
    #from src.loftr import LoFTR, default_cfg
    #print(img0_raw.shape)
    #print(img1_raw.shape)

    # The default config uses dual-softmax.
    # The outdoor and indoor models share the same config.
    # You can change the default values like thr and coarse_match_type.
    matcher = LoFTR(config=default_cfg, apply_mask=apply_mask)
    matcher.load_state_dict(torch.load("weights/outdoor_ds.ckpt")['state_dict'])
    # matcher = matcher.eval().cuda()
    matcher = matcher.eval()
    #np.save("img0.npy", img0_raw)
    #np.save("img1.npy", img1_raw)


    # Load example images
    #img0_pth = "assets/phototourism_sample_images/st_pauls_cathedral_30776973_2635313996.jpg"
    #img1_pth = "assets/phototourism_sample_images/st_pauls_cathedral_37347628_10902811376.jpg"
    #img0_raw = cv2.imread(img0_pth, cv2.IMREAD_GRAYSCALE)
    #img1_raw = cv2.imread(img1_pth, cv2.IMREAD_GRAYSCALE)
    img0_raw = cv2.resize(img0_raw, (img0_raw.shape[1]//8*8, img0_raw.shape[0]//8*8))  # input size shuold be divisible by 8
    img1_raw = cv2.resize(img1_raw, (img1_raw.shape[1]//8*8, img1_raw.shape[0]//8*8))


    img0 = torch.from_numpy(img0_raw)[None][None]
    img1 = torch.from_numpy(img1_raw)[None][None]
    img0 = img0.float()
    img1 = img1.float()

    """ Display both images: """
    if(0):
        fig, ax = plt.subplots(1, 2, figsize=(19.2, 10.8))
        ax[0].set_title("Img0 Raw")
        ax[0].imshow(img0_raw)#, cmap="gray")
        ax[1].set_title("Img1 Raw")
        ax[1].imshow(img1_raw)#, cmap="gray")
        plt.show()
        exit()

    #img0 = torch.from_numpy(img0_raw)[None][None] / 255.
    #img1 = torch.from_numpy(img1_raw)[None][None] / 255.

    mask0 = torch.ones_like(img0).squeeze().int()
    mask1 = torch.ones_like(img1).squeeze().int()

    # mask0 = torch.zeros_like(img0)
    # mask1 = torch.zeros_like(img1)

    # print(mask0.size())
    # exit()

    if(0):
        plt.figure()
        plt.imshow((img0 * mask0).squeeze(), cmap="gray")
        plt.show()
        exit()

    batch = {'image0': img0, 'image1': img1, "new_mask0": mask0, "new_mask1": mask1}

    # Inference with LoFTR and get prediction
    with torch.no_grad():
        matcher(batch)
        # exit()
        mkpts0 = batch['mkpts0_f'].cpu().numpy()
        mkpts1 = batch['mkpts1_f'].cpu().numpy()
        mconf = batch['mconf'].cpu().numpy()

        # print(f"img0_raw.size(): {img0.size()}")
        # print(f"mkpts0 size: {mkpts0.shape}")
        # exit()
        # print(f"Max mkpts0 along row: {mkpts0[:, 0].max()}")
        # print(f"Max mkpts0 along col: {mkpts0[:, 1].max()}")
        # exit()


    # In[7]:


    # Draw
    #color = cm.jet(mconf)
    #text = [
    #    'LoFTR',
    #    'Matches: {}'.format(len(mkpts0)),
    #]
    #fig = make_matching_figure(img0_raw, img1_raw, mkpts0, mkpts1, color, text=text)
    #show()

    fm0 = batch['feat_0_matches'].cpu().numpy()
    fm1 = batch['feat_1_matches'].cpu().numpy()
    fm0_size = fm0.shape
    fm1_size = fm1.shape
    mkpts0_size = mkpts0.shape
    mkpts1_size = mkpts1.shape
    print([fm0_size, fm1_size, mkpts0_size, mkpts1_size])
    fm0 = fm0.flatten()
    fm1 = fm1.flatten()
    mkpts0 = mkpts0.flatten()
    mkpts1 = mkpts1.flatten()
    fm0 = np.insert(fm0, 0, fm0_size[1])
    fm1 = np.insert(fm1, 0, fm1_size[1])
    mkpts0 = np.insert(mkpts0, 0, mkpts0_size[1])
    mkpts1 = np.insert(mkpts1, 0, mkpts1_size[1])
    fm0 = np.insert(fm0, 0, fm0_size[0])
    fm1 = np.insert(fm1, 0, fm1_size[0])
    mkpts0 = np.insert(mkpts0, 0, mkpts0_size[0])
    mkpts1 = np.insert(mkpts1, 0, mkpts1_size[0])
    fm0 = fm0.tolist()
    fm1 = fm1.tolist()
    mkpts0 = mkpts0.tolist()
    mkpts1 = mkpts1.tolist()
    print("finished python")
    return [fm0, fm1, mkpts0, mkpts1]
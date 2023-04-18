import os
from copy import deepcopy
import sys
if not hasattr(sys, 'argv'):
    sys.argv  = ['']

import torch
import cv2
import numpy as np
from src.utils.plotting import make_matching_figure
from src.loftr import LoFTR, default_cfg
from demo.masking import build_mask, apply_mask

def match_images(img0_raw, img1_raw):
    # The default config uses dual-softmax.
    # The outdoor and indoor models share the same config.
    # You can change the default values like thr and coarse_match_type.
    matcher = LoFTR(config=default_cfg, apply_mask=apply_mask)
    matcher.load_state_dict(torch.load("weights/outdoor_ds.ckpt")['state_dict'])
    # matcher = matcher.eval().cuda()
    matcher = matcher.eval()
    #np.save("img0.npy", img0_raw)
    #np.save("img1.npy", img1_raw)

    img0_raw = cv2.resize(img0_raw, (img0_raw.shape[1]//8*8, img0_raw.shape[0]//8*8))  # input size shuold be divisible by 8
    img1_raw = cv2.resize(img1_raw, (img1_raw.shape[1]//8*8, img1_raw.shape[0]//8*8))

    img0 = torch.from_numpy(img0_raw)[None][None] / 255.
    img1 = torch.from_numpy(img1_raw)[None][None] / 255.
    img0 = img0.float()
    img1 = img1.float()

    mask0 = torch.ones_like(img0).squeeze().int()
    mask1 = torch.ones_like(img1).squeeze().int()

    # mask0 = torch.zeros_like(img0)
    # mask1 = torch.zeros_like(img1)

    batch = {'image0': img0, 'image1': img1, "new_mask0": mask0, "new_mask1": mask1}

    # Inference with LoFTR and get prediction
    with torch.no_grad():
        matcher(batch)
        mkpts0 = batch['mkpts0_f'].cpu().numpy()
        mkpts1 = batch['mkpts1_f'].cpu().numpy()
        mconf = batch['mconf'].cpu().numpy()

    fm0 = batch['feat_0_matches'].cpu().numpy()
    fm1 = batch['feat_1_matches'].cpu().numpy()
    if (fm0.shape[0] > 1000):
        fm0 = fm0[:1000]
    if (fm1.shape[0] > 1000):
        fm1 = fm1[:1000]
    if (mkpts0.shape[0] > 1000):
        mkpts0 = mkpts0[:1000]
    if (mkpts1.shape[0] > 1000):
        mkpts1 = mkpts1[:1000]
    fm0_size = fm0.shape
    fm1_size = fm1.shape
    mkpts0_size = mkpts0.shape
    mkpts1_size = mkpts1.shape
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
    return [fm0, fm1, mkpts0, mkpts1]
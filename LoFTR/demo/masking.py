import matplotlib.pyplot as plt
import cv2 as cv
import numpy as np
import torch

def build_mask(img):

    if(len(img.shape) > 2):
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    else:
        gray = img

    edges = cv.Canny(gray, 50, 150, apertureSize=3)
    lines = cv.HoughLines(edges, 1, np.pi / 180, 200)

    mask = np.zeros_like(gray)

    for line in lines:
        rho, theta = line[0]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        cv.line(mask, (x1, y1), (x2, y2), (255), 20)


    mask //= mask.max()
    return mask


def apply_mask(data, method="random"):

    if(method == "random"):
        idxs = torch.randint(0, data["mkpts0_c"].size(0), (200,))

        data["mkpts0_c"] = data["mkpts0_c"][idxs, :]
        data["mkpts1_c"] = data["mkpts1_c"][idxs, :]

        data["coarse_i_ids"] = data["coarse_i_ids"][idxs]
        data["coarse_j_ids"] = data["coarse_j_ids"][idxs]


    elif(method == "canny+hough"):

        mask0 = data["new_mask0"].squeeze()
        mask1 = data["new_mask1"].squeeze()

        # print("")

        data["coarse_i_ids"] = data["coarse_i_ids"][mask0[data["mkpts0_c"][:, 1].int(), data["mkpts0_c"][:, 0].int()] == 1]
        data["coarse_j_ids"] = data["coarse_j_ids"][mask1[data["mkpts1_c"][:, 1].int(), data["mkpts1_c"][:, 0].int()] == 1]

        data["mkpts0_c"] = data["mkpts0_c"][mask0[data["mkpts0_c"][:, 1].int(), data["mkpts0_c"][:, 0].int()] == 1, :]
        data["mkpts1_c"] = data["mkpts1_c"][mask1[data["mkpts1_c"][:, 1].int(), data["mkpts1_c"][:, 0].int()] == 1, :]



if __name__ == "__main__":

    img = cv.imread("assets/phototourism_sample_images/london_bridge_19481797_2295892421.jpg")

    mask = build_mask(img)

    fig, ax = plt.subplots(1, 2)
    ax[0].imshow(img)
    ax[1].imshow(mask, cmap="gray")
    plt.show()
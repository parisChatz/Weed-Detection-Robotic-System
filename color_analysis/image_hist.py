import cv2
from matplotlib import pyplot as plt
import numpy as np
import time as t

target_image = ['plant.png','weed.png']

for image in target_image:

    img = cv2.imread(image)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    hsv_image = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    hsv_image = cv2.GaussianBlur(hsv_image,  ksize=(17, 17), sigmaX=10)


    hue, sat, val = hsv_image[:,:,0], hsv_image[:,:,1], hsv_image[:,:,2]
    # plt.figure(figsize=(10,8))
    plt.subplot(311)                             #plot in the first cell
    plt.subplots_adjust(hspace=.5)
    plt.title("Hue")
    plt.hist(np.ndarray.flatten(hue), alpha=0.5, bins=180,label=image)
    plt.legend(loc='upper right')
    plt.subplot(312)                             #plot in the second cell
    plt.title("Saturation")
    plt.hist(np.ndarray.flatten(sat), alpha=0.5, bins=125,label=image)
    plt.legend(loc='upper right')
    plt.subplot(313)                             #plot in the third cell
    plt.title("Luminosity Value")
    plt.hist(np.ndarray.flatten(val), alpha=0.5, bins=125,label=image)
    plt.legend(loc='upper right')
plt.show()
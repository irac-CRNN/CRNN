from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Flatten, LSTM
from keras.layers.convolutional import Conv2D, MaxPooling2D
from keras.layers.wrappers import TimeDistributed
from tensorflow.keras.preprocessing import image
from tensorflow.keras.utils import to_categorical
from keras.models import load_model
import numpy as np
import pandas as pd
from tqdm import tqdm
import cv2
import sys

new_model=load_model('model/crnn_model.h5')
cnt = 0
while True:
    frames = []

    img = cv2.imread('real_time_img.jpg')

    if cnt <= 25:
        frames = frames.append(img)
        cnt += 1
    else:
        cnt = 0

    if cnt == 25:
        input = np.array(frames) #??
        
        



import cv2     # for capturing videos
import math   # for mathematical operations
import pandas as pd
from glob import glob
from tqdm import tqdm
import natsort

# getting the names of all the images
images_train = glob("train/*.jpg")
images_train = natsort.natsorted(images_train)

images_test = glob("test/*.jpg")
images_test = natsort.natsorted(images_test)

train_image = []
train_class = []
test_image = []
test_class = []

for i in tqdm(range(len(images_train))):
    # creating the image name
    train_image.append(images_train[i].split('/')[1])
    # creating the class of image
    train_class.append(images_train[i].split('/')[1].split('_')[1])

for i in tqdm(range(len(images_test))):
    # creating the image name
    test_image.append(images_test[i].split('/')[1])
    # creating the class of image
    test_class.append(images_test[i].split('/')[1].split('_')[1])
    
# storing the images and their class in a dataframe
train_data = pd.DataFrame()
train_data['image'] = train_image
train_data['class'] = train_class

# storing the images and their class in a dataframe
test_data = pd.DataFrame()
test_data['image'] = test_image
test_data['class'] = test_class

# converting the dataframe into csv file 
train_data.to_csv('train_list.csv',header=True, index=False)
test_data.to_csv('test_list.csv',header=True, index=False)
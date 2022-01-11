import cv2
import depthai as dai
from tensorflow.keras.preprocessing import image
from tensorflow.keras.models import load_model

pipeline = dai.Pipeline()


model = load_model('model/crnn_model.h5')
print('Loaded model successfully!')



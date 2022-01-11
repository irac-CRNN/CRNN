from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Flatten
from keras.layers.convolutional import Conv2D, Conv3D, MaxPooling2D, MaxPooling3D
from keras.layers.wrappers import TimeDistributed
from tensorflow.keras.layers import LSTM, BatchNormalization
from tensorflow.keras.preprocessing import image
from tensorflow.keras.utils import timeseries_dataset_from_array
import numpy as np
import pandas as pd
from tensorflow.python.keras.engine.training import Model
from tqdm import tqdm

train = pd.read_csv('train_list.csv')
test = pd.read_csv('test_list.csv')

train_image = []
test_image = []

frame = []
k = 0

# for loop to read and store frames
for i in tqdm(range(train.shape[0])):
    train_img = image.load_img('train/'+train['image'][i], target_size=(128,72,3))
    train_img = image.img_to_array(train_img)
    train_img = train_img/255
    frame.append(train_img)
    k += 1
    if k > 25:
        train_image.append(frame)
        frame = []
        k = 0
    
for i in tqdm(range(test.shape[0])):
    test_img = image.load_img('test/'+test['image'][i], target_size=(128,72,3))
    test_img = image.img_to_array(test_img)
    test_img = test_img/255
    frame.append(test_img)
    k += 1
    if k > 25:
        test_image.append(frame)
        frame = []
        k = 0
    
X_train = np.array(train_image)
X_test = np.array(test_image)
print(X_test.shape)

Y_train = np.zeros((len(X_train),))
Y_train[0:10] = 0
Y_train[10:20] = 1
Y_train[20:30] = 2
Y_train[30:40] = 3
Y_train[40:50] = 4

Y_test = np.zeros((len(X_test),))
Y_test[0:2] = 0
Y_test[2:4] = 1
Y_test[4:6] = 2
Y_test[6:8] = 3
Y_test[8:10] = 4

model = Sequential()
model.add(Conv3D(32, (3,3,3), activation='relu', padding='same', input_shape=(3,128,72,26)))
model.add(BatchNormalization())
model.add(MaxPooling3D(pool_size=(2,2,2)))
model.add(Conv3D(64, (3,3,3), activation='relu', padding='same'))
model.add(BatchNormalization())
model.add(MaxPooling3D(pool_size=(2,2,2)))
model.add(Conv3D(128, (3,3,3), activation='relu', padding='same'))
model.add(BatchNormalization())
model.add(MaxPooling3D(pool_size=(2,2,2)))
model.add(Flatten())
model.add(Dense(128, activation='relu'))
model.add(Dense(5, activation='softmax'))
model.compile('adam', loss='categorical_crossentropy', metrics=['accuracy'])
model.summary()

history = model.fit(X_train, Y_train, validation_data=(X_test, Y_test), batch_size=1, epochs=10, shuffle=True)

model.save('model/3d_model.h5')
"""
model = Sequential()
model.add(TimeDistributed(Conv2D(32, (3,3), activation='relu', padding='same'), input_shape=(None,128,72,3)))
model.add(TimeDistributed(MaxPooling2D(pool_size=(2,2))))
model.add(TimeDistributed(Conv2D(64, (3,3), activation='relu', padding='same')))
model.add(TimeDistributed(MaxPooling2D(pool_size=(2,2))))
model.add(TimeDistributed(Conv2D(128, (3,3), activation='relu', padding='same')))
model.add(TimeDistributed(MaxPooling2D(pool_size=(2,2))))
model.add(TimeDistributed(Flatten()))
model.add(LSTM(128, activation='relu'))
model.add(Dense(64, activation='relu'))
model.add(Dense(5, activation='softmax'))
model.compile('adam', loss='categorical_crossentropy', metrics=['accuracy'])
model.summary()

history = model.fit(X_train, y_train, batch_size=16, epochs=10, validation_split = 0.2)

model.save('model/crnn_model.h5')
print('Saved model successfully')
"""

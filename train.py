from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Flatten, LSTM
from keras.layers.convolutional import Conv2D, MaxPooling2D
from keras.layers.wrappers import TimeDistributed
from tensorflow.keras.preprocessing import image
from tensorflow.keras.utils import to_categorical
import numpy as np
import pandas as pd
from tqdm import tqdm

train = pd.read_csv('train_list.csv')
test = pd.read_csv('test_list.csv')

train_image = []
test_image = []

frame = []
k = 0

# for loop to read and store frames
for i in tqdm(range(train.shape[0])):
    img = image.load_img('train/'+train['image'][i], target_size=(64,64,3))
    img = image.img_to_array(img)
    img = img/255
    frame.append(img)
    k += 1
    if k == 10:
        train_image.append(frame)
        frame = []
        k = 0
    
for i in tqdm(range(test.shape[0])):
    img = image.load_img('test/'+test['image'][i], target_size=(64,64,3))
    img = image.img_to_array(img)
    img = img/255
    frame.append(img)
    k += 1
    if k == 10:
        test_image.append(frame)
        frame = []
        k = 0
 
X_train = np.array(train_image)
X_test = np.array(test_image)

Y_train = np.zeros((len(X_train),), dtype=int)
Y_train[0:10] = 0
Y_train[10:20] = 1
Y_train[20:30] = 2
Y_train[30:40] = 3
Y_train[40:50] = 4
Y_train[50:60] = 5


Y_test = np.zeros((len(X_test),), dtype=int)
Y_test[0:3] = 0
Y_test[3:6] = 1
Y_test[6:9] = 2
Y_test[9:12] = 3
Y_test[12:15] = 4
Y_test[15:18] = 5

Y_train = to_categorical(Y_train,5)
Y_test = to_categorical(Y_test,5)

model = Sequential()
model.add(TimeDistributed(Conv2D(32, (3,3), activation='relu', padding='same'), input_shape=(None,64,64,3)))
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

model.fit(X_train, Y_train, batch_size=1, epochs=10, validation_data=(X_test, Y_test))

model.save('model/crnn_model.h5')
print('Saved model successfully')
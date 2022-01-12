from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Flatten, Bidirectional
from tensorflow.keras.layers import LSTM, BatchNormalization
from keras.layers.convolutional import Conv2D, MaxPooling2D
from keras.layers.wrappers import TimeDistributed
from tensorflow.keras.preprocessing import image
from tensorflow.keras.utils import to_categorical
import numpy as np
import pandas as pd
from tqdm import tqdm
import matplotlib.pyplot as plt

train = pd.read_csv('train_list.csv')
test = pd.read_csv('test_list.csv')

train_image = []
test_image = []

frame = []
k = 0
"""
# for loop to read and store frames
for i in tqdm(range(train.shape[0])):
    train_img = image.load_img('train/'+train['image'][i], target_size=(108,192,3))
    train_img = image.img_to_array(train_img)
    train_img = train_img/255
    frame.append(train_img)
    k += 1
    if (k > 3) & (k < 22) & (k % 4 == 0):
        frame.append(train_img)
    if k > 25:
        train_image.append(frame)
        frame = []
        k = 0
    
for i in tqdm(range(test.shape[0])):
    test_img = image.load_img('test/'+test['image'][i], target_size=(108,192,3))
    test_img = image.img_to_array(test_img)
    test_img = test_img/255
    frame.append(test_img)
    k += 1
    if (k > 3) & (k < 22) & (k % 4 == 0):
        frame.append(test_img)
    if k > 25:
        test_image.append(frame)
        frame = []
        k = 0
"""

# for loop to read and store frames
for i in tqdm(range(train.shape[0])):
    train_img = image.load_img('train/'+train['image'][i], target_size=(108,192,3))
    train_img = image.img_to_array(train_img)
    train_img = train_img/255
    frame.append(train_img)
    k += 1
    if k > 25:
        train_image.append(frame)
        frame = []
        k = 0
    
for i in tqdm(range(test.shape[0])):
    test_img = image.load_img('test/'+test['image'][i], target_size=(108,192,3))
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

Y_train = np.zeros((len(X_train),))
Y_train[0:10] = 0
Y_train[10:20] = 1
Y_train[20:30] = 2
Y_train[30:40] = 3
Y_train[40:50] = 4
Y_train[50:60] = 5
Y_train = to_categorical(Y_train)

Y_test = np.zeros((len(X_test),))
Y_test[0:2] = 0
Y_test[2:4] = 1
Y_test[4:6] = 2
Y_test[6:8] = 3
Y_test[8:10] = 4
Y_test[10:12] = 5
Y_test = to_categorical(Y_test)

model = Sequential()
model.add(TimeDistributed(Conv2D(32, (3,3), activation='relu', padding='same'), input_shape=(None,108,192,3)))
model.add(BatchNormalization())
model.add(TimeDistributed(MaxPooling2D(pool_size=(2,2), strides=(2,2))))
model.add(TimeDistributed(Conv2D(64, (3,3), activation='relu', padding='same')))
model.add(BatchNormalization())
model.add(TimeDistributed(MaxPooling2D(pool_size=(2,2), strides=(2,2))))
model.add(TimeDistributed(Conv2D(128, (3,3), activation='relu', padding='same')))
model.add(BatchNormalization())
model.add(TimeDistributed(MaxPooling2D(pool_size=(2,2), strides=(2,2))))
model.add(TimeDistributed(Conv2D(256, (3,3), activation='relu', padding='same')))
model.add(BatchNormalization())
model.add(TimeDistributed(MaxPooling2D(pool_size=(2,2), strides=(2,2))))
model.add(TimeDistributed(Flatten()))
model.add(Bidirectional(LSTM(512, activation='tanh', return_sequences=True)))
model.add(Bidirectional(LSTM(512, activation='tanh')))
#model.add(Dense(512, activation='relu'))
model.add(Dense(256, activation='relu'))
model.add(Dense(6, activation='softmax'))
model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])
model.summary()

hist = model.fit(X_train, Y_train, batch_size=1, epochs=100, shuffle=True, validation_data=(X_test, Y_test))

model.save('model/crnn_model2.h5')
print('Saved model successfully')

output = model.predict(X_test)
classes_x = np.argmax(output, axis=1)
print(classes_x)

_,train_accuracy = model.evaluate(X_train, Y_train)
_,test_accuracy = model.evaluate(X_test, Y_test)
print('Accuracy on train set: {:.2f}%'.format(100*train_accuracy))
print('Accuracy on test set: {:.2f}%'.format(100*test_accuracy))

train_loss=hist.history['loss']
val_loss=hist.history['val_loss']
train_acc=hist.history['accuracy']
val_acc=hist.history['val_accuracy']
xc=range(100)

timedisloss=plt.figure(1,figsize=(7,5))
plt.plot(xc,train_acc)
plt.plot(xc,train_loss)
plt.xlabel('num of Epochs')
plt.ylabel('acc & loss')
plt.title('train')
plt.grid(True)
plt.legend(['acc','loss'])
print("plotting")

timedisacc=plt.figure(2,figsize=(7,5))
plt.plot(xc,val_acc)
plt.plot(xc,val_loss)
plt.xlabel('num of Epochs')
plt.ylabel('acc & loss')
plt.title('validation')
plt.grid(True)
plt.legend(['acc','loss'])

plt.show()

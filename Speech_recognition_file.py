import tensorflow as tf
from keras.layers import Flatten,Dense,Dropout,Input
from tensorflow.keras.optimizers import Adam,SGD
import keras
from keras.regularizers import l2,l1
from keras.models import load_model
import csv
import numpy as np
from sklearn.preprocessing import StandardScaler
import sklearn as sk

x_train = []
y_train = []
with open('data.csv','r') as file:
    csvreader = csv.reader(file)
    next(csvreader)
    for row in csvreader:
        x_train.append([float(row[0]),float(row[1])])
        y_train.append([float(row[2]),float(row[3])])
    file.close()
x_train = np.array(x_train)
y_train = np.array(y_train)

#scaler = StandardScaler().fit(x_train)
#x_train = scaler.transform(x_train)
#print(x_train.shape)
#print(y_train.shape)

model = keras.models.Sequential([
    Input(shape=(x_train.shape[1])),
    Dense(20,activation='relu',kernel_regularizer=l2(0.001)),
    Dropout(0.2),
    Dense(40,activation='relu',kernel_regularizer=l2(0.001)),
    Dropout(0.2),
    Dense(20,activation='relu',kernel_regularizer=l2(0.001)),
    Dropout(0.2),
    Dense(2,activation='relu'),
])

epoches = 1000

opt = Adam(learning_rate=0.001,decay=0.001/epoches)
#opt = SGD(learning_rate=0.01,decay=0.01/epoches)
model.compile(
    optimizer=opt,
    loss='mse',
    metrics=['accuracy']
)
model.fit(x_train,y_train,batch_size=64,epochs=epoches,verbose=2)

model.save('Thymio.h5')
del model
'''
model = load_model('Thymio.h5')
index = 5
answer = model.predict(np.array([140,800]).reshape(-1,2))
print(answer)
#print(y_train[index])
'''
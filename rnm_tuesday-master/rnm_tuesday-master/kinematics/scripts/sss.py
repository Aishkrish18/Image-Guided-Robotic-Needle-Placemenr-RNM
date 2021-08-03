import pickle
import cv2
with open("/home/cenkt/rnm/files/scanning_output/scanning.pkl", "rb") as fp:   # Unpickling
    b = pickle.load(fp)

for i in b:
    print("image")
    print(i[0].shape)
    cv2.imshow("img",i[0])
    cv2.waitKey(0)
    print("tf")
    print(i[1])
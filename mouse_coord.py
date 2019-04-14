import pymouse
import keyboard
import numpy as np
import cv2

def get_circle_coord(top_left = [1151, 202], h_sep = 142, v_sep = 71, l_sep = 71):
    circ = []
    for y in range(11):
        p0 = [top_left[0]+y%2*l_sep, top_left[1]+y*v_sep]
        for x in range(4):
            p = [p0[0]+x*h_sep, p0[1]]
            circ.append(p)
    return circ

mouse = pymouse.PyMouse()
while True:
    if keyboard.is_pressed('esc'):
        break
    print(mouse.position())


# img = cv2.imread('calibration/Capture.PNG')
# circ = get_circle_coord()
# print("circ: ", np.array(circ).shape)
# for c in circ:
#     c = np.array(c)
#     img = cv2.circle(img, tuple(c.astype(np.int32)), 3, (255,0,255), cv2.FILLED)
# cv2.imshow('circle', img)
# cv2.waitKey(0)

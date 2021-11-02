# Python program to illustrate HoughLine
# method for line detection
import cv2
import numpy as np

# Reading the required image in
# which operations are to be done.
# Make sure that the image is in the same
# directory in which this python program is
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

while(1):
    ret, img = cap.read()
    cv2.imshow('original', img)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 75, 150)
    #lines = cv2.HoughLinesP(edges, 1, np.pi/180, 30, maxLineGap=250)
    lines = cv2.HoughLinesP(edges, 2, np.pi/180, 100, minLineLength = 30, maxLineGap=50)
    if lines is not None:
        for line in lines:
           x1, y1, x2, y2 = line[0]
           cv2.line(img, (x1, y1), (x2, y2), (0, 0, 128), 5)
    cv2.imshow("linesEdges", edges)
    cv2.imshow("linesDetected", img)
    
    # Wait for Esc key to stop
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

# Close the window
cap.release()

# De-allocate any associated memory usage
cv2.destroyAllWindows()

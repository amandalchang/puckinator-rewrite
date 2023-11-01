import cv2
import numpy as np
import time

cap = cv2.VideoCapture(0)  # Use 0 for the default webcam

lower_red = np.array([0, 120, 70])
upper_red = np.array([10, 255, 255])


# Initialize variables for FPS calculation
prev_time = time.time()
curr_time = time.time()


cv2.namedWindow("Frame")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask to isolate the bright red color
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(mask, (9, 9), 2)

    # Apply Hough Circle Transform
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=20, param1=50, param2=30, minRadius=50, maxRadius=100)

    if circles is not None:
        # Convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")
        print("i see circles")

        # Draw circles on the frame
        for (x, y, r) in circles:
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow("Circle Detection", frame)
    cv2.imshow("Mask", mask)

    # Exit the loop when the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
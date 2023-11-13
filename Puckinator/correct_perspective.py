import cv2 as cv
import numpy as np
import serial


# Constants
CALIB_FRAME = 10  # Number of frames grabbed
table_width = 3925
table_height = 1875
desired_fps = 35
arm_length = 11 # arm legnth in inches
displacement = 10 # distance between motors in inches

def coordinateconverter(cX, cY, arm_length, displacement):
    """
    Note:
        The origin is defined to be at the axis of movement of the left motor.
        This function is designed to return the desired angles of the two
        motors on a five-bar parallel robot relative to the horizontal given
        the length of the arms (assumed to be of equal length) and the distance
        between the motors. They must be in the same length units.
    Args:
        cX: The x coordinate of the center of the puck
        cY: The y coordinate of the center of the puck
        length: The length of each of the four arms (inches)
        displacement: The distance between the motors (inches)
    Returns:
        q1: the CCW angle of the left motor from the horizontal
        q2: the CCW angle of the right motor from the horizontal
    """
    # Length of the diagonal between the origin and (cX, cY)
    diag1 = np.sqrt(cX**2 + cY**2)
    # Calculating left motor angle
    q1 = np.arctan(cY/cX) + np.arccos(diag1/(2*arm_length))

    # Length of the diagonal between the center of the right motor and (cX, cY)
    diag2 = np.sqrt((displacement - cX)**2 + cY**2)
    # Calculating right motor angle
    q2 = np.pi - np.arctan(cY/(displacement - cX)) - np.arccos(diag2/(2*arm_length))
    
    return (q1, q2)

class PerspectiveCorrector:
    def __init__(self, width, height) -> None:
        # Load a predefined dictionary for ArUco marker detection
        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        # Create an instance of DetectorParameters for configuring ArUco detection
        self.parameters = cv.aruco.DetectorParameters()
        # Create an ArucoDetector object with the predefined dictionary and custom parameters
        self.detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)

        self.calibrated_transform = None
        self.width = width
        self.height = height

    def calibrate(self, frame):
        # Detect ArUco markers in the frame
        markerCorners, markerIds, _ = self.detector.detectMarkers(frame)

        # Check if any ArUco markers were detected
        if markerIds is not None:
            print(f"There are{len(markerCorners)}")
            detectedMarkers = list(zip(markerCorners, markerIds))
            # Draw the boundaries of the detected ArUco markers on the frame
            cv.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
            # Proceed if exactly four ArUco markers are detected
            if len(markerCorners) == 4:
                sorted_markers = list(
                    zip(*sorted(detectedMarkers, key=lambda marker: marker[1]))
                )[0]

                print(f"Sorted markers:\n{sorted_markers}")

                desired_corners = np.array(
                    [marker[0][0] for marker in sorted_markers]
                )  # Extracting the first corner of each marker

                print(
                    f"Desired corners (has shape {desired_corners.shape}):\n{desired_corners}"
                )

                # Define the coordinates of the corners of the paper in the output image
                output_pts = np.array(
                    [
                        [0, 0],
                        [table_width - 1, 0],
                        [table_width - 1, table_height - 1],
                        [0, table_height - 1],
                    ],
                    dtype="float32",
                )

                # Compute the perspective transform matrix to transform the perspective
                # of the captured frame to match the dimensions of the paper
                self.calibrated_transform = cv.getPerspectiveTransform(
                    desired_corners, output_pts
                )

    def correct_frame(self, frame):
        if self.calibrated_transform is not None:
            return cv.warpPerspective(
                frame, self.calibrated_transform, (self.width, self.height)
            )
        else:
            return None


class PuckDetector:
    def __init__(self) -> None:
        # Load a predefined dictionary for ArUco marker detection
        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        # Create an instance of DetectorParameters for configuring ArUco detection
        self.parameters = cv.aruco.DetectorParameters()
        # Create an ArucoDetector object with the predefined dictionary and custom parameters
        self.detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)

    def detect_puck(self, frame):
        markerCorners, markerIds, _ = self.detector.detectMarkers(frame)
        # print("detect puck called")
        # Check if any ArUco markers were detected
        center = None
        if markerIds is not None:
            # print("marker IDs present")
            detectedMarkers = list(zip(markerCorners, markerIds))
            # Draw the boundaries of the detected ArUco markers on the frame
            cv.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
            # print(detectedMarkers)
            # Search for the target marker
            for corners, id in detectedMarkers:
                if id == 4:
                    # print(f"Corners list for id 4:\n{corners}")
                    x_avg = np.mean([corner[0] for corner in corners[0]])
                    y_avg = np.mean([corner[1] for corner in corners[0]])
                    center = (x_avg, y_avg)
                    # print(f"calculated center: {center}")
        return (frame, center)


def main():
    # Initialize the video capture object to capture video from the default camera (camera 0)
    cap = cv.VideoCapture(0)
    cap.set(12, 2)
    corrector = PerspectiveCorrector(3925, 1875)
    detector = PuckDetector()

    arduino = serial.Serial(port="/dev/tty.usbmodem1101", baudrate=115200)
    # Initialize the number of frames
    num_frames = 0

    while True:
        # Capture a frame from the camera
        ret, frame = cap.read()
        # Check if the frame was successfully captured
        if not ret:
            print("Failed to grab frame")
            break  # Exit the loop if frame capture failed
        else:
            # Apply the perspective transformation to the captured frame
            corrected_frame = corrector.correct_frame(frame)
            if corrected_frame is not None:
                # # Display the result of the perspective transformation
                # # print("corrected frame is not none")
                # detect_result = detector.detect_puck(corrected_frame)
                # if detect_result is not None:
                #     detected_frame, center = detect_result
                #     # print("detect result is not none")
                #     if detected_frame is not None:
                #         # print("showing perspective corrected frame")

                #         cv.imshow("Perspective Transform", detected_frame)
                #         if center is not None:
                #             print(center)

                #             x_in = round((float(center[1]) / 100) - 9.375, 2)

                #             arduino.write(f"{x_in}\n".encode("utf-8"))
                #             print(f"{str(x_in)} written to serial port")
                # Convert the image to grayscale
                # Convert the image from BGR to HSV color space
                hsv = cv.cvtColor(corrected_frame, cv.COLOR_BGR2HSV)

                # Define the lower and upper bounds of the HSV values for thresholding
                # Adjust these values according to your object's color
                lower_hsv = (0, 0, 100)
                upper_hsv = (50, 255, 255)

                # Threshold the HSV image to get only the desired colors
                mask = cv.inRange(hsv, lower_hsv, upper_hsv)

                cv.imshow("mask", mask)

                # Erode to remove noise
                kernel = np.ones((5, 5), np.uint8)
                eroded = cv.erode(mask, kernel, iterations=1)

                # Dilate to enhance the features
                dilated = cv.dilate(eroded, kernel, iterations=1)

                # Find contours in the thresholded image
                contours, _ = cv.findContours(
                    dilated, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE
                )

                if contours:
                    # Find the largest contour in the frame and assume it is the object
                    largest_contour = max(contours, key=cv.contourArea)
                    cv.drawContours(
                        corrected_frame, [largest_contour], -1, (0, 255, 0), 3
                    )
                    M = cv.moments(largest_contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        center = (cX, cY)

                        # Draw the largest contour and center on the image
                        cv.drawContours(
                            corrected_frame, [largest_contour], -1, (0, 255, 0), 3
                        )
                        cv.circle(corrected_frame, center, 7, (255, 255, 255), -1)

                        # Show the image with the detected object
                        cv.imshow("Processed Frame", corrected_frame)
                        print(center)

                        # Calculate the x_in value based on the center coordinates
                        x_in = round((float(center[1]) / 100) - 9.375, 2)

                        # Send the x_in value to the Arduino
                        arduino.write(f"{x_in}\n".encode("utf-8"))
                        print(f"{str(x_in)} written to serial port")
                    else:
                        print("No contour detected.")

            num_frames = num_frames + 1

        # Display the original frame with the detected ArUco markers
        cv.imshow("Frame", frame)

        # Wait for a key press for 1 millisecond; exit if 'Esc' is pressed
        key = cv.waitKey(1)
        if key == 27:
            break
        if key == ord("c"):
            corrector.calibrate(frame)

    serial_thread.stop()
    # Release the video capture object to free resources
    cap.release()
    # Destroy all OpenCV-created windows to free resources
    cv.destroyAllWindows()


# Execute the main function when this script is run
if __name__ == "__main__":
    main()

import cv2 as cv
import numpy as np
import serial
import time
import math

# Constants
CALIB_FRAME = 10  # Number of frames grabbed
TABLE_WIDTH = 3925 + 150
TABLE_HEIGHT = 1875 + 150
ARM_LENGTH = 8  # arm legnth in inches
DISPLACEMENT = 5  # distance between motors in inches
SERIAL_DELAY = 0.01

WAITING_POSITION = 4.0
HITTING_POSITION = 9.0

ARDUINO_ENABLED = True  # disable arduino comms for debugging


def coordinateconverter(cX, cY, arm_length, displacement):
    """
    Note:
        The origin is defined to be at the axis of movement of the left motor.
        This function is designed to return the desired angles of the two
        motors on a five-bar parallel robot relative to the horizontal given
        the length of the arms (assumed to be of equal length) and the distance
        between the motors. They must be in the same length units.
    Args:
        cX: The x coordinate of the center of the striker
        cY: The y coordinate of the center of the striker
        length: The length of each of the four arms (inches)
        displacement: The distance between the motors (inches)
    Returns:
        q1: the radian CCW angle of the left motor from the horizontal
        q2: the radian CCW angle of the right motor from the horizontal
    """
    # Length of the diagonal between the origin and (cX, cY)
    diag1 = np.sqrt(cX**2 + cY**2)
    # Calculating left motor angle
    if cX == 0.0:
        cX = 0.001
    theta = np.arctan2(cY, cX) + np.arccos(diag1 / (2 * arm_length))

    # Length of the diagonal between the center of the right motor and (cX, cY)
    diag2 = np.sqrt((displacement - cX) ** 2 + cY**2)
    # Calculating right motor angle
    phi = (
        np.pi - np.arctan2(cY, displacement - cX) - np.arccos(diag2 / (2 * arm_length))
    )

    return (theta, phi)


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
            print(f"There are {len(markerCorners)}")
            detectedMarkers = list(zip(markerCorners, markerIds))
            # Draw the boundaries of the detected ArUco markers on the frame
            cv.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
            # Proceed if exactly four ArUco markers are detected
            if len(markerCorners) == 4:
                sorted_markers = list(
                    zip(*sorted(detectedMarkers, key=lambda marker: marker[1]))
                )[0]

                # print(f"Sorted markers:\n{sorted_markers}")

                desired_corners = np.array(
                    [marker[0][0] for marker in sorted_markers]
                )  # Extracting the first corner of each marker

                # print(
                #     f"Desired corners (has shape {desired_corners.shape}):\n{desired_corners}"
                # )
                # Define the coordinates of the corners of the table in the output image
                output_pts = np.array(
                    [
                        [0, 0],
                        [TABLE_WIDTH - 1, 0],
                        [TABLE_WIDTH - 1, TABLE_HEIGHT - 1],
                        [0, TABLE_HEIGHT - 1],
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
        timestamp = None
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
                    timestamp = time.perf_counter()
                    # print(f"calculated center: {center}")
        return (frame, center, timestamp)


def main():
    # Initialize the video capture object to capture video from the default camera (camera 0)
    cap = cv.VideoCapture(4)
    # cap.set(12, 2)
    corrector = PerspectiveCorrector(TABLE_WIDTH, TABLE_HEIGHT)
    detector = PuckDetector()
    if ARDUINO_ENABLED:
        arduino = serial.Serial(port="/dev/ttyACM0", baudrate=115200, write_timeout=0.1)
    # Initialize the number of frames
    num_frames = 0
    previous_center = None
    previous_timestamp = None

    while True:
        # Capture a frame from the camera
        ret, frame = cap.read()
        # Converting the image to grayscale and then to binary
        # frame = cv.threshold(cv.cvtColor(frame, cv.COLOR_BGR2GRAY), 127, 255, 0)

        # Check if the frame was successfully captured
        if not ret:
            print("Failed to grab frame")
            break  # Exit the loop if frame capture failed
        else:
            # Apply the perspective transformation to the captured frame
            corrected_frame = corrector.correct_frame(frame)
            if corrected_frame is not None:
                # Display the result of the perspective transformation
                # print("corrected frame is not none")
                detect_result = detector.detect_puck(corrected_frame)
                if detect_result is not None:
                    detected_frame, center, timestamp = detect_result
                    # print("detect result is not none")
                    if detected_frame is not None:
                        # print("showing perspective corrected frame")
                        resize = cv.resize(
                            detected_frame,
                            (int(TABLE_WIDTH / 4), int(TABLE_HEIGHT / 4)),
                        )

                        if center is not None:
                            center_float = tuple([float(x) for x in center])

                            if previous_center is not None:
                                # print(center)
                                # print(previous_center)
                                x1, y1 = previous_center
                                x2, y2 = center_float
                                time_elapsed = timestamp - previous_timestamp
                                # in inches? check exact conversion
                                speed = (
                                    math.dist([x1, y1], [x2, y2]) / 100
                                ) / time_elapsed
                                # print(f"{time_elapsed} seconds have passed")
                                # print(f"{distance_traveled} is distance traveled")
                                # print(
                                #    f"{distance_traveled/time_elapsed} inches per second"
                                # )

                                # Calculate the slope
                                # x1, x2, y1, y2 = slope_predictor(x1, x2, y1, y2)
                                if x2 != x1:
                                    m = (y2 - y1) / (x2 - x1)
                                    # Calculate the y-intercept
                                    b = y1 - m * x1
                                    # Calculate the end point of the line
                                    x3 = x2 + (x2 - x1)
                                    y3 = m * x3 + b
                                    y_int = m * (HITTING_POSITION * 100) + b
                                else:
                                    # This `is a special case where the line is vertical
                                    print("the line is vertical")
                                    b = None
                                    y_int = y2

                                y_int = min(1750, max(y_int, 135))
                                print(f"speed: {speed}")
                                if speed < 10:
                                    y_int = y2
                                # print("line drawn on frame")
                                # Draw the line on the image
                                # print(
                                #     f"line coords: ({int(x2)}, {int(y2)}), ({int(x3)}, {int(y3)})"
                                # )
                                resize = cv.arrowedLine(
                                    resize,
                                    (int(x2 / 4), int(y2 / 4)),
                                    (int(x3 / 4), int(y3 / 4)),
                                    (255, 0, 0),
                                    10,
                                )

                                # print(center)
                                (theta, phi) = coordinateconverter(
                                    # round((float(center[1]) / 100) - 6, 2),
                                    round(y_int / 100 - 6, 2),
                                    HITTING_POSITION,
                                    ARM_LENGTH,
                                    DISPLACEMENT,
                                )
                                print(
                                    f"go to position {round(y_int / 100 - 6, 2), HITTING_POSITION}"
                                )
                                resize = cv.circle(
                                    resize,
                                    (
                                        int(HITTING_POSITION * 100 / 4),
                                        int((y_int) / 4),
                                    ),
                                    25,
                                    (0, 0, 255),
                                    3,
                                )

                                if ARDUINO_ENABLED:
                                    arduino.write(
                                        f"{theta - (3.14 / 2)},{phi - (3.14 / 2)}\n".encode(
                                            "utf-8"
                                        )
                                    )
                                # print(
                                #     f"raw values: ({theta}, {phi}) written to serial: ({theta - (3.14 / 2)},{phi - (3.14 / 2)}) radians "
                                # )

                            previous_center = center_float
                            previous_timestamp = timestamp

                        cv.imshow("Perspective Transform", resize)
                        # x_in = round((float(center[1]) / 100) - 9.375, 2)
                        # arduino.write(f"{x_in}\n".encode())
                        # print(f"{str(x_in)} written to serial port")
                        # if (time.perf_counter() - timer) > SERIAL_DELAY:
                        #     try:
                        #         arduino.write(f"{x_in}\n".encode())
                        #     except serial.serialutil.SerialTimeoutException:
                        #         print("Serial timeout exception occured")
                        #     else:
                        #         print(f"{str(x_in)} written to serial port")
                        #     timer = time.perf_counter()

            num_frames = num_frames + 1

        # Display the original frame with the detected ArUco markers
        cv.imshow("Frame", frame)

        # Wait for a key press for 1 millisecond; exit if 'Esc' is pressed
        key = cv.waitKey(1)
        if key == 27:
            break
        if key == ord("c"):
            corrector.calibrate(frame)

    # Release the video capture object to free resources
    cap.release()
    # Destroy all OpenCV-created windows to free resources
    cv.destroyAllWindows()


# Execute the main function when this script is run
if __name__ == "__main__":
    main()

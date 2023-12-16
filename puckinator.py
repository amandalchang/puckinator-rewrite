import cv2 as cv
import numpy as np
import serial
import time
import math
from dataclasses import dataclass
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import imutils

# Constants
PIXELS_PER_INCH = 25  # number of pixels per physical inch on the air hockey table
TABLE_WIDTH = int((39.25 + 1.5) * PIXELS_PER_INCH)
TABLE_HEIGHT = int((18.75 + 1.5) * PIXELS_PER_INCH)
ARM_LENGTH = 8.0  # arm legnth in inches
DISPLACEMENT = 5.0  # distance between motors in inches
SPEED_THRESHOLD = 5.0  # inches per second


TOP_BOUNCE_Y = 44.25
BOTTOM_BOUNCE_Y = 457
WAITING_POSITION = 4.0
HITTING_POSITION = 7.0  # x-direction inches from the upper left corner
X_OFFSET = 4  # inches coordinate converter is offset from the upper left corner of the upper left ArUco
Y_OFFSET = 7.90  # inches coordinate converter offset from the upper left corner of the upper left ArUco
PORT = "/dev/ttyACM0"

ARDUINO_ENABLED = True  # disable arduino comms for debugging
DRAW_ENABLED = True  # disable arrow and circle drawing to fix latency
TRAJECTORY_PREDICT = True  # disable trajectory prediction to optimize latency
BOUNCE_PREDICT = True
PERF_COUNTER_ENABLED = False  # disable perf_counter for less print statements


@dataclass
class TimestampedPos:
    """
    This organizes position and time information for the puck. 
    x and y are defined in OpenCV pixel coordinates and timestamp is in seconds
    """

    x: float  # pixels
    y: float  # pixels
    timestamp: float

    def pos_as_tuple(self):
        return (self.x, self.y)


@dataclass
class TrajectoryPrediction:
    """
    This organizes the predicted point and the predicted time it will reach it.
    It also includes the prediction vector.

    x_int and y_int are in OpenCV pixel coordinates while vx and vy are in 
    inches/sec
    """

    x_int: float  # Inches
    y_int: float  # Inches
    vx: float  # pixels
    vy: float  # pixels
    int_time: float  # in seconds


def send_coordinates_to_arduino(desired_y, arduino, desired_x=HITTING_POSITION):
    """
    Sends the angles necessary to move the striker to a desired coordinate over 
    serial to the arduino

    Args: 
        desired_y: a float representing the desired y coordinate of the striker
        arduino: an instance of the serial.Serial class
    Returns:
        None
    """
    (theta, phi) = coordinateconverter(
        desired_x + X_OFFSET,
        round((desired_y / PIXELS_PER_INCH) - Y_OFFSET, 2),
        ARM_LENGTH,
        DISPLACEMENT,
    )
    print(f"desired y position: {desired_y}")
    print(
        f"go to position {desired_x + X_OFFSET, round((desired_y / PIXELS_PER_INCH) - Y_OFFSET, 2)}"
    )

    if ARDUINO_ENABLED:
        arduino.write(f"{theta - (3.14 / 2)},{phi - (3.14 / 2)}\n".encode("utf-8"))
    # print(
    #     f"raw values: ({theta}, {phi}) written to serial: ({theta - (3.14 / 2)},{phi - (3.14 / 2)}) radians "
    # )


def predict_trajectory(
    prev_pos: TimestampedPos, latest_pos: TimestampedPos, intersect_x=HITTING_POSITION
):
    """
    Predicts the trajectory of the puck based on previous and latest positions.

    This function calculates the speed of the puck based on the distance between
    the previous and latest positions and the time elapsed. It then calculates
    the y-intercept and the endpoint of the velocity vector. If the y-intercept
    is predicted to be beyond the edges of the table, it runs a bounce
    prediction algorithm until it is predicted to be within the table. If the 
    vector is vertical or the speed is lower than the speed threshold, the 
    y-intercept matches the y-position of the puck.

    Args:
        prev_pos (TimedstampedPos): The previous position of the puck.
        latest_pos (TimedstampedPos): The latest position of the puck.
        intersect_x (float, optional): The x position of the predicted spot
            for the puck. Defaults to HITTING_POSITION.

    Returns:
        TrajectoryPrediction: An instance of TrajectoryPrediction containing
            the predicted x and y values of the puck, the dx and dy values, and
            the intercept time. If the dy, dx, or intercept_time is None, the
            function returns None.
        puck_points: A list containing tuples of each predicted bounce point
    """
    time_elapsed = latest_pos.timestamp - prev_pos.timestamp
    dy = None
    dx = None
    b = None
    intercept_time = None

    speed = (
        math.dist(
            prev_pos.pos_as_tuple(),
            latest_pos.pos_as_tuple(),
        )
    ) / time_elapsed
    if latest_pos.x != prev_pos.x:
        dx = latest_pos.x - prev_pos.x
        dy = latest_pos.y - prev_pos.y
        # Calculate the y-intercept
        b = prev_pos.y - dy / dx * prev_pos.x
        # Calculate the end point of the line
        # x3 = latest_pos.x + (latest_pos.x - prev_pos.x)
        # y3 = m * x3 + b
        y_int = dy / dx * (intersect_x * PIXELS_PER_INCH) + b
        # print("different X values :)")
        intercept_time = (
            latest_pos.timestamp
            + math.dist(
                latest_pos.pos_as_tuple(), (intersect_x * PIXELS_PER_INCH, y_int)
            )
            / speed
        )
    else:
        # This is a special case where the line is vertical
        # print("the line is vertical")
        y_int = latest_pos.y

    # print(f"speed: {speed}")

    if (speed / PIXELS_PER_INCH) < SPEED_THRESHOLD or (latest_pos.x - prev_pos.x > 0):
        y_int = latest_pos.y

    if dy is None or dx is None or intercept_time is None or b is None:
        return None

    m = dy / dx
    latest_m = m
    latest_b = b
    i = 0

    puck_points: list[tuple] = [(int(latest_pos.x), int(latest_pos.y))]
    if (speed / PIXELS_PER_INCH) > SPEED_THRESHOLD and (latest_pos.x - prev_pos.x < 0):
        print(f"initial y int: {y_int}")
        if BOUNCE_PREDICT:
            while y_int > BOTTOM_BOUNCE_Y or y_int < TOP_BOUNCE_Y:
                i = i + 1
                print(f"Calculating bounce number {i}")
                if i > 2:
                    y_int = latest_pos.y
                    puck_points: list[tuple] = [(int(latest_pos.x), int(latest_pos.y))]
                    break
                (latest_m, latest_b), bounce_pos = predict_next_bounce(
                    latest_m, latest_b
                )
                print(f"Bounce at {bounce_pos}")
                puck_points.append(bounce_pos)
                print(f"Calculated Y intercept: {y_int}")
                y_int = latest_m * (intersect_x * PIXELS_PER_INCH) + latest_b
    y_int = min(BOTTOM_BOUNCE_Y, max(TOP_BOUNCE_Y, y_int))
    puck_points.append((int(intersect_x * PIXELS_PER_INCH), int(y_int)))

    return (
        TrajectoryPrediction(intersect_x, y_int, dx, dy, intercept_time),
        puck_points,
    )


def predict_next_bounce(m, b):
    """
    Predicts the next bounce point and velocity line based off the current 
    velocity line using the law of reflection; this is only called when the
    puck is heading towards the robot, eliminating the need for directionality
    on the velocity line

    Args:
        m: a float representing the slope of a velocity line
        b: a float representing the y-intercept of a velocity line

    Returns:
        (bounce_m, c): a tuple of floats containing the predicted slope and
            y-intercept of after a bounce
        (x1, y1): a tuple of ints containing the pixel coordinate of the
            predicted bounce.
    """
    if m == 0:
        m = 0.0001
    if m > 0:
        y1 = TOP_BOUNCE_Y
    else:
        y1 = BOTTOM_BOUNCE_Y

    x1 = (y1 - b) / m
    bounce_m = -1 * m
    c = y1 + (-bounce_m) * x1
    return ((bounce_m, c), (int(x1), int(y1)))


def coordinateconverter(cY, cX, arm_length, displacement):
    """
    This function returns the desired angles of the two
    motors on a five-bar parallel robot relative to the horizontal given
    the length of the arms (assumed to be of equal length) and the distance
    between the motors. They must be in the same length units.

    Note:
        The origin is defined to be at the axis of movement of the left motor.
        The x direction here is the y direction outside of this function.
    Args:
        cX: a float representing the x coordinate of the center of the striker 
            based on arm coordinates
        cY: a float representing he y coordinate of the center of the striker
        arm_length: The float length of each of the four arms (inches)
        displacement: The float distance between the motors (inches)
    Returns:
        theta: the float radian CCW angle of the left motor from the horizontal
        phi: the float radian CCW angle of the right motor from the horizontal
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
        """
        Initialize a PerspectiveCorrector instance.

        Args:
            width (int): Width of the corrected frame.
            height (int): Height of the corrected frame.

        Attributes:
        - dictionary: Predefined dictionary for ArUco marker detection.
        - parameters: Detector parameters for configuring ArUco detection.
        - detector: ArucoDetector object with the predefined dictionary and 
            custom parameters.
        - calibrated_transform (numpy.ndarray): Perspective transformation 
            matrix after calibration.
        - width (int): Width of the corrected frame.
        - height (int): Height of the corrected frame.
        """
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
        """
        Computes the necessary transform to crop and distort the frame based on
        the positions of the four ArUco markers. Stores the calibration 
        distortion matrix in self.calibrated_transform.

        Args:
            frame (numpy.ndarray): Input image frame.

        Returns:
            None
        """
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

                desired_corners = np.array(
                    [marker[0][0] for marker in sorted_markers]
                )  # Extracting the first corner of each marker
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
        """
        Correct the perspective of the input frame using a provided calibration
        matrix

        Args:
            frame (numpy.ndarray): Input image frame.

        Returns:
            numpy.ndarray or None: Corrected frame if calibration matrix is
            provided, otherwise None.
        """
        if self.calibrated_transform is not None:
            return cv.warpPerspective(
                frame, self.calibrated_transform, (self.width, self.height)
            )
        else:
            return None


class PuckDetector:
    def __init__(self) -> None:
        """
        Initializes a PuckDetector instance.

        Loads a predefined dictionary for ArUco marker detection and configures parameters.
        """
        # Load a predefined dictionary for ArUco marker detection
        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        # Create an instance of DetectorParameters for configuring ArUco detection
        self.parameters = cv.aruco.DetectorParameters()
        # self.parameters.maxMarkerPerimeterRate = 1
        self.parameters.polygonalApproxAccuracyRate = 0.1
        # Create an ArucoDetector object with the predefined dictionary and custom parameters
        self.detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)

    def detect_puck(self, frame, starttime):
        """
        Detects a puck based on the presence of an ID 4 ArUco marker in the
        provided frame

        Args:
            frame (numpy.ndarray): Input image frame.
            starttime: a float start time for performance measurement.

        Returns:
        tuple: A tuple containing the processed frame and a TimestampedPos 
        object representing the detected puck's position. Returns (frame, None)
        if no puck is detected.
        """
        if PERF_COUNTER_ENABLED:
            print(f"detect puck started at {time.perf_counter() - starttime}")
        markerCorners, markerIds, _ = self.detector.detectMarkers(frame)
        if PERF_COUNTER_ENABLED:
            print(f"detect marker finished at {time.perf_counter() - starttime}")
        # print("detect puck called")
        # Check if any ArUco markers were detected

        if markerIds is not None:
            # print("marker IDs present")
            detectedMarkers = list(zip(markerCorners, markerIds))
            # Draw the boundaries of the detected ArUco markers on the frame
            cv.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
            # print(detectedMarkers)
            # Search for the target marker
            for corners, marker_id in detectedMarkers:
                if marker_id == 4:
                    # print(f"Corners list for id 4:\n{corners}")
                    x_avg = float(np.mean([corner[0] for corner in corners[0]]))
                    y_avg = float(np.mean([corner[1] for corner in corners[0]]))
                    timestamp = time.perf_counter()
                    return (frame, TimestampedPos(x_avg, y_avg, timestamp))
        if PERF_COUNTER_ENABLED:
            print(f"detect puck finished at {time.perf_counter() - starttime}")
        return (frame, None)


def main():
    """
    Run the main program for puck detection and trajectory prediction.

    This function initializes the webcam stream, perspective corrector, puck 
    detector, and other necessary components. It continuously captures frames,
    applies perspective transformation, detects the puck, predicts its 
    trajectory, and displays the result with visualizations. If ARDUINO_ENABLED
    is True, it also sends puck coordinates to an Arduino.

    The program exits when the 'Esc' key is pressed.

    Key Controls:
    - 'Esc': Exit the program.
    - 'c': Calibrate the perspective corrector using the current frame.

    Note:
        Make sure to remove the striker before running the program to avoid
        triggering motor calibration while the two arms are physically connected
    Returns:
        None
    """
    # Initialize the video capture object to capture video from the default camera (camera 0)
    stream = WebcamVideoStream(src=4).start()
    fps = FPS().start()

    corrector = PerspectiveCorrector(TABLE_WIDTH, TABLE_HEIGHT)
    detector = PuckDetector()
    input("Take the striker off or else it will explode")
    if ARDUINO_ENABLED:
        arduino = serial.Serial(port=PORT, baudrate=115200, write_timeout=0.1)
    # Initialize the number of frames
    num_frames = 0
    previous_position = None
    frame = None
    last_trajectory = None

    while True:
        # Capture a frame from the camera
        if frame is None or not np.array_equal(frame, stream.read()):
            frame = stream.read()
            starttime = time.perf_counter()
            if PERF_COUNTER_ENABLED:
                print(f"frame read at {time.perf_counter()}")
            # Converting the image to grayscale and then to binary
            # frame = cv.threshold(cv.cvtColor(frame, cv.COLOR_BGR2GRAY), 127, 255, 0)

            # Apply the perspective transformation to the captured frame
            # resized_frame = imutils.resize(frame, width=640)
            corrected_frame = corrector.correct_frame(frame)

            if PERF_COUNTER_ENABLED:
                print(f"frame corrected at {time.perf_counter() - starttime}")
            if corrected_frame is not None:
                # Display the result of the perspective transformation
                detect_result = detector.detect_puck(corrected_frame, starttime)
                if detect_result is not None:
                    detected_frame, latest_position = detect_result
                    if detected_frame is not None:
                        if latest_position is not None:
                            print(
                                f"latest puck position: {latest_position.x, latest_position.y}"
                            )
                            if previous_position is not None:
                                if latest_position.x > TABLE_WIDTH / 3:
                                    result = predict_trajectory(
                                        previous_position,
                                        latest_position,
                                    )
                                    if result is not None:
                                        last_trajectory, puck_points = result
                                        if DRAW_ENABLED:
                                            print(puck_points)
                                            # Define the color, thickness and tipLength
                                            color = (255, 0, 0)  # Red color in BGR
                                            thickness = 2
                                            tipLength = 0.1

                                            # Draw arrowed lines between all pairs of points
                                            for i, start_point in enumerate(
                                                puck_points[:-1]
                                            ):
                                                end_point = puck_points[i + 1]
                                                corrected_frame = cv.arrowedLine(
                                                    detected_frame,
                                                    start_point,
                                                    end_point,
                                                    color,
                                                    thickness,
                                                    tipLength=tipLength,
                                                )
                                else:
                                    print("im on the other side")
                                    # calculate_offensive(
                                    #     last_trajectory
                                    # )  # Potentially check if most recent trajectory is recent enough

                                if ARDUINO_ENABLED and last_trajectory is not None:
                                    send_coordinates_to_arduino(
                                        last_trajectory.y_int, arduino
                                    )

                            if (
                                previous_position is None
                                or latest_position.x != previous_position.x
                            ):
                                previous_position = latest_position
                        if PERF_COUNTER_ENABLED:
                            print(f"frame show at {time.perf_counter() - starttime}")
                        if DRAW_ENABLED and last_trajectory is not None:
                            corrected_frame = cv.circle(
                                corrected_frame,
                                (
                                    int(HITTING_POSITION * PIXELS_PER_INCH),
                                    int(last_trajectory.y_int),
                                ),
                                25,
                                (0, 0, 255),
                                3,
                            )
                        cv.imshow("Perspective Transform", corrected_frame)

            num_frames += 1
            fps.update()

            # Display the original frame with the detected ArUco markers
            cv.imshow("Frame", frame)

        # Wait for a key press for 1 millisecond; exit if 'Esc' is pressed
        key = cv.waitKey(1)
        if key == 27:
            break
        if key == ord("c") and frame is not None:
            corrector.calibrate(frame)

    fps.stop()
    print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

    # Release the video capture object to free resources
    # cap.release()
    # Destroy all OpenCV-created windows to free resources
    cv.destroyAllWindows()


# Execute the main function when this script is run
if __name__ == "__main__":
    main()

import itertools
import cv2 as cv
import numpy as np

# Constants
CALIB_FRAME = 10  # Number of frames grabbed
table_width = 3925
table_height = 1875


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


def main():
    # Initialize the video capture object to capture video from the default camera (camera 0)
    cap = cv.VideoCapture(0)
    corrector = PerspectiveCorrector(3925, 1875)

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
                # Display the result of the perspective transformation
                cv.imshow("Perspective Transform", corrected_frame)
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

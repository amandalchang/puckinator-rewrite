import cv2 as cv
import numpy as np
import json


def load_calibration_data(json_file):
    with open(json_file, "r") as f:
        loaded_data = json.load(f)

    camera_matrix = np.array(loaded_data["camera_matrix"])
    distortion_coefficients = np.array(loaded_data["distortion_coefficients"])

    return camera_matrix, distortion_coefficients


def main():
    # Initialize the webcam
    cap = cv.VideoCapture(0)

    # Load camera calibration data
    camera_matrix, distortion_coefficients = load_calibration_data(
        "camera_calibration.json"
    )

    # Chessboard settings
    pattern_size = (9, 6)
    square_size = 1.0  # Set this to your actual square size

    # Prepare object points
    objp = np.zeros((np.prod(pattern_size), 3), dtype=np.float32)
    objp[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
    objp *= square_size

    frame_warp = None

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Convert to grayscale
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv.findChessboardCorners(gray, pattern_size)

        if ret:
            # Refine the corner positions
            corners_subpix = cv.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )

            # Draw the corners
            cv.drawChessboardCorners(frame, pattern_size, corners_subpix, ret)

            # Get the new camera matrix based on the free scaling parameter
            newcameramtx, roi = cv.getOptimalNewCameraMatrix(
                camera_matrix,
                distortion_coefficients,
                gray.shape[::-1],
                1,
                gray.shape[::-1],
            )

            # Compute the homography matrix
            H, _ = cv.findHomography(
                corners_subpix,
                cv.perspectiveTransform(objp[:, :2].reshape(-1, 1, 2), newcameramtx),
            )

            # Apply the perspective transformation
            h, w = frame.shape[:2]
            frame_warp = cv.warpPerspective(frame, H, (w, h))
        else:
            frame_warp = np.zeros_like(frame)

        # Combine original and warped images for display
        combined_frame = np.hstack((frame, frame_warp))
        cv.imshow("Original (left) vs Perspective Corrected (right)", combined_frame)

        key = cv.waitKey(50)
        if key == 27:  # ESC key to break
            break

    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()

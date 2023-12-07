import math
from Puckinator.puckinator import (
    coordinateconverter,
    HITTING_POSITION,
    ARM_LENGTH,
    DISPLACEMENT,
    ARDUINO_ENABLED,
)
import serial


def main():
    if ARDUINO_ENABLED:
        arduino = serial.Serial(port="/dev/ttyACM0", baudrate=115200, write_timeout=0.1)

    while True:
        y_int = float(input("y-coordinate: "))

        (theta, phi) = coordinateconverter(
            # round((float(center[1]) / 100) - 6, 2),
            y_int,
            9,
            ARM_LENGTH,
            DISPLACEMENT,
        )
        print(math.degrees(theta), math.degrees(phi))
        print(
            f"angles with 90 deg subtracted: {math.degrees(theta) - 90},{math.degrees(phi) - 90}"
        )

        if ARDUINO_ENABLED:
            arduino.write(f"{theta - (3.14 / 2)},{phi - (3.14 / 2)}\n".encode("utf-8"))


if __name__ == "__main__":
    main()

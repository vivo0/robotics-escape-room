# type: ignore

import argparse
import time

import cv2
import robomaster.camera
import robomaster.config  # noqa
import robomaster.conn
from robomaster import logger, logging, robot  # noqa

# IP = "127.0.0.1"
IP = ""
if IP:
    robomaster.config.LOCAL_IP_STR = IP
    robomaster.config.ROBOT_IP_STR = IP


def main():
    parser = argparse.ArgumentParser(description="Test the camera SDK")
    parser.add_argument(
        "--resolution", default=360, type=int, help="video resolution {360, 540, 720}"
    )
    parser.add_argument(
        "--log_level",
        default="WARN",
        type=str,
        help="log level {DEBUG, INFO, WARN, ERROR}",
    )
    parser.add_argument(
        "--frames",
        default=100,
        type=int,
        help="Number of frames to acquire and display. Set to negative for infinite",
    )

    args = parser.parse_args()
    logger.setLevel(args.log_level)
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta")
    time.sleep(1)

    print("Camera version", ep_robot.camera.get_version())
    print("Camera configuration", vars(ep_robot.camera.conf))
    print("Camera address", ep_robot.camera.video_stream_addr)
    print("Camera resolution", f"{args.resolution}p")
    ep_robot.camera.start_video_stream(display=False, resolution=f"{args.resolution}p")
    i = 0
    while True:
        try:
            frame = ep_robot.camera.read_cv2_image()
            if i == 0:
                print("Frames have shape", frame.shape)
            cv2.imshow("image", frame)
            cv2.waitKey(1)
        except:
            time.sleep(0.01)
        i += 1
        if args.frames > 0 and args.frames <= i:
            break

    ep_robot.camera.stop_video_stream()
    ep_robot.close()


if __name__ == "__main__":
    main()

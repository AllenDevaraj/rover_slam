#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2

W, H, FPS = 640, 480, 30  # tweak if you like


def main():
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)

    try:
        print("Starting pipeline…")
        pipe.start(cfg)
        while True:
            frames = pipe.wait_for_frames(5000)   # 5s timeout
            color = frames.get_color_frame()
            if not color:
                continue
            img = np.asanyarray(color.get_data())  # already BGR
            cv2.imshow("RealSense D435 - Color", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        print("Stopping pipeline.")
        pipe.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

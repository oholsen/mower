#!/usr/bin/env python3
import logging
import time

from edge_control.camera.driver import Camera


def main1():
    camera = Camera()
    while True:
        camera.start_stream()
        time.sleep(10)
        camera.stop_stream()
        time.sleep(10)


def main2():
    camera = Camera()
    camera.start_stream()
    i = 0
    while True:
        time.sleep(5)
        camera.snapshot(f"image{i}.jpeg")
        i += 1


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    main2()

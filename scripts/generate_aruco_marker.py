#!/usr/bin/env python3
"""
Generate ArUco marker image (DICT_4X4_50, id=0) and save to models directory.
Run once: python3 scripts/generate_aruco_marker.py
"""
import cv2
import numpy as np
import os

def main():
    out_dir = os.path.join(os.path.dirname(__file__), '..', 'models', 'aruco_marker', 'meshes')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'aruco_marker.png')

    aruco_dict  = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_size = 512       # pixels for the marker itself
    border_px   = 64        # white border

    # Generate marker (black squares on white)
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, 0, marker_size)

    # Add white border
    total = marker_size + 2 * border_px
    canvas = np.ones((total, total), dtype=np.uint8) * 255
    canvas[border_px:border_px + marker_size, border_px:border_px + marker_size] = marker_img

    # Convert to BGR (Gazebo OGRE2 expects 3-channel)
    canvas_bgr = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)

    cv2.imwrite(out_path, canvas_bgr)
    print(f'ArUco marker saved → {os.path.abspath(out_path)}')
    print(f'  dict : DICT_4X4_50   id : 0   size : {total}x{total}px')

if __name__ == '__main__':
    main()

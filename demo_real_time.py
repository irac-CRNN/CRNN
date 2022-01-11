#!/usr/bin/env python3

import sys
import time
sys.path.append("../..")
from HandTrackerRenderer import HandTrackerRenderer
from HandTrackerEdge import HandTracker
from Filters import LandmarksSmoothingFilter
import argparse
import numpy as np
import cv2
from o3d_utils_real_time import Visu3D

start_time = -2**100

LINES_HAND = [[0, 1], [1, 2], [2, 3], [3, 4],                   # 엄지
              [0, 5], [5, 6], [6, 7], [7, 8],                   # 검지
              [5, 9], [9, 10], [10, 11], [11, 12],              # 중지
              [9, 13], [13, 14], [14, 15], [15, 16],            # 약지
              [13, 17], [17, 18], [18, 19], [19, 20], [0, 17]]  # 새끼


class HandTracker3DRenderer:
    def __init__(self, tracker, mode_3d="image", smoothing=True):
        self.count = 0
        self.tracker = tracker
        self.mode_3d = mode_3d
        if self.mode_3d == "image":
            self.vis3d = Visu3D(bg_color=(255,255,255), zoom=0.7, segment_radius=10)
            z = min(tracker.img_h, tracker.img_w) / 3
            self.vis3d.create_grid([0, 0, z], [tracker.img_w, 0, z], [tracker.img_w, tracker.img_h, z],
                                   [0, tracker.img_h, z], 5, 2)  # Wall
            self.vis3d.init_view()

        self.smoothing = smoothing
        self.filter = None
        if self.smoothing:
            if tracker.solo:
                if self.mode_3d == "image":
                    self.filter = [LandmarksSmoothingFilter(min_cutoff=0.01, beta=40, derivate_cutoff=1)]
                else:
                    self.filter = [
                        LandmarksSmoothingFilter(min_cutoff=1, beta=20, derivate_cutoff=10, disable_value_scaling=True)]
            else:
                if self.mode_3d == "image":
                    self.filter = [
                        LandmarksSmoothingFilter(min_cutoff=0.01, beta=40, derivate_cutoff=1),
                        LandmarksSmoothingFilter(min_cutoff=0.01, beta=40, derivate_cutoff=1)
                    ]
                else:
                    self.filter = [
                        LandmarksSmoothingFilter(min_cutoff=1, beta=20, derivate_cutoff=10, disable_value_scaling=True),
                        LandmarksSmoothingFilter(min_cutoff=1, beta=20, derivate_cutoff=10, disable_value_scaling=True)
                    ]

        self.nb_hands_in_previous_frame = 0

    def draw_hand(self, hand, i):
        if self.mode_3d == "image":
            # Denormalize z-component of 'norm_landmarks'
            lm_z = (hand.norm_landmarks[:, 2:3] * hand.rect_w_a / 0.4).astype(np.int)
            # ... and concatenates with x and y components of 'landmarks'
            points = np.hstack((hand.landmarks, lm_z))
            radius = hand.rect_w_a / 30  # Thickness of segments depends on the hand size

        if self.smoothing:
            points = self.filter[i].apply(points, object_scale=hand.rect_w_a)

        for i, a_b in enumerate(LINES_HAND):
            a, b = a_b
            self.vis3d.add_segment(points[a], points[b], radius=radius,
                                   color=[1, 0, 0])  # if hand.handedness<0.5 else [0,1,0])

    def draw(self, hands):
        if self.smoothing and len(hands) != self.nb_hands_in_previous_frame:
            for f in self.filter: f.reset()
        self.vis3d.clear()
        self.vis3d.add_geometries()
        for i, hand in enumerate(hands):
            self.draw_hand(hand, i)
        self.vis3d.render()
        self.nb_hands_in_previous_frame = len(hands)
          
        self.vis3d.save_image() 


parser = argparse.ArgumentParser()
parser_tracker = parser.add_argument_group("Tracker arguments")
parser_tracker.add_argument('-i', '--input', type=str,
                            help="Path to video or image file to use as input (if not specified, use OAK color camera)")
parser_tracker.add_argument("--pd_model", type=str,
                            help="Path to a blob file for palm detection model")
parser_tracker.add_argument("--lm_model", type=str,
                            help="Path to a blob file for landmark model")
parser_tracker.add_argument('-s', '--solo', action="store_true",
                            help="Detect one hand max")
parser_tracker.add_argument('-f', '--internal_fps', type=int,
                            help="Fps of internal color camera. Too high value lower NN fps (default= depends on the model)")
parser_tracker.add_argument('--internal_frame_height', type=int,
                            help="Internal color camera frame height in pixels")
parser_tracker.add_argument('--single_hand_tolerance_thresh', type=int, default=0,
                            help="(Duo mode only) Number of frames after only one hand is detected before calling palm detection (default=%(default)s)")

parser_renderer3d = parser.add_argument_group("3D Renderer arguments")
parser_renderer3d.add_argument('-m', '--mode_3d', nargs='?',
                               choices=['image', 'world', 'raw_world', 'mixed'], const='image', default='image',
                               help="Specify the 3D coordinates used. See README for description (default=%(default)s)")
parser_renderer3d.add_argument('--no_smoothing', action="store_true",
                               help="Disable smoothing filter (smoothing works only in solo mode)")
args = parser.parse_args()

dargs = vars(args)
tracker_args = {a: dargs[a] for a in ['pd_model', 'lm_model', 'internal_fps', 'internal_frame_height'] if
                dargs[a] is not None}

tracker = HandTracker(
    input_src=args.input,
    use_world_landmarks=args.mode_3d != "image",
    solo=args.solo,
    xyz=args.mode_3d == "image",
    stats=True,
    single_hand_tolerance_thresh=args.single_hand_tolerance_thresh,
    lm_nb_threads=1,
    **tracker_args
)

renderer3d = HandTracker3DRenderer(tracker, mode_3d=args.mode_3d, smoothing=not args.no_smoothing)
renderer2d = HandTrackerRenderer(tracker)

pause = False
hands = []

while True:
    # Run hand tracker on next frame
    if not pause:
        frame, hands, bag = tracker.next_frame()
        if frame is None: break
        # Render 2d frame
        frame = renderer2d.draw(frame, hands, bag)
        cv2.imshow("Hand Tracker", frame)
    key = cv2.waitKey(1)
    # Draw hands on open3d canvas
    renderer3d.draw(hands)
    if key == 27 or key == ord('q'):
        break
    elif key == 32:  # space
        pause = not pause
    elif key == ord('w'):
                print("Snapshot saved in snapshot.jpg")
                cv2.imwrite("snapshot.jpg", frame)
               
renderer2d.exit()
tracker.exit()

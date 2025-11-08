#!/usr/bin/env python3
import argparse
import json
import shutil
import traceback
from argparse import ArgumentParser
from pathlib import Path
import time
from datetime import datetime, timedelta
from collections import deque
import traceback
import itertools
import cv2
import depthai as dai
import numpy as np
import depthai_calibration.calibration_utils as calibUtils

"""
Example:
python3 calibrate_depthai_v3.py -s 4.0 -brd OAK-FFC-3P.json -nx 12 -ny 9 --ssh-preview

No scaling
python3 calibrate_depthai_v3.py -s 4.0 -brd OAK-FFC-3P.json -nx 12 -ny 9

Calibrate on saved images from dataset folder:
python3 calibrate_depthai_v3.py -s 4.0 -brd OAK-FFC-3P.json -m process -nx 12 -ny 9
"""

font = cv2.FONT_HERSHEY_SIMPLEX
red = (255, 0, 0)
green = (0, 255, 0)


# =================================================================
# NEW: HELPER FUNCTION FOR CLAMPING VALUES
# =================================================================
def clamp(num, v0, v1):
    """Clamp a number between a minimum and maximum value."""
    return max(v0, min(num, v1))


stringToCam = {
    "RGB": dai.CameraBoardSocket.CAM_A,
    "LEFT": dai.CameraBoardSocket.CAM_B,
    "RIGHT": dai.CameraBoardSocket.CAM_C,
    "CAM_A": dai.CameraBoardSocket.CAM_A,
    "CAM_B": dai.CameraBoardSocket.CAM_B,
    "CAM_C": dai.CameraBoardSocket.CAM_C,
    "CAM_D": dai.CameraBoardSocket.CAM_D,
    "CAM_E": dai.CameraBoardSocket.CAM_E,
    "CAM_F": dai.CameraBoardSocket.CAM_F,
    "CAM_G": dai.CameraBoardSocket.CAM_G,
    "CAM_H": dai.CameraBoardSocket.CAM_H,
}


camToMonoRes = {
    "OV7251": dai.MonoCameraProperties.SensorResolution.THE_480_P,
    "OV9282": dai.MonoCameraProperties.SensorResolution.THE_800_P,
    "OV9782": dai.MonoCameraProperties.SensorResolution.THE_800_P,
}

camToRgbRes = {
    "IMX378": dai.ColorCameraProperties.SensorResolution.THE_4_K,
    "IMX214": dai.ColorCameraProperties.SensorResolution.THE_4_K,
    "IMX577": dai.ColorCameraProperties.SensorResolution.THE_4_K,
    "OV9782": dai.ColorCameraProperties.SensorResolution.THE_800_P,
    "IMX582": dai.ColorCameraProperties.SensorResolution.THE_12_MP,
    "AR0234": dai.ColorCameraProperties.SensorResolution.THE_1200_P,
}

antibandingOpts = {
    "off": dai.CameraControl.AntiBandingMode.OFF,
    "50": dai.CameraControl.AntiBandingMode.MAINS_50_HZ,
    "60": dai.CameraControl.AntiBandingMode.MAINS_60_HZ,
}


def create_blank(width, height, rgb_color=(0, 0, 0)):
    """Create new image(numpy array) filled with certain color in RGB"""
    # Create black blank image
    image = np.zeros((height, width, 3), np.uint8)

    # Since OpenCV uses BGR, convert the color first
    color = tuple(reversed(rgb_color))
    # Fill image with color
    image[:] = color

    return image


class ParseKwargs(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        setattr(namespace, self.dest, dict())
        for value in values:
            key, value = value.split("=")
            getattr(namespace, self.dest)[key] = value


def parse_args():
    epilog_text = """
    Captures and processes images for disparity depth calibration, generating a `<device id>.json` file or `depthai_calib.json`
    that should be loaded when initializing depthai. By default, captures one image for each of the 8 calibration target poses.

    Image capture requires the use of a printed OpenCV charuco calibration target applied to a flat surface(ex: sturdy cardboard).
    Default board size used in this script is 22x16. However you can send a customized one too.
    When taking photos, ensure enough amount of markers are visible and images are crisp. 
    The board does not need to fit within each drawn red polygon shape, but it should mimic the display of the polygon.

    If the calibration checkerboard corners cannot be found, the user will be prompted to try that calibration pose again.

    The script requires a RMS error < 1.0 to generate a calibration file. If RMS exceeds this threshold, an error is displayed.
    An average epipolar error of <1.5 is considered to be good, but not required. 

    Example usage:
    
    Stop ROS to access cameras:
    sudo systemctl stop rb_xavier_ros
    cd ~/hyper
    source venv/bin/activate

    Run this script for 2 x OV9782 and the IMX577 camera:
    python3 ssh_camera_filter_control_calibrate.py -s 4 --brd OAK-FFC-3P.json -nx 17 -ny 9 --ssh-preview
    python3 ssh_camera_filter_control_calibrate.py -s 4 --brd OAK-FFC-3P.json -nx 17 -ny 9

    Run calibration with a checkerboard square size of 4.0cm on board config file OAK-FFC-3P.json:
    python3 ssh_camera_filter_control_calibrate.py -s 4.0 -brd OAK-FFC-3P.json -nx 17 -ny 9

    Run with a fast, low-quality preview optimized for SSH connections:
    python3 ssh_camera_filter_control_calibrate.py -s 4.0 -brd OAK-FFC-3P.json -nx 17 -ny 9 --ssh-preview 

    Only run image processing only with same board setup. Requires a set of saved capture images:
    python3 ssh_camera_filter_control_calibrate.py -s 4.0 -brd OAK-FFC-3P.json -m process -nx 17 -ny 9
    
    Delete all existing images before starting image capture:
    python3 ssh_camera_filter_control_calibrate.py -s 4.0 -brd OAK-FFC-3P.json -m process -nx 17 -ny 9 -i delete

    Running with stereo camera only
    python3 ssh_camera_filter_control_calibrate.py -s 4.0 --brd OAK-FFC-3P.json -nx 17 -ny 9 -dsb rgb

    Running with RGB only 
    python3 ssh_camera_filter_control_calibrate.py -s 4.0 --brd OAK-FFC-3P-HQ83.json -nx 17 -ny 9 -dsb left right

    """
    parser = ArgumentParser(
        epilog=epilog_text, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "-c",
        "--count",
        default=3,
        type=int,
        required=False,
        help="Number of images per polygon to capture. Default: 1.",
    )
    parser.add_argument(
        "-s",
        "--squareSizeCm",
        type=float,
        required=True,
        help="Square size of calibration pattern used in centimeters. Default: 2.0cm.",
    )
    parser.add_argument(
        "-ms",
        "--markerSizeCm",
        type=float,
        required=False,
        help="Marker size in charuco boards.",
    )
    parser.add_argument(
        "-db",
        "--defaultBoard",
        default=None,
        type=str,
        help="Calculates the size of markers, numbers of squareX and squareY base on the choosing board from charuco_boards directory.",
    )
    parser.add_argument(
        "-nx",
        "--squaresX",
        default="11",
        type=int,
        required=False,
        help="number of chessboard squares in X direction in charuco boards.",
    )
    parser.add_argument(
        "-ny",
        "--squaresY",
        default="8",
        type=int,
        required=False,
        help="number of chessboard squares in Y direction in charuco boards.",
    )
    parser.add_argument(
        "-rd",
        "--rectifiedDisp",
        default=True,
        action="store_false",
        help="Display rectified images with lines drawn for epipolar check",
    )
    parser.add_argument(
        "-m",
        "--mode",
        default=["capture", "process"],
        nargs="*",
        type=str,
        required=False,
        help="Space-separated list of calibration options to run. By default, executes the full 'capture process' pipeline. To execute a single step, enter just that step (ex: 'process').",
    )
    parser.add_argument(
        "-brd",
        "--board",
        default=None,
        type=str,
        help="BW1097, BW1098OBC - Board type from resources/depthai_boards/boards (not case-sensitive). "
        "Or path to a custom .json board config. Mutually exclusive with [-fv -b -w]",
    )
    parser.add_argument(
        "-iv",
        "--invertVertical",
        dest="invert_v",
        default=False,
        action="store_true",
        help="Invert vertical axis of the camera for the display",
    )
    parser.add_argument(
        "-ih",
        "--invertHorizontal",
        dest="invert_h",
        default=False,
        action="store_true",
        help="Invert horizontal axis of the camera for the display",
    )
    parser.add_argument(
        "-ep",
        "--maxEpiploarError",
        default="0.8",
        type=float,
        required=False,
        help="Sets the maximum epiploar allowed with rectification. Default: %(default)s",
    )
    parser.add_argument(
        "-cm",
        "--cameraMode",
        default="perspective",
        type=str,
        required=False,
        help="Choose between perspective and Fisheye",
    )
    parser.add_argument(
        "-rlp",
        "--rgbLensPosition",
        nargs="*",
        action=ParseKwargs,
        required=False,
        default={},
        help="Set the manual lens position of the camera for calibration. Example -rlp rgb=135 night=135",
    )
    parser.add_argument(
        "-dsb",
        "--disableCamera",
        nargs="+",
        required=False,
        default=[],
        help="Set which camera should be disabled. Example -dsb rgb left right",
    )
    parser.add_argument(
        "-cd",
        "--captureDelay",
        default=2,
        type=int,
        required=False,
        help="Choose how much delay to add between pressing the key and capturing the image. Default: %(default)s",
    )
    parser.add_argument(
        "-fac",
        "--factoryCalibration",
        default=False,
        action="store_true",
        help="Enable writing to Factory Calibration.",
    )
    parser.add_argument(
        "-osf",
        "--outputScaleFactor",
        type=float,
        default=0.5,
        help="set the scaling factor for output visualization. Default: 0.5.",
    )
    parser.add_argument(
        "-fps",
        "--framerate",
        type=float,
        default=10,
        help="FPS to set for all cameras. Default: %(default)s",
    )
    parser.add_argument(
        "-ab",
        "--antibanding",
        default="50",
        choices={"off", "50", "60"},
        help="Set antibanding/antiflicker algo for lights that flicker at mains frequency. Default: %(default)s [Hz]",
    )
    parser.add_argument(
        "-scp",
        "--saveCalibPath",
        type=str,
        default="",
        help="Save calibration file to this path",
    )
    parser.add_argument(
        "-dst",
        "--datasetPath",
        type=str,
        default="dataset",
        help="Path to dataset used for processing images",
    )
    parser.add_argument(
        "-mdmp",
        "--minDetectedMarkersPercent",
        type=float,
        default=0.4,
        help="Minimum percentage of detected markers to consider a frame valid",
    )
    parser.add_argument(
        "-mt",
        "--mouseTrigger",
        default=False,
        action="store_true",
        help="Enable mouse trigger for image capture",
    )
    parser.add_argument(
        "-nic",
        "--noInitCalibration",
        default=False,
        action="store_true",
        help="Don't take the board calibration for initialization but start with an empty one",
    )
    parser.add_argument(
        "-trc",
        "--traceLevel",
        type=int,
        default=0,
        help="Set to trace the steps in calibration. Number from 1 to 5. If you want to display all, set trace number to 10.",
    )
    parser.add_argument(
        "-mst",
        "--minSyncTimestamp",
        type=float,
        default=0.2,
        help="Minimum time difference between pictures taken from different cameras.  Default: %(default)s ",
    )
    parser.add_argument(
        "-it",
        "--numPictures",
        type=float,
        default=None,
        help="Number of pictures taken.",
    )
    parser.add_argument(
        "-ebp",
        "--enablePolygonsDisplay",
        default=False,
        action="store_true",
        help="Enable the display of polynoms.",
    )
    parser.add_argument(
        "-dbg",
        "--debugProcessingMode",
        default=False,
        action="store_true",
        help="Enable processing of images without using the camera.",
    )
    parser.add_argument(
        "-v3",
        "--useDepthaiV3",
        default=True,
        action="store_true",
        help="Use depthai v3 (default: True).",
    )
    parser.add_argument(
        "--ssh-preview",
        action="store_true",
        help="Enable a fast, low-quality preview optimized for SSH, showing only marker counts.",
    )
    # =================================================================
    options = parser.parse_args()
    # Set some extra defaults, `-brd` would override them
    if options.defaultBoard is not None:
        try:
            board_name = options.defaultBoard
            try:
                board_name, _ = board_name.split(".")
            except:
                board_name = board_name
            _, size, charcuo_num = board_name.split("_")
            numX, numY = charcuo_num.split("x")
            options.squaresX = int(numX)
            options.squaresY = int(numY)
        except:
            raise argparse.ArgumentTypeError(
                options.defaultBoard, "Board name has not been found."
            )
    if options.markerSizeCm is None:
        options.markerSizeCm = options.squareSizeCm * 0.75
    if options.squareSizeCm < 2.2:
        raise argparse.ArgumentTypeError(
            "-s / --squareSizeCm needs to be greater than 2.2 cm"
        )
    if options.traceLevel == 1:
        print(
            f"Charuco board selected is: board_name = {board_name}, numX = {numX}, numY = {numY}, squareSize {options.squareSizeCm} cm, markerSize {options.markerSizeCm} cm"
        )
    if options.debugProcessingMode:
        options.mode = "process"
        if options.board is None:
            raise argparse.ArgumentError(
                options.board,
                "Board name (-brd) of camera must be specified in case of using debug mode (-dbg).",
            )
    return options


class HostSync:
    def __init__(self, deltaMilliSec):
        self.arrays = {}
        self.arraySize = 15
        self.recentFrameTs = None
        self.deltaMilliSec = timedelta(milliseconds=deltaMilliSec)
        # self.synced = queue.Queue()

    def remove(self, t1):
        return timedelta(milliseconds=500) < (self.recentFrameTs - t1)

    def add_msg(self, name, data, ts):
        if name not in self.arrays:
            self.arrays[name] = deque(maxlen=self.arraySize)
        # Add msg to array
        self.arrays[name].appendleft({"data": data, "timestamp": ts})
        if self.recentFrameTs == None or self.recentFrameTs - ts:
            self.recentFrameTs = ts

    def clearQueues(self):
        print("Clearing Queues...")
        for name, msgList in self.arrays.items():
            self.arrays[name].clear()
            print(len(self.arrays[name]))

    def get_synced(self):
        synced = {}
        for name, msgList in self.arrays.items():
            if len(msgList) != self.arraySize:
                return False

        for name, pivotMsgList in self.arrays.items():
            print("len(pivotMsgList)")
            print(len(pivotMsgList))
            pivotMsgListDuplicate = pivotMsgList
            while pivotMsgListDuplicate:
                currPivot = pivotMsgListDuplicate.popleft()
                synced[name] = currPivot["data"]

                for subName, msgList in self.arrays.items():
                    print(f"len of {subName}")
                    print(len(msgList))
                    if name == subName:
                        continue
                    msgListDuplicate = msgList.copy()
                    while msgListDuplicate:
                        print(f"---len of dup {subName} is {len(msgListDuplicate)}")
                        currMsg = msgListDuplicate.popleft()
                        time_diff = abs(currMsg["timestamp"] - currPivot["timestamp"])
                        print(
                            f"---Time diff is {time_diff} and delta is {self.deltaMilliSec}"
                        )
                        if time_diff < self.deltaMilliSec:
                            print(
                                f"--------Adding {subName} to sync. Messages left is {len(msgListDuplicate)}"
                            )
                            synced[subName] = currMsg["data"]
                            break
                    print(
                        f"Size of Synced is {len(synced)} amd array size is {len(self.arrays)}"
                    )
                    if len(synced) == len(self.arrays):
                        self.clearQueues()
                        return synced

            # raise SystemExit(1)
            self.clearQueues()
            return False


class MessageSync:
    def __init__(
        self, num_queues, min_diff_timestamp, max_num_messages=4, min_queue_depth=3
    ):
        self.num_queues = num_queues
        self.min_diff_timestamp = min_diff_timestamp
        self.max_num_messages = max_num_messages
        # self.queues = [deque() for _ in range(num_queues)]
        self.queues = dict()
        self.queue_depth = min_queue_depth
        # self.earliest_ts = {}

    def add_msg(self, name, msg):
        if name not in self.queues:
            self.queues[name] = deque(maxlen=self.max_num_messages)
        self.queues[name].append(msg)
        # if msg.getTimestampDevice() < self.earliest_ts:
        #     self.earliest_ts = {name: msg.getTimestampDevice()}

        # print('Queues: ', end='')
        # for name in self.queues.keys():
        #     print('\t: ', name, end='')
        #     print(self.queues[name], end=', ')
        #     print()
        # print()

    def get_synced(self):

        # Atleast 3 messages should be buffered
        min_len = min([len(queue) for queue in self.queues.values()])
        if min_len == 0:
            if self.traceLevel > 0:
                print("Status:", "exited due to min len == 0", self.queues)
            return None

        # initializing list of list
        queue_lengths = []
        for name in self.queues.keys():
            queue_lengths.append(range(0, len(self.queues[name])))
        permutations = list(itertools.product(*queue_lengths))
        # print ("All possible permutations are : " +  str(permutations))

        # Return a best combination after being atleast 3 messages deep for all queues
        min_ts_diff = None
        for indicies in permutations:
            tmp = {}
            i = 0
            for n in self.queues.keys():
                tmp[n] = indicies[i]
                i = i + 1
            indicies = tmp

            acc_diff = 0.0
            min_ts = None
            for name in indicies.keys():
                msg = self.queues[name][indicies[name]]
                if min_ts is None:
                    min_ts = msg.getTimestampDevice().total_seconds()
            for name in indicies.keys():
                msg = self.queues[name][indicies[name]]
                acc_diff = acc_diff + abs(
                    min_ts - msg.getTimestampDevice().total_seconds()
                )

            # Mark minimum
            if min_ts_diff is None or (
                acc_diff < min_ts_diff["ts"]
                and abs(acc_diff - min_ts_diff["ts"]) > 0.03
            ):
                min_ts_diff = {"ts": acc_diff, "indicies": indicies.copy()}
                if self.traceLevel > 0:
                    print(
                        "new minimum:",
                        min_ts_diff,
                        "min required:",
                        self.min_diff_timestamp,
                    )

            if min_ts_diff["ts"] < self.min_diff_timestamp:
                # Check if atleast 5 messages deep
                min_queue_depth = None
                for name in indicies.keys():
                    if min_queue_depth is None or indicies[name] < min_queue_depth:
                        min_queue_depth = indicies[name]
                if min_queue_depth >= self.queue_depth:
                    # Retrieve and pop the others
                    synced = {}
                    for name in indicies.keys():
                        synced[name] = self.queues[name][min_ts_diff["indicies"][name]]
                        # pop out the older messages
                        for i in range(0, min_ts_diff["indicies"][name] + 1):
                            self.queues[name].popleft()
                    if self.traceLevel == 1:
                        print(
                            "Returning synced messages with error:",
                            min_ts_diff["ts"],
                            min_ts_diff["indicies"],
                        )
                    return synced


class Main:
    # =================================================================
    # --- USER-CONFIGURABLE DEFAULTS ---
    # These values will be used when the script starts.
    # They can be changed interactively during runtime.
    # =================================================================
    # Image Quality
    DEFAULT_SHARPNESS = 1  # Range 0-4
    DEFAULT_LUMA_DENOISE = 3  # Range 0-4
    DEFAULT_CHROMA_DENOISE = 2  # Range 0-4

    # Exposure Settings
    DEFAULT_MANUAL_EXPOSURE = False  # True for Manual, False for Auto
    DEFAULT_AE_LIMIT_US = 10000  # Auto Exposure limit in microseconds (e.g., 10000)

    # Defaults for Manual Mode (if enabled by toggling)
    DEFAULT_MANUAL_ISO = 800  # Range 100-1600
    DEFAULT_MANUAL_EXPOSURE_TIME_US = 10000
    # =================================================================

    output_scale_factor = 0.5
    polygons = None
    width = None
    height = None
    current_polygon = 0
    images_captured_polygon = 0
    images_captured = 0

    def __init__(self):
        self.args = parse_args()

        # Apply the user-configurable defaults to the instance variables
        self.sharpness = self.DEFAULT_SHARPNESS
        self.luma_denoise = self.DEFAULT_LUMA_DENOISE
        self.chroma_denoise = self.DEFAULT_CHROMA_DENOISE
        self.manual_exposure = self.DEFAULT_MANUAL_EXPOSURE
        self.ae_limit_us = self.DEFAULT_AE_LIMIT_US
        self.manual_iso = self.DEFAULT_MANUAL_ISO
        self.manual_exposure_time_us = self.DEFAULT_MANUAL_EXPOSURE_TIME_US

        # Override with a calculated value if framerate is set, otherwise use default
        if self.args.framerate > 0:
            max_ae_for_fps = int(1_000_000 / self.args.framerate)
            self.ae_limit_us = min(self.ae_limit_us, max_ae_for_fps)

        self.traceLevel = self.args.traceLevel
        self.output_scale_factor = self.args.outputScaleFactor

        # Updated ArUco initialization for modern OpenCV
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_1000
        )
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary)

        self.enablePolygonsDisplay = self.args.enablePolygonsDisplay
        self.board_name = None
        self.device = None  # Will be initialized in run() method

        if self.args.board:
            self.board_name = self.args.board
            board_path = Path(self.args.board)
            if not board_path.exists():
                board_path = (
                    (
                        Path(__file__).parent
                        / "resources/depthai_boards/boards"
                        / self.args.board.upper()
                    )
                    .with_suffix(".json")
                    .resolve()
                )
                if not board_path.exists():
                    raise ValueError("Board config not found: {}".format(board_path))
            with open(board_path) as fp:
                self.board_config = json.load(fp)["board_config"]

        # Initialize self.polygons here to prevent TypeError
        self.polygons = calibUtils.setPolygonCoordinates(1000, 600)
        if self.args.numPictures:
            self.total_images = self.args.numPictures
        else:
            self.total_images = self.args.count * len(self.polygons)

        if self.traceLevel == 1:
            print("Using Arguments=", self.args)
        if self.args.datasetPath:
            Path(self.args.datasetPath).mkdir(parents=True, exist_ok=True)

        self.coverageImages = {}
        if self.board_config:
            for cam_id in self.board_config["cameras"]:
                name = self.board_config["cameras"][cam_id]["name"]
                self.coverageImages[name] = None

        # Updated CharucoBoard creation for modern OpenCV
        self.charuco_board = cv2.aruco.CharucoBoard(
            (self.args.squaresX, self.args.squaresY),
            self.args.squareSizeCm,
            self.args.markerSizeCm,
            self.aruco_dictionary,
        )

        # Create a CharucoDetector for board detection
        self.charuco_detector = cv2.aruco.CharucoDetector(self.charuco_board)

    def mouse_event_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.mouseTrigger = True

    def startPipeline(self):
        """
        Create pipeline, start it, and create output queues.
        Must be called with an open device.
        Uses V3 API by default.
        """
        if self.device is None or self.device.isClosed():
            raise RuntimeError("Device must be open before starting pipeline")

        # Initialize storage for outputs
        self.camera_outputs = {}
        self.control_queues = {}

        # V3 API path (default)
        # Create the pipeline structure
        pipeline = self.create_pipeline_v3()

        # Create output queues from the stored outputs
        self.camera_queue = {}
        for name, output in self.camera_outputs.items():
            self.camera_queue[name] = output.createOutputQueue(
                maxSize=1, blocking=False
            )

        # Start the pipeline
        pipeline.start()

        self.pipeline = pipeline

    def is_markers_found(self, frame):
        """Check if sufficient charuco corners are detected for calibration"""
        _, _, charuco_corners, charuco_ids = self.detect_markers_corners(frame)

        # Total number of charuco corners is (squaresX-1) * (squaresY-1)
        total_charuco_corners = (self.args.squaresX - 1) * (self.args.squaresY - 1)
        min_required_corners = int(
            total_charuco_corners * self.args.minDetectedMarkersPercent
        )

        detected_corners = len(charuco_corners) if charuco_corners is not None else 0

        if self.traceLevel > 0:
            print(
                f"Detected {detected_corners}/{total_charuco_corners} charuco corners, need {min_required_corners}"
            )

        return detected_corners >= min_required_corners

    # Updated function for modern OpenCV - single source of truth for detection
    def detect_markers_corners(self, frame):
        """Detect ArUco markers and interpolate charuco corners using modern OpenCV API"""
        # Use CharucoDetector to find everything in one go
        charuco_corners, charuco_ids, marker_corners, marker_ids = (
            self.charuco_detector.detectBoard(frame)
        )

        # The old function returned marker IDs as a column vector. Reshape to maintain compatibility.
        if marker_ids is None:
            marker_ids_col = None
        else:
            marker_ids_col = marker_ids.reshape(-1, 1)

        return marker_corners, marker_ids_col, charuco_corners, charuco_ids

    def draw_markers(self, frame, charuco_corners=None, charuco_ids=None):
        """Draw detected charuco corners directly on the frame using modern OpenCV API"""
        # If corners not provided, detect them
        if charuco_corners is None or charuco_ids is None:
            charuco_corners, charuco_ids, _, _ = self.charuco_detector.detectBoard(frame)

        # Draw the detected corners
        if charuco_ids is not None and len(charuco_ids) > 0:
            return cv2.aruco.drawDetectedCornersCharuco(
                frame, charuco_corners, charuco_ids, green
            )
        return frame

    def draw_corners(self, frame, displayframe, color):
        _, _, charuco_corners, _ = self.detect_markers_corners(frame)
        if charuco_corners is not None:
            for corner in charuco_corners:
                corner_int = (int(corner[0][0]), int(corner[0][1]))
                cv2.circle(
                    displayframe,
                    corner_int,
                    4 * displayframe.shape[1] // 1900,
                    color,
                    -1,
                )
        return displayframe

    def test_camera_orientation(self, frame_l, frame_r):
        marker_corners_l, id_l, _, _ = self.detect_markers_corners(frame_l)
        marker_corners_r, id_r, _, _ = self.detect_markers_corners(frame_r)

        if id_l is None or id_r is None:
            return True

        for i, left_id in enumerate(id_l):
            idx = np.where(id_r == left_id)
            if idx[0].size == 0:
                continue
            for left_corner, right_corner in zip(
                marker_corners_l[i], marker_corners_r[idx[0][0]]
            ):
                if left_corner[0][0] - right_corner[0][0] < 0:
                    return False
        return True

    def create_pipeline(self):
        pipeline = dai.Pipeline()
        fps = self.args.framerate

        for cam_id in self.board_config["cameras"]:
            cam_info = self.board_config["cameras"][cam_id]
            if cam_info["name"] in self.args.disableCamera:
                continue

            cam_node = None
            if cam_info["type"] == "mono":
                cam_node = pipeline.createMonoCamera()
                sensorName = cam_info.get("sensorName", "OV9282")
                cam_node.setBoardSocket(stringToCam[cam_id])
                if sensorName in camToMonoRes:
                    cam_node.setResolution(camToMonoRes[sensorName])
                else:
                    print(
                        f"Warning: Mono sensor '{sensorName}' not in resolution map. Using 800P default."
                    )
                    cam_node.setResolution(
                        dai.MonoCameraProperties.SensorResolution.THE_800_P
                    )
                cam_node.setFps(fps)
                xout = pipeline.createXLinkOut()
                xout.setStreamName(cam_info["name"])
                cam_node.out.link(xout.input)
                cam_node.initialControl.setSharpness(self.sharpness)

            else:  # Assumes 'color' type
                cam_node = pipeline.createColorCamera()
                sensorName = cam_info.get("sensorName", "IMX577")
                cam_node.setBoardSocket(stringToCam[cam_id])
                if sensorName.upper() in camToRgbRes:
                    cam_node.setResolution(camToRgbRes[sensorName.upper()])
                else:
                    print(
                        f"Warning: Color sensor '{sensorName}' not in resolution map. Using 4K default."
                    )
                    cam_node.setResolution(
                        dai.ColorCameraProperties.SensorResolution.THE_4_K
                    )
                cam_node.setFps(fps)

                xout = pipeline.createXLinkOut()
                xout.setStreamName(cam_info["name"])

                # Handle different ISP outputs
                if (
                    cam_info.get("sensorName") == "OV9782"
                    or cam_info.get("model") == "OV9782"
                ):
                    cam_node.out.link(xout.input)  # Use raw bayer output
                else:
                    cam_node.isp.link(xout.input)  # Use ISP processed output

                cam_node.initialControl.setSharpness(self.sharpness)
                cam_node.initialControl.setLumaDenoise(self.luma_denoise)
                cam_node.initialControl.setChromaDenoise(self.chroma_denoise)

                if self.manual_exposure:
                    cam_node.initialControl.setManualExposure(
                        self.manual_exposure_time_us, self.manual_iso
                    )
                else:
                    cam_node.initialControl.setAutoExposureEnable()
                    if self.ae_limit_us:
                        max_exp = int(1_000_000 / fps) if fps > 0 else 33333
                        ae_limit = clamp(self.ae_limit_us, 106, max_exp)
                        cam_node.initialControl.setAutoExposureLimit(ae_limit)

                if cam_info.get("hasAutofocus", False):
                    lens_pos = 135
                    if cam_info["name"] in self.args.rgbLensPosition:
                        lens_pos = int(self.args.rgbLensPosition[cam_info["name"]])
                    elif stringToCam[cam_id].name.lower() in self.args.rgbLensPosition:
                        lens_pos = int(
                            self.args.rgbLensPosition[stringToCam[cam_id].name.lower()]
                        )
                    print(f"Setting manual focus for {cam_info['name']} to {lens_pos}")
                    cam_node.initialControl.setManualFocus(lens_pos)

            controlIn = pipeline.createXLinkIn()
            controlIn.setStreamName(f"{cam_info['name']}-control")
            controlIn.out.link(cam_node.inputControl)

            cam_node.initialControl.setAntiBandingMode(
                antibandingOpts[self.args.antibanding]
            )
            xout.input.setBlocking(False)
            xout.input.setQueueSize(1)

        return pipeline

    def create_pipeline_v3(self):
        """
        Create the DepthAI V3 pipeline with camera nodes and control inputs.
        Returns the pipeline object WITHOUT starting it.

        Returns:
            dai.Pipeline: The configured pipeline ready to start
        """
        pipeline = dai.Pipeline(self.device)
        fps = self.args.framerate

        # Add resolution mapping for V3 API
        resolutionToSize = {
            dai.MonoCameraProperties.SensorResolution.THE_400_P: (640, 400),
            dai.MonoCameraProperties.SensorResolution.THE_480_P: (640, 480),
            dai.MonoCameraProperties.SensorResolution.THE_720_P: (1280, 720),
            dai.MonoCameraProperties.SensorResolution.THE_800_P: (1280, 800),
            dai.ColorCameraProperties.SensorResolution.THE_720_P: (1280, 720),
            dai.ColorCameraProperties.SensorResolution.THE_800_P: (1280, 800),
            dai.ColorCameraProperties.SensorResolution.THE_1080_P: (1920, 1080),
            dai.ColorCameraProperties.SensorResolution.THE_1200_P: (1920, 1200),
            dai.ColorCameraProperties.SensorResolution.THE_4_K: (3840, 2160),
            dai.ColorCameraProperties.SensorResolution.THE_5_MP: (2592, 1944),
            dai.ColorCameraProperties.SensorResolution.THE_12_MP: (4056, 3040),
            dai.ColorCameraProperties.SensorResolution.THE_13_MP: (4208, 3120),
        }

        for cam_id in self.board_config["cameras"]:
            cam_info = self.board_config["cameras"][cam_id]
            if cam_info["name"] in self.args.disableCamera:
                continue

            socket = stringToCam[cam_id]

            if cam_info["type"] == "mono":
                # Build mono camera with FPS in build() call
                cam = pipeline.create(dai.node.Camera).build(socket, sensorFps=fps)
                resolution = camToMonoRes.get(cam_info.get("sensorName", "OV9282"))
                if resolution:
                    output_size = resolutionToSize[resolution]
                    cam_output = cam.requestOutput(
                        output_size, type=dai.ImgFrame.Type.GRAY8
                    )
                else:
                    cam_output = cam.requestFullResolutionOutput(
                        type=dai.ImgFrame.Type.GRAY8
                    )

                # Set initial control for mono cameras
                cam.initialControl.setSharpness(self.sharpness)

            elif cam_info["type"] == "color":
                # Build color camera - FPS is set in build() or requestOutput()
                cam = pipeline.create(dai.node.Camera).build(socket)

                # Set manual focus for cameras with autofocus capability
                if cam_info.get("hasAutofocus"):
                    lens_pos = int(self.args.rgbLensPosition.get(cam_info["name"], 135))
                    cam.initialControl.setManualFocus(lens_pos)

                # Set initial image quality controls
                cam.initialControl.setSharpness(self.sharpness)
                cam.initialControl.setLumaDenoise(self.luma_denoise)
                cam.initialControl.setChromaDenoise(self.chroma_denoise)

                # Set exposure controls
                if self.manual_exposure:
                    cam.initialControl.setManualExposure(
                        self.manual_exposure_time_us, self.manual_iso
                    )
                else:
                    cam.initialControl.setAutoExposureEnable()
                    if self.ae_limit_us:
                        max_exp = int(1_000_000 / fps) if fps > 0 else 33333
                        ae_limit = clamp(self.ae_limit_us, 106, max_exp)
                        cam.initialControl.setAutoExposureLimit(ae_limit)

                resolution = camToRgbRes.get(cam_info.get("sensorName", "IMX577"))
                if resolution:
                    output_size = resolutionToSize[resolution]
                    # FPS set in requestOutput call
                    cam_output = cam.requestOutput(
                        output_size, type=dai.ImgFrame.Type.BGR888i, fps=fps
                    )
                else:
                    cam_output = cam.requestFullResolutionOutput(
                        type=dai.ImgFrame.Type.BGR888i
                    )
            else:
                raise ValueError(
                    f"Unknown camera type: {cam_info['type']} for camera {cam_info['name']}"
                )

            # Set antibanding mode
            cam.initialControl.setAntiBandingMode(
                antibandingOpts[self.args.antibanding]
            )

            # Store the output with the camera name for later queue creation
            self.camera_outputs[cam_info["name"]] = cam_output

            # Create control input for runtime control
            controlIn = cam.inputControl.createInputQueue()
            self.control_queues[cam_info["name"]] = controlIn

        return pipeline

    def parse_frame(self, frame, stream_name):
        """Parse frame and save if sufficient corners detected"""
        _, _, charuco_corners, charuco_ids = self.detect_markers_corners(frame)

        # Total number of corners on the board is (X-1) * (Y-1)
        total_charuco_corners = (self.args.squaresX - 1) * (self.args.squaresY - 1)
        min_corners_needed = int(
            total_charuco_corners * self.args.minDetectedMarkersPercent
        )

        detected_corners = len(charuco_corners) if charuco_corners is not None else 0

        if detected_corners < min_corners_needed:
            if self.traceLevel > 0:
                print(
                    f"Insufficient corners: {detected_corners}/{total_charuco_corners} (need {min_corners_needed})"
                )
            return False

        filename = calibUtils.image_filename(self.current_polygon, self.images_captured)
        path = Path(self.args.datasetPath) / stream_name / filename
        path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(path), frame)
        print(f"py: Saved image as: {str(path)} with {detected_corners} corners")
        return True

    def debug_charuco_detection(self, frame):
        """Debug function to understand what's being detected"""
        marker_corners, ids, charuco_corners, charuco_ids = self.detect_markers_corners(
            frame
        )
        if self.traceLevel > 0:
            print(
                f"Board configuration: {self.args.squaresX}x{self.args.squaresY} squares"
            )
            print(
                f"Expected charuco corners: {(self.args.squaresX-1) * (self.args.squaresY-1)}"
            )
            print(
                f"ArUco markers detected: {len(marker_corners) if marker_corners else 0}"
            )
            print(
                f"Charuco corners interpolated: {len(charuco_corners) if charuco_corners is not None else 0}"
            )

            if charuco_corners is not None and len(charuco_corners) > 0:
                # Find the bounding box of detected corners
                corners_array = np.array([corner[0] for corner in charuco_corners])
                min_x, min_y = corners_array.min(axis=0)
                max_x, max_y = corners_array.max(axis=0)
                print(
                    f"Corner detection area: ({min_x:.0f},{min_y:.0f}) to ({max_x:.0f},{max_y:.0f})"
                )

                # Check corner ID distribution
                if charuco_ids is not None:
                    min_id, max_id = charuco_ids.min(), charuco_ids.max()
                    print(f"Corner IDs range: {min_id} to {max_id}")

                    # For a 9x17 board, IDs should go from 0 to 127 (8x16 corners)
                    expected_max_id = (self.args.squaresX - 1) * (
                        self.args.squaresY - 1
                    ) - 1
                    print(f"Expected max corner ID: {expected_max_id}")

        return marker_corners, ids, charuco_corners, charuco_ids

    def print_controls(self):
        print("\n" + "=" * 40)
        print("      Interactive Camera Controls")
        print("=" * 40)
        print("  SHARPNESS:       [g]--      --[h]")
        print("  LUMA DENOISE:    [k]--      --[l]")
        print("  CHROMA DENOISE:  [v]--      --[c]")
        print("  ------------------------------------")
        print("  MANUAL EXPOSURE: [m] to toggle ON/OFF")
        print("  ------------------------------------")
        print("  IF MANUAL ON:")
        print("    ISO:           [u]-- 100  --[i]")
        print("    EXPOSURE TIME: [w]-- 1000us --[e]")
        print("  ------------------------------------")
        print("  IF AUTO ON:")
        print("    AE LIMIT:      [z]-- 1000us --[a]")
        print("\n  FPS cannot be changed at runtime.")
        print("  Press [space] to capture, [q] to quit.")
        print("=" * 40 + "\n")

    def print_current_settings(self):
        settings = [
            f"Sharpness: {self.sharpness}",
            f"Luma: {self.luma_denoise}",
            f"Chroma: {self.chroma_denoise}",
        ]
        if self.manual_exposure:
            settings.append("Mode: Manual")
            settings.append(f"ISO: {self.manual_iso}")
            settings.append(f"ExpTime: {self.manual_exposure_time_us}us")
        else:
            settings.append("Mode: Auto")
            settings.append(f"AE Limit: {self.ae_limit_us}us")

        print("\r" + " " * 120, end="")
        print("\rCurrent settings: " + " | ".join(settings), end="", flush=True)

    def print_all_settings_block(self):
        print("\n\n" + "=" * 40)
        print("    Current Camera Settings Updated")
        print("-" * 40)
        print(f"  Sharpness:            {self.sharpness}")
        print(f"  Luma Denoise:         {self.luma_denoise}")
        print(f"  Chroma Denoise:       {self.chroma_denoise}")
        print("-" * 40)
        if self.manual_exposure:
            print("  Mode:                 Manual")
            print(f"  ISO:                  {self.manual_iso}")
            print(f"  Exposure Time (us):   {self.manual_exposure_time_us}")
        else:
            print("  Mode:                 Auto")
            print(f"  AE Limit (us):        {self.ae_limit_us}")
        print("=" * 40)
        print("           Controls Reminder")
        print("-" * 40)
        print("  SHARPNESS: [g]/[h]   LUMA: [k]/[l]   CHROMA: [v]/[c]")
        print("  EXPOSURE MODE: [m] (toggle auto/manual)")
        print("  MANUAL-> ISO: [u]/[i]   EXP TIME: [w]/[e]")
        print("  AUTO->   AE LIMIT: [z]/[a]")
        print("=" * 40 + "\n")

    def show_info_frame(self):
        info_frame = np.zeros((600, 1100, 3), np.uint8)
        print("Starting image capture. Press the [ESC] key to abort.")
        if self.enablePolygonsDisplay:
            print(
                f"Will take {self.total_images} total images, {self.args.count} per each polygon."
            )
        else:
            print(f"Will take {self.total_images} total images.")

        def show(position, text):
            cv2.putText(
                info_frame, text, position, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0)
            )

        show((25, 40), f"Calibration of camera {self.board_name}")
        show((25, 100), "Information about image capture:")
        show((25, 160), "Press the [ESC] key to abort.")
        show((25, 220), "Press the [spacebar] key to capture the image.")
        if self.args.ssh_preview:
            show((25, 280), "SSH PREVIEW MODE: Focus on getting high marker counts.")
        else:
            show(
                (25, 280),
                'Press the "s" key to stop capturing images and begin calibration.',
            )
        if self.enablePolygonsDisplay:
            show((25, 360), "Polygon on the image represents the desired chessboard")
            show((25, 420), "position, that will provide best calibration score.")
            show(
                (25, 480),
                f"Will take {self.total_images} total images, {self.args.count} per each polygon.",
            )
        else:
            show((25, 480), f"Will take {self.total_images} total images.")

        show((25, 520), f"Progress: 0 / {self.total_images}")
        show((25, 560), "To continue, press [spacebar]...")

        cv2.imshow("info", info_frame)
        while True:
            key = cv2.waitKey(1)
            if key & 0xFF == ord(" "):
                cv2.destroyAllWindows()
                return
            elif key & 0xFF == 27 or key == ord("q"):
                cv2.destroyAllWindows()
                raise SystemExit(0)

    def show_failed_capture_frame(self):
        print(
            "py: Capture failed, unable to find sufficient markers! Reposition and try again."
        )
        time.sleep(1)

    def show_failed_sync_images(self):
        print(
            f"py: Capture failed, unable to sync images! Threshold: {self.minSyncTimestamp}"
        )
        time.sleep(2)

    def empty_calibration(self, calib: dai.CalibrationHandler):
        data = calib.getEepromData()
        for attr in ["boardName", "boardRev"]:
            if getattr(data, attr):
                return False
        return True

    def capture_images_sync(self):
        finished = False
        capturing = False
        start_timer = False
        timer = self.args.captureDelay
        prev_time = time.time()

        self.display_name = "Calibration Preview"
        self.minSyncTimestamp = self.args.minSyncTimestamp
        syncCollector = MessageSync(len(self.camera_queue), self.minSyncTimestamp)
        syncCollector.traceLevel = self.args.traceLevel
        self.mouseTrigger = False
        sync_trys = 0

        self.print_current_settings()

        while not finished:
            if capturing:
                syncedMsgs = syncCollector.get_synced()
                if not syncedMsgs:
                    if sync_trys > 10:
                        self.show_failed_sync_images()
                        finished = True
                        break
                    sync_trys += 1
                else:
                    allPassed = True
                    frames_to_save = {}
                    for name, frameMsg in syncedMsgs.items():
                        frame_to_save = frameMsg.getCvFrame()
                        if self.parse_frame(frame_to_save, name):
                            frames_to_save[name] = frame_to_save
                        else:
                            allPassed = False
                            break

                    if allPassed:
                        self.images_captured += 1
                        self.images_captured_polygon += 1
                        print(
                            f"\n[PROGRESS] {self.images_captured} / {self.total_images} images taken."
                        )

                        color = (
                            np.random.randint(0, 255),
                            np.random.randint(0, 255),
                            np.random.randint(0, 255),
                        )
                        for name, frame in frames_to_save.items():
                            if self.coverageImages[name] is None:
                                self.coverageImages[name] = (
                                    np.ones(frame.shape, np.uint8) * 255
                                )
                            bgr_frame = (
                                cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                                if len(frame.shape) == 2
                                else frame
                            )
                            bgr_coverage = (
                                cv2.cvtColor(
                                    self.coverageImages[name], cv2.COLOR_GRAY2BGR
                                )
                                if len(self.coverageImages[name].shape) == 2
                                else self.coverageImages[name]
                            )
                            self.coverageImages[name] = self.draw_corners(
                                bgr_frame, bgr_coverage, color
                            )

                        self.print_current_settings()
                    else:
                        self.show_failed_capture_frame()
                        for name in frames_to_save.keys():
                            filename = calibUtils.image_filename(
                                self.current_polygon, self.images_captured
                            )
                            path_to_remove = (
                                Path(self.args.datasetPath) / name / filename
                            )
                            if path_to_remove.exists():
                                path_to_remove.unlink()

                capturing = False

            full_res_frames = {}
            for key in self.camera_queue.keys():
                frameMsg = self.camera_queue[key].get()
                if frameMsg:
                    syncCollector.add_msg(key, frameMsg)
                    full_res_frames[key] = frameMsg.getCvFrame()

            display_frames = {}
            # Total corners are (squaresX-1) * (squaresY-1)
            total_corners = (self.args.squaresX - 1) * (self.args.squaresY - 1)
            min_required = int(total_corners * self.args.minDetectedMarkersPercent)

            for name, frame in full_res_frames.items():
                gray_frame = (
                    cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    if len(frame.shape) == 3
                    else frame
                )
                bgr_frame = (
                    cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    if len(frame.shape) == 2
                    else frame
                )
                if self.traceLevel > 0:
                    print(
                        f"Camera {name}: Frame shape = {frame.shape}, dtype = {frame.dtype}"
                    )
                    gray_frame_check = (
                        cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        if len(frame.shape) == 3
                        else frame
                    )
                    print(f"Camera {name}: Gray frame shape = {gray_frame_check.shape}")
                    h, w = gray_frame.shape
                    print(f"Camera {name}: Resolution = {w}x{h}")

                _, _, charuco_corners, charuco_ids = self.detect_markers_corners(
                    gray_frame
                )
                # Add debug output (only for first camera to avoid spam)
                if self.traceLevel > 0 and name == list(full_res_frames.keys())[0]:
                    self.debug_charuco_detection(gray_frame)
                    print("-" * 50)  # Separator line

                detected_count = len(charuco_ids) if charuco_ids is not None else 0

                if self.args.ssh_preview:
                    preview_frame_small = cv2.resize(
                        bgr_frame, (640, 400), interpolation=cv2.INTER_AREA
                    )
                    text = f"{name}: {detected_count}/{total_corners}"
                    color = green if detected_count >= min_required else red
                    cv2.putText(
                        preview_frame_small, text, (10, 30), font, 1, (0, 0, 0), 4
                    )
                    cv2.putText(preview_frame_small, text, (10, 30), font, 1, color, 2)
                    display_frames[name] = preview_frame_small
                else:
                    frame_with_markers = self.draw_markers(
                        bgr_frame, charuco_corners, charuco_ids
                    )
                    text = f"{name}: {detected_count}/{total_corners}"
                    color = green if detected_count >= min_required else red
                    cv2.putText(
                        frame_with_markers, text, (20, 60), font, 2, (0, 0, 0), 6
                    )
                    cv2.putText(frame_with_markers, text, (20, 60), font, 2, color, 3)
                    scaled_frame = cv2.resize(
                        frame_with_markers,
                        (0, 0),
                        fx=self.output_scale_factor,
                        fy=self.output_scale_factor,
                    )
                    display_frames[name] = scaled_frame

            if display_frames:
                if not self.args.ssh_preview:
                    # Arrange frames in a 2-column grid for better screen fit
                    frame_list = list(display_frames.values())
                    num_frames = len(frame_list)
                    rows = []
                    # Standardize width for stacking
                    max_width = max(f.shape[1] for f in frame_list) if frame_list else 0

                    for i in range(0, num_frames, 2):
                        # Get a pair of frames for the row
                        frame1 = frame_list[i]
                        frame2 = (
                            frame_list[i + 1]
                            if (i + 1) < num_frames
                            else create_blank(frame1.shape[1], frame1.shape[0])
                        )

                        # Resize to match heights
                        h1, w1 = frame1.shape[:2]
                        h2, w2 = frame2.shape[:2]
                        if h1 != h2:
                            if h1 > h2:
                                new_w2 = int(w2 * (h1 / h2))
                                frame2 = cv2.resize(
                                    frame2, (new_w2, h1), interpolation=cv2.INTER_AREA
                                )
                            else:  # h2 > h1
                                new_w1 = int(w1 * (h2 / h1))
                                frame1 = cv2.resize(
                                    frame1, (new_w1, h2), interpolation=cv2.INTER_AREA
                                )

                        rows.append(np.hstack((frame1, frame2)))

                    if rows:
                        # Before vstack, ensure all rows have the same width
                        max_row_width = max(r.shape[1] for r in rows)
                        processed_rows = []
                        for row in rows:
                            if row.shape[1] < max_row_width:
                                # Pad the row to match the max width
                                padding = np.zeros(
                                    (row.shape[0], max_row_width - row.shape[1], 3),
                                    dtype=np.uint8,
                                )
                                processed_rows.append(np.hstack((row, padding)))
                            else:
                                processed_rows.append(row)

                        combinedImage = np.vstack(processed_rows)
                    else:
                        combinedImage = create_blank(1280, 720, (0, 0, 0))

                else:  # ssh_preview logic
                    combinedImage = np.hstack(list(display_frames.values()))
            else:
                combinedImage = create_blank(1280, 400, (0, 0, 0))

            key = cv2.waitKey(1)
            control_update_needed = False
            ctrl = dai.CameraControl()
            max_exp_for_fps = (
                int(1_000_000 / self.args.framerate)
                if self.args.framerate > 0
                else 33333
            )

            if key == ord("g"):
                self.sharpness = clamp(self.sharpness - 1, 0, 4)
                ctrl.setSharpness(self.sharpness)
                control_update_needed = True
            elif key == ord("h"):
                self.sharpness = clamp(self.sharpness + 1, 0, 4)
                ctrl.setSharpness(self.sharpness)
                control_update_needed = True
            elif key == ord("k"):
                self.luma_denoise = clamp(self.luma_denoise - 1, 0, 4)
                ctrl.setLumaDenoise(self.luma_denoise)
                control_update_needed = True
            elif key == ord("l"):
                self.luma_denoise = clamp(self.luma_denoise + 1, 0, 4)
                ctrl.setLumaDenoise(self.luma_denoise)
                control_update_needed = True
            elif key == ord("v"):
                self.chroma_denoise = clamp(self.chroma_denoise - 1, 0, 4)
                ctrl.setChromaDenoise(self.chroma_denoise)
                control_update_needed = True
            elif key == ord("c"):
                self.chroma_denoise = clamp(self.chroma_denoise + 1, 0, 4)
                ctrl.setChromaDenoise(self.chroma_denoise)
                control_update_needed = True
            elif key == ord("m"):
                self.manual_exposure = not self.manual_exposure
                if self.manual_exposure:
                    ctrl.setManualExposure(
                        self.manual_exposure_time_us, self.manual_iso
                    )
                else:
                    ctrl.setAutoExposureEnable()
                    ctrl.setAutoExposureLimit(self.ae_limit_us)
                control_update_needed = True
            elif self.manual_exposure:
                if key == ord("u"):
                    self.manual_iso = clamp(self.manual_iso - 100, 100, 1600)
                    ctrl.setManualExposure(
                        self.manual_exposure_time_us, self.manual_iso
                    )
                    control_update_needed = True
                elif key == ord("i"):
                    self.manual_iso = clamp(self.manual_iso + 100, 100, 1600)
                    ctrl.setManualExposure(
                        self.manual_exposure_time_us, self.manual_iso
                    )
                    control_update_needed = True
                elif key == ord("w"):
                    self.manual_exposure_time_us = clamp(
                        self.manual_exposure_time_us - 1000, 5, max_exp_for_fps
                    )
                    ctrl.setManualExposure(
                        self.manual_exposure_time_us, self.manual_iso
                    )
                    control_update_needed = True
                elif key == ord("e"):
                    self.manual_exposure_time_us = clamp(
                        self.manual_exposure_time_us + 1000, 5, max_exp_for_fps
                    )
                    ctrl.setManualExposure(
                        self.manual_exposure_time_us, self.manual_iso
                    )
                    control_update_needed = True
            elif not self.manual_exposure:
                if key == ord("z"):
                    self.ae_limit_us = clamp(
                        self.ae_limit_us - 1000, 106, max_exp_for_fps
                    )
                    ctrl.setAutoExposureLimit(self.ae_limit_us)
                    control_update_needed = True
                elif key == ord("a"):
                    self.ae_limit_us = clamp(
                        self.ae_limit_us + 1000, 106, max_exp_for_fps
                    )
                    ctrl.setAutoExposureLimit(self.ae_limit_us)
                    control_update_needed = True

            if control_update_needed:
                self.print_all_settings_block()
                self.print_current_settings()
                for q in self.control_queues.values():
                    q.send(ctrl)

            if (key & 0xFF) == 27 or (key & 0xFF) == ord("q"):
                print("\npy: Calibration has been interrupted!")
                raise SystemExit(0)
            elif key == ord(" ") or self.mouseTrigger:
                if not start_timer:
                    start_timer = True
                    prev_time = time.time()
                    timer = self.args.captureDelay
                self.mouseTrigger = False
            if key == ord("s") and not self.args.ssh_preview:
                finished = True
                print(
                    "\nCapturing interrupted by user, proceeding with processing of images."
                )
                break

            display_image = combinedImage
            progress_text = f"Images: {self.images_captured} / {self.total_images}"
            font_scale = 1.0
            font_thick = 2
            (text_width, text_height), _ = cv2.getTextSize(
                progress_text, font, font_scale, font_thick
            )
            if display_image.shape[1] > text_width + 10:
                text_x = display_image.shape[1] - text_width - 10
                cv2.putText(
                    display_image,
                    progress_text,
                    (text_x, 30),
                    font,
                    font_scale,
                    (0, 0, 0),
                    4,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    display_image,
                    progress_text,
                    (text_x, 30),
                    font,
                    font_scale,
                    (255, 255, 255),
                    font_thick,
                    cv2.LINE_AA,
                )

            if start_timer:
                curr_time = time.time()
                if curr_time - prev_time >= 1:
                    prev_time = curr_time
                    timer -= 1
                if timer <= 0:
                    start_timer = False
                    capturing = True
                else:
                    image_shape = display_image.shape
                    cv2.putText(
                        display_image,
                        str(timer),
                        (image_shape[1] // 2, image_shape[0] // 2),
                        font,
                        7,
                        (0, 0, 255),
                        4,
                        cv2.LINE_AA,
                    )

            cv2.imshow(self.display_name, display_image)
            if self.args.mouseTrigger:
                cv2.setMouseCallback(self.display_name, self.mouse_event_callback)

            if self.images_captured_polygon == self.args.count:
                self.images_captured_polygon = 0
                self.current_polygon += 1
                if self.args.numPictures is None and self.current_polygon == len(
                    self.polygons
                ):
                    finished = True
                elif (
                    self.args.numPictures is not None
                    and self.images_captured >= self.args.numPictures
                ):
                    finished = True
            if self.images_captured >= self.total_images:
                finished = True
            if finished:
                cv2.destroyAllWindows()
                break
        print()

    def calibrate(self):
        print("Starting image processing")
        stereo_calib = calibUtils.StereoCalibration(
            self.args.traceLevel, self.args.outputScaleFactor, self.args.disableCamera
        )
        dest_path = str(Path("resources").absolute())

        try:
            status, result_config = stereo_calib.calibrate(
                self.board_config,
                self.dataset_path,
                self.args.squareSizeCm,
                self.args.markerSizeCm,
                self.args.squaresX,
                self.args.squaresY,
                self.args.cameraMode,
                self.args.rectifiedDisp,
            )

            if self.args.noInitCalibration or self.args.debugProcessingMode:
                calibration_handler = dai.CalibrationHandler()
            else:
                calibration_handler = self.device.readCalibration()

            board_keys = self.board_config.keys()
            if self.empty_calibration(calibration_handler):
                if "name" in board_keys and "revision" in board_keys:
                    calibration_handler.setBoardInfo(
                        self.board_config["name"], self.board_config["revision"]
                    )

            target_file_path = Path(self.dataset_path) / "target_info.txt"
            with target_file_path.open("w") as target_file:
                target_file.write(f"Marker Size: {self.args.markerSizeCm} cm\n")
                target_file.write(f"Square Size: {self.args.squareSizeCm} cm\n")
                target_file.write(f"Number of squaresX: {self.args.squaresX}\n")
                target_file.write(f"Number of squaresY: {self.args.squaresY}\n")

                error_text = []
                for camera, cam_info in result_config["cameras"].items():
                    if cam_info["name"] in self.args.disableCamera:
                        continue

                    reprojection_error_threshold = 1.0
                    if cam_info["size"][1] > 720:
                        reprojection_error_threshold *= cam_info["size"][1] / 720
                    if (
                        cam_info["name"] == "rgb" or cam_info["name"] == "middle"
                    ):  # Allow higher error for color
                        reprojection_error_threshold = 3.0

                    if cam_info["reprojection_error"] > reprojection_error_threshold:
                        error_text.append(
                            f"High Reprojection Error for {cam_info['name']}: {cam_info['reprojection_error']:.4f} > {reprojection_error_threshold:.2f}"
                        )

                    print(
                        f"{cam_info['name']} Reprojection Error: {cam_info['reprojection_error']:.6f}"
                    )
                    target_file.write(
                        f"{cam_info['name']}-reprojection: {cam_info['reprojection_error']:.6f}\n"
                    )

                    calibration_handler.setDistortionCoefficients(
                        stringToCam[camera], cam_info["dist_coeff"]
                    )
                    calibration_handler.setCameraIntrinsics(
                        stringToCam[camera],
                        cam_info["intrinsics"],
                        cam_info["size"][0],
                        cam_info["size"][1],
                    )
                    calibration_handler.setFov(stringToCam[camera], cam_info["hfov"])

                    if "extrinsics" in cam_info and "to_cam" in cam_info["extrinsics"]:
                        to_cam_name = result_config["cameras"][
                            cam_info["extrinsics"]["to_cam"]
                        ]["name"]
                        if to_cam_name in self.args.disableCamera:
                            continue

                        epipolar_error = cam_info["extrinsics"]["epipolar_error"]
                        if epipolar_error > self.args.maxEpiploarError:
                            error_text.append(
                                f"High Epipolar Error ({cam_info['name']}-{to_cam_name}): {epipolar_error:.4f} > {self.args.maxEpiploarError}"
                            )

                        target_file.write(
                            f"{cam_info['name']} and {to_cam_name} epipolar_error: {epipolar_error:.6f}\n"
                        )

                        specTranslation = np.array(
                            [
                                cam_info["extrinsics"]["specTranslation"]["x"],
                                cam_info["extrinsics"]["specTranslation"]["y"],
                                cam_info["extrinsics"]["specTranslation"]["z"],
                            ],
                            dtype=np.float32,
                        )
                        calibration_handler.setCameraExtrinsics(
                            stringToCam[camera],
                            stringToCam[cam_info["extrinsics"]["to_cam"]],
                            cam_info["extrinsics"]["rotation_matrix"],
                            cam_info["extrinsics"]["translation"],
                            specTranslation,
                        )

                # Stereo settings
                if "stereo_config" in result_config:
                    stereo_conf = result_config["stereo_config"]
                    left_cam_socket = stringToCam[stereo_conf["left_cam"]]
                    right_cam_socket = stringToCam[stereo_conf["right_cam"]]
                    calibration_handler.setStereoLeft(
                        left_cam_socket, stereo_conf["rectification_left"]
                    )
                    calibration_handler.setStereoRight(
                        right_cam_socket, stereo_conf["rectification_right"]
                    )

            if not error_text and not self.args.debugProcessingMode:
                print("Flashing calibration data to device...")
                mx_serial_id = self.device.getMxId()
                date_time_string = datetime.now().strftime("_%Y-%m-%d_%H-%M")
                file_name = f"{mx_serial_id}{date_time_string}.json"
                calib_dest_path = Path("resources") / file_name
                calib_dest_path.parent.mkdir(exist_ok=True, parents=True)
                calibration_handler.eepromToJsonFile(str(calib_dest_path))
                print(f"Calibration file also saved to: {calib_dest_path}")

                if self.args.saveCalibPath:
                    calibration_handler.eepromToJsonFile(self.args.saveCalibPath)

                status = self.device.flashCalibration(calibration_handler)
                if status:
                    print("EEPROM written successfully.")
                    resImage = create_blank(900, 512, rgb_color=green)
                    cv2.putText(
                        resImage,
                        "EEPROM written successfully",
                        (10, 250),
                        font,
                        2,
                        (0, 0, 0),
                        2,
                    )
                else:
                    print("Error writing to EEPROM.")
                    resImage = create_blank(900, 512, rgb_color=red)
                    cv2.putText(
                        resImage,
                        "EEPROM write FAILED!",
                        (10, 250),
                        font,
                        2,
                        (0, 0, 0),
                        2,
                    )
                cv2.imshow("Result", resImage)
                cv2.waitKey(0)

            else:
                print(
                    "\nCalibration failed or has high errors. Not flashing to device."
                )
                for error in error_text:
                    print(f"- {error}")
                    resImage = create_blank(1200, 512, rgb_color=red)
                    cv2.putText(resImage, error, (10, 250), font, 1.5, (0, 0, 0), 2)
                    cv2.imshow("Result", resImage)
                    cv2.waitKey(0)

        except Exception as e:
            print(f"An error occurred during calibration processing: {e}")
            traceback.print_exc()
        finally:
            if not self.args.debugProcessingMode:
                self.device.close()

    def run(self):
        """
        Main execution flow for calibration capture and processing.
        Properly manages device lifecycle using context manager pattern.
        """
        try:
            if "capture" in self.args.mode:
                self.dataset_path = self.args.datasetPath
                dataset_p = Path(self.dataset_path)

                if dataset_p.exists():
                    answer = input(
                        f"Folder '{self.dataset_path}' will be deleted. Proceed? (y/n): "
                    )
                    if answer.lower() == "y":
                        shutil.rmtree(dataset_p)
                    else:
                        print("Calibration cancelled.")
                        return

                # Open device using context manager
                with dai.Device() as self.device:
                    # Load Board Config & Augment with Live Sensor Info
                    if self.board_config is None:
                        dev_name = self.device.getDeviceName()
                        print(f"Device name: {dev_name}")
                        filtered_parts = [
                            p for p in dev_name.split("-") if p not in ("AF", "FF", "9782")
                        ]
                        self.board_name = "-".join(filtered_parts)
                        board_path = (
                            Path(__file__).parent
                            / "resources/depthai_boards/boards"
                            / f"{self.board_name.upper()}.json"
                        ).resolve()
                        if not board_path.exists():
                            raise ValueError(
                                f"Auto-detected board config not found: {board_path}"
                            )
                        with open(board_path) as fp:
                            self.board_config = json.load(fp)["board_config"]

                    cameraProperties = self.device.getConnectedCameraFeatures()
                    for properties in cameraProperties:
                        for in_cam, cam_info in self.board_config["cameras"].items():
                            if properties.socket == stringToCam.get(in_cam):
                                cam_info["sensorName"] = properties.sensorName
                                cam_info["hasAutofocus"] = properties.hasAutofocus
                                print(
                                    f"Cam: {cam_info['name']} | Sensor: {properties.sensorName} | Focus: {properties.hasAutofocus}"
                                )
                                break

                    self.coverageImages = {
                        cam["name"]: None
                        for cam in self.board_config["cameras"].values()
                    }

                    # Create dataset directories
                    for cam_info in self.board_config["cameras"].values():
                        (dataset_p / cam_info["name"]).mkdir(
                            parents=True, exist_ok=True
                        )

                    # Create and start pipeline with the open device
                    self.startPipeline()

                    # Show info and begin capture
                    self.print_controls()
                    self.show_info_frame()
                    self.capture_images_sync()

            if "process" in self.args.mode:
                self.dataset_path = self.args.datasetPath
                print(f"Using dataset path: {self.dataset_path}")

                # For processing mode, open device only if needed for writing calibration
                if (
                    "capture" not in self.args.mode
                    and not self.args.debugProcessingMode
                ):
                    with dai.Device() as self.device:
                        self.calibrate()
                else:
                    self.calibrate()

        except Exception as e:
            print(f"An error occurred during execution: {e}")
            traceback.print_exc()
        finally:
            print("py: DONE.")


if __name__ == "__main__":
    Main().run()

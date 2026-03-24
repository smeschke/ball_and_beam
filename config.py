import os

CAM_INDEX   = 0
SERIAL_PORT = "/dev/ttyACM0"
BAUD        = 115200
DESKTOP     = os.path.expanduser("~/Desktop")

MAX_VEL    = 8000   # steps/sec — must match Arduino MAX_VEL
MANUAL_VEL = 3000   # steps/sec when holding S or D

KP = 400.0
KI =  20.0
KD = 4260.0

GRAPH_HISTORY = 4500   # samples kept for graph (~150 s at 30 fps)
GRAPH_H       = 1000   # pixel height of graph panel
GRAPH_W       = 3000   # pixel width of graph panel

# detection slider defaults
V_MIN_INIT     = 78
S_MAX_INIT     = 56
AREA_INIT      = 0
ROI_TOP_INIT   = 28
ROI_BOT_INIT   = 65
ROI_LEFT_INIT  = 0
ROI_RIGHT_INIT = 100

# window names
WIN       = "Ball & Beam"
WIN_TUNE  = "Detection"
WIN_ROI   = "ROI"
WIN_PID   = "PID"
WIN_GRAPH = "Graphs"
WIN_TABLE = "PID Computation"

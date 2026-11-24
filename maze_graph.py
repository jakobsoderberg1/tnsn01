"""
Maze graph extracted from the map image (pixel coordinates, y grows downward).

Edit this file if the physical maze differs from the drawing.
Every edge must be horizontal or vertical (the robot thinks in N/E/S/W).

Node naming:
  B   = base (start/exit)          J* = junction (3+ lines)
  C*  = corner (90 degree bend)    D* = dead end
  H   = junction that lies INSIDE the second hole (no line under it!)
"""

MM_PER_PX = 1.0   # TODO: set from the real maze, e.g. real_width_mm / 965

NODES = {
    "B":   (545, 972),
    "A":   (418, 972),
    "J4":  (418, 837),
    "J3":  (280, 837),
    "C8":  (552, 837),
    "C1":  (135, 837),
    "C6":  (280, 972),
    "D2":  (140, 972),
    "C2":  (135, 143),
    "C3":  (280, 143),
    "C4":  (280, 275),
    "J1":  (418, 275),
    "D1":  (418, 143),
    "J2":  (418, 415),
    "C5":  (280, 415),
    "C7":  (418, 550),
    "J7":  (690, 550),
    "J6":  (690, 690),
    "H":   (690, 415),
    "C9":  (690, 275),
    "C13": (552, 415),
    "C12": (552, 143),
    "D5":  (688, 143),
    "C10": (962, 275),
    "C11": (962, 143),
    "D4":  (818, 143),
    "J5":  (552, 690),
    "D3":  (425, 690),
    "C14": (812, 690),
    "C15": (812, 837),
    "C16": (685, 837),
    "C17": (685, 972),
    "C18": (962, 972),
    "C19": (962, 563),
    "C20": (820, 563),
    "C21": (820, 415),
    "D7":  (968, 415),
}

# (u, v, gaps) - gaps are (start_px, end_px) measured from u along the edge.
EDGES = [
    ("B", "A", []), ("A", "J4", []),
    ("J4", "J3", []), ("J4", "C8", []),
    ("J3", "C1", []), ("J3", "C6", []), ("J3", "C5", []),
    ("C6", "D2", []),
    ("C1", "C2", []), ("C2", "C3", []), ("C3", "C4", []), ("C4", "J1", []),
    ("J1", "D1", []), ("J1", "J2", []),
    ("J2", "C5", []), ("J2", "C7", []),
    # Hole 1: horizontal line missing between x=478 and x=632
    ("C7", "J7", [(60, 214)]),
    ("J7", "J6", []),
    # Hole 2: blank region around (690, 415); junction H sits inside it
    ("J7", "H",   [(60, 135)]),   # line from J7 up to y=490, then blank
    ("H",  "C9",  [(0, 65)]),     # blank up to y=350, then line
    ("H",  "C13", [(0, 68)]),     # blank left to x=622, then line
    ("C13", "C12", []), ("C12", "D5", []),
    ("C9", "C10", []), ("C10", "C11", []), ("C11", "D4", []),
    ("J6", "J5", []), ("J5", "D3", []), ("J5", "C8", []),
    ("J6", "C14", []), ("C14", "C15", []), ("C15", "C16", []),
    ("C16", "C17", []), ("C17", "C18", []), ("C18", "C19", []),
    ("C19", "C20", []), ("C20", "C21", []), ("C21", "D7", []),
]

BASE = "B"
START_HEADING = "W"          # robot leaves the base driving west
BLIND_NODES = {"H"}          # junctions with no line: arrive by distance, turn by gyro

KNOWN_ITEMS = [("J3", "C5", (280, 548))]   # red dot

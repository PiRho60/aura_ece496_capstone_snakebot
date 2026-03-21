import cv2
import numpy as np
import math
import socket
import time

# ---------- IDs ----------
REF_IDS = [3, 4, 5, 6]  # fixed reference tags
SNAKE_ID = 1
OBJECT_ID = 2

# ---------- Real-world reference tag centers (meters) ----------
WORLD_REF_METERS = {
3: (0.00, 0.00),
4: (2.4384, 0.00),
5: (0.00, 2.4384),
6: (2.4384, 2.4384),
}

# WORLD_REF_METERS = {
#     3: (0.00, 0.00),
#     4: (1.83, 0.00),
#     5: (0.00, 1.83),
#     6: (1.83, 1.83),
# }

# ---------- Camera ----------
CAM_INDEX = 1
REQ_W, REQ_H = 3840, 2160

# ---------- ESP32 UDP ----------
ESP_IP = "100.66.148.72"
ESP_PORT = 6657

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ---------- Window / UI ----------
WINDOW_NAME = "Snake Robot Control"

pending_command = None
quit_requested = False

system_state = "IDLE"
last_command = "None"
status_message = "Ready"

# Small compact buttons
# (x1, y1, x2, y2)
BUTTONS = {
    "go": (20, 52, 110, 88),
    "stop": (120, 52, 235, 88),
    "quit": (245, 52, 340, 88),
    "north": (1720, 45, 1780, 85),
    "west": (1655, 90, 1715, 130),
    "east": (1785, 90, 1845, 130),
    "south": (1720, 135, 1780, 175),
}


# Sends a raw UDP text message to the ESP32.
# Used as the base helper for all robot commands.
def send_udp_message(message: str):
    try:
        sock.sendto(message.encode(), (ESP_IP, ESP_PORT))
        print(f"[UDP SENT] {message}")
    except Exception as e:
        print(f"[UDP ERROR] Failed to send '{message}': {e}")


# Formats and sends a GO command containing:
# - direction: turn angle relative to snake heading
# - distance_m: distance to target in meters
def send_go_command(direction: float, distance_m: float):
    msg = f"cmd=GO,direction={direction:.2f},distance_m={distance_m:.3f}"
    send_udp_message(msg)


# Sends a STOP command to the ESP32 to stop robot motion.
def send_stop_command():
    send_udp_message("cmd=STOP")


# Computes the center point of a detected AprilTag
# by averaging its 4 corner coordinates.
def tag_center_from_corners(corners_4x2: np.ndarray):
    return corners_4x2.mean(axis=0)


# Returns the predefined AprilTag dictionary used for detection.
# This code uses AprilTag 36h11 markers through OpenCV ArUco.
def get_marker_dict():
    aruco = cv2.aruco
    return aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)


# Detects AprilTags in the current frame.
# Returns:
# - out: dictionary mapping tag_id -> 4x2 corner array
# - corners: raw OpenCV corners result
# - ids: raw OpenCV ids result
def detect_markers(detector, frame):
    corners, ids, rejected = detector.detectMarkers(frame)

    if ids is None:
        return {}, corners, ids

    out = {}
    for i, tag_id in enumerate(ids.flatten().tolist()):
        out[tag_id] = corners[i].reshape(4, 2).astype(np.float32)

    return out, corners, ids


# Builds a homography matrix H that maps image pixel coordinates
# into world coordinates in meters.
# It uses the centers of the 4 reference tags with known real positions.
def compute_homography(id_to_corners):
    """
    Build homography H that maps image pixel coords -> world meters using the centers of reference tags.
    """

    img_pts = []
    world_pts = []

    for tid in REF_IDS:
        if tid in id_to_corners:

            c = tag_center_from_corners(id_to_corners[tid])
            img_pts.append([c[0], c[1]])

            wx, wy = WORLD_REF_METERS[tid]
            world_pts.append([wx, wy])

    if len(img_pts) < 4:
        return None

    img_pts = np.array(img_pts, dtype=np.float32)
    world_pts = np.array(world_pts, dtype=np.float32)

    H, _ = cv2.findHomography(img_pts, world_pts, method=0)

    return H


# Uses the homography matrix to convert a single image pixel point
# into world coordinates in meters.
def img_to_world(H, xy_pixel):

    pt = np.array([[[xy_pixel[0], xy_pixel[1]]]], dtype=np.float32)

    out = cv2.perspectiveTransform(pt, H)

    return float(out[0, 0, 0]), float(out[0, 0, 1])


# Normalizes a 2D vector to unit length.
# Returns None if the vector is too small to normalize safely.
def normalize(vx, vy, eps=1e-9):

    n = math.hypot(vx, vy)

    if n < eps:
        return None

    return vx / n, vy / n


# Computes the signed angle in degrees from vector 1 to vector 2.
# Positive means counterclockwise, negative means clockwise.
def signed_angle_deg(v1x, v1y, v2x, v2y):

    dot = v1x * v2x + v1y * v2y
    cross = v1x * v2y - v1y * v2x

    return math.degrees(math.atan2(cross, dot))


# Computes the snake pose in world coordinates from its AprilTag.
# Returns:
# - snake center position (sx, sy)
# - forward unit direction (hx, hy)
# - heading angle in degrees
#
# Forward is defined using the LEFT edge midpoint of the tag,
# so this assumes the snake forward direction corresponds to the tag -x axis.
def get_snake_pose_world(H, snake_corners_px):
    """
    Returns:
        sx, sy            = snake center in world frame
        hx, hy            = unit vector of snake FORWARD axis in world frame (defined as tag -x)
        heading_deg       = absolute heading in world frame (0 = -X, CCW positive)
    """

    center_px = tag_center_from_corners(snake_corners_px)

    # corner order assumed [TL, TR, BR, BL]
    # use LEFT edge midpoint so forward = tag -x
    left_mid_px = 0.5 * (snake_corners_px[0] + snake_corners_px[3])

    sx, sy = img_to_world(H, center_px)
    fx, fy = img_to_world(H, left_mid_px)

    hx = fx - sx
    hy = fy - sy

    hnorm = normalize(hx, hy)

    if hnorm is None:
        return None

    hx, hy = hnorm

    # now hx,hy already points along tag -x
    # define 0 deg at world -X
    heading_deg = (math.degrees(math.atan2(hy, hx)) - 180.0 + 360.0) % 360.0

    return sx, sy, hx, hy, heading_deg


# Checks whether a mouse click point (x, y) lies inside a rectangular button.
def point_in_rect(x, y, rect):

    x1, y1, x2, y2 = rect

    return x1 <= x <= x2 and y1 <= y <= y2


# Mouse callback for the OpenCV window.
# When the user clicks a button, it stores the requested command
# into pending_command for the main loop to process.
def mouse_callback(event, x, y, flags, param):

    global pending_command

    if event != cv2.EVENT_LBUTTONDOWN:
        return

    if point_in_rect(x, y, BUTTONS["go"]):
        pending_command = "go"

    elif point_in_rect(x, y, BUTTONS["stop"]):
        pending_command = "stop"

    elif point_in_rect(x, y, BUTTONS["quit"]):
        pending_command = "quit"

    elif point_in_rect(x, y, BUTTONS["north"]):
        pending_command = "north"
    elif point_in_rect(x, y, BUTTONS["south"]):
        pending_command = "south"
    elif point_in_rect(x, y, BUTTONS["east"]):
        pending_command = "east"
    elif point_in_rect(x, y, BUTTONS["west"]):
        pending_command = "west"


# Draws a semi-transparent filled rectangle on top of the frame.
# Used for dashboard backgrounds and buttons.
def draw_translucent_box(frame, x1, y1, x2, y2, color=(30, 30, 30), alpha=0.28):

    overlay = frame.copy()

    cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)

    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)


# Draws a labeled UI button using a translucent colored box
# and centered text.
def draw_button(frame, rect, label, fill_color, text_color=(255, 255, 255)):

    x1, y1, x2, y2 = rect

    # lighter / cleaner compact button
    draw_translucent_box(frame, x1, y1, x2, y2, fill_color, alpha=0.88)

    cv2.rectangle(frame, (x1, y1), (x2, y2), (245, 245, 245), 1)

    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.55
    thick = 1

    (tw, th), _ = cv2.getTextSize(label, font, scale, thick)

    tx = x1 + (x2 - x1 - tw) // 2
    ty = y1 + (y2 - y1 + th) // 2

    cv2.putText(frame, label, (tx, ty), font, scale, text_color, thick, cv2.LINE_AA)


# Draws the top dashboard text and the GO / STOP / QUIT buttons.
# It displays:
# - system state
# - last command
# - status message
# - visible reference tags
# - current distance / angle / heading if available
def draw_dashboard(frame, latest_nav, visible_refs):

    # Top status line
    if latest_nav is not None and latest_nav["relative_deg"] is not None:

        dist_str = f"{latest_nav['distance_m']:.3f}m"
        ang_str = f"{latest_nav['relative_deg']:.1f}deg"
        heading_str = f"{latest_nav['snake_heading_deg']:.1f}deg"

    else:

        dist_str = "N/A"
        ang_str = "N/A"
        heading_str = "N/A"

    top_line = (
        f"STATE: {system_state}, "
        f"CMD: {last_command}, "
        f"STATUS: {status_message}, "
        f"REFS: {visible_refs}, "
        f"DIST: {dist_str}, "
        f"ANGLE: {ang_str}, "
        f"HEADING: {heading_str}"
    )

    # small translucent background just for the line
    draw_translucent_box(frame, 12, 10, 1180, 40, color=(25, 25, 25), alpha=0.32)

    cv2.putText(
        frame,
        top_line,
        (20, 31),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )

    # compact buttons row
    draw_button(frame, BUTTONS["go"], "GO", (0, 165, 0))
    draw_button(frame, BUTTONS["stop"], "STOP", (0, 140, 230))
    draw_button(frame, BUTTONS["quit"], "QUIT", (0, 0, 210))

    # directional pad (D-Pad)
    draw_button(frame, BUTTONS["north"], "N", (70, 70, 70))
    draw_button(frame, BUTTONS["west"], "W", (70, 70, 70))
    draw_button(frame, BUTTONS["east"], "E", (70, 70, 70))
    draw_button(frame, BUTTONS["south"], "S", (70, 70, 70))


# Main program loop.
# Responsibilities:
# - initialize AprilTag detector and camera
# - read frames continuously
# - detect reference / snake / object tags
# - compute world mapping and navigation values
# - draw UI and tag overlays
# - process GO / STOP / QUIT commands
# - send UDP commands to the ESP32
def main():

    global pending_command
    global quit_requested

    global system_state
    global last_command
    global status_message

    aruco = cv2.aruco

    dictionary = get_marker_dict()

    params = aruco.DetectorParameters()

    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5
    params.cornerRefinementMaxIterations = 30
    params.cornerRefinementMinAccuracy = 0.01

    detector = aruco.ArucoDetector(dictionary, params)

    cap = cv2.VideoCapture(CAM_INDEX)

    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {CAM_INDEX}")
    
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, REQ_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, REQ_H)
    cap.set(cv2.CAP_PROP_FPS, 60.0)

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    cv2.setMouseCallback(WINDOW_NAME, mouse_callback)

    latest_nav = None
    stored_nav = None

    while True:

        ok, frame = cap.read()

        if not ok:
            break

        id_to_corners, corners, ids = detect_markers(detector, frame)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

        H = compute_homography(id_to_corners)

        latest_nav = None

        if H is not None and (SNAKE_ID in id_to_corners) and (OBJECT_ID in id_to_corners):

            snake_px = tag_center_from_corners(id_to_corners[SNAKE_ID])
            obj_px = tag_center_from_corners(id_to_corners[OBJECT_ID])

            spt = (int(snake_px[0]), int(snake_px[1]))
            opt = (int(obj_px[0]), int(obj_px[1]))

            cv2.line(frame, spt, opt, (255, 0, 255), 2)
            cv2.circle(frame, spt, 5, (255, 0, 255), -1)
            cv2.circle(frame, opt, 5, (255, 0, 255), -1)

            snake_pose = get_snake_pose_world(H, id_to_corners[SNAKE_ID])

            if snake_pose is not None:

                sx, sy, hx, hy, heading_deg = snake_pose

                ox, oy = img_to_world(H, obj_px)

                vx = ox - sx
                vy = oy - sy

                d = math.hypot(vx, vy)

                rel_deg = None

                vnorm = normalize(vx, vy)

                if vnorm is not None:

                    vxn, vyn = vnorm

                    rel_deg = signed_angle_deg(hx, hy, vxn, vyn)

                latest_nav = {
                    "snake_xy": (sx, sy),
                    "object_xy": (ox, oy),
                    "distance_m": d,
                    "snake_heading_deg": heading_deg,
                    "relative_deg": rel_deg,
                }
                stored_nav = latest_nav

        vis = [tid for tid in REF_IDS if tid in id_to_corners]

        draw_dashboard(frame, latest_nav, vis)

        cmd_to_run = pending_command
        pending_command = None

        if cmd_to_run == "go":

            last_command = "GO"

            if stored_nav is None or stored_nav["relative_deg"] is None:

                system_state = "WAITING"
                status_message = "Nav unavailable"

                print("GO requested, but no navigation data has been seen yet.")

            else:

                rel_deg = stored_nav["relative_deg"]
                d = stored_nav["distance_m"]

                if abs(rel_deg) < 5.0:
                    turn_msg = "Straight"
                elif rel_deg > 0:
                    turn_msg = f"Left {abs(rel_deg):.1f}"
                else:
                    turn_msg = f"Right {abs(rel_deg):.1f}"

                system_state = "RUNNING"
                status_message = turn_msg
                
                send_go_command(rel_deg, d)

        elif cmd_to_run == "stop":

            last_command = "STOP"
            system_state = "STOPPED"
            status_message = "Stop sent"

            print("snake stop")

            send_stop_command()

        elif cmd_to_run == "north":
            last_command = "NORTH"
            system_state = "RUNNING"
            status_message = "Heading North"
            print("snake direction: north")
            send_udp_message("cmd=NORTH")
        
        elif cmd_to_run == "south":
            last_command = "SOUTH"
            system_state = "RUNNING"
            status_message = "Heading South"
            print("snake direction: south")
            send_udp_message("cmd=SOUTH")

        elif cmd_to_run == "east":
            last_command = "EAST"
            system_state = "RUNNING"
            status_message = "Heading East"
            print("snake direction: east")
            send_udp_message("cmd=EAST")

        elif cmd_to_run == "west":
            last_command = "WEST"
            system_state = "RUNNING"
            status_message = "Heading West"
            print("snake direction: west")
            send_udp_message("cmd=WEST")

        elif cmd_to_run == "quit":

            last_command = "QUIT"
            system_state = "EXIT"
            status_message = "Quitting"

            print("quitting...")

            send_stop_command()

            break


        cv2.imshow(WINDOW_NAME, frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q") or key == 27:

            send_stop_command()

            break

    quit_requested = True

    cap.release()
    cv2.destroyAllWindows()
    sock.close()


if __name__ == "__main__":
    main()

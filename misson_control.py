from pymavlink import mavutil
import time

MISSION_TIMEOUT_S = 30
POSITION_READY_TIMEOUT_S = 60
ARM_RETRY_COUNT = 5
ARM_RETRY_DELAY_S = 3
CURRENT_ITEM_TIMEOUT_S = 10
MODE_CHANGE_DELAY_S = 2
POST_ARM_DELAY_S = 2
GUIDED_TAKEOFF_ALT_M = 10.0
TAKEOFF_TIMEOUT_S = 60

print("Mission control starting")
connection = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
connection.wait_heartbeat()
print(f"Connected to system {connection.target_system}, component {connection.target_component}")

def set_current_mission_item(seq: int = 0):
    connection.waypoint_set_current_send(seq)
    print(f"Requested current mission item {seq}")

def wait_for_current_mission_item(expected_seq: int = 0, timeout_s: float = CURRENT_ITEM_TIMEOUT_S):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = connection.recv_match(
            type=["MISSION_CURRENT", "STATUSTEXT"],
            blocking=True,
            timeout=1,
        )
        if msg is None:
            continue

        msg_type = msg.get_type()
        if msg_type == "STATUSTEXT":
            print(f"[AP] {msg.text}")
            continue

        seq = int(getattr(msg, "seq", -1))
        print(f"Mission current item reported as {seq}")
        if seq == expected_seq:
            return

    print(f"Did not observe MISSION_CURRENT={expected_seq}; continuing anyway")

def set_mode(mode_name: str):
    mode_id = connection.mode_mapping().get(mode_name)
    if mode_id is None:
        raise ValueError(f"Unknown mode: {mode_name}")

    connection.mav.set_mode_send(
        connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    print(f"Requested mode change to {mode_name}")

def arm():
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0, 0, 0, 0, 0, 0,
    )
    print("Sent arm command")

def wait_for_arm_ack(timeout_s: float = 15.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = connection.recv_match(
            type=["COMMAND_ACK", "STATUSTEXT", "HEARTBEAT"],
            blocking=True,
            timeout=1,
        )
        if msg is None:
            continue

        msg_type = msg.get_type()
        if msg_type == "STATUSTEXT":
            print(f"[AP] {msg.text}")
            continue

        if msg_type == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            print(f"Arm COMMAND_ACK result={msg.result}")
            return msg.result

        if msg_type == "HEARTBEAT":
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed:
                print("Vehicle is armed")
                return mavutil.mavlink.MAV_RESULT_ACCEPTED

    raise TimeoutError("Timed out waiting for arm acknowledgement")

def arm_with_retries(retries: int = ARM_RETRY_COUNT, delay_s: float = ARM_RETRY_DELAY_S):
    for attempt in range(1, retries + 1):
        print(f"Arm attempt {attempt}/{retries}")
        arm()
        try:
            arm_result = wait_for_arm_ack()
        except TimeoutError:
            arm_result = None
            print("Arm acknowledgement timed out")

        if arm_result in (
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
            mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
        ):
            print("Arming accepted")
            return

        print(f"Arming not accepted yet, waiting {delay_s} seconds before retry")
        time.sleep(delay_s)

    raise RuntimeError("Vehicle failed to arm after multiple attempts")

def guided_takeoff(altitude_m: float):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0, altitude_m,
    )
    print(f"Sent GUIDED takeoff command to {altitude_m} m")

def wait_until_altitude_reached(target_alt_m: float, tolerance_m: float = 0.7, timeout_s: float = TAKEOFF_TIMEOUT_S):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = connection.recv_match(
            type=["GLOBAL_POSITION_INT", "STATUSTEXT"],
            blocking=True,
            timeout=1,
        )
        if msg is None:
            continue

        msg_type = msg.get_type()

        if msg_type == "STATUSTEXT":
            print(f"[AP] {msg.text}")
            continue

        rel_alt_m = float(msg.relative_alt) / 1000.0
        print(f"Relative altitude: {rel_alt_m:.2f} m")

        if rel_alt_m >= (target_alt_m - tolerance_m):
            print(f"Reached takeoff altitude: {rel_alt_m:.2f} m")
            return

    raise TimeoutError(f"Did not reach {target_alt_m} m in time")

def start_mission():
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0,
        0, 0, 0, 0, 0, 0, 0,
    )
    print("Sent MISSION_START")

def wait_for_mission_start_ack(timeout_s: float = 10.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = connection.recv_match(
            type=["COMMAND_ACK", "STATUSTEXT", "MISSION_CURRENT", "HEARTBEAT"],
            blocking=True,
            timeout=1,
        )
        if msg is None:
            continue

        msg_type = msg.get_type()
        if msg_type == "STATUSTEXT":
            print(f"[AP] {msg.text}")
            continue

        if msg_type == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_MISSION_START:
            print(f"MISSION_START COMMAND_ACK result={msg.result}")
            return

        if msg_type == "MISSION_CURRENT":
            print(f"Mission current item reported as {int(getattr(msg, 'seq', -1))}")
            return

        if msg_type == "HEARTBEAT":
            custom_mode = int(getattr(msg, "custom_mode", -1))
            print(f"Heartbeat custom_mode={custom_mode}")

    print("No explicit MISSION_START acknowledgement observed")

def make_takeoff_item(seq, lat, lon, alt):
    return mavutil.mavlink.MAVLink_mission_item_int_message(
        connection.target_system,
        connection.target_component,
        seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        1 if seq == 0 else 0,
        1,
        0, 0, 0, 0,
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
    )

def make_waypoint_item(seq, lat, lon, alt):
    return mavutil.mavlink.MAVLink_mission_item_int_message(
        connection.target_system,
        connection.target_component,
        seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,
        1,
        0, 0, 0, 0,
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
    )

def make_rtl_item(seq):
    return mavutil.mavlink.MAVLink_mission_item_int_message(
        connection.target_system,
        connection.target_component,
        seq,
        mavutil.mavlink.MAV_FRAME_MISSION,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0,
        1,
        0, 0, 0, 0,
        0, 0, 0,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
    )

def drain_old_mission_messages(duration_s: float = 2.0):
    deadline = time.time() + duration_s
    while time.time() < deadline:
        msg = connection.recv_match(
            type=["MISSION_ACK", "STATUSTEXT"],
            blocking=True,
            timeout=0.2,
        )
        if msg is None:
            continue
        if msg.get_type() == "MISSION_ACK":
            print(f"Ignoring pre-upload MISSION_ACK type={msg.type}")
        elif msg.get_type() == "STATUSTEXT":
            print(f"[AP] {msg.text}")

def upload_mission(waypoints):
    print("Clearing previous mission")
    connection.mav.mission_clear_all_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
    )

    drain_old_mission_messages()

    print(f"Sending mission count: {len(waypoints)}")
    connection.mav.mission_count_send(
        connection.target_system,
        connection.target_component,
        len(waypoints),
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
    )

    sent_items = set()
    deadline = time.time() + MISSION_TIMEOUT_S
    while time.time() < deadline:
        msg = connection.recv_match(
            type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK", "STATUSTEXT"],
            blocking=True,
            timeout=1,
        )

        if msg is None:
            continue

        msg_type = msg.get_type()
        if msg_type == "STATUSTEXT":
            print(f"[AP] {msg.text}")
            continue

        if msg_type in ["MISSION_REQUEST_INT", "MISSION_REQUEST"]:
            seq = int(msg.seq)
            if seq < 0 or seq >= len(waypoints):
                raise IndexError(f"Vehicle requested out-of-range mission item {seq}")
            print(f"Vehicle requested mission item {seq}")
            connection.mav.send(waypoints[seq])
            sent_items.add(seq)
            deadline = time.time() + MISSION_TIMEOUT_S
            continue

        if msg_type == "MISSION_ACK":
            if len(sent_items) == len(waypoints):
                print(f"Mission upload complete: type={msg.type}")
                return
            print(f"Ignoring early MISSION_ACK type={msg.type} before all mission items were sent")

    raise TimeoutError("Mission upload timed out before all items were requested and acknowledged")

def wait_for_position_ready(timeout_s: float = POSITION_READY_TIMEOUT_S):
    print("Waiting for GPS fix / position estimate")
    have_fix = False
    have_global_position = False
    deadline = time.time() + timeout_s

    while time.time() < deadline:
        msg = connection.recv_match(
            type=["GPS_RAW_INT", "GLOBAL_POSITION_INT", "STATUSTEXT"],
            blocking=True,
            timeout=1,
        )
        if msg is None:
            continue

        msg_type = msg.get_type()
        if msg_type == "GPS_RAW_INT":
            fix_type = int(getattr(msg, "fix_type", 0))
            satellites_visible = int(getattr(msg, "satellites_visible", 0))
            print(f"GPS fix_type={fix_type} satellites={satellites_visible}")
            have_fix = fix_type >= 3
        elif msg_type == "GLOBAL_POSITION_INT":
            have_global_position = not (msg.lat == 0 and msg.lon == 0)
        elif msg_type == "STATUSTEXT":
            print(f"[AP] {msg.text}")

        if have_fix and have_global_position:
            print("GPS fix and global position available; waiting 5 more seconds for home/EKF")
            time.sleep(5)
            return

    raise TimeoutError("Position estimate did not become ready in time")

def wait_until_altitude_reached(target_alt_m: float, tolerance_m: float = 0.7, timeout_s: float = TAKEOFF_TIMEOUT_S):
    deadline = time.time() + timeout_s
    saw_position = False

    while time.time() < deadline:
        msg = connection.recv_match(
            type=["GLOBAL_POSITION_INT", "STATUSTEXT"],
            blocking=True,
            timeout=1,
        )
        if msg is None:
            continue

        msg_type = msg.get_type()

        if msg_type == "STATUSTEXT":
            print(f"[AP] {msg.text}")
            continue

        saw_position = True
        rel_alt_m = float(msg.relative_alt) / 1000.0
        print(f"Relative altitude: {rel_alt_m:.2f} m")

        if rel_alt_m >= (target_alt_m - tolerance_m):
            print(f"Reached takeoff altitude: {rel_alt_m:.2f} m")
            return

    if not saw_position:
        raise TimeoutError("Never received GLOBAL_POSITION_INT during takeoff wait")

    raise TimeoutError(f"Did not reach {target_alt_m} m in time")

def guided_takeoff(altitude_m: float):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0, altitude_m,
    )
    print(f"Sent GUIDED takeoff command to {altitude_m} m")

    deadline = time.time() + 10
    while time.time() < deadline:
        msg = connection.recv_match(
            type=["COMMAND_ACK", "STATUSTEXT"],
            blocking=True,
            timeout=1,
        )
        if msg is None:
            continue

        if msg.get_type() == "STATUSTEXT":
            print(f"[AP] {msg.text}")
            continue

        if msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            print(f"TAKEOFF COMMAND_ACK result={msg.result}")
            if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                raise RuntimeError(f"Takeoff rejected with MAV_RESULT={msg.result}")
            return

    print("No explicit TAKEOFF COMMAND_ACK observed")

    
home_lat = -35.363261
home_lon = 149.165230

mission_waypoints = [
    make_takeoff_item(0, home_lat, home_lon, 20),
    make_waypoint_item(1, home_lat + 0.0002, home_lon + 0.0002, 20),
    make_waypoint_item(2, home_lat + 0.0004, home_lon + 0.0004, 20),
    make_waypoint_item(3, home_lat + 0.0006, home_lon + 0.0002, 20),
    make_waypoint_item(4, home_lat + 0.0008, home_lon + 0.0000, 20),
    make_waypoint_item(5, home_lat + 0.0010, home_lon - 0.0002, 20),
    make_rtl_item(6),
]

print("Uploading mission")
upload_mission(mission_waypoints)
set_current_mission_item(0)
wait_for_current_mission_item(0)

wait_for_position_ready()

set_mode("GUIDED")
time.sleep(MODE_CHANGE_DELAY_S)

arm_with_retries()
time.sleep(POST_ARM_DELAY_S)

print("About to send GUIDED takeoff")
guided_takeoff(GUIDED_TAKEOFF_ALT_M)

print("Waiting to reach takeoff altitude")
wait_until_altitude_reached(GUIDED_TAKEOFF_ALT_M)

print("Takeoff complete, switching to AUTO")
set_mode("AUTO")
time.sleep(MODE_CHANGE_DELAY_S)

print("Starting mission")
start_mission()
wait_for_mission_start_ack()

print("Drone ready for mission")

while True:
    msg = connection.recv_match(
        type=["HEARTBEAT", "GLOBAL_POSITION_INT", "STATUSTEXT", "MISSION_CURRENT"],
        blocking=True,
        timeout=1,
    )
    if msg is not None:
        print(msg)
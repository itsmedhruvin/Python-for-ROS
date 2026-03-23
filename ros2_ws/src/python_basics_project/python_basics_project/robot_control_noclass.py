#!/usr/bin/python3

# python imports
import time
import math
import threading
import traceback
# ros2 imports
import rclpy
from rclpy.executors import MultiThreadedExecutor
# module imports
from robot_interface import RobotInterface


#~#~#~#~#~# start your function definitions after this line #~#~#~#~#~#

# ─────────────────────────────────────────────────────────────────────────────
# MOVEMENT FUNCTIONS  (14)
# ─────────────────────────────────────────────────────────────────────────────

def get_linear_angular_velocity():
    """Return the current linear and angular velocities as a dict."""
    global robot_interface
    return {
        "linear":  robot_interface.linear_velocity,
        "angular": robot_interface.angular_velocity,
    }


def stop_robot():
    """Stop all robot movement immediately."""
    global robot_interface
    robot_interface.linear_velocity  = 0.0
    robot_interface.angular_velocity = 0.0
    return None


def move_front(linear_speed):
    """Move the robot forward at the given linear speed."""
    global robot_interface
    robot_interface.linear_velocity  =  abs(linear_speed)
    robot_interface.angular_velocity = 0.0
    return None


def move_back(linear_speed):
    """Move the robot backward at the given linear speed."""
    global robot_interface
    robot_interface.linear_velocity  = -abs(linear_speed)
    robot_interface.angular_velocity = 0.0
    return None


def turn_left(angular_speed):
    """Turn the robot left (counter-clockwise) at the given angular speed."""
    global robot_interface
    robot_interface.linear_velocity  = 0.0
    robot_interface.angular_velocity =  abs(angular_speed)
    return None


def turn_right(angular_speed):
    """Turn the robot right (clockwise) at the given angular speed."""
    global robot_interface
    robot_interface.linear_velocity  = 0.0
    robot_interface.angular_velocity = -abs(angular_speed)
    return None


def timed_move_front(linear_speed, duration):
    """Move forward at linear_speed for duration seconds, then stop."""
    move_front(linear_speed)
    time.sleep(duration)
    stop_robot()
    return None


def timed_move_back(linear_speed, duration):
    """Move backward at linear_speed for duration seconds, then stop."""
    move_back(linear_speed)
    time.sleep(duration)
    stop_robot()
    return None


def timed_turn_left(angular_speed, duration):
    """Turn left at angular_speed for duration seconds, then stop."""
    turn_left(angular_speed)
    time.sleep(duration)
    stop_robot()
    return None


def timed_turn_right(angular_speed, duration):
    """Turn right at angular_speed for duration seconds, then stop."""
    turn_right(angular_speed)
    time.sleep(duration)
    stop_robot()
    return None


def move_distance_front(linear_speed, distance):
    """
    Move forward by 'distance' metres using open-loop control.
    distance = linear_speed × time  →  time = distance / linear_speed
    """
    speed    = abs(linear_speed)
    duration = abs(distance) / speed if speed > 0 else 0.0
    timed_move_front(speed, duration)
    return None


def move_distance_back(linear_speed, distance):
    """
    Move backward by 'distance' metres using open-loop control.
    distance = linear_speed × time  →  time = distance / linear_speed
    """
    speed    = abs(linear_speed)
    duration = abs(distance) / speed if speed > 0 else 0.0
    timed_move_back(speed, duration)
    return None


def turn_angle_left(angular_speed, angle):
    """
    Turn left by 'angle' radians using open-loop control.
    angle = angular_speed × time  →  time = angle / angular_speed
    """
    speed    = abs(angular_speed)
    duration = abs(angle) / speed if speed > 0 else 0.0
    timed_turn_left(speed, duration)
    return None


def turn_angle_right(angular_speed, angle):
    """
    Turn right by 'angle' radians using open-loop control.
    angle = angular_speed × time  →  time = angle / angular_speed
    """
    speed    = abs(angular_speed)
    duration = abs(angle) / speed if speed > 0 else 0.0
    timed_turn_right(speed, duration)
    return None


# ─────────────────────────────────────────────────────────────────────────────
# LASER SCANNER FUNCTIONS  (13)
# ─────────────────────────────────────────────────────────────────────────────

def get_min_scan_angle():
    """Return the minimum scan angle from the LaserScan message (radians)."""
    global robot_interface
    return robot_interface.scan_angle_min


def get_max_scan_angle():
    """Return the maximum scan angle from the LaserScan message (radians)."""
    global robot_interface
    return robot_interface.scan_angle_max


def get_angle_increment():
    """Return the angular step between consecutive scan rays (radians)."""
    global robot_interface
    return robot_interface.scan_angle_increment


def get_min_scan_range():
    """Return the sensor's minimum measurable range (metres)."""
    global robot_interface
    return robot_interface.scan_range_min


def get_max_scan_range():
    """Return the sensor's maximum measurable range (metres)."""
    global robot_interface
    return robot_interface.scan_range_max


def get_all_scan_ranges():
    """Return the full list of range readings."""
    global robot_interface
    return list(robot_interface.scan_ranges)


def get_scan_range_by_index(index):
    """Return the range reading at the specified index."""
    global robot_interface
    return robot_interface.scan_ranges[index]


def get_front_scan_range():
    """
    Return the range directly in front of the robot.
    Index 0 corresponds to angle_min (front) for a forward-facing scanner.
    """
    global robot_interface
    return robot_interface.scan_ranges[0]


def get_back_scan_range():
    """
    Return the range directly behind the robot.
    Back index ≈ halfway through the array (π radians from front).
    """
    global robot_interface
    back_index = len(robot_interface.scan_ranges) // 2
    return robot_interface.scan_ranges[back_index]


def get_left_scan_range():
    """
    Return the range to the left of the robot.
    Left index ≈ one-quarter through the array (π/2 radians from front).
    """
    global robot_interface
    left_index = len(robot_interface.scan_ranges) // 4
    return robot_interface.scan_ranges[left_index]


def get_right_scan_range():
    """
    Return the range to the right of the robot.
    Right index ≈ three-quarters through the array (3π/2 radians from front).
    """
    global robot_interface
    right_index = (3 * len(robot_interface.scan_ranges)) // 4
    return robot_interface.scan_ranges[right_index]


def get_min_range_with_index():
    """
    Return (min_range, index) ignoring inf and NaN values.
    This is the closest detected obstacle — NOT the sensor's minimum range spec.
    """
    global robot_interface
    finite_ranges = [(v, i) for i, v in enumerate(robot_interface.scan_ranges)
                     if v != float("inf") and not math.isnan(v)]
    if not finite_ranges:
        return (float("inf"), -1)
    min_val, min_idx = min(finite_ranges, key=lambda x: x[0])
    return (min_val, min_idx)


def get_max_range_with_index():
    """
    Return (max_range, index) ignoring inf and NaN values.
    This is the farthest detected point — NOT the sensor's maximum range spec.
    """
    global robot_interface
    finite_ranges = [(v, i) for i, v in enumerate(robot_interface.scan_ranges)
                     if v != float("inf") and not math.isnan(v)]
    if not finite_ranges:
        return (0.0, -1)
    max_val, max_idx = max(finite_ranges, key=lambda x: x[0])
    return (max_val, max_idx)


# ─────────────────────────────────────────────────────────────────────────────
# ODOMETRY FUNCTIONS  (3)
# ─────────────────────────────────────────────────────────────────────────────

def get_position():
    """Return the robot's current position as a dict {x, y, z}."""
    global robot_interface
    return {
        "x": robot_interface.odom_position_x,
        "y": robot_interface.odom_position_y,
        "z": robot_interface.odom_position_z,
    }


def get_orientation():
    """Return the robot's current orientation as a dict {roll, pitch, yaw}."""
    global robot_interface
    return {
        "roll":  robot_interface.odom_orientation_r,
        "pitch": robot_interface.odom_orientation_p,
        "yaw":   robot_interface.odom_orientation_y,
    }


def get_euclidean_distance(x1, y1, x2, y2):
    """
    Calculate and return the 2-D Euclidean distance between two points.
    distance = sqrt((x2-x1)² + (y2-y1)²)
    """
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


# ─────────────────────────────────────────────────────────────────────────────
# TEST FUNCTION
# ─────────────────────────────────────────────────────────────────────────────

def run_tests():
    """Exercise every function once and print the results."""
    global robot_interface

    sep = "-" * 52

    # ── Velocities ────────────────────────────────────────
    print(sep)
    print("TEST: get_linear_angular_velocity()")
    print(get_linear_angular_velocity())

    # ── Forward / backward movement ───────────────────────
    print(sep)
    print("TEST: move_front(0.1)  — 2 s")
    timed_move_front(0.1, 2.0)

    print(sep)
    print("TEST: move_back(0.1)  — 2 s")
    timed_move_back(0.1, 2.0)

    # ── Turning ───────────────────────────────────────────
    print(sep)
    print("TEST: turn_left(0.3)  — 2 s")
    timed_turn_left(0.3, 2.0)

    print(sep)
    print("TEST: turn_right(0.3)  — 2 s")
    timed_turn_right(0.3, 2.0)

    # ── Distance-based movement ───────────────────────────
    print(sep)
    print("TEST: move_distance_front(0.1, 0.20 m)")
    move_distance_front(0.1, 0.20)

    print(sep)
    print("TEST: move_distance_back(0.1, 0.20 m)")
    move_distance_back(0.1, 0.20)

    # ── Angle-based turning ───────────────────────────────
    print(sep)
    print("TEST: turn_angle_left(0.3, π/2 rad)")
    turn_angle_left(0.3, robot_interface.pi / 2)

    print(sep)
    print("TEST: turn_angle_right(0.3, π/2 rad)")
    turn_angle_right(0.3, robot_interface.pi / 2)

    # ── Stop ──────────────────────────────────────────────
    print(sep)
    print("TEST: stop_robot()")
    stop_robot()
    time.sleep(0.5)

    # ── Laser scanner ─────────────────────────────────────
    print(sep)
    print("TEST: get_min_scan_angle()       →", get_min_scan_angle())
    print("TEST: get_max_scan_angle()       →", get_max_scan_angle())
    print("TEST: get_angle_increment()      →", get_angle_increment())
    print("TEST: get_min_scan_range()       →", get_min_scan_range())
    print("TEST: get_max_scan_range()       →", get_max_scan_range())
    ranges = get_all_scan_ranges()
    print("TEST: get_all_scan_ranges()      → len =", len(ranges),
          "  first =", ranges[0] if ranges else "N/A")
    print("TEST: get_scan_range_by_index(0) →", get_scan_range_by_index(0))
    print("TEST: get_front_scan_range()     →", get_front_scan_range())
    print("TEST: get_back_scan_range()      →", get_back_scan_range())
    print("TEST: get_left_scan_range()      →", get_left_scan_range())
    print("TEST: get_right_scan_range()     →", get_right_scan_range())
    print("TEST: get_min_range_with_index() →", get_min_range_with_index())
    print("TEST: get_max_range_with_index() →", get_max_range_with_index())

    # ── Odometry ──────────────────────────────────────────
    print(sep)
    pos = get_position()
    ori = get_orientation()
    print("TEST: get_position()    →", pos)
    print("TEST: get_orientation() →", ori)
    dist = get_euclidean_distance(0.0, 0.0, pos["x"], pos["y"])
    print("TEST: get_euclidean_distance(0,0, x,y) →", round(dist, 4), "m")

    print(sep)
    print("ALL TESTS COMPLETE.")
    return None

#~#~#~#~#~# finish your function definitions before this line #~#~#~#~#~#


def spin_node():
    """
    Run the robot interface node in a separate thread.
    NOTE: THE ROBOT WILL NOT WORK IF THIS FUNCTION IS REMOVED !!!
    """
    global executor
    executor.spin()
    return None


if __name__ == "__main__":

    # initialize ros2 with python
    rclpy.init(args=None)
    # instantiate robot interface program module
    robot_interface = RobotInterface()
    # start robot interface program execution
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(robot_interface)
    # run robot interface program in a separate thread
    threading.Thread(target=spin_node).start()
    # wait for a few seconds for program to initialize
    print("Getting Ready in 5 Seconds...")
    time.sleep(5.0)
    print("READY !!!")

    try:
        #~#~#~#~#~# start your program after this line #~#~#~#~#~#

        run_tests()

        #~#~#~#~#~# finish your program before this line #~#~#~#~#~#

    except Exception as error:
        # report exception
        print("~~~~~~~~~~~ ERROR: ~~~~~~~~~~~")
        print(traceback.print_exception(error))
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        # clean up before shutdown
        executor.shutdown()
        robot_interface.destroy_node()

    finally:
        # shutdown ros2
        rclpy.shutdown()

# End of Code
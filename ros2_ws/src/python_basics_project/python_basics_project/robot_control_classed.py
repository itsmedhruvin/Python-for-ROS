#!/usr/bin/python3

# python imports
import time
import threading
import traceback
import math
from statistics import multimode
# ros2 imports
import rclpy
from rclpy.executors import MultiThreadedExecutor
# module imports
from robot_interface import RobotInterface


#~#~#~#~#~# start your class after this line #~#~#~#~#~#

class RobotControl:

    def __init__(self, robot_interface):
        """Class constructor — store a reference to the shared RobotInterface node."""
        self.ri = robot_interface
        return None

    def __del__(self):
        """Class destructor."""
        return None

    # ─────────────────────────────────────────────────────────────────
    # MOVEMENT FUNCTIONS  (14)
    # ─────────────────────────────────────────────────────────────────

    def get_linear_angular_velocity(self):
        """Return the current linear and angular velocities as a dict."""
        return {
            "linear":  self.ri.linear_velocity,
            "angular": self.ri.angular_velocity,
        }

    def stop_robot(self):
        """Stop all robot movement immediately."""
        self.ri.linear_velocity  = 0.0
        self.ri.angular_velocity = 0.0
        return None

    def move_front(self, linear_speed):
        """Move the robot forward at the given linear speed."""
        self.ri.linear_velocity  =  abs(linear_speed)
        self.ri.angular_velocity = 0.0
        return None

    def move_back(self, linear_speed):
        """Move the robot backward at the given linear speed."""
        self.ri.linear_velocity  = -abs(linear_speed)
        self.ri.angular_velocity = 0.0
        return None

    def turn_left(self, angular_speed):
        """Turn the robot left (counter-clockwise) at the given angular speed."""
        self.ri.linear_velocity  = 0.0
        self.ri.angular_velocity =  abs(angular_speed)
        return None

    def turn_right(self, angular_speed):
        """Turn the robot right (clockwise) at the given angular speed."""
        self.ri.linear_velocity  = 0.0
        self.ri.angular_velocity = -abs(angular_speed)
        return None

    def timed_move_front(self, linear_speed, duration):
        """Move forward at linear_speed for duration seconds, then stop."""
        self.move_front(linear_speed)
        time.sleep(duration)
        self.stop_robot()
        return None

    def timed_move_back(self, linear_speed, duration):
        """Move backward at linear_speed for duration seconds, then stop."""
        self.move_back(linear_speed)
        time.sleep(duration)
        self.stop_robot()
        return None

    def timed_turn_left(self, angular_speed, duration):
        """Turn left at angular_speed for duration seconds, then stop."""
        self.turn_left(angular_speed)
        time.sleep(duration)
        self.stop_robot()
        return None

    def timed_turn_right(self, angular_speed, duration):
        """Turn right at angular_speed for duration seconds, then stop."""
        self.turn_right(angular_speed)
        time.sleep(duration)
        self.stop_robot()
        return None

    def move_distance_front(self, linear_speed, distance):
        """
        Move forward by 'distance' meters using open-loop control.
        distance = linear_speed x time  ->  time = distance / linear_speed
        """
        speed    = abs(linear_speed)
        duration = abs(distance) / speed if speed > 0 else 0.0
        self.timed_move_front(speed, duration)
        return None

    def move_distance_back(self, linear_speed, distance):
        """
        Move backward by 'distance' meters using open-loop control.
        distance = linear_speed x time  ->  time = distance / linear_speed
        """
        speed    = abs(linear_speed)
        duration = abs(distance) / speed if speed > 0 else 0.0
        self.timed_move_back(speed, duration)
        return None

    def turn_angle_left(self, angular_speed, angle):
        """
        Turn left by 'angle' radians using open-loop control.
        angle = angular_speed x time  ->  time = angle / angular_speed
        """
        speed    = abs(angular_speed)
        duration = abs(angle) / speed if speed > 0 else 0.0
        self.timed_turn_left(speed, duration)
        return None

    def turn_angle_right(self, angular_speed, angle):
        """
        Turn right by 'angle' radians using open-loop control.
        angle = angular_speed x time  ->  time = angle / angular_speed
        """
        speed    = abs(angular_speed)
        duration = abs(angle) / speed if speed > 0 else 0.0
        self.timed_turn_right(speed, duration)
        return None

    # ─────────────────────────────────────────────────────────────────
    # LASER SCANNER FUNCTIONS  (13)
    # ─────────────────────────────────────────────────────────────────

    def get_min_scan_angle(self):
        """Return the minimum scan angle from the LaserScan message (radians)."""
        return self.ri.scan_angle_min

    def get_max_scan_angle(self):
        """Return the maximum scan angle from the LaserScan message (radians)."""
        return self.ri.scan_angle_max

    def get_angle_increment(self):
        """Return the angular step between consecutive scan rays (radians)."""
        return self.ri.scan_angle_increment

    def get_min_scan_range(self):
        """Return the sensor's minimum measurable range (metres)."""
        return self.ri.scan_range_min

    def get_max_scan_range(self):
        """Return the sensor's maximum measurable range (metres)."""
        return self.ri.scan_range_max

    def get_all_scan_ranges(self):
        """Return the full list of range readings."""
        return list(self.ri.scan_ranges)

    def get_scan_range_by_index(self, index):
        """Return the range reading at the specified index."""
        return self.ri.scan_ranges[index]

    def get_front_scan_range(self):
        """
        Return the range directly in front of the robot.
        Index 0 corresponds to angle_min (front) for a forward-facing scanner.
        """
        return self.ri.scan_ranges[0]

    def get_back_scan_range(self):
        """
        Return the range directly behind the robot.
        Back index = halfway through the array (pi radians from front).
        """
        back_index = len(self.ri.scan_ranges) // 2
        return self.ri.scan_ranges[back_index]

    def get_left_scan_range(self):
        """
        Return the range to the left of the robot.
        Left index = one-quarter through the array (pi/2 radians from front).
        """
        left_index = len(self.ri.scan_ranges) // 4
        return self.ri.scan_ranges[left_index]

    def get_right_scan_range(self):
        """
        Return the range to the right of the robot.
        Right index = three-quarters through the array (3*pi/2 radians from front).
        """
        right_index = (3 * len(self.ri.scan_ranges)) // 4
        return self.ri.scan_ranges[right_index]

    def get_min_range_with_index(self):
        """
        Return (min_range, index) ignoring inf and NaN values.
        This is the closest detected obstacle, NOT the sensor minimum range spec.
        """
        finite_ranges = [(v, i) for i, v in enumerate(self.ri.scan_ranges)
                         if v != float("inf") and not math.isnan(v)]
        if not finite_ranges:
            return (float("inf"), -1)
        min_val, min_idx = min(finite_ranges, key=lambda x: x[0])
        return (min_val, min_idx)

    def get_max_range_with_index(self):
        """
        Return (max_range, index) ignoring inf and NaN values.
        This is the farthest detected point, NOT the sensor maximum range spec.
        """
        finite_ranges = [(v, i) for i, v in enumerate(self.ri.scan_ranges)
                         if v != float("inf") and not math.isnan(v)]
        if not finite_ranges:
            return (0.0, -1)
        max_val, max_idx = max(finite_ranges, key=lambda x: x[0])
        return (max_val, max_idx)

    # ─────────────────────────────────────────────────────────────────
    # ODOMETRY FUNCTIONS  (3)
    # ─────────────────────────────────────────────────────────────────

    def get_position(self):
        """Return the robot's current position as a dict {x, y, z}."""
        return {
            "x": self.ri.odom_position_x,
            "y": self.ri.odom_position_y,
            "z": self.ri.odom_position_z,
        }

    def get_orientation(self):
        """Return the robot's current orientation as a dict {roll, pitch, yaw}."""
        return {
            "roll":  self.ri.odom_orientation_r,
            "pitch": self.ri.odom_orientation_p,
            "yaw":   self.ri.odom_orientation_y,
        }

    def get_euclidean_distance(self, x1, y1, x2, y2):
        """
        Calculate and return the 2-D Euclidean distance between two points.
        distance = sqrt((x2-x1)^2 + (y2-y1)^2)
        """
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    # ─────────────────────────────────────────────────────────────────
    # PART 3 - ALGORITHM 1: OBSTACLE PREDICTION
    # ─────────────────────────────────────────────────────────────────

    def predict_obstacle(self, threshold=0.300):
        """
        Predict what is in front of the robot using the frontal 90 degree arc.

        Steps:
          1. Find indices 45 degrees left and 45 degrees right of front (index 0).
          2. Collect all ranges in that 90 degree window.
          3. Remove inf and NaN values.
          4. If min > threshold  -> print "none"
             If min <= threshold -> run multimode on the finite list:
               len(modes) > 1   -> print "obstacle"
               len(modes) == 1  -> print "wall"

        Returns the prediction string.
        """
        ranges    = self.get_all_scan_ranges()
        total     = len(ranges)
        increment = self.get_angle_increment()

        # number of indices that span 45 degrees
        if increment > 0:
            half_window = int((math.pi / 4) / increment)
        else:
            half_window = total // 8    # fallback for unknown increment

        # front is index 0; left is positive direction, right is negative (wraps)
        left_idx  = half_window % total          # +45 degrees
        right_idx = (total - half_window) % total  # -45 degrees

        # collect the 90 degree frontal window (handles wrap-around at index 0)
        if right_idx > left_idx:
            # window wraps: [right_idx .. end] + [0 .. left_idx]
            frontal = ranges[right_idx:] + ranges[: left_idx + 1]
        else:
            frontal = ranges[right_idx : left_idx + 1]

        # remove inf and NaN
        frontal_finite = [v for v in frontal
                          if v != float("inf") and not math.isnan(v)]

        if not frontal_finite:
            prediction = "none"
            print("[Obstacle Prediction] " + prediction +
                  "  (no valid readings in frontal arc)")
            return prediction

        min_val = min(frontal_finite)

        if min_val > threshold:
            prediction = "none"
        else:
            modes = multimode(frontal_finite)
            if len(modes) > 1:
                prediction = "obstacle"
            else:
                prediction = "wall"

        print("[Obstacle Prediction] " + prediction +
              "  (front_min=" + str(round(min_val, 3)) +
              " m, threshold=" + str(threshold) + " m)")
        return prediction

    # ─────────────────────────────────────────────────────────────────
    # PART 3 - ALGORITHM 2: DIRECTION TRACKING
    # ─────────────────────────────────────────────────────────────────

    def get_direction(self):
        """
        Map the current yaw to one of 16 compass directions (3-char strings).

        ROS yaw convention:
          0       = North  (robot facing forward at spawn)
          +pi/2   = West   (turned left / CCW)
          +/-pi   = South  (facing backward)
          -pi/2   = East   (turned right / CW)

        16 segments, each 22.5 degrees (pi/8 radians) wide, ordered CCW from North.
        Returns the 3-character direction string and prints it.
        """
        directions = [
            "-N-", "NNW", "N-W", "WNW",
            "-W-", "WSW", "S-W", "SSW",
            "-S-", "SSE", "S-E", "ESE",
            "-E-", "ENE", "N-E", "NNE",
        ]

        yaw     = self.get_orientation()["yaw"]   # range: (-pi, +pi]
        segment = math.pi / 8                     # 22.5 degrees per segment

        # offset yaw by half a segment so North boundary is centred on "-N-"
        idx       = int((yaw + segment / 2) / segment) % 16
        direction = directions[idx]

        print("[Direction Tracking] " + direction +
              "  (yaw=" + str(round(yaw, 5)) + " rad)")
        return direction

    # ─────────────────────────────────────────────────────────────────
    # PART 3 - ALGORITHM 3: NAIVE OBSTACLE AVOIDER
    # ─────────────────────────────────────────────────────────────────

    def run_obstacle_avoider(self, linear_speed=0.100, angular_speed=0.300,
                             threshold=0.300, duration=300.0):
        """
        Naive Obstacle Avoider - runs for 'duration' seconds (default 5 min).

        The 360-degree laser scan is divided into 8 x 45-degree segments.
        Only five segments are used:
          front, front-left, front-right, left, right.

        Decision logic each iteration:
          1. All frontal segments clear           -> move forward
          2. Front clear, front-left blocked      -> curve right
          3. Front clear, front-right blocked     -> curve left
          4. Front blocked                        -> turn toward more open side
          5. All frontal segments blocked         -> turn toward more open side

        Distance travelled is accumulated using incremental Euclidean steps.
        Linear and angular velocities are printed each iteration.
        """
        print("=" * 60)
        print("Naive Obstacle Avoider - STARTING")
        print("  linear_speed  = " + str(linear_speed) + " m/s")
        print("  angular_speed = " + str(angular_speed) + " rad/s")
        print("  threshold     = " + str(threshold) + " m")
        print("  duration      = " + str(duration) + " s")
        print("=" * 60)

        total_distance = 0.0
        start_time     = time.time()
        prev_pos       = self.get_position()

        while (time.time() - start_time) < duration:

            ranges = self.get_all_scan_ranges()
            total  = len(ranges)
            seg    = total // 8     # indices per 45-degree segment

            # ── segment slice helpers ────────────────────────────────
            # front         : index 0 only
            # front-left    : indices 1 .. seg      (+1 to +45 degrees)
            # left          : indices seg+1 .. 2*seg (+45 to +90 degrees)
            # front-right   : indices total-seg .. total-1  (-45 to -1 degrees)
            # right         : indices total-2*seg .. total-seg-1 (-90 to -45 deg)

            front_seg        = [ranges[0]]
            front_left_seg   = ranges[1        : seg + 1]
            left_seg         = ranges[seg + 1  : 2 * seg + 1]
            front_right_seg  = ranges[total - seg  : total]
            right_seg        = ranges[total - 2 * seg : total - seg]

            def seg_min(seg_list):
                """Return minimum finite value in a segment, or inf if all inf."""
                finite = [v for v in seg_list
                          if v != float("inf") and not math.isnan(v)]
                return min(finite) if finite else float("inf")

            min_front        = seg_min(front_seg)
            min_front_left   = seg_min(front_left_seg)
            min_front_right  = seg_min(front_right_seg)
            min_left         = seg_min(left_seg)
            min_right        = seg_min(right_seg)

            front_clear       = min_front       > threshold
            front_left_clear  = min_front_left  > threshold
            front_right_clear = min_front_right > threshold

            # ── decision tree ────────────────────────────────────────
            if front_clear and front_left_clear and front_right_clear:
                self.move_front(linear_speed)
                action = "FORWARD"

            elif front_clear and (not front_left_clear) and front_right_clear:
                # front-left too close -> steer right while moving
                self.ri.linear_velocity  = linear_speed
                self.ri.angular_velocity = -abs(angular_speed)
                action = "CURVE RIGHT (front-left blocked)"

            elif front_clear and front_left_clear and (not front_right_clear):
                # front-right too close -> steer left while moving
                self.ri.linear_velocity  = linear_speed
                self.ri.angular_velocity =  abs(angular_speed)
                action = "CURVE LEFT  (front-right blocked)"

            elif not front_clear:
                # front blocked -> turn in place toward more open side
                if min_left >= min_right:
                    self.turn_left(angular_speed)
                    action = "TURN LEFT  (front blocked, left more open)"
                else:
                    self.turn_right(angular_speed)
                    action = "TURN RIGHT (front blocked, right more open)"

            else:
                # both flanks blocked, front still clear -> keep going
                self.move_front(linear_speed)
                action = "FORWARD    (flanks tight)"

            # ── distance tracking ────────────────────────────────────
            curr_pos       = self.get_position()
            step_dist      = self.get_euclidean_distance(
                prev_pos["x"], prev_pos["y"],
                curr_pos["x"], curr_pos["y"]
            )
            total_distance += step_dist
            prev_pos        = curr_pos

            # ── current velocities ───────────────────────────────────
            vel     = self.get_linear_angular_velocity()
            elapsed = round(time.time() - start_time, 1)

            print("[" + str(elapsed).rjust(6) + "s] " +
                  action.ljust(44) +
                  "| dist=" + str(round(total_distance, 3)).rjust(7) + " m" +
                  "  lin=" + str(round(vel["linear"],  3)).rjust(7) +
                  "  ang=" + str(round(vel["angular"], 3)).rjust(7))

            time.sleep(0.1)     # 10 Hz control loop

        # ── clean stop and summary ───────────────────────────────────
        self.stop_robot()
        print("=" * 60)
        print("Naive Obstacle Avoider - FINISHED after " +
              str(round(duration, 1)) + " s")
        print("Total distance travelled : " +
              str(round(total_distance, 3)) + " m")
        print("=" * 60)
        return None

    # ─────────────────────────────────────────────────────────────────
    # PART 2 TEST FUNCTION  (commented out as instructed)
    # ─────────────────────────────────────────────────────────────────

    def run_tests(self):
        """Part 2 test code - kept but commented out."""

        sep = "-" * 52

        print(sep)
        print("TEST: get_linear_angular_velocity()")
        print(self.get_linear_angular_velocity())

        print(sep)
        print("TEST: move_front(0.1)  - 2 s")
        self.timed_move_front(0.1, 2.0)

        print(sep)
        print("TEST: move_back(0.1)  - 2 s")
        self.timed_move_back(0.1, 2.0)

        print(sep)
        print("TEST: turn_left(0.3)  - 2 s")
        self.timed_turn_left(0.3, 2.0)

        print(sep)
        print("TEST: turn_right(0.3)  - 2 s")
        self.timed_turn_right(0.3, 2.0)

        print(sep)
        print("TEST: move_distance_front(0.1, 0.20 m)")
        self.move_distance_front(0.1, 0.20)

        print(sep)
        print("TEST: move_distance_back(0.1, 0.20 m)")
        self.move_distance_back(0.1, 0.20)

        print(sep)
        print("TEST: turn_angle_left(0.3, pi/2 rad)")
        self.turn_angle_left(0.3, self.ri.pi / 2)

        print(sep)
        print("TEST: turn_angle_right(0.3, pi/2 rad)")
        self.turn_angle_right(0.3, self.ri.pi / 2)

        print(sep)
        print("TEST: stop_robot()")
        self.stop_robot()
        time.sleep(0.5)

        print(sep)
        print("TEST: get_min_scan_angle()       ->", self.get_min_scan_angle())
        print("TEST: get_max_scan_angle()       ->", self.get_max_scan_angle())
        print("TEST: get_angle_increment()      ->", self.get_angle_increment())
        print("TEST: get_min_scan_range()       ->", self.get_min_scan_range())
        print("TEST: get_max_scan_range()       ->", self.get_max_scan_range())
        ranges = self.get_all_scan_ranges()
        print("TEST: get_all_scan_ranges()      -> len =", len(ranges),
              "  first =", ranges[0] if ranges else "N/A")
        print("TEST: get_scan_range_by_index(0) ->", self.get_scan_range_by_index(0))
        print("TEST: get_front_scan_range()     ->", self.get_front_scan_range())
        print("TEST: get_back_scan_range()      ->", self.get_back_scan_range())
        print("TEST: get_left_scan_range()      ->", self.get_left_scan_range())
        print("TEST: get_right_scan_range()     ->", self.get_right_scan_range())
        print("TEST: get_min_range_with_index() ->", self.get_min_range_with_index())
        print("TEST: get_max_range_with_index() ->", self.get_max_range_with_index())

        print(sep)
        pos = self.get_position()
        ori = self.get_orientation()
        print("TEST: get_position()    ->", pos)
        print("TEST: get_orientation() ->", ori)
        dist = self.get_euclidean_distance(0.0, 0.0, pos["x"], pos["y"])
        print("TEST: get_euclidean_distance(0,0, x,y) ->", round(dist, 4), "m")

        print(sep)
        print("ALL TESTS COMPLETE.")
        return None

#~#~#~#~#~# finish your class before this line #~#~#~#~#~#


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
    # wait a few seconds for program to initialise
    print("Getting Ready in 5 Seconds...")
    time.sleep(5.0)
    print("READY !!!")

    try:
        #~#~#~#~#~# start your program after this line #~#~#~#~#~#

        rc = RobotControl(robot_interface)

        # Part 2 tests - commented out as instructed
        rc.move_distance_front(0.1, 0.20)

        rc.stop_robot()
        rc.predict_obstacle()

        rc.move_distance_front(0.1, 0.20)
        rc.stop_robot()
        rc.predict_obstacle()

        '''# ── Part 3 Algorithm 1: Obstacle Prediction ─────────────────
        print("\n--- Obstacle Prediction (5 samples) ---")
        for _ in range(5):
            rc.predict_obstacle(threshold=0.300)
            time.sleep(0.5)

        # ── Part 3 Algorithm 2: Direction Tracking ───────────────────
        print("\n--- Direction Tracking (5 samples) ---")
        for _ in range(5):
            rc.get_direction()
            time.sleep(0.5)

        # ── Part 3 Algorithm 3: Naive Obstacle Avoider ───────────────
        # Runs for 300 seconds (5 minutes).
        # Reduce duration value for a quick test, e.g. duration=30.0
        rc.run_obstacle_avoider(
            linear_speed  = 0.100,
            angular_speed = 0.300,
            threshold     = 0.300,
            duration      = 300.0,
        )

        #~#~#~#~#~# finish your program before this line #~#~#~#~#~#'''

    except Exception as error:
        print("~~~~~~~~~~~ ERROR: ~~~~~~~~~~~")
        print(traceback.print_exception(error))
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        executor.shutdown()
        robot_interface.destroy_node()

    finally:
        rclpy.shutdown()

# End of Code
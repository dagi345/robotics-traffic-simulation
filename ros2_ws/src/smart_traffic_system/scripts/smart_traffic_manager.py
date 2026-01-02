#!/usr/bin/env python3
"""
Smart Traffic Manager with Zone Monitoring, Yellow Light, and Error Handling

This module manages traffic light states based on vehicle density
with intersection zone monitoring to prevent collisions.

Requirements: 1.1-1.6, 3.1-3.5, 7.1-7.5, 10.1, 10.3, 10.4
"""
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import LogicalCameraImage, Light
from std_msgs.msg import String, Bool
import time
import json


class SmartTrafficManager(Node):
    def __init__(self):
        super().__init__('smart_traffic_manager')

        # Constants from traffic-flow-logic.txt
        self.BASE_GREEN = 10.0  # Base green time (seconds)
        self.MAX_GREEN = 60.0   # Maximum green time (seconds)
        self.MIN_GREEN = 10.0   # Minimum green time (seconds)
        self.YELLOW_TIME = 3.0  # Yellow light duration (fixed, Requirement 7.2)
        self.ALL_RED_TIME = 2.0 # All-red clearance time (fixed, 1-2 seconds)
        self.CLEARANCE_TIME = 3.0  # Minimum time for intersection to clear
        self.SENSOR_TIMEOUT = 1.0  # Seconds before sensor is considered failed
        
        # Adaptive green time calculation parameters (from traffic-flow-logic.txt)
        self.k_vehicle = 1.0  # seconds per vehicle
        self.k_ped = 2.0      # seconds per pedestrian
        self.alpha = 3.0      # pedestrian priority factor (α ∈ [2, 5])
        
        # Phases (including yellow states - Requirement 7.1)
        self.NS_GREEN = "NS_GREEN"
        self.NS_YELLOW = "NS_YELLOW"
        self.EW_GREEN = "EW_GREEN"
        self.EW_YELLOW = "EW_YELLOW"
        self.ALL_RED = "ALL_RED"
        
        # State
        self.current_state = self.NS_GREEN
        self.last_switch_time = time.time()
        self.target_state = self.EW_GREEN
        self.waiting_for_clearance = False
        self.clearance_wait_start = None
        
        # Count per sensor (vehicle and pedestrian)
        self.vehicle_counts = {'n': 0, 's': 0, 'e': 0, 'w': 0}
        self.pedestrian_counts = {'n': 0, 's': 0, 'e': 0, 'w': 0}
        
        # Intersection zone status
        self.zone_is_clear = True
        self.zone_ns_clear = True
        self.zone_ew_clear = True
        self.vehicles_in_zone = []
        self.last_zone_update = time.time()
        
        # Sensor failure handling
        self.last_sensor_update = {
            'n': time.time(), 's': time.time(),
            'e': time.time(), 'w': time.time()
        }
        self.sensor_failed = {'n': False, 's': False, 'e': False, 'w': False}
        
        # Subscriptions for lane sensors
        self.create_subscription(
            LogicalCameraImage, '/sensors/north',
            lambda msg: self.sensor_cb(msg, 'n'), 10)
        self.create_subscription(
            LogicalCameraImage, '/sensors/south',
            lambda msg: self.sensor_cb(msg, 's'), 10)
        self.create_subscription(
            LogicalCameraImage, '/sensors/east',
            lambda msg: self.sensor_cb(msg, 'e'), 10)
        self.create_subscription(
            LogicalCameraImage, '/sensors/west',
            lambda msg: self.sensor_cb(msg, 'w'), 10)
        
        # Subscriptions for intersection zone status
        self.create_subscription(
            String, '/intersection/zone_status',
            self.zone_status_cb, 10)
        self.create_subscription(
            Bool, '/intersection/is_clear',
            self.zone_clear_cb, 10)
        self.create_subscription(
            Bool, '/intersection/ns_clear',
            self.ns_clear_cb, 10)
        self.create_subscription(
            Bool, '/intersection/ew_clear',
            self.ew_clear_cb, 10)

        # Publishers
        self.light_pub = self.create_publisher(
            Light, '/traffic_lights/commands', 10)
        self.state_pub = self.create_publisher(
            String, '/traffic_lights/state', 10)
        
        # Individual light state publishers
        # These control which vehicles can proceed
        self.light_north_pub = self.create_publisher(
            String, '/traffic/light/north', 10)
        self.light_south_pub = self.create_publisher(
            String, '/traffic/light/south', 10)
        self.light_east_pub = self.create_publisher(
            String, '/traffic/light/east', 10)
        self.light_west_pub = self.create_publisher(
            String, '/traffic/light/west', 10)

        self.timer = self.create_timer(0.5, self.logic_loop)
        self.get_logger().info("Adaptive Traffic Manager Started with Zone Monitoring")
        
        # Initialize lights to current state
        self.update_lights()

    def zone_status_cb(self, msg):
        """Process detailed zone status from Intersection Zone Monitor."""
        try:
            status = json.loads(msg.data)
            self.zone_is_clear = status.get('is_clear', True)
            self.zone_ns_clear = status.get('ns_clear', True)
            self.zone_ew_clear = status.get('ew_clear', True)
            self.vehicles_in_zone = status.get('vehicles_in_zone', [])
            self.last_zone_update = time.time()
        except json.JSONDecodeError as e:
            self.get_logger().warning(f"Failed to parse zone status: {e}")
    
    def zone_clear_cb(self, msg):
        """Process simple zone clear flag."""
        self.zone_is_clear = msg.data
        self.last_zone_update = time.time()
    
    def ns_clear_cb(self, msg):
        """Process NS clear flag."""
        self.zone_ns_clear = msg.data
    
    def ew_clear_cb(self, msg):
        """Process EW clear flag."""
        self.zone_ew_clear = msg.data

    def sensor_cb(self, msg, dir_key):
        """Process sensor data with validation."""
        if not self._validate_sensor_data(msg, dir_key):
            self.get_logger().warning(
                f"Invalid sensor data from {dir_key} sensor, ignoring")
            return
        
        # Count vehicles (models starting with 'car_')
        vehicle_count = sum(1 for model in msg.models if model.name.startswith('car_'))
        self.vehicle_counts[dir_key] = vehicle_count
        self.last_sensor_update[dir_key] = time.time()
        
        if self.sensor_failed[dir_key]:
            self.sensor_failed[dir_key] = False
            self.get_logger().info(f"Sensor {dir_key} recovered")
    
    def _validate_sensor_data(self, msg, dir_key):
        """Validate sensor data before processing."""
        try:
            if not hasattr(msg, 'models'):
                return False
            _ = len(msg.models)
            for model in msg.models:
                if not hasattr(model, 'name'):
                    return False
            return True
        except Exception as e:
            self.get_logger().error(f"Sensor data validation error: {e}")
            return False
    
    def _check_sensor_failures(self):
        """Check for sensor failures and log warnings."""
        now = time.time()
        for dir_key in ['n', 's', 'e', 'w']:
            elapsed = now - self.last_sensor_update[dir_key]
            if elapsed > self.SENSOR_TIMEOUT and not self.sensor_failed[dir_key]:
                self.sensor_failed[dir_key] = True
                self.get_logger().warning(
                    f"Sensor {dir_key} timeout ({elapsed:.1f}s), "
                    f"using last known count: {self.vehicle_counts[dir_key]}")
    
    def _is_safe_to_transition(self, target_state):
        """Check if it's safe to transition to the target state."""
        now = time.time()
        
        # Check if zone monitor data is stale
        zone_data_stale = (now - self.last_zone_update) > 2.0
        
        if zone_data_stale:
            # Fallback: use time-based clearance
            if self.clearance_wait_start is None:
                self.clearance_wait_start = now
            elapsed_clearance = now - self.clearance_wait_start
            if elapsed_clearance >= self.CLEARANCE_TIME:
                self.get_logger().warning(
                    "Zone data stale, using time-based clearance")
                return True
            return False
        
        # Check zone clearance based on target state
        if target_state == self.NS_GREEN:
            if not self.zone_ns_clear:
                if self.vehicles_in_zone:
                    self.get_logger().debug(
                        f"Waiting for EW vehicles to clear: {self.vehicles_in_zone}")
                return False
        
        elif target_state == self.EW_GREEN:
            if not self.zone_ew_clear:
                if self.vehicles_in_zone:
                    self.get_logger().debug(
                        f"Waiting for NS vehicles to clear: {self.vehicles_in_zone}")
                return False
        
        # Enforce minimum clearance time
        if self.clearance_wait_start is not None:
            elapsed_clearance = now - self.clearance_wait_start
            if elapsed_clearance < self.CLEARANCE_TIME:
                return False
        
        return True

    def _calculate_demand(self):
        """
        Calculate demand per phase according to traffic-flow-logic.txt.
        
        Returns:
            tuple: (NS_total_demand, EW_total_demand, NS_vehicle_demand, EW_vehicle_demand)
        """
        # Aggregate vehicle demand per phase
        NS_vehicle_demand = self.vehicle_counts['n'] + self.vehicle_counts['s']
        EW_vehicle_demand = self.vehicle_counts['e'] + self.vehicle_counts['w']
        
        # Aggregate pedestrian demand per phase
        NS_pedestrian_demand = self.pedestrian_counts['n'] + self.pedestrian_counts['s']
        EW_pedestrian_demand = self.pedestrian_counts['e'] + self.pedestrian_counts['w']
        
        # Weighted total demand (pedestrians have higher priority)
        NS_total_demand = NS_vehicle_demand + self.alpha * NS_pedestrian_demand
        EW_total_demand = EW_vehicle_demand + self.alpha * EW_pedestrian_demand
        
        return NS_total_demand, EW_total_demand, NS_vehicle_demand, EW_vehicle_demand
    
    def _calculate_green_time(self, vehicle_demand, pedestrian_demand):
        """
        Calculate adaptive green time based on demand (from traffic-flow-logic.txt).
        
        Formula:
        green_time = BASE_GREEN + k_vehicle * vehicle_demand + k_ped * pedestrian_demand
        green_time = clamp(green_time, BASE_GREEN, MAX_GREEN)
        
        Args:
            vehicle_demand: Number of vehicles waiting
            pedestrian_demand: Number of pedestrians waiting
        
        Returns:
            float: Green time in seconds
        """
        green_time = (
            self.BASE_GREEN +
            self.k_vehicle * vehicle_demand +
            self.k_ped * pedestrian_demand
        )
        # Clamp between BASE_GREEN and MAX_GREEN
        green_time = max(self.BASE_GREEN, min(green_time, self.MAX_GREEN))
        return green_time
    
    def logic_loop(self):
        """
        Main control loop with adaptive green time and proper state machine.
        Implements state machine from traffic-flow-logic.txt:
        NS_GREEN → NS_YELLOW → ALL_RED_1 → EW_GREEN → EW_YELLOW → ALL_RED_2 → NS_GREEN
        """
        self._check_sensor_failures()
        
        now = time.time()
        elapsed = now - self.last_switch_time
        
        # Calculate demand per phase
        NS_total_demand, EW_total_demand, NS_vehicle_demand, EW_vehicle_demand = self._calculate_demand()
        
        # Get pedestrian demands
        NS_pedestrian_demand = self.pedestrian_counts['n'] + self.pedestrian_counts['s']
        EW_pedestrian_demand = self.pedestrian_counts['e'] + self.pedestrian_counts['w']
        
        previous_state = self.current_state
        next_state = self.current_state
        
        # Track green time for current phase
        if not hasattr(self, 'current_green_time'):
            self.current_green_time = self.BASE_GREEN

        if self.current_state == self.NS_GREEN:
            # Calculate adaptive green time for NS phase
            self.current_green_time = self._calculate_green_time(
                NS_vehicle_demand, NS_pedestrian_demand)
            
            # Switch conditions (from traffic-flow-logic.txt):
            # 1. Minimum green time must pass
            # 2. Either: (a) EW has demand and NS doesn't, OR (b) max green time reached
            should_switch = (
                elapsed >= self.current_green_time or
                (elapsed >= self.MIN_GREEN and EW_total_demand > 0 and NS_total_demand == 0)
            )
            
            if should_switch:
                # Transition to yellow first (Requirement 7.2)
                next_state = self.NS_YELLOW
                self.target_state = self.EW_GREEN
                self.get_logger().info(
                    f"NS_GREEN ending after {elapsed:.1f}s (calculated: {self.current_green_time:.1f}s), "
                    f"demand: NS={NS_total_demand:.1f} EW={EW_total_demand:.1f}")
        
        elif self.current_state == self.NS_YELLOW:
            # Yellow light duration (fixed 3 seconds, Requirement 7.5)
            if elapsed >= self.YELLOW_TIME:
                next_state = self.ALL_RED
                self.waiting_for_clearance = True
                self.clearance_wait_start = now
        
        elif self.current_state == self.EW_GREEN:
            # Calculate adaptive green time for EW phase
            self.current_green_time = self._calculate_green_time(
                EW_vehicle_demand, EW_pedestrian_demand)
            
            # Switch conditions
            should_switch = (
                elapsed >= self.current_green_time or
                (elapsed >= self.MIN_GREEN and NS_total_demand > 0 and EW_total_demand == 0)
            )
            
            if should_switch:
                # Transition to yellow first (Requirement 7.2)
                next_state = self.EW_YELLOW
                self.target_state = self.NS_GREEN
                self.get_logger().info(
                    f"EW_GREEN ending after {elapsed:.1f}s (calculated: {self.current_green_time:.1f}s), "
                    f"demand: NS={NS_total_demand:.1f} EW={EW_total_demand:.1f}")
        
        elif self.current_state == self.EW_YELLOW:
            # Yellow light duration (fixed 3 seconds, Requirement 7.5)
            if elapsed >= self.YELLOW_TIME:
                next_state = self.ALL_RED
                self.waiting_for_clearance = True
                self.clearance_wait_start = now
        
        elif self.current_state == self.ALL_RED:
            # All-red clearance time (fixed 1-2 seconds, from traffic-flow-logic.txt)
            if elapsed >= self.ALL_RED_TIME:
                if self._is_safe_to_transition(self.target_state):
                    next_state = self.target_state
                    self.waiting_for_clearance = False
                    self.clearance_wait_start = None
                    # Reset green time calculation for new phase
                    self.current_green_time = self.BASE_GREEN
                else:
                    if self.vehicles_in_zone:
                         # Log exactly WHICH vehicle is holding up the light
                        self.get_logger().info(
                            f"[SAFETY HOLD] Waiting for vehicles to clear intersection: {self.vehicles_in_zone}. Target State: {self.target_state}")
                    elif not self.zone_is_clear:
                        self.get_logger().info(
                            f"[SAFETY HOLD] Zone reported NOT CLEAR (Sensor active). Target State: {self.target_state}")

        if next_state != self.current_state:
            self.current_state = next_state
            self.last_switch_time = now
            self.update_lights()
            self._log_state_transition(previous_state, next_state, now)

        # Broadcast state
        state_msg = String()
        state_msg.data = self.current_state
        self.state_pub.publish(state_msg)
    
    def _log_state_transition(self, previous_state, new_state, timestamp):
        """Log state transitions with timestamp."""
        self.get_logger().info(
            f"State transition: {previous_state} -> {new_state} "
            f"at {timestamp:.2f}")

    def set_lamp(self, model, color, intensity):
        msg = Light()
        msg.name = f"{model}::housing::{color}_lamp"
        msg.intensity = float(intensity)
        self.light_pub.publish(msg)

    def update_lights(self):
        """Update traffic light states based on current state including yellow."""
        traffic_lights = {
            'north': 'tl_north',
            'south': 'tl_south',
            'east': 'tl_east',
            'west': 'tl_west'
        }
        
        # Determine individual light states
        light_states = {}
        
        if self.current_state == self.ALL_RED:
            for direction, light_name in traffic_lights.items():
                self.set_lamp(light_name, 'green', 0.0)
                self.set_lamp(light_name, 'yellow', 0.0)
                self.set_lamp(light_name, 'red', 1.0)
                light_states[direction] = 'RED'
        
        elif self.current_state == self.NS_GREEN:
            # NS green, EW red
            self.set_lamp(traffic_lights['north'], 'green', 1.0)
            self.set_lamp(traffic_lights['north'], 'yellow', 0.0)
            self.set_lamp(traffic_lights['north'], 'red', 0.0)
            self.set_lamp(traffic_lights['south'], 'green', 1.0)
            self.set_lamp(traffic_lights['south'], 'yellow', 0.0)
            self.set_lamp(traffic_lights['south'], 'red', 0.0)
            self.set_lamp(traffic_lights['east'], 'green', 0.0)
            self.set_lamp(traffic_lights['east'], 'yellow', 0.0)
            self.set_lamp(traffic_lights['east'], 'red', 1.0)
            self.set_lamp(traffic_lights['west'], 'green', 0.0)
            self.set_lamp(traffic_lights['west'], 'yellow', 0.0)
            self.set_lamp(traffic_lights['west'], 'red', 1.0)
            light_states['north'] = 'GREEN'
            light_states['south'] = 'GREEN'
            light_states['east'] = 'RED'
            light_states['west'] = 'RED'
        
        elif self.current_state == self.NS_YELLOW:
            # NS yellow, EW red (Requirement 7.2)
            self.set_lamp(traffic_lights['north'], 'green', 0.0)
            self.set_lamp(traffic_lights['north'], 'yellow', 1.0)
            self.set_lamp(traffic_lights['north'], 'red', 0.0)
            self.set_lamp(traffic_lights['south'], 'green', 0.0)
            self.set_lamp(traffic_lights['south'], 'yellow', 1.0)
            self.set_lamp(traffic_lights['south'], 'red', 0.0)
            self.set_lamp(traffic_lights['east'], 'green', 0.0)
            self.set_lamp(traffic_lights['east'], 'yellow', 0.0)
            self.set_lamp(traffic_lights['east'], 'red', 1.0)
            self.set_lamp(traffic_lights['west'], 'green', 0.0)
            self.set_lamp(traffic_lights['west'], 'yellow', 0.0)
            self.set_lamp(traffic_lights['west'], 'red', 1.0)
            light_states['north'] = 'YELLOW'
            light_states['south'] = 'YELLOW'
            light_states['east'] = 'RED'
            light_states['west'] = 'RED'
        
        elif self.current_state == self.EW_GREEN:
            # EW green, NS red
            self.set_lamp(traffic_lights['north'], 'green', 0.0)
            self.set_lamp(traffic_lights['north'], 'yellow', 0.0)
            self.set_lamp(traffic_lights['north'], 'red', 1.0)
            self.set_lamp(traffic_lights['south'], 'green', 0.0)
            self.set_lamp(traffic_lights['south'], 'yellow', 0.0)
            self.set_lamp(traffic_lights['south'], 'red', 1.0)
            self.set_lamp(traffic_lights['east'], 'green', 1.0)
            self.set_lamp(traffic_lights['east'], 'yellow', 0.0)
            self.set_lamp(traffic_lights['east'], 'red', 0.0)
            self.set_lamp(traffic_lights['west'], 'green', 1.0)
            self.set_lamp(traffic_lights['west'], 'yellow', 0.0)
            self.set_lamp(traffic_lights['west'], 'red', 0.0)
            light_states['north'] = 'RED'
            light_states['south'] = 'RED'
            light_states['east'] = 'GREEN'
            light_states['west'] = 'GREEN'
        
        elif self.current_state == self.EW_YELLOW:
            # EW yellow, NS red (Requirement 7.2)
            self.set_lamp(traffic_lights['north'], 'green', 0.0)
            self.set_lamp(traffic_lights['north'], 'yellow', 0.0)
            self.set_lamp(traffic_lights['north'], 'red', 1.0)
            self.set_lamp(traffic_lights['south'], 'green', 0.0)
            self.set_lamp(traffic_lights['south'], 'yellow', 0.0)
            self.set_lamp(traffic_lights['south'], 'red', 1.0)
            self.set_lamp(traffic_lights['east'], 'green', 0.0)
            self.set_lamp(traffic_lights['east'], 'yellow', 1.0)
            self.set_lamp(traffic_lights['east'], 'red', 0.0)
            self.set_lamp(traffic_lights['west'], 'green', 0.0)
            self.set_lamp(traffic_lights['west'], 'yellow', 1.0)
            self.set_lamp(traffic_lights['west'], 'red', 0.0)
            light_states['north'] = 'RED'
            light_states['south'] = 'RED'
            light_states['east'] = 'YELLOW'
            light_states['west'] = 'YELLOW'
        
        # Publish individual light states
        self._publish_individual_light_states(light_states)
    
    def _publish_individual_light_states(self, light_states):
        """
        Publish individual traffic light states.
        
        Args:
            light_states: Dict mapping direction to state ('RED', 'YELLOW', 'GREEN')
        """
        # Log state changes (only once per update, not per direction)
        self.get_logger().info(f"Traffic light states: N={light_states.get('north', '?')} "
                               f"S={light_states.get('south', '?')} "
                               f"E={light_states.get('east', '?')} "
                               f"W={light_states.get('west', '?')}")
        
        for direction, state in light_states.items():
            msg = String()
            msg.data = state
            
            if direction == 'north':
                self.light_north_pub.publish(msg)
            elif direction == 'south':
                self.light_south_pub.publish(msg)
            elif direction == 'east':
                self.light_east_pub.publish(msg)
            elif direction == 'west':
                self.light_west_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    manager = SmartTrafficManager()
    rclpy.spin(manager)
    manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

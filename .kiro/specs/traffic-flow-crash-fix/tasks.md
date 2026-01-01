# Implementation Plan: Traffic Flow Crash Fix

## Overview

This implementation plan fixes the critical bugs causing intersection crashes. Tasks are prioritized to fix the most severe issues first (velocity axis, position tracking, duplicate vehicles), then add safety enhancements (smooth transitions, red light enforcement), and finally validate the complete system.

## Tasks

- [-] 1. Fix Velocity Axis Mapping (CRITICAL)
  - [x] 1.1 Add velocity axis selection method
    - Create `get_velocity_axis_and_sign(vehicle_name)` method
    - Return tuple of (axis, sign) based on vehicle direction
    - Handle n/s/e/w directions correctly
    - _Requirements: 1.1, 1.2, 1.3, 1.4, 1.5_

  - [x] 1.2 Create apply_velocity method
    - Replace direct velocity publishing with `apply_velocity(vehicle_name, velocity)`
    - Use correct axis (x or y) based on vehicle direction
    - Apply correct sign (positive or negative)
    - _Requirements: 1.1, 1.2, 1.3, 1.4_

  - [x] 1.3 Update timer_callback to use new velocity method
    - Replace all `msg.linear.y = -velocity` with `apply_velocity(name, velocity)`
    - Remove old Twist message creation
    - Test that all 16 vehicles move correctly
    - _Requirements: 1.1, 1.2, 1.3, 1.4, 1.5_

  - [ ] 1.4 Write unit tests for velocity axis selection
    - Test northbound returns ('y', 1)
    - Test southbound returns ('y', -1)
    - Test eastbound returns ('x', 1)
    - Test westbound returns ('x', -1)
    - _Requirements: 1.5_

- [-] 2. Fix Vehicle Position Tracking (CRITICAL)
  - [x] 2.1 Remove VehiclePositionTracker node instantiation
    - Delete `self.position_tracker = VehiclePositionTracker()` line
    - Remove import of VehiclePositionTracker
    - _Requirements: 2.1, 2.2_

  - [x] 2.2 Integrate position tracking into traffic_flow.py
    - Add `self.vehicle_positions = {}` dictionary
    - Add `self.vehicle_lanes = {}` dictionary
    - Create `_initialize_lane_mapping()` method
    - _Requirements: 2.1, 2.3_

  - [x] 2.3 Add position update from sensor callbacks
    - Modify `sensor_cb` to extract and store vehicle positions
    - Update positions for all detected vehicles
    - _Requirements: 2.2, 2.5_

  - [x] 2.4 Implement distance calculation methods
    - Create `get_distance_to_vehicle_ahead(vehicle_name)` method
    - Create `_calculate_lane_distance(lane, my_pos, other_pos)` helper
    - Handle all four lane directions correctly
    - _Requirements: 2.4, 9.1, 9.2_

  - [ ] 2.5 Write unit tests for position tracking
    - Test position updates from sensor data
    - Test distance calculations for each lane direction
    - Test handling of missing position data
    - _Requirements: 2.2, 2.3, 2.4_

- [-] 3. Fix Duplicate Vehicle Positions (CRITICAL)
  - [x] 3.1 Update world file vehicle positions
    - Change car_w2 from (25, 7) to (25, 2)
    - Change car_w3 from (30, 12) to (35, 5)
    - Change car_w4 from (25, 7) to (30, 5)
    - Verify all 16 vehicles have unique positions
    - _Requirements: 3.1, 3.2, 3.3_

  - [x] 3.2 Verify minimum spacing between vehicles
    - Check that all vehicles are at least 5m apart
    - Verify vehicles are in correct lanes
    - _Requirements: 3.2, 3.3_

  - [ ] 3.3 Write validation script for vehicle positions
    - Parse world file and extract all vehicle positions
    - Check for duplicates
    - Check minimum spacing
    - _Requirements: 3.1, 3.2, 3.4_

- [ ] 4. Checkpoint - Test Basic Vehicle Movement
  - Launch simulation
  - Verify all 16 vehicles move in correct directions
  - Verify no startup collisions
  - Verify position tracking works

- [ ] 5. Add Smooth Velocity Transitions
  - [ ] 5.1 Add velocity smoothing state variables
    - Add `self.current_velocities = {}` dictionary
    - Add `self.target_velocities = {}` dictionary
    - Add MAX_ACCELERATION and MAX_DECELERATION constants
    - _Requirements: 6.1, 6.2, 6.3, 6.4_

  - [ ] 5.2 Implement smooth_velocity_transition method
    - Calculate velocity delta
    - Limit acceleration/deceleration rates
    - Update current velocity
    - Return smoothed velocity
    - _Requirements: 6.1, 6.2, 6.3, 6.4_

  - [ ] 5.3 Integrate smoothing into timer_callback
    - Call smooth_velocity_transition before apply_velocity
    - Use smoothed velocity for all vehicle commands
    - _Requirements: 6.1, 6.2, 6.5_

  - [ ] 5.4 Write unit tests for velocity smoothing
    - Test acceleration limiting
    - Test deceleration limiting
    - Test smooth transitions
    - _Requirements: 6.3, 6.4_

- [ ] 6. Strengthen Red Light Enforcement
  - [ ] 6.1 Add stop line distance calculation
    - Create `get_distance_to_stop_line(vehicle_name)` method
    - Calculate distance based on vehicle direction
    - Return negative if past stop line
    - _Requirements: 5.3, 5.4_

  - [ ] 6.2 Enhance red light checking logic
    - Create `should_stop_for_red_light(vehicle_name)` method
    - Check light state, detection zone, and stop line distance
    - Return (should_stop, reason) tuple
    - _Requirements: 5.1, 5.2, 5.3, 5.4_

  - [ ] 6.3 Update timer_callback with enhanced logic
    - Use should_stop_for_red_light for all vehicles
    - Log reasons for stopping/moving
    - Ensure 100% red light compliance
    - _Requirements: 5.1, 5.2, 5.4, 5.5_

  - [ ] 6.4 Write property tests for red light enforcement
    - **Property 1: Red Light Stopping**
    - *For any* vehicle in detection zone with red light, velocity SHALL be zero
    - **Validates: Requirements 5.1, 5.2, 5.4**

- [ ] 7. Improve Sensor Direction Mapping Documentation
  - [ ] 7.1 Add comprehensive comments to sensor subscriptions
    - Document which sensor detects which traffic direction
    - Explain why north sensor maps to 's' direction
    - Add visual diagram in comments
    - _Requirements: 8.1, 8.2, 8.3, 8.4, 8.5_

  - [ ] 7.2 Create sensor mapping constants
    - Define SENSOR_TO_DIRECTION mapping dictionary
    - Use constants instead of hardcoded strings
    - _Requirements: 8.1_

  - [ ] 7.3 Write unit tests for sensor mapping
    - Test that north sensor data goes to southbound control
    - Test all four sensor-direction mappings
    - _Requirements: 8.2, 8.3, 8.4, 8.5_

- [ ] 8. Checkpoint - Test Red Light Compliance
  - Launch simulation
  - Observe vehicles stopping at red lights
  - Verify no vehicles run red lights
  - Check smooth deceleration

- [ ] 9. Enhance Queue Management
  - [ ] 9.1 Verify collision avoidance integration
    - Ensure distance_to_vehicle_ahead is used
    - Check SAFE_FOLLOWING_DISTANCE enforcement
    - Test STOP_DISTANCE emergency braking
    - _Requirements: 9.1, 9.2, 9.3_

  - [ ] 9.2 Add queue formation logging
    - Log when vehicles form queues
    - Log spacing between queued vehicles
    - _Requirements: 9.4_

  - [ ] 9.3 Test queue release behavior
    - Verify vehicles resume in order when light turns green
    - Check that following vehicles wait for leader
    - _Requirements: 9.5_

  - [ ] 9.4 Write property tests for queue management
    - **Property 2: Safe Following Distance**
    - *For any* two vehicles in same lane, distance SHALL be >= 5m or velocity SHALL be reduced
    - **Validates: Requirements 9.1, 9.2, 9.3**

- [ ] 10. Add Intersection Collision Prevention Validation
  - [ ] 10.1 Verify intersection zone monitor integration
    - Check that zone status is received
    - Verify NS_clear and EW_clear flags work
    - Test zone conflict detection
    - _Requirements: 7.1, 7.2, 7.3_

  - [ ] 10.2 Add near-miss detection and logging
    - Calculate minimum distance between vehicles
    - Log when vehicles come within 2m
    - Track near-miss events
    - _Requirements: 7.5_

  - [ ] 10.3 Write property tests for intersection safety
    - **Property 3: No Intersection Collisions**
    - *For any* two vehicles from perpendicular directions, they SHALL NOT be in intersection zone simultaneously
    - **Validates: Requirements 7.1, 7.2, 7.4**

- [ ] 11. Comprehensive System Testing
  - [ ] 11.1 Create 10-minute crash-free test
    - Run simulation for 10 minutes
    - Monitor for collisions
    - Log all vehicle interactions
    - Verify zero crashes
    - _Requirements: 7.4, 10.5_

  - [ ] 11.2 Create red light compliance test
    - Monitor all vehicles for 5 minutes
    - Track red light violations
    - Verify 100% compliance
    - _Requirements: 5.4, 10.4_

  - [ ] 11.3 Create queue formation test
    - Observe vehicles queuing at red lights
    - Measure spacing between vehicles
    - Verify orderly release on green
    - _Requirements: 9.4, 9.5_

  - [ ] 11.4 Write integration tests
    - Test complete NS_GREEN → EW_GREEN cycle
    - Test ALL_RED transitions
    - Test intersection clearance
    - _Requirements: 7.3, 10.5_

- [ ] 12. Add Comprehensive Logging and Debugging
  - [ ] 12.1 Add velocity command logging
    - Log velocity changes with reasons
    - Include vehicle name, old velocity, new velocity, reason
    - _Requirements: 10.1_

  - [ ] 12.2 Add red light event logging
    - Log when vehicles stop for red lights
    - Log when vehicles resume on green
    - _Requirements: 10.1_

  - [ ] 12.3 Add collision avoidance logging
    - Log when vehicles slow for following distance
    - Log emergency stops
    - _Requirements: 10.1_

  - [ ] 12.4 Create debug visualization script
    - Show vehicle positions in real-time
    - Display detection zones
    - Show traffic light states
    - _Requirements: 10.1_

- [ ] 13. Final Validation and Documentation
  - [ ] 13.1 Run all unit tests
    - Verify all tests pass
    - Fix any failing tests
    - _Requirements: 10.1, 10.2_

  - [ ] 13.2 Run all integration tests
    - Verify complete traffic cycles work
    - Check collision avoidance
    - Validate intersection safety
    - _Requirements: 10.3, 10.4_

  - [ ] 13.3 Run full simulation test
    - 10-minute crash-free test
    - Red light compliance test
    - Queue management test
    - _Requirements: 10.5_

  - [ ] 13.4 Document system behavior
    - Create user guide for running simulation
    - Document expected behavior
    - List known limitations
    - _Requirements: All_

- [ ] 14. Final Checkpoint - System Validated
  - All tests pass
  - Zero crashes in 10-minute simulation
  - 100% red light compliance
  - Smooth, realistic traffic flow

## Notes

- Tasks 1-3 are CRITICAL and must be completed first
- Checkpoint after task 4 ensures basic functionality before adding enhancements
- Tasks 5-7 add safety and realism improvements
- Tasks 8-10 focus on collision prevention and validation
- Tasks 11-13 provide comprehensive testing and validation
- Each task references specific requirements for traceability
- Property tests validate universal correctness properties
- Integration tests validate complete system behavior

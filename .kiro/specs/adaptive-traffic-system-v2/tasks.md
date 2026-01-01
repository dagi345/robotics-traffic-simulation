# Implementation Plan: Adaptive Traffic System V2

## Overview

This implementation plan completes the Smart Adaptive Traffic Light System by fixing intersection collisions, adding pedestrian simulation, and implementing traffic flow metrics. Tasks are organized to fix critical safety issues first, then add pedestrian features, and finally implement metrics comparison.

## Tasks

- [x] 1. Fix Intersection Collision Logic
  - [x] 1.1 Create Intersection Zone Monitor component
    - Create new file `intersection_zone_monitor.py`
    - Subscribe to intersection zone sensor
    - Track vehicles entering/exiting the zone
    - Publish zone clearance status
    - _Requirements: 1.4, 1.6, 10.1, 10.2, 10.3_

  - [x] 1.2 Add intersection zone sensor to world file
    - Add logical camera sensor covering intersection zone (-13 to +13 in X and Y)
    - Position sensor above intersection looking down
    - Configure appropriate range and update rate
    - _Requirements: 10.1_

  - [x] 1.3 Integrate zone monitor with Traffic Manager
    - Subscribe to zone status in Traffic Manager
    - Block phase transitions when zone not clear
    - Add clearance verification before green phases
    - _Requirements: 1.4, 1.6_

  - [x] 1.4 Update Flow Controller for intersection safety
    - Add zone awareness to vehicle control
    - Slow vehicles approaching non-clear intersection
    - Ensure vehicles stop before entering during conflicting phase
    - _Requirements: 1.1, 1.2, 1.3_

  - [x] 1.5 Write property tests for intersection safety
    - **Property 1: Conflicting Traffic Exclusion**
    - **Property 2: Intersection Clearance Before Phase Change**
    - **Property 22: Intersection Zone Vehicle Tracking**
    - **Validates: Requirements 1.1, 1.2, 1.3, 1.4, 1.6, 10.2, 10.3**

- [x] 2. Checkpoint - Verify intersection collision fix
  - Ensure all tests pass, ask the user if questions arise.
  - Run simulation and verify no intersection collisions

- [ ] 3. Add Yellow Light Support
  - [ ] 3.1 Update traffic light model for yellow lamp
    - Modify `detailed_traffic_light/model.sdf` to include yellow lamp
    - Add yellow lamp visual and light elements
    - _Requirements: 7.1_

  - [ ] 3.2 Extend Traffic Manager state machine
    - Add NS_YELLOW and EW_YELLOW states
    - Implement yellow light timing (default 3 seconds)
    - Update state transitions to include yellow phases
    - _Requirements: 7.2, 7.5_

  - [ ] 3.3 Update Flow Controller for yellow light response
    - Detect yellow light state
    - Reduce vehicle speed on yellow (50% slowdown)
    - _Requirements: 7.3, 7.4_

  - [ ] 3.4 Write property tests for yellow light
    - **Property 18: Yellow Light Duration**
    - **Property 19: Vehicle Slowdown on Yellow**
    - **Validates: Requirements 7.2, 7.3, 7.4, 7.5**

- [x] 4. Add Pedestrian Actors to World
  - [x] 4.1 Verify gazebo-ros-actor-plugin is available
    - Check that libgazebo_ros_actor_plugin.so exists in install directory
    - Review example world file at `ros2_ws/src/gazebo-ros-actor-plugin/config/worlds/move_actor.world`
    - Understand plugin parameters: follow_mode, vel_topic, animation_factor, linear_velocity
    - _Requirements: 2.3_

  - [x] 4.2 Add pedestrian actors to world file
    - Add 6 pedestrian actors per crosswalk (24 total)
    - **North crosswalk waiting areas**: 
      - West side: x=-4 to -6, y=14 to 16 (3 actors)
      - East side: x=4 to 6, y=14 to 16 (3 actors)
    - **South crosswalk waiting areas**:
      - West side: x=-4 to -6, y=-14 to -16 (3 actors)
      - East side: x=4 to 6, y=-14 to -16 (3 actors)
    - **East crosswalk waiting areas**:
      - North side: x=14 to 16, y=4 to 6 (3 actors)
      - South side: x=14 to 16, y=-4 to -6 (3 actors)
    - **West crosswalk waiting areas**:
      - North side: x=-14 to -16, y=4 to 6 (3 actors)
      - South side: x=-14 to -16, y=-4 to -6 (3 actors)
    - Use DoctorFemaleWalk model: `<uri>model://DoctorFemaleWalk</uri>`
    - Configure velocity control plugin for each actor with unique topic: `/pedestrian_<direction>_<side>_<num>/cmd_vel`
    - Set animation_factor=4.0, linear_velocity=1.2, angular_velocity=2.5
    - _Requirements: 2.1, 2.2, 2.3_

  - [x] 4.3 Update pedestrian waiting area sensors (already exist, verify coverage)
    - Verify existing sensors cover new pedestrian positions:
      - sensor_pedestrian_north: covers y=7 to y=14, x=-6 to x=6
      - sensor_pedestrian_south: covers y=-14 to y=-7, x=-6 to x=6
      - sensor_pedestrian_east: covers x=7 to x=14, y=-6 to y=6
      - sensor_pedestrian_west: covers x=-14 to x=-7, y=-6 to y=6
    - Adjust sensor positions/FOV if needed to ensure all waiting areas are covered
    - _Requirements: 3.1, 3.5_

  - [x] 4.4 Update bridge configuration for pedestrian actors
    - Add 24 pedestrian cmd_vel topics to bridge_config.yaml
    - Map Gazebo topics to ROS topics for each pedestrian actor
    - Example: `/model/pedestrian_north_west_1/cmd_vel` → `/pedestrian_north_west_1/cmd_vel`
    - _Requirements: 2.3, 5.5_

  - [ ] 4.5 Write unit tests for pedestrian configuration
    - Test: Verify 24 pedestrian actors exist in world file
    - Test: Verify each actor has unique name and position
    - Test: Verify each actor has velocity control plugin configured
    - Test: Verify pedestrian sensors cover all waiting areas
    - Test: Verify bridge config includes all 24 pedestrian cmd_vel topics
    - **Validates: Requirements 2.1, 2.2, 2.3, 3.1, 3.5**

- [ ] 5. Implement Pedestrian Detection System
  - [ ] 5.1 Create Pedestrian Detector component
    - Create new file `pedestrian_detector.py`
    - Subscribe to pedestrian sensors at each crosswalk
    - Count waiting pedestrians per crosswalk
    - Publish pedestrian demand to Traffic Manager
    - _Requirements: 3.2, 3.3, 3.4_

  - [ ] 5.2 Update bridge configuration for pedestrian topics
    - Add pedestrian sensor topics to bridge config
    - Add pedestrian demand topic
    - _Requirements: 3.1_

  - [ ] 5.3 Write property tests for pedestrian detection
    - **Property 6: Pedestrian Count Accuracy**
    - **Validates: Requirements 3.3, 3.4**

- [ ] 6. Checkpoint - Verify pedestrian detection
  - Ensure all tests pass, ask the user if questions arise.
  - Verify pedestrians are detected in simulation

- [ ] 7. Implement Pedestrian Crossing Logic in Traffic Manager
  - [ ] 7.1 Add pedestrian crossing states to state machine
    - Add NS_PED_CROSSING and EW_PED_CROSSING states
    - Define valid transitions for pedestrian phases
    - _Requirements: 6.1_

  - [ ] 7.2 Implement pedestrian demand handling
    - Subscribe to pedestrian demand topic
    - Track pedestrian wait times
    - Trigger crossing phase when demand exists (max 60s wait)
    - _Requirements: 4.1_

  - [ ] 7.3 Implement pedestrian phase timing
    - Configure Safe_Crossing_Time (default 15 seconds)
    - Configure Clearance_Interval (default 2 seconds)
    - Implement crossing phase duration logic
    - _Requirements: 4.3, 4.5_

  - [ ] 7.4 Implement crosswalk clearance check
    - Subscribe to crosswalk occupancy sensor
    - Extend red light if pedestrians still crossing
    - Transition to yellow/green when clear
    - _Requirements: 4.7_

  - [ ] 7.5 Add pedestrian signal state publisher
    - Publish walk/don't-walk state for each crosswalk
    - Synchronize with Traffic Manager state
    - _Requirements: 11.4, 11.5_

  - [ ] 7.6 Write property tests for pedestrian crossing logic
    - **Property 7: Pedestrian Crossing Initiation Time Bound**
    - **Property 8: Pedestrian Phase Traffic Stop**
    - **Property 9: Clearance Interval Before Walk Signal**
    - **Property 11: Crossing Phase Duration**
    - **Property 12: Yellow-to-Green Transition After Crossing**
    - **Property 13: Red Light Extension for Occupied Crosswalk**
    - **Property 17: No Conflicting Pedestrian-Vehicle Movements**
    - **Validates: Requirements 4.1, 4.2, 4.3, 4.5, 4.6, 4.7, 6.2, 6.3, 6.5, 6.6**

- [ ] 8. Implement Pedestrian Controller
  - [ ] 8.1 Create Pedestrian Controller component
    - Create new file `pedestrian_controller.py`
    - Subscribe to pedestrian signal state
    - Manage pedestrian actor states (waiting, crossing, crossed)
    - _Requirements: 5.5_

  - [ ] 8.2 Implement pedestrian movement commands
    - Publish velocity commands to pedestrian actors
    - Use varied speeds (1.0-1.5 m/s) for natural behavior
    - Command zero velocity when waiting
    - _Requirements: 2.5, 2.6, 5.1, 5.4_

  - [ ] 8.3 Implement pedestrian path following
    - Calculate crossing direction based on crosswalk
    - Track pedestrian position during crossing
    - Stop pedestrian when reached opposite side
    - _Requirements: 5.2, 5.3_

  - [ ] 8.4 Update bridge configuration for pedestrian control
    - Add pedestrian actor cmd_vel topics to bridge config
    - _Requirements: 5.5_

  - [ ] 8.5 Write property tests for pedestrian control
    - **Property 4: Pedestrian Crossing Speed Bounds**
    - **Property 5: Pedestrian Speed Variation**
    - **Property 10: Walk Signal Triggers Pedestrian Movement**
    - **Property 14: Pedestrian Stop at Destination**
    - **Property 15: Don't Walk Keeps Pedestrians Stationary**
    - **Property 16: Pedestrian Spacing**
    - **Validates: Requirements 2.5, 2.6, 4.4, 5.1, 5.3, 5.4, 5.6**

- [ ] 9. Checkpoint - Verify pedestrian crossing system
  - Ensure all tests pass, ask the user if questions arise.
  - Run simulation and verify pedestrians cross safely

- [ ] 10. Add Pedestrian Signal Visualization
  - [ ] 10.1 Create pedestrian signal models
    - Create simple walk/don't-walk signal model
    - Add controllable light elements (green walk, red stop)
    - _Requirements: 11.1_

  - [ ] 10.2 Add pedestrian signals to world file
    - Position signals at each crosswalk
    - Configure signal control topics
    - _Requirements: 11.1_

  - [ ] 10.3 Implement signal control in Traffic Manager
    - Publish signal commands synchronized with state
    - Update signals on state transitions
    - _Requirements: 11.2, 11.3, 11.4_

  - [ ] 10.4 Write property tests for signal visualization
    - **Property 23: Pedestrian Signal State Consistency**
    - **Validates: Requirements 11.2, 11.3, 11.4**

- [ ] 11. Implement Metrics Collection System
  - [ ] 11.1 Create Metrics Collector component
    - Create new file `metrics_collector.py`
    - Subscribe to vehicle positions, light states, pedestrian crossings
    - Track entry/exit times for wait time calculation
    - _Requirements: 8.1, 8.2, 8.3, 8.4, 8.5_

  - [ ] 11.2 Implement vehicle metrics
    - Calculate average wait time per direction
    - Calculate throughput (vehicles per minute)
    - Track maximum queue length per direction
    - _Requirements: 8.1, 8.2, 8.3_

  - [ ] 11.3 Implement pedestrian metrics
    - Calculate average pedestrian wait time
    - Count total pedestrian crossings
    - _Requirements: 8.4, 8.5_

  - [ ] 11.4 Add metrics publisher
    - Publish metrics to ROS topic
    - Include timestamp and mode indicator
    - _Requirements: 8.6_

  - [ ] 11.5 Write property tests for metrics
    - **Property 20: Metrics Calculation Correctness**
    - **Validates: Requirements 8.1, 8.2, 8.3, 8.4, 8.5**

- [ ] 12. Implement Fixed-Time Baseline Mode
  - [ ] 12.1 Add mode parameter to Traffic Manager
    - Add configurable mode parameter (adaptive/fixed)
    - Log active mode for metric correlation
    - _Requirements: 9.1, 9.5_

  - [ ] 12.2 Implement fixed-time cycle logic
    - Implement fixed phase durations (configurable, default 60s cycle)
    - Cycle through phases regardless of traffic
    - Include fixed pedestrian phases
    - _Requirements: 9.2, 9.3, 9.4_

  - [ ] 12.3 Write property tests for fixed-time mode
    - **Property 21: Fixed-Time Mode Constant Intervals**
    - **Validates: Requirements 9.2, 9.3**

- [ ] 13. Checkpoint - Verify metrics and modes
  - Ensure all tests pass, ask the user if questions arise.
  - Run simulation in both modes and compare metrics

- [ ] 14. Integration and Final Testing
  - [ ] 14.1 Update launch file for all new components
    - Add Intersection Zone Monitor node
    - Add Pedestrian Detector node
    - Add Pedestrian Controller node
    - Add Metrics Collector node
    - _Requirements: All_

  - [ ] 14.2 Run full integration test
    - Launch complete simulation
    - Verify no vehicle collisions at intersection
    - Verify pedestrians cross safely
    - Verify metrics are collected
    - _Requirements: All_

  - [ ] 14.3 Run comparison test
    - Run simulation in fixed-time mode for 5 minutes
    - Run simulation in adaptive mode for 5 minutes
    - Compare metrics (wait times, throughput, queue lengths)
    - Document results
    - _Requirements: 8.7, 9.1_

- [ ] 15. Final Checkpoint - Complete system validation
  - Ensure all tests pass, ask the user if questions arise.
  - Verify system meets all requirements

## Notes

- All tasks including tests are required for comprehensive validation
- Critical safety tasks (1, 3, 7) should be completed first
- Pedestrian simulation (4, 5, 8) can be done in parallel with safety fixes
- Metrics (11, 12) can be added last as they don't affect core functionality
- Each checkpoint verifies incremental progress before moving forward
- Property tests validate universal correctness properties using Hypothesis
- Unit tests validate specific examples and edge cases
- All 23 correctness properties from the design document must be tested

# Person Detection & Tracking Module

This module is responsible only for detecting people and tracking the current target.

## 1. Module Scope

### What this module should do
- Detect people in camera or depth images.
- Track a selected target over time.
- Provide bounding boxes, track IDs, and optional 2D/3D positions.
- Help the task manager decide who is the current guest or host.

### What this module should not do
- Do not perform navigation.
- Do not perform speech recognition.
- Do not allocate seats.
- Do not control the arm or gripper.
- Do not decide task logic.

## 2. Inputs

Typical inputs:
- RGB image stream
- Depth image stream
- Camera info
- Optional odometry for coordinate conversion

## 3. Outputs

Recommended outputs:
- `person_id`
- `bbox` / `keypoints`
- `track_id`
- `confidence`
- Optional `position_2d`
- Optional `position_3d`

## 4. ROS Interface Suggestions

### Subscriptions
- `/camera/rgb/image_raw`
- `/camera/depth/image_raw`
- `/camera/rgb/camera_info`

### Publications
- `/person_tracker/targets`
- `/person_tracker/current_target`
- `/person_tracker/debug_image`

### Optional Services / Actions
- `/person_tracker/reset`
- `/person_tracker/select_target`

## 5. Target Selection Rule

For the HRI receptionist task, the tracker should prioritize:
1. The person closest to the door region when the robot arrives.
2. The person the robot is currently facing.
3. The person with the most stable detection result.

## 6. State Machine Handoff

The tracking module should send results to the task manager only.

Example handoff:
- Detection node finds a person at the door.
- Tracker keeps following that person.
- Task manager receives the selected target and starts greeting.

## 7. Implementation Priority

1. Person detection.
2. Multi-person tracking.
3. Target selection at the door.
4. Stable ID maintenance across frames.
5. Optional 2D/3D position estimation.

## 8. Acceptance Criteria

The module is acceptable if it can:
- Detect multiple people in the same scene.
- Keep one selected target stable.
- Return the target information continuously.
- Avoid mixing guests with unrelated people.

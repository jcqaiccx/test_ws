# HRI Task 1 Blueprint (Receptionist)

This document turns the rulebook into an implementable ROS workflow.

## 1. State Machine Table

| State | Purpose | Trigger In | Core Actions | Success Out | Failure Handling |
|---|---|---|---|---|---|
| S0_INIT | Initialize all runtime data | Node start | Load waypoints, reset guest memory, reset seat map | S1_WAIT_ARRIVAL | Retry init and alarm |
| S1_WAIT_ARRIVAL | Wait for guest arrival signal | Timer / bell / schedule | Optional bell detection, then decide to check door | S2_GO_TO_DOOR | Timeout then proactive door check |
| S2_GO_TO_DOOR | Navigate to door to greet | Goal request | Send nav goal to `door_wp`, align gaze to moving direction | S3_LOCK_GUEST | Replan once, else fallback manual waypoint |
| S3_LOCK_GUEST | Select the correct guest at door | Person detections | Detect/track nearest standing person in door ROI, face target | S4_ASK_NAME_DRINK | Repeat scan and short rotate sweep |
| S4_ASK_NAME_DRINK | Collect name and favorite drink | Person locked | Ask questions, call ASR, parse entities, persist memory | S5_GUIDE_TO_SEAT | One retry with concise re-ask |
| S5_GUIDE_TO_SEAT | Guide guest to free seat | Guest info ready | Choose free seat, navigate, ask guest to sit, mark seat occupied | S1_WAIT_ARRIVAL or S6_INTRODUCE | If seat blocked, choose next free seat |
| S6_INTRODUCE | Introduce two guests to each other | Two guests seated | Look at guest A and introduce guest B, then swap | S7_RECEIVE_BAG | If missing data, use minimal safe intro |
| S7_RECEIVE_BAG | Receive bag from second guest | Introduction done | Ask for handover, move arm to handover pose, grasp check | S8_FOLLOW_HOST | Retry grasp with one alternate pose |
| S8_FOLLOW_HOST | Follow host to drop zone | Bag held | Person-follow host while carrying bag | S9_PLACE_BAG | If host lost, local search and reacquire |
| S9_PLACE_BAG | Place bag and finish | Drop context available | Put bag at indicated area, release gripper, announce done | TASK_DONE | If place fails, reposition and retry once |

## 2. Step-by-Step Build Plan (Implementation Order)

1. Build `hri_task_manager` state machine shell with S0-S9 transitions only.
2. Add navigation action client (`move_base`) and waypoint YAML loader.
3. Add seat manager and guest memory store.
4. Integrate voice service (`/voice/recognize`) and TTS topic (`/tts_text`).
5. Add entity parsing for `name` and `drink` from ASR text.
6. Integrate person tracking topic for target lock and gaze target.
7. Integrate bag handover and follow-host interfaces.
8. Add logging/timeout/retry policy per state.
9. Dry-run in simulation with mocked outputs, then on robot.

## 3. State Input/Output Contract

| State | Required Inputs | Produced Outputs |
|---|---|---|
| S0_INIT | Waypoints, seat config | `guest_count=0`, `seat_status`, `current_state=S1` |
| S1_WAIT_ARRIVAL | Bell event or scheduler tick | `arrival_detected` |
| S2_GO_TO_DOOR | `door_wp` | `nav_result`, `pose_at_door` |
| S3_LOCK_GUEST | Person detections/tracks | `current_guest_track_id`, `guest_pose` |
| S4_ASK_NAME_DRINK | ASR text + parse result | `guest.name`, `guest.drink`, optional features |
| S5_GUIDE_TO_SEAT | seat map + nav + track | `guest.seat_id`, updated `seat_status` |
| S6_INTRODUCE | two guest memories + seat poses | spoken intro events + gaze switch events |
| S7_RECEIVE_BAG | arm state + gripper feedback | `bag_in_gripper=true/false` |
| S8_FOLLOW_HOST | host track + local planner | `follow_ok`, `drop_zone_reached` |
| S9_PLACE_BAG | place pose + arm control | `task_done=true` |

## 4. Recommended ROS Node Split

- `rcj_home/hri_task_manager.py`: Global state machine and orchestration.
- `rcj_home/guest_memory.py`: In-memory guest data store and query API.
- `rcj_home/seat_manager.py`: Seat allocation and occupancy management.
- `robot_voice/iat_publish` (already refactored): Voice recognition service `/voice/recognize`.
- `robot_voice/tts_subscribe`: Speech output subscriber for `/tts_text`.
- `vision/person_tracker`: Person detection/tracking output.
- `head_controller/look_at_target`: Gaze/head orientation control.
- `arm_controller/handover_place`: Bag handover and placement.

## 5. Minimal Interface Definitions

- Subscribed by manager:
  - `/person_tracker/targets` (custom or standard tracked person msg)
  - `/follow_host/status` (`std_msgs/String` or custom)
  - `/arm/status` (`std_msgs/String` or custom)
- Published by manager:
  - `/tts_text` (`std_msgs/String`)
  - `/head/look_at` (`geometry_msgs/PointStamped` or custom)
  - `/task/state` (`std_msgs/String`)
- Called by manager:
  - `/voice/recognize` (`robot_voice/RecognizeVoice`)
  - `/move_base` (`move_base_msgs/MoveBaseAction`)

## 6. Acceptance Checklist (Task 1 Main Score)

- Robot proactively goes to door for each guest.
- Robot asks and stores each guest's `name` and `favorite drink`.
- Robot guides each guest to a different free seat.
- Robot introduces guest A and B with correct name+drink.
- Gaze behavior matches speaking and introducing requirements.
- Robot receives bag hand-to-hand from guest 2.
- Robot follows host and places bag at destination.

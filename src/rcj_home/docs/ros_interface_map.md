# HRI Task 1 ROS Interface Map

This document maps each task state to the ROS interfaces that should be used in implementation.

## 1. Overall Node Split

| Node | Responsibility | Main Interfaces |
|---|---|---|
| `rcj_home/hri_task_manager.py` | Global state machine and orchestration | `/task/state`, `/tts_text`, `/tts_done`, `/voice/recognize`, navigation and perception callbacks |
| `robot_voice/iat_publish` | Speech recognition service | `/voice/recognize` service |
| `robot_voice/tts_subscribe` | Speech synthesis playback | `/tts_text` topic, `/tts_done` topic |
| `navigation` / `move_base` | Autonomous navigation | `move_base` action |
| `person_tracker` | Human detection and tracking | `/person_tracker/targets` or custom tracking topic |
| `head_controller` | Gaze / head orientation control | `/head/look_at` or pan-tilt control topic |
| `arm_controller` | Bag handover / place motion | arm / gripper topics or services |
| `follow_controller` | Follow host behavior | follow status topic / action |

## 2. State-to-Interface Mapping

### S0_INIT
- Purpose: initialize map, seats, guest memory, and waypoints.
- Inputs: YAML config / parameters.
- Outputs: state ready signal.
- Interfaces:
  - `rosparam` / YAML for waypoints
  - `/task/state` publish

### S1_GO_TO_DOOR / S1_WAIT_ARRIVAL
- Purpose: navigate to the door and wait for the next guest.
- Inputs: door waypoint.
- Outputs: arrival at door.
- Interfaces:
  - `move_base` action goal to `door_wp`
  - `/head/look_at` or heading control toward the goal direction
  - optional bell detector topic for arrival trigger

### S2_LOCK_GUEST
- Purpose: select the arriving guest at the door and ignore other people.
- Inputs: person detections, bounding boxes, track IDs.
- Outputs: current guest lock result.
- Interfaces:
  - `/person_tracker/targets`
  - `/head/look_at`
  - optional face/person re-identification topic

### S3_ASK_NAME_DRINK / S4_ASK_INFO
- Purpose: ask the guest for name and favorite drink.
- Inputs: TTS prompt, microphone audio.
- Outputs: ASR text.
- Interfaces:
  - `/tts_text` publish question
  - `/voice/recognize` service call
  - `/tts_done` wait before opening microphone

### S4_PARSE_AND_STORE
- Purpose: parse the ASR text and store extracted guest information.
- Inputs: ASR result text.
- Outputs: `guest.name`, `guest.drink`.
- Interfaces:
  - internal regex / dictionary parser
  - `/task/state` publish parsed result for debugging

### S5_GUIDE_TO_SEAT
- Purpose: guide the guest to a free seat.
- Inputs: seat map, guest info, seat waypoint.
- Outputs: seat assignment and occupancy update.
- Interfaces:
  - `move_base` action goal to `seat_wp_x`
  - `/tts_text` guidance prompt
  - optional occupancy sensor / vision seat check

### S6_WAIT_SECOND_GUEST
- Purpose: repeat the same flow for the second guest.
- Inputs: next door arrival.
- Outputs: second guest stored.
- Interfaces:
  - same as S1-S5

### S7_INTRODUCE_GUESTS
- Purpose: introduce the two guests to each other after both are seated.
- Inputs: two guest memories and seat locations.
- Outputs: introduction speech and gaze switching.
- Interfaces:
  - `/tts_text` introduction message
  - `/head/look_at` toward the guest being referenced
  - guest memory store

### S8_RECEIVE_BAG
- Purpose: receive the bag from the second guest by handover.
- Inputs: handover pose, gripper state.
- Outputs: bag grasp success.
- Interfaces:
  - arm/gripper control topic or service
  - gripper state feedback topic
  - `/tts_text` prompt for handover

### S9_FOLLOW_HOST
- Purpose: follow the host to the placement location.
- Inputs: host track, follow policy.
- Outputs: follow success and placement zone reached.
- Interfaces:
  - `/person_tracker/targets`
  - follow controller action/topic
  - `/head/look_at` toward host

### S10_PLACE_BAG
- Purpose: place the bag at the indicated location and finish.
- Inputs: place pose, arm state.
- Outputs: bag released, task complete.
- Interfaces:
  - arm/place service or trajectory
  - gripper release control
  - `/tts_text` completion message

## 3. Minimal Message Contracts

### `/task/state`
- Type: `std_msgs/String`
- Values: `S0_INIT`, `S1_ASK_INFO`, `S2_OUTPUT_INFO`, etc.

### `/tts_text`
- Type: `std_msgs/String`
- Meaning: text to be spoken.

### `/tts_done`
- Type: `std_msgs/String`
- Meaning: playback finished; payload can echo the spoken text.

### `/voice/recognize`
- Type: `robot_voice/RecognizeVoice`
- Request: `timeout`
- Response: `text`

## 4. Recommended Implementation Order

1. Navigation to door and seat waypoints.
2. Person detection and target lock at the door.
3. Guest memory and seat allocation.
4. Gaze control during speaking and introducing.
5. Bag handover and placement.
6. Host following.
7. Optional bell detection and automatic door opening.

## 5. Notes

- Keep navigation, perception, speech, and arm control as separate modules.
- The state machine should only orchestrate; it should not directly implement low-level motion.
- Always wait for `/tts_done` before opening the microphone for ASR.

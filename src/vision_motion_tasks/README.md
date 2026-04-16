# vision_motion_tasks

Motion control package for gaze lock and person following.

## Nodes

- `gaze_controller.py`
  - Input: `/vision/person_target`
  - Input: `vision` commands (`start_gaze`, `stop_gaze`)
  - Output: `/cmd_vel_mux/input/navi` (angular only)
  - Output: `vision_back` (`gaze_lost`, `gaze_tracking`, `gaze_locked`)

- `person_follow_controller.py`
  - Input: `/vision/person_target`
  - Input: `/camera/depth/image_raw`
  - Input: `pid` commands (`follow_people`, `pid_stop`)
  - Output: `/cmd_vel_mux/input/navi` (linear + angular)
  - Output: `pid_back` (`follow_people_finish` on stop)

## Launch

```bash
roslaunch vision_motion_tasks motion_pipeline.launch
```

Full pipeline launch:

```bash
roslaunch vision_motion_tasks vision_tasks_all.launch
```

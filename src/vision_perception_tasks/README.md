# vision_perception_tasks

Perception package for competition tasks.

## Nodes

- `person_detector.py`
  - Input: `~image_topic` (default: `/camera/rgb/image_color`)
  - Output: `~person_topic` (default: `/vision/person_target`, JSON in `std_msgs/String`)
  - Output: `~debug_topic` (default: `/vision/person_debug`, `sensor_msgs/Image`)

- `object_shelf_detector.py`
  - Input: `~image_topic` (default: `/camera/rgb/image_color`)
  - Output: `~object_topic` (default: `/vision/object_detections`, JSON in `std_msgs/String`)
  - Output: `~vision_back_topic` (default: `vision_back`, emits `left/right/center` by best object)
  - Output: `~debug_topic` (default: `/vision/object_debug`, `sensor_msgs/Image`)

## Model Download

The package auto-downloads MobileNet-SSD model files on first run into:

- `vision_perception_tasks/models/deploy.prototxt`
- `vision_perception_tasks/models/mobilenet_iter_73000.caffemodel`

The robot needs internet access for first startup.

## Launch

```bash
roslaunch vision_perception_tasks perception_pipeline.launch
```

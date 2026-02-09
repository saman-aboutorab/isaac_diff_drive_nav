# Isaac Sim Computer Vision Demo Roadmap

**Branch:** `feature/isaac-cv`
**Estimated Time:** 2-3 hours
**Status:** 🚧 Not Started

## Objective
Demonstrate Isaac Sim integration with computer vision for synthetic data generation and object detection.

## Why This Demo

1. **Leverages Existing Work:** Isaac Sim already set up with ROS2 bridge
2. **Modern & Impressive:** Synthetic data generation is cutting-edge
3. **Avoids TF Issue:** Doesn't need dynamic transform tree
4. **AI/ML Focus:** Perfect for AI Engineer roles
5. **Unique:** Less common than SLAM demos

## What We'll Build

```
Isaac Sim
    ├── Nova Carter Robot
    ├── RTX Camera (instead of just LiDAR)
    └── Objects in scene (labeled)
           ↓
    ROS2 Image Topic
           ↓
    Computer Vision Pipeline
    ├── Object Detection (YOLO or TensorRT)
    ├── Semantic Segmentation
    └── Depth Estimation
           ↓
    Reactive Navigation
    ├── Our gap_follower.py (already works!)
    ├── Enhanced with CV decisions
    └── "Follow closest object" behavior
           ↓
    RViz + rqt_image_view Visualization
```

## Implementation Steps

### Phase 1: Add Camera to Isaac Sim (45 min)
- [ ] Add RTX RGB camera to Action Graph
- [ ] Configure ROS2 Camera Helper node
- [ ] Publish to `/camera/image_raw` and `/camera/camera_info`
- [ ] Test image visualization in RViz/rqt

### Phase 2: Basic Object Detection (1 hour)
**Option A: Pre-trained YOLO (Easier)**
- [ ] Install `ultralytics` (YOLOv8)
- [ ] Create ROS2 node for inference
- [ ] Publish detected objects to topic
- [ ] Visualize bounding boxes

**Option B: Isaac Sim Synthetic Labels (More Impressive)**
- [ ] Use Isaac Sim's semantic segmentation
- [ ] Generate labeled training data
- [ ] Show data generation pipeline

### Phase 3: CV-Enhanced Navigation (45 min)
- [ ] Enhance `gap_follower.py` with vision
- [ ] Add behaviors:
  - "Follow red object"
  - "Avoid blue obstacles"
  - "Stop at face detection"
- [ ] Combine LiDAR + Camera decisions

### Phase 4: Documentation & Demo (30 min)
- [ ] Record demo video showing:
  - Object detection in real-time
  - Robot reacting to visual cues
  - Data generation pipeline
- [ ] Create technical README
- [ ] Prepare resume talking points

## Files Structure

### Keep from ros2-laserscan
- ✅ Isaac Sim world and setup
- ✅ ROS2 bridge configuration
- ✅ `gap_follower.py` (enhance it!)

### Remove
- ❌ SLAM configurations
- ❌ Nav2 configs (keep if using global planner)
- ❌ Static TF complexity

### Add New
- ➕ `isaac/action_graphs/camera_lidar_graph.py`
- ➕ `ros2_ws/vision_nav/` (new package)
  - `object_detector.py`
  - `cv_gap_follower.py`
- ➕ `isaac/worlds/objects_scene.usd` (scene with various objects)

## Demo Scenarios

### Scenario 1: "Follow the Ball"
- Red ball in Isaac Sim scene
- Robot detects ball with camera
- Uses LiDAR to avoid obstacles while approaching ball
- **Shows:** Sensor fusion, behavior-based navigation

### Scenario 2: "Synthetic Data Generator"
- Multiple objects at different poses
- Script randomizes positions
- Captures labeled images (bounding boxes + segmentation masks)
- **Shows:** ML pipeline, data generation for training

### Scenario 3: "Visual Obstacle Classification"
- Blue objects = walls (stop far away)
- Green objects = soft obstacles (ok to get close)
- Red objects = targets (approach)
- **Shows:** Decision-making based on visual features

## Technical Stack

### Computer Vision
- **YOLOv8** or **TensorRT** for object detection
- **OpenCV** for image processing
- **Isaac Sim Synthetic Data Tools** for labeling

### ROS2 Integration
- `cv_bridge` for image conversion
- `vision_msgs` for detection results
- `sensor_msgs` for camera info

## Expected Results

### Functional Demo
1. Isaac Sim publishes camera images
2. Object detection runs in real-time (>10 FPS)
3. Robot navigates based on visual input
4. RViz shows images with bounding boxes

### Deliverables
- [ ] Working launch with camera + detection
- [ ] 2-3 minute demo video showing CV pipeline
- [ ] Screenshots of:
  - Raw camera feed
  - Detection results overlay
  - Robot responding to visual cues
  - Synthetic data samples

## Resume Talking Points

**"Synthetic Data Generation Pipeline with NVIDIA Isaac Sim"**

- Integrated RTX camera sensors with ROS2
- Implemented real-time object detection pipeline
- Developed behavior-based navigation using computer vision
- Generated synthetic training data with automatic labeling
- Demonstrated sensor fusion (LiDAR + Camera)

**Technical Keywords:**
Isaac Sim, Computer Vision, Object Detection, YOLO, TensorRT, Synthetic Data, ROS2, Sensor Fusion, Behavior-Based Robotics

## Alternative: Depth-Based Navigation

**If object detection is too complex:**
- Use depth camera from Isaac Sim
- Implement 3D obstacle avoidance
- Show point cloud visualization
- Simpler but still impressive

## Quick Start Commands (After Implementation)

```bash
# Terminal 1: Isaac Sim with camera
/home/saman-aboutorab/isaacsim/python.sh src/isaac_cv_demo.py

# Terminal 2: Object Detection
cd ros2_ws && source install/setup.bash
ros2 run vision_nav object_detector

# Terminal 3: CV-Enhanced Navigation
ros2 run vision_nav cv_gap_follower

# Terminal 4: Visualization
ros2 run rqt_image_view rqt_image_view
rviz2
```

## Success Criteria

- ✅ Camera images publish from Isaac Sim
- ✅ Object detection runs reliably
- ✅ Robot responds to detected objects
- ✅ Demo shows clear CV → action pipeline
- ✅ Synthetic data generation documented

## Next Steps After Completion

1. Merge to `main` as Demo #2
2. Create release tag: `v2.0.0-isaac-cv`
3. Update main README
4. Create side-by-side comparison with SLAM demo

---

**This demo showcases Isaac Sim properly while avoiding the TF issue!**

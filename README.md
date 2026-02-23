# FlexBE States and Behaviors for Unseen Object Clustering

FlexBE service states and behavior pipelines for integrating **Unseen Object Clustering (UOC)** into ROS 2 manipulation workflows.

This repository provides:

- **FlexBE service states** for calling UOC ROS 2 segmentation servers
- **FlexBE behavior pipelines** that use UOC as the perception front-end before grasp planning
- Integrated pipelines for:
  - **UOC + Contact-GraspNet (RGB-D)** (recommended)
  - **UOC + GraspSAM (RGB-D)** (recommended)
- An experimental/test state for **UOC with point cloud input** (not recommended)

## Overview

This package is a **FlexBE-based perception integration layer** centered on UOC.

It includes two UOC segmentation service states:

1. **`unseen_obj_seg_rgbd_service_state.py`** (recommended)
   - Calls the UOC ROS 2 server using **RGB-D inputs**
   - This is the intended and reliable usage mode for UOC in our pipeline
   - Produces segmentation outputs used by downstream target selection and grasp modules

2. **`unseen_obj_seg_cloud_service_state.py`** (experimental, not recommended)
   - Calls UOC with **point cloud inputs**
   - Included mainly for testing / experimentation
   - In our setup, UOC performs poorly with point-cloud-only inputs

This repository also includes two FlexBE behaviors that connect UOC to downstream grasp planners:

- **UOC -> Contact-GraspNet (RGB-D)**
- **UOC -> GraspSAM (RGB-D)**

## Important Recommendation

Use **UOC with RGB-D inputs**.

- UOC is designed to work well with RGB-D segmentation.
- The point-cloud UOC state is kept for testing, but segmentation quality is generally poor.

Also note:

- **Contact-GraspNet can still be used with point cloud inputs** through other perception/filtering modules (e.g., Euclidean clustering / point cloud filtering), but that is a different integration path and is generally less robust than RGB-D.
- The weak performance is specifically about **UOC + point cloud**, not all point-cloud-based pipelines in general.

## Repository Structure

```text
├── uoc_flexbe
│   ├── CHANGELOG.rst
│   ├── CMakeLists.txt
│   └── package.xml
├── uoc_flexbe_behaviors
│   ├── bin
│   │   └── copy_behavior
│   ├── CHANGELOG.rst
│   ├── CMakeLists.txt
│   ├── config
│   │   └── example.yaml
│   ├── manifest
│   │   ├── unseenobjclustercontactgraspnetpipeine.xml
│   │   └── unseenobjclustergraspsampipeine.xml
│   ├── package.xml
│   ├── resource
│   │   └── uoc_flexbe_behaviors
│   ├── setup.cfg
│   ├── setup.py
│   └── uoc_flexbe_behaviors
│       ├── __init__.py
│       ├── unseenobjclustercontactgraspnetpipeine_sm.py
│       └── unseenobjclustergraspsampipeine_sm.py
└── uoc_flexbe_states
    ├── CHANGELOG.rst
    ├── package.xml
    ├── resource
    │   └── uoc_flexbe_states
    ├── setup.cfg
    ├── setup.py
    └── uoc_flexbe_states
        ├── __init__.py
        ├── unseen_obj_seg_cloud_service_state.py
        └── unseen_obj_seg_rgbd_service_state.py
```

## Quick Start

This section is tailored to the service names used by the uploaded UOC states/behaviors and the downstream integration behaviors.

### 1) Build the workspace

```bash
cd ~/your_ws
colcon build --symlink-install
source install/setup.bash
```

### 2) Setup required ROS 2 servers

For the UOC-based pipelines, you will typically need:

- `/segmentation_rgbd` (UOC segmentation server, recommended)
- `/get_grasps` (Contact-GraspNet server) for the CGN pipeline
- `/run_graspsam` (GraspSAM server) for the GraspSAM pipeline
- `/move_to_pose` (MoveIt / OMPL motion execution service)

### 3) Start FlexBE and run a behavior

Open FlexBE App / onboard execution and run one of:

- `UnseenObjClusterContactGraspnetPipeine` (UOC + CGN, recommended)
- `UnseenObjClusterGraspSamPipeine` (UOC + GraspSAM, recommended)

### 4) Verify services

```bash
ros2 service list | grep -E "segmentation_rgbd|get_grasps|run_graspsam|move_to_pose"
```

## Provided FlexBE States

### `UnseenObjSegRGBDServiceState` (recommended)
**File:** `uoc_flexbe_states/unseen_obj_seg_rgbd_service_state.py`

Calls the UOC ROS 2 segmentation service in **RGB-D mode**.

This is the primary and recommended UOC state for real use.

**Typical outputs**
- Segmentation results / instance metadata (e.g., masks, selected instance candidates, scene mapping info)
- JSON/path/string metadata for downstream state selection (depending on your UOC server interface)

**Default service**
- `/segmentation_rgbd`

**Notes**
- Intended to be used before target selection and downstream grasp planning
- Works well with both the Contact-GraspNet RGB-D pipeline and the GraspSAM pipeline

---

### `UnseenObjSegCloudServiceState` (experimental, not recommended)
**File:** `uoc_flexbe_states/unseen_obj_seg_cloud_service_state.py`

Calls the UOC segmentation service using **point cloud inputs**.

**Default service**
- (typically a cloud segmentation service, depending on your server setup)

**Notes**
- Included for testing / experimentation
- UOC generally performs poorly in this mode in our setup
- Prefer `UnseenObjSegRGBDServiceState` unless you are explicitly testing point-cloud UOC behavior

## Provided FlexBE Behaviors (Pipelines)

### 1) `UnseenObjClusterContactGraspnetPipeine` (recommended)
**File:** `uoc_flexbe_behaviors/uoc_flexbe_behaviors/unseenobjclustercontactgraspnetpipeine_sm.py`

Pipeline:
1. `UnseenObjSegRGBDServiceState` (`/segmentation_rgbd`)
2. `SelectInstanceToSceneNameState` (map selected target to CGN scene naming convention)
3. `CGNGraspRGBDServiceState` (`/get_grasps`)
4. `MoveToPoseServiceState` (`/move_to_pose`)

Why recommended:
- Strongest integration path for UOC-based grasping
- UOC segmentation pairs well with CGN RGB-D grasp generation
- Clean and reliable segmentation-to-grasp handoff

---

### 2) `UnseenObjClusterGraspSamPipeine` (recommended)
**File:** `uoc_flexbe_behaviors/uoc_flexbe_behaviors/unseenobjclustergraspsampipeine_sm.py`

Pipeline:
1. `UnseenObjSegRGBDServiceState` (`/segmentation_rgbd`)
2. `SelectInstanceToSceneNameState` (map selected target to GraspSAM scene convention)
3. `GraspSAMServiceState` (`/run_graspsam`)
4. `MoveToPoseServiceState` (`/move_to_pose`)

Why recommended:
- Reuses the same UOC RGB-D segmentation front-end
- Clean integration into GraspSAM grasp generation
- Outputs base-frame grasp poses for direct motion planning

## Tables for Easier Documentation

### State summary

| State file | Main class | Inputs | Outputs | Service called | Notes |
|---|---|---|---|---|---|
| `unseen_obj_seg_rgbd_service_state.py` | `UnseenObjSegRGBDServiceState` | RGB-D scene inputs / request config | segmentation outputs (instances, masks, metadata) | `/segmentation_rgbd` | Recommended UOC state. |
| `unseen_obj_seg_cloud_service_state.py` | `UnseenObjSegCloudServiceState` | PointCloud2 / cloud-based request | segmentation outputs (cloud mode) | cloud segmentation service (setup-dependent) | Experimental only; poor performance in our setup. |

### Behavior summary

| Behavior (FlexBE) | Main file | Pipeline type | Services used | Recommended |
|---|---|---|---|---|
| `UnseenObjClusterContactGraspnetPipeine` | `unseenobjclustercontactgraspnetpipeine_sm.py` | UOC (RGB-D) -> CGN (RGB-D) -> MoveIt | `/segmentation_rgbd`, `/get_grasps`, `/move_to_pose` | Yes (primary) |
| `UnseenObjClusterGraspSamPipeine` | `unseenobjclustergraspsampipeine_sm.py` | UOC (RGB-D) -> GraspSAM -> MoveIt | `/segmentation_rgbd`, `/run_graspsam`, `/move_to_pose` | Yes (primary) |

## Architecture

### UOC + Contact-GraspNet (RGB-D) pipeline

```text
RGB-D Camera
   |
   v
UOC Segmentation (ROS 2 service: /segmentation_rgbd)
   |
   v
Target instance selection / scene mapping
   |
   v
CGNGraspRGBDServiceState
   |
   v
Contact-GraspNet ROS 2 Server (/get_grasps)
   |
   v
Grasp poses (Pose[])
   |
   v
MoveIt / OMPL (/move_to_pose)
   |
   v
Robot motion execution
```

### UOC + GraspSAM (RGB-D) pipeline

```text
RGB-D Camera
   |
   v
UOC Segmentation (ROS 2 service: /segmentation_rgbd)
   |
   v
Target instance selection / scene mapping
   |
   v
GraspSAMServiceState
   |
   v
GraspSAM ROS 2 Server (/run_graspsam)
   |
   v
Grasp results -> pose_base extraction
   |
   v
MoveIt / OMPL (/move_to_pose)
   |
   v
Robot motion execution
```

## Dependencies

This repository assumes the following are available in your ROS 2 workspace:

- **FlexBE** (core + onboard/app tooling)
- **UOC ROS 2 server**
  - `/segmentation_rgbd` (recommended)
  - Optional cloud segmentation service (if testing cloud mode)
- **Contact-GraspNet ROS 2 server** (for the CGN behavior)
  - `/get_grasps`
- **GraspSAM ROS 2 server** (for the GraspSAM behavior)
  - `/run_graspsam`
- **MoveIt / motion execution service**
  - `/move_to_pose`

Companion FlexBE state packages used by the behaviors may include:

- `compare_flexbe_states` (e.g., `MoveToPoseServiceState`, target selection utilities)
- `cgn_flexbe_states` (for `CGNGraspRGBDServiceState`)
- `gsam_flexbe_states` (for `GraspSAMServiceState`)

## Installation

Clone into your ROS 2 workspace `src/` folder:

```bash
cd ~/your_ws/src
git clone <your_repo_url>
```

Build and source:

```bash
cd ~/your_ws
colcon build --symlink-install
source install/setup.bash
```

## Notes and Recommendations

- **Use `UnseenObjSegRGBDServiceState` for production use.**  
  This is the intended and reliable segmentation mode for UOC in these pipelines.

- The cloud-based UOC state is kept as an experimental/test path. It is not recommended for normal use due to poor segmentation quality.

- If your goal is point-cloud-based grasping with Contact-GraspNet, consider using other point-cloud perception/filtering modules (e.g., Euclidean clustering) instead of UOC cloud mode.

- These behavior files are generated by FlexBE. Manual edits outside `[MANUAL]` regions may be overwritten if regenerated.

## Acknowledgments

This repository builds on:

- Unseen Object Clustering (UOC)
- FlexBE
- ROS 2
- Contact-GraspNet
- GraspSAM
- MoveIt

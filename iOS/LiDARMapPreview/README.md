# LiDAR Map Preview

This folder contains the iPhone app used as the sensor head for the current
`iPhone RGB-D -> ORB-SLAM3 -> ORB Nav Desk` workflow.

The app is no longer just a local mesh demo. Its current primary role is to:

- run an `ARSession` with world tracking and LiDAR depth
- stream `RGB + Depth + Intrinsics + ARKit pose` to the Mac backend
- expose ARKit tracking and mapping quality so the backend can make better
  startup and diagnostics decisions

## Current behavior

The app currently:

- uses `ARWorldTrackingConfiguration`
- enables `smoothedSceneDepth` when available, otherwise `sceneDepth`
- enables `sceneReconstruction = .meshWithClassification` when available
- shows live status for:
  - ARKit tracking quality
  - world mapping status
  - mesh anchor count
  - depth availability
  - camera position
  - streaming status
- includes a restart button to reset the `ARSession`
- streams to the Mac over TCP with:
  - JPEG RGB frames
  - `UInt16` millimeter depth
  - rescaled intrinsics
  - 4x4 camera transform
  - ARKit tracking state and world-mapping status

Transport behavior in the current implementation:

- target send rate is `10 FPS`
- RGB is capped at `640` pixels on the long edge
- JPEG compression quality is `0.62`
- the client keeps only the latest frame in flight to avoid building a stale queue
- the client auto-reconnects after disconnects or backend restarts

## Build and install

Regenerate the project if needed:

```bash
cd /Users/xy/work/ORB_SLAM3/iOS/LiDARMapPreview
xcodegen generate
open LiDARMapPreview.xcodeproj
```

Or use:

```bash
/Users/xy/work/ORB_SLAM3/iOS/LiDARMapPreview/open_project.sh
```

Then:

1. Open the project in `Xcode`.
2. Choose your signing team for the `LiDARMapPreview` target.
3. Run on a LiDAR-capable iPhone.

Command-line validation for the project currently uses:

```bash
xcodebuild -project /Users/xy/work/ORB_SLAM3/iOS/LiDARMapPreview/LiDARMapPreview.xcodeproj -scheme LiDARMapPreview -sdk iphonesimulator CODE_SIGNING_ALLOWED=NO build
```

The simulator build is only a compile check. Real LiDAR capture and ARKit depth
behavior still need a physical device.

## End-to-end run with ORB Nav Desk

The current supported path is:

1. Build the repository backend and launch `ORB Nav Desk`.
2. Let `ORB Nav Desk` start or adopt `rgbd_iphone_stream` on port `9000`.
3. On the iPhone, enter the Mac IP, keep port `9000`, and tap `Start Stream`.
4. Wait for the backend warm-up gate to finish. The backend now delays
   `ORB-SLAM3 TrackRGBD` startup until ARKit tracking and mapping look stable.
5. Use `ORB Nav Desk` to inspect RGB, depth, map growth, guidance, and logs.

The matching Mac-side pieces are:

- [StreamClient.swift](/Users/xy/work/ORB_SLAM3/iOS/LiDARMapPreview/Sources/StreamClient.swift)
- [rgbd_iphone_stream.cc](/Users/xy/work/ORB_SLAM3/Examples/RGB-D/rgbd_iphone_stream.cc)
- [AppModel.swift](/Users/xy/work/ORB_SLAM3/macOS/ORBNavDesk/Sources/AppModel.swift)

## Diagnostics and logs

For live debugging and run review, the current system records:

- backend log: `/tmp/iphone_rgbd_nav_backend.log`
- desktop app log: `/tmp/orb_nav_desk.log`
- runtime state: `/tmp/iphone_rgbd_nav_state.json`

The stream header now includes:

- `arTrackingState`
- `arTrackingReason`
- `arWorldMappingStatus`

Those fields are used for:

- startup warm-up gating
- scale diagnostics
- post-run log analysis

## Capture guidance

For the cleanest mapping run:

- use the rear camera
- start with textured, static surfaces in view
- move smoothly instead of whipping the phone around
- avoid long stretches of blank walls, mirrors, or moving people
- wait until ARKit reports a healthy mapping state before expecting stable SLAM
- if tracking drops to `limited` or `not_available`, pause and let the session recover

## Legacy note

The older Python receiver path in `tools/lidar_stream_receiver.py` is now a
legacy reference flow. The primary maintained path in this fork is:

`LiDARMapPreview -> rgbd_iphone_stream -> ORB Nav Desk`

## Apple API references

- [ARCamera.transform](https://developer.apple.com/documentation/arkit/arcamera/transform)
- [ARFrame.sceneDepth](https://developer.apple.com/documentation/arkit/arframe/scenedepth)
- [ARFrame.smoothedSceneDepth](https://developer.apple.com/documentation/arkit/arframe/smoothedscenedepth)
- [ARWorldTrackingConfiguration.sceneReconstruction](https://developer.apple.com/documentation/arkit/arworldtrackingconfiguration/scenereconstruction)
- [Visualizing and interacting with a reconstructed scene](https://developer.apple.com/documentation/ARKit/visualizing-and-interacting-with-a-reconstructed-scene)
- [Displaying a point cloud using scene depth](https://developer.apple.com/documentation/ARKit/displaying-a-point-cloud-using-scene-depth)

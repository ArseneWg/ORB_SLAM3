# LiDAR Map Preview

This folder contains the source files for a minimal iPhone app aimed at the
`iPhone 16 Pro` path we discussed: use `ARKit` world tracking, `LiDAR` scene
depth, and `RealityKit` mesh visualization to show a room-scale map that is
actually understandable in a demo.

## What this app does

- Uses `ARWorldTrackingConfiguration`
- Enables `sceneReconstruction = .meshWithClassification` when available
- Enables `smoothedSceneDepth` or `sceneDepth` when available
- Shows Apple's built-in scene mesh overlay with `ARView.debugOptions`
- Displays live status for:
  - tracking quality
  - world mapping status
  - mesh anchor count
  - depth availability
  - camera position
- Includes a restart button for quickly resetting the session before a demo

This follows Apple's official APIs for:

- `ARCamera.transform`
- `ARFrame.sceneDepth`
- `ARFrame.smoothedSceneDepth`
- `ARWorldTrackingConfiguration.sceneReconstruction`
- `ARWorldTrackingConfiguration.supportsSceneReconstruction(_:)`
- `ARWorldTrackingConfiguration.supportsFrameSemantics(_:)`

References:

- [ARCamera.transform](https://developer.apple.com/documentation/arkit/arcamera/transform)
- [ARFrame.sceneDepth](https://developer.apple.com/documentation/arkit/arframe/scenedepth)
- [ARFrame.smoothedSceneDepth](https://developer.apple.com/documentation/arkit/arframe/smoothedscenedepth)
- [ARWorldTrackingConfiguration.sceneReconstruction](https://developer.apple.com/documentation/arkit/arworldtrackingconfiguration/scenereconstruction)
- [Visualizing and interacting with a reconstructed scene](https://developer.apple.com/documentation/ARKit/visualizing-and-interacting-with-a-reconstructed-scene)
- [Displaying a point cloud using scene depth](https://developer.apple.com/documentation/ARKit/displaying-a-point-cloud-using-scene-depth)

## Current limitation on this Mac

This Mac currently has the Command Line Tools active, but not the full `Xcode`
app installed. That means I can prepare the iPhone project here, but I can't
compile or install the app onto your phone from this machine yet.

I confirmed that:

- `xcodebuild` is unavailable because the full `Xcode.app` is missing
- your `iPhone 16 Pro` is visible to the Mac as a Continuity Camera device

## Fastest way to use these files

1. Install full `Xcode` from the App Store on this Mac.
2. Open [project.yml](/Users/xy/work/ORB_SLAM3/iOS/LiDARMapPreview/project.yml) with `XcodeGen`:
   ```bash
   cd /Users/xy/work/ORB_SLAM3/iOS/LiDARMapPreview
   xcodegen generate
   open LiDARMapPreview.xcodeproj
   ```
   Or just run:
   ```bash
   /Users/xy/work/ORB_SLAM3/iOS/LiDARMapPreview/open_project.sh
   ```
3. In Xcode, choose your signing team for the `LiDARMapPreview` target.
4. Run on your `iPhone 16 Pro`.

`XcodeGen` is already installed on this Mac, so once `Xcode` is available the
project file can be regenerated directly from [project.yml](/Users/xy/work/ORB_SLAM3/iOS/LiDARMapPreview/project.yml).

## Demo guidance

For the cleanest room-map demo:

- Use the rear camera, not the front camera.
- Start 1 to 2 meters away from a wall, desk, or sofa.
- Move slowly in an arc instead of pacing back and forth.
- Keep textured, static surfaces in view.
- Avoid pointing mostly at people or mirrors.
- Wait until the app shows `Mapping: mapped` before presenting it.

## Next step after Xcode is available

Once full `Xcode` is installed, I can continue with either:

1. polishing this on-device mesh preview for demos, or
2. building a second stage that streams `RGB + depth + pose` back to the Mac.

## Mac streaming stage

The second stage now has a matching Mac-side receiver:

- [StreamClient.swift](/Users/xy/work/ORB_SLAM3/iOS/LiDARMapPreview/Sources/StreamClient.swift)
- [lidar_stream_receiver.py](/Users/xy/work/ORB_SLAM3/tools/lidar_stream_receiver.py)
- [run_lidar_stream_receiver.sh](/Users/xy/work/ORB_SLAM3/tools/run_lidar_stream_receiver.sh)

Quick start:

1. On the Mac, run:
   ```bash
   /Users/xy/work/ORB_SLAM3/tools/run_lidar_stream_receiver.sh
   ```
2. The script prints one or more local IP addresses such as `192.168.x.x:9000`.
3. On the iPhone, enter that IP in the `Mac IP` field, keep the default port `9000`, and tap `Start Stream`.
4. The Mac viewer will show:
   - live RGB
   - live depth
   - a growing top-down map using streamed pose + depth

Keys in the Mac viewer:

- `q` or `Esc`: quit
- `c`: clear the accumulated map
- `p`: save a `PLY` map and a PNG screenshot

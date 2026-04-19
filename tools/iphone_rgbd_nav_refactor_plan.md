# iPhone RGB-D Mac Navigation UI Refactor Plan

## Goal

Turn the current macOS receiver from an experimental debug window into a standard navigation workstation UI that is:

1. Understandable without memorizing keyboard shortcuts
2. Readable on a 13-inch laptop screen
3. Stable for repeated testing with iPhone RGB-D input
4. Structured so later migration to vehicle hardware is straightforward

## Current Problems

The current implementation lives mostly in:

- `/Users/xy/work/ORB_SLAM3/Examples/RGB-D/rgbd_iphone_stream.cc`

Main issues:

1. One file is doing too much
   Rendering, network transport, ORB-SLAM3 control, occupancy mapping, planning, mouse handling, screenshot logic, and UI strings are all mixed together in a single file of about 2000 lines.

2. The interaction model is not standard
   Important controls are hidden behind keyboard shortcuts, and the user has to remember mode toggles instead of reading visible controls.

3. The Chinese UI is not layout-safe
   Labels are drawn directly onto the canvas with fixed offsets. When text changes or the panel is narrow, titles and help strings overlap.

4. Waiting / live / paused states are inconsistent
   Different states render differently, but they do not share a unified scene layout or status model.

5. Navigation output is mixed with debug output
   The same panel tries to be a debug console, screenshot target, path planner, and navigation operator screen.

6. Handheld collection and vehicle collection are mixed
   The current logic supports both "handheld adaptive height" and "fixed vehicle height", but the UI does not clearly expose that this is a core mode switch.

## Refactor Principles

1. Separate app layers
   Network input, SLAM session, navigation map model, and UI rendering should be independent modules.

2. Make all important controls visible
   Keyboard shortcuts remain as secondary controls only. Primary operations should be clickable.

3. Make one screen serve one operator task
   The operator should be able to answer:
   "Am I connected?"
   "Am I tracking?"
   "Where am I?"
   "What is blocked?"
   "Where will I go next?"

4. Use explicit app states
   Waiting, Streaming, Paused, Lost, and Localization-Only should be top-level UI states.

5. Optimize for later vehicle deployment
   Vehicle mode should become the default serious mode. Handheld mode should remain as a diagnostic mode.

## Proposed Target Architecture

Split the current single-file implementation into these files:

1. `Examples/RGB-D/nav_receiver_main.cc`
   Application entry, top-level state machine, wiring

2. `Examples/RGB-D/nav_transport.h/.cc`
   TCP input, packet parsing, frame reception

3. `Examples/RGB-D/nav_session.h/.cc`
   ORB-SLAM3 lifecycle, reconnect behavior, localization-only toggle, atlas behavior

4. `Examples/RGB-D/nav_map_model.h/.cc`
   Occupancy grid, inflation layer, floor estimation, trajectory, guidance extraction

5. `Examples/RGB-D/nav_ui_types.h`
   Shared view models and UI state structs

6. `Examples/RGB-D/nav_ui_renderer.h/.cc`
   All OpenCV drawing, layout, font handling, panel rendering

7. `Examples/RGB-D/nav_controls.h/.cc`
   Mouse handling, button dispatch, mode changes, visible control model

8. `Examples/RGB-D/nav_export.h/.cc`
   Screenshot, latest image policy, guidance JSON, optional future route export

## Proposed UI Standard

### Layout

Use a fixed 3-zone layout:

1. Top status bar
   Connection, tracking state, mode, timestamp, scale ratio

2. Main content area
   Left: RGB and depth preview stacked or tabbed
   Center: navigation map as the primary operator surface
   Right: inspector panel with controls and current measurements

3. Bottom action bar
   Visible actions and shortcuts, compact and non-overlapping

### Control Surface

Replace hidden key-heavy interaction with visible controls:

1. Mode chips
   `Vehicle`
   `Handheld`
   `SLAM`
   `Localization`
   `Follow`
   `Overview`

2. Explicit buttons
   `Reset View`
   `Overview`
   `Clear Goal`
   `Pause Mapping`
   `Resume`
   `Save Snapshot`

3. Sliders or step controls
   `Camera Height`
   `Inflation Radius`
   `Lookahead Distance`

4. Inspector readouts
   `ORB distance`
   `Reference distance`
   `Scale ratio`
   `Next waypoint`
   `Heading error`
   `Goal state`

### Typography and Localization

1. Measure all text before layout
2. Do not place text with fixed hand-tuned offsets unless the containing box is computed from measured text
3. Separate "title", "value", "hint", and "warning" text styles
4. Truncate long hints or move them into the inspector instead of overlaying them over the map

## Navigation Model Improvements

### Near-term

1. Make `Vehicle` mode the preferred navigation mode
   In this mode, fixed camera height is explicit and visible.

2. Separate map layers conceptually
   Free layer, occupied layer, inflated danger layer, and path layer should be treated as separate render layers.

3. Keep guidance operator-friendly
   Show:
   `distance to next waypoint`
   `heading error`
   `goal reached / no path / path valid`

4. Export structured guidance continuously
   Keep `/tmp/iphone_rgbd_nav_guidance.json`, but define it as a stable interface for future vehicle control integration.

### Later

1. Add route smoothing
2. Add local target selection based on curvature limits
3. Add optional deadband for heading correction
4. Add persistent map reload and relocalization workflow for vehicle startup

## State Machine

Use explicit app states:

1. `WAITING_FOR_STREAM`
2. `STREAMING_TRACKING`
3. `STREAMING_RECENTLY_LOST`
4. `STREAMING_LOCALIZATION_ONLY`
5. `PAUSED_INSPECTION`
6. `DISCONNECTED_WITH_HISTORY`

Each state should define:

1. What controls are enabled
2. What top-level message is shown
3. Whether map updates are allowed
4. Whether operator actions are allowed

## Acceptance Criteria

The refactor should be considered successful only if all of the following are true:

1. On a 13-inch MacBook display, no Chinese labels overlap in waiting, streaming, or paused states.
2. A first-time user can operate the UI without knowing any keyboard shortcuts.
3. The map remains understandable in both `Follow` and `Overview`.
4. Switching between `Handheld` and `Vehicle` modes is visible and obvious.
5. The screenshot timestamp is always visible and current.
6. The latest screenshot updates in both waiting and streaming states.
7. The guidance JSON is always consistent with what the UI shows.

## Implementation Phases

### Phase 1

Stabilize the existing monolith without changing functionality:

1. Unify state rendering
2. Standardize timestamp handling
3. Move all UI strings and metrics into one render model
4. Replace overlapping text with measured layout boxes

### Phase 2

Split the monolith into modules:

1. Transport
2. SLAM session
3. Navigation model
4. Renderer
5. Controls

### Phase 3

Upgrade operator interaction:

1. Clickable controls
2. Visible settings
3. Stronger `Vehicle` workflow
4. Better inspector panel

### Phase 4

Prepare for actual vehicle integration:

1. Stable guidance JSON contract
2. Route smoothing
3. Better relocalization workflow
4. Optional control output interface

## Recommended Immediate Next Step

Do Phase 1 and the beginning of Phase 2 first.

Concretely:

1. Keep the current algorithmic behavior
2. Refactor rendering and control code out of `rgbd_iphone_stream.cc`
3. Replace the current overlay-heavy layout with a standard `status bar + map + inspector + action bar` UI

That is the highest-value change for usability without destabilizing ORB-SLAM3 integration.

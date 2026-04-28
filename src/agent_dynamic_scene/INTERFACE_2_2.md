# Stage 2.2 Interface Freeze

## Canonical future topic for stage 2.3 risk layer
- Topic: `/dynamic_agents/future_path`
- Type: `nav_msgs/msg/Path`

## Debug / visualization topics
- `/dynamic_agents/current_pose` (`geometry_msgs/msg/PoseStamped`)
- `/dynamic_agents/current_marker` (`visualization_msgs/msg/Marker`)
- `/dynamic_agents/future_markers` (`visualization_msgs/msg/MarkerArray`)

## Semantics of `/dynamic_agents/future_path`
- `header.frame_id = map`
- poses are ordered in time
- `poses[0]` is the near-current future start
- horizon is controlled by `future.horizon_sec`
- time step is controlled by `future.dt`

## Current scope
- single actor only
- no custom msg package yet
- marker topics are for RViz only
- stage 2.3 risk layer should subscribe only to `/dynamic_agents/future_path`
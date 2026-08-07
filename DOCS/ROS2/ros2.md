# ROS2 Module

## Function

Reserve the host-side integration boundary for future radar, LaserScan, SLAM,
localization, navigation, and mission supervision.

## Source Location

`ROS2_WIN/`, `DOCS/ROS2/`, and `DOCS/ROS2_WIN_Radar/`.

## Entry File

No current Smart_Car runtime entry is proven. Existing documents and archived
materials are planning, porting, or audit records.

## Inputs

Future S3 gateway data over a defined host transport, timestamps, odometry, and
operator/autonomy intent.

## Outputs

Future LaserScan, maps, localization, navigation state, and bounded autonomy
intent back to the gateway/operator.

## Public Interfaces

No current Smart_Car ROS2 API is established. Future interfaces require an
explicit S3 gateway transport, topic/schema, timing, and safety contract.

## Dependencies

ROS2/DDS, YDLIDAR SDK/driver, time/TF/odometry contracts, Windows/Docker/USB
transport decisions.

## Current Status

Planned. Radar acquisition remains assigned to S3; direct ROS2 hardware-driver
work is not current implementation authority.

## Known Issues

No current transport, DDS discovery, timestamp, TF, odometry, safety arbitration,
or vehicle acceptance contract is frozen.

## Modification Notes

Do not add autonomous control or move radar parsing into ROS2 without an explicit
architecture and safety decision.

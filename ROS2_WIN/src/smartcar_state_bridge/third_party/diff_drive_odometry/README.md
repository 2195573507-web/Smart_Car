# diff_drive_controller Odometry subset

This directory contains the upstream Humble `diff_drive_controller::Odometry`
implementation from `ros2_controllers` commit
`eb4ca17d610eb4315f7241c0134de1bdfc5748ea`.

Only `odometry.cpp` is compiled here.  The public header is supplied by the
ROS Humble `diff_drive_controller` package, while controller, plugin, and
`ros2_control` code are deliberately excluded.  The source is licensed under
Apache-2.0; see the upstream license URL in the source header.

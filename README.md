mc-panda
=======

This package provides the following robot modules for [mc_rtc]:
- `PandaDefault`: panda robot without an end-effector mounted
- `PandaFoot`: panda robot with a static foot-like end-effector
- `PandaHand`: panda robot with the standard panda gripper
- `PandaPump`: panda robot with the standard panda pump

It also contains the definition of two devices:
- `mc_panda::PandaDevice` is an asynchronous interface for `franka::Robot` commands and `franka::RobotState` which are connected to the actual robot when [mc_franka] is running the controller;
- `mc_panda::Pump` is an asynchronous interface for `franka::VacuumGripper` which is connected to the pump when [mc_franka] is running the controller. It is only available in the `PandaPump` variant.

And it provides some panda-specific FSM states:
- `PandaStop`: immediately stop panda motion execution
- `WaitForCollisionState`: monitor panda contact sensor until certain threshold are reached
- `PumpDropoffState`: sends a dropoff command to the panda's pump and wait for the command completion
- `PumpStopState`: interrupt the pump operation
- `PumpVacuumState`: sends a vacuum command to the panda's pump and wait for the command completion

Dependencies
------------

This package requires:
- [mc_rtc]
- [libfranka](https://github.com/frankaemika/libfranka)
- [franka_ros](https://github.com/frankaemika/franka_ros)

If [mc_openrtm](https://github.com/jrl-umi3218/mc_openrtm) is installed this will also install compatible Choreonoid projects for the panda robot.

Original xacro files from [franka_gazebo](https://github.com/mkrizmancic/franka_gazebo)

[mc_rtc]: https://github.com/jrl-umi3218/mc_rtc
[mc_franka]: https://github.com/jrl-umi3218/mc_franka

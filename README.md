mc-panda
=======

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)

This package provides the following robot modules for [mc_rtc]:
- `PandaDefault`: panda robot without an end-effector mounted
- `PandaFoot`: panda robot with a static foot-like end-effector
- `PandaHand`: panda robot with the standard panda gripper
- `PandaPump`: panda robot with the standard panda pump

It also contains the definition of two devices:
- `mc_panda::Robot` is an asynchronous interface for `franka::Robot` commands and `franka::RobotState` which are connected to the actual robot when [mc_franka] is running the controller;
- `mc_panda::Pump` is an asynchronous interface for `franka::VacuumGripper` which is connected to the pump when [mc_franka] is running the controller. It is only available in the `PandaPump` variant.

And it provides some panda-specific FSM states:
- `PandaStop`: immediately stop panda motion execution
- `PandaWaitForCollision`: monitor panda contact sensor until certain threshold are reached
- `PumpDropOff`: sends a dropOff command to the panda's pump and wait for the command completion
- `PumpStop`: interrupt the pump operation
- `PumpVacuum`: sends a vacuum command to the panda's pump and wait for the command completion

See [state examples](src/states/examples.yaml) for details on the available parameters for each state.

Dependencies
------------

This package requires:
- [mc_rtc]
- [libfranka](https://github.com/frankaemika/libfranka)
- [franka_ros]

If [mc_openrtm](https://github.com/jrl-umi3218/mc_openrtm) is installed this will also install compatible Choreonoid projects for the panda robot.

Original xacro files from:
- [franka_ros] for the main definitions and collisions specifications
- [franka_gazebo](https://github.com/mkrizmancic/franka_gazebo) for the inertial parameters

Pump model was downloaded from [Schmalz](https://www.schmalz.com/en/10.03.01.00314)

[mc_rtc]: https://github.com/jrl-umi3218/mc_rtc
[mc_franka]: https://github.com/jrl-umi3218/mc_franka
[franka_ros]: https://github.com/frankaemika/franka_ros

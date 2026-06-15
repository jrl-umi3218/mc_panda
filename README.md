mc_panda
========

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![CI](https://github.com/jrl-umi3218/mc_panda/workflows/CI%20of%20mc_panda/badge.svg?branch=master)](https://github.com/jrl-umi3218/mc_panda/actions?query=workflow%3A%22CI+of+mc_panda%22)

## Video presentation

A video demonstrating panda motion generation and simultaneous pump actuation employing this implementation is available here:

[![Video presentation](https://img.youtube.com/vi/juynq6x9JJ8/0.jpg)](https://youtu.be/juynq6x9JJ8 "Safe Impacts with Soft Contacts Based on Learned Deformations")

## Robots

This package provides the following robot modules for [mc_rtc]:

| Robot Module                | Description                                 |
|-----------------------------|---------------------------------------------|
| `Panda_FR1_Default`         | FR1 robot without an end-effector           |
| `Panda_FR1_Hand`            | FR1 robot with the standard panda gripper   |
| `Panda_FR1_Pump`            | FR1 robot with the standard panda pump      |
| `Panda_FR1_Foot`            | FR1 robot with a static foot-like end-effector |
| `Panda_FR1_Mukca`           | FR1 robot with Mukca tool                   |
| `Panda_FR1_PandaToPandaCalib` | FR1 robot with PandaToPandaCalib tool     |
| `Panda_FR3_Default`         | FR3 robot without an end-effector           |
| `Panda_FR3_Hand`            | FR3 robot with the standard panda gripper   |
| `Panda_FR3_Pump`            | FR3 robot with the standard panda pump      |
| `Panda_FR3_Foot`            | FR3 robot with a static foot-like end-effector |
| `Panda_FR3_Mukca`           | FR3 robot with Mukca tool                   |
| `Panda_FR3_PandaToPandaCalib` | FR3 robot with PandaToPandaCalib tool     |

### Naming Convention Update (between v1.1.1 and v2.0.0)

Robot modules now follow the naming convention:  
```
Panda_<Robot>_<Tool>
```
where `<Robot>` is one of `FR1`, `FR3` and `<Tool>` is one of `Default`, `Hand`, `Pump`, `Foot`, `Mukca`, `PandaToPandaCalib`.

#### Migration Guide

| Old Name      | New Name                   |
|-------------- |---------------------------|
| PandaDefault  | Panda_FR3_Default         |
| PandaHand     | Panda_FR3_Hand            |
| PandaPump     | Panda_FR3_Pump            |
| PandaFoot     | Panda_FR3_Foot            |

If you previously used e.g. `PandaDefault`, update your configuration to use `Panda_FR3_Default`.  
For FR1 robots, use the corresponding `Panda_FR1_*` variant.

## Tools

Here are all the tools defined in the project:

````markdown
| Tool                | Description                        |
|---------------------|------------------------------------|
| `Panda_Tool_Hand`              | Standard panda gripper             |
| `Panda_Tool_Pump`              | Standard panda pump                |
| `Panda_Tool_Foot`              | Static foot-like end-effector      |
| `Panda_Tool_Mukca`             | Mukca tool                         |
| `Panda_Tool_PandaToPandaCalib` | Panda-to-Panda calibration tool    |
````

## Devices

It also contains the following devices:
- `mc_panda::Robot` is an asynchronous interface for `franka::Robot` commands and `franka::RobotState` which are connected to the actual robot when [mc_franka] is running the controller;
- `mc_panda::Pump` is an asynchronous interface for `franka::VacuumGripper` which is connected to the pump when [mc_franka] is running the controller. It is only available in the `PandaPump` variant.

## FSM States
And it provides some panda-specific FSM states:
- `PandaStop`: immediately stop panda motion execution
- `PandaWaitForCollision`: monitor panda contact sensor until certain threshold are reached
- `PumpDropOff`: sends a dropOff command to the panda's pump and wait for the command completion
- `PumpStop`: interrupt the pump operation
- `PumpVacuum`: sends a vacuum command to the panda's pump and wait for the command completion

See [state examples](src/states/examples.yaml) for details on the available parameters for each state.

## Dependencies

This package requires:
- [mc_rtc]
- [libfranka](https://github.com/frankaemika/libfranka)

If [mc_openrtm](https://github.com/jrl-umi3218/mc_openrtm) is installed this will also install compatible Choreonoid projects for the `FR1` panda robot.

Models:
- FR1 model was generated from an (unknown) older franka_description version
- FR3 model was generated from [franka_description v2.1.0](https://github.com/frankarobotics/franka_description)
- Pump model was downloaded from [Schmalz](https://www.schmalz.com/en/10.03.01.00314)

### Use with Nix

You can build and develop this package using [Nix flakes](https://nixos.wiki/wiki/Flakes):

1. **Enter the development shell:**
   ```
   nix develop
   ```
   This will provide a shell with all dependencies (including mc-rtc, libfranka, the current local version of mc-panda repository etc.) and build tools.

2. **Build the project (optional):**
   ```
   cmake -B build
   cmake --build build
   ```
   Note that using the version built manually in this step will require a custom `mc_rtc.yaml` configuration (documentation coming soon)

3. **Run the GUI or list available robots:**
   ```
   mc-rtc-magnum &
   mc_robot_visualization
   ```
   This displays the robot built from the latest local sources of the repository available when `nix develop` was last executed.

4. **List all available robot variants:**
   ```
   mc_robot_visualization
   ```

## Reference

Writing code takes time.
If this implementation is useful for your research, please cite the related publication:

```
@INPROCEEDINGS{Dehio2021ICRA,
  title={Robot-Safe Impacts with Soft Contacts Based on Learned Deformations}, 
  author={Dehio, Niels and Kheddar, Abderrahmane},
  booktitle={IEEE Int. Conf. on Robotics and Automation},
  pages={1357-1363},
  year={2021},
  pdf = {https://hal.archives-ouvertes.fr/hal-02973947/document},
  url = {https://hal.archives-ouvertes.fr/hal-02973947}
}
```

[![I.AM.Logo](https://i-am-project.eu/templates/yootheme/cache/iam_logo-horizontaal_XL-9e4a8a2a.png)](https://i-am-project.eu/index.php)

This work was partially supported by the Research Project I.AM. through the European Union H2020 program under GA 871899.

[mc_rtc]: https://github.com/jrl-umi3218/mc_rtc
[mc_franka]: https://github.com/jrl-umi3218/mc_franka
[franka_ros]: https://github.com/frankaemika/franka_ros

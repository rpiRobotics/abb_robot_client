# abb_robot_client

[![Python](https://img.shields.io/badge/python-3.6+-blue.svg)](https://github.com/rpiRobotics/abb_robot_client)
![PyPI](https://img.shields.io/pypi/v/abb-robot-client)

Python package providing clients for ABB robots using RWS (Robot Web Services) and Externally Guided Motion (EGM). 
This package currently supports IRC5 controllers running RobotWare 6.xx. It does not support RobotWare 7+.

This package is typically used with [abb-motion-program-exec](https://pypi.org/project/abb-motion-program-exec/),
which provides a higher level interface to generate motion programs. `abb-motion-program-exec` includes the ability
to initialize EGM operations, using this package to communicate with EGM.

`abb_robot_client` includes three modules: `rws`, `rws_aio`, and `egm`. `rws` provides a synhronous client for Robot 
Web Services (RWS) using HTTP REST, and the ability to create subscriptions using websockets. `rws_aio` provides 
identical functionality to `rws`, but uses asyncio, with each method being `async`. `egm` provides an Externally
Guided Motion (EGM) client.

A Robot Raconteur driver is also provided. This driver provides access to RWS and EGM.

Documentation can be found at: https://abb_robot_client.readthedocs.org

## Installation

`abb-robot-client` is avaliable on PyPi. Use the `[aio]` option to include support for asyncio:

```
pip install abb-robot-client[aio]
```

## Examples

See the `examples/` directory for examples using the modules.

## Robot Raconteur Driver

The Robot Raconteur driver provides access to the features of RWS and EGM, along with standard Robot Raconteur
data structures such as `RobotInfo` and `RobotState`. See `examples/robotraconteur` for examples using
the Robot Raconteur driver. See `src/abb_robot_client/robotraconteur/experimental.abb_robot.rws.robdef` and
`src/abb_robot_client/robotraconteur/experimental.abb_robot.egm.robdef` service definitions
for full information on the objects and data types provided by the service.

### Installation

Install the `abb-robot-client` with the `[robotraconteur]` feature:

```bash
python -m pip install abb-robot-client[robotraconteur]
```

### Start the Driver

Start the driver:

```
abb-robot-client-robotraconteur --robot-info-file=config\abb_1200_5_90_rws_default_config.yml --robot-url=http://127.0.0.1:80
```

or

```
python -m pip abb_robot_client.robotraconteur --robot-info-file=config\abb_1200_5_90_rws_default_config.yml --robot-url=http://127.0.0.1:80
```

Change the `--robot-info-file=` to the appropriate Robot Raconteur format yaml file for your robot,
and change `--robot-url=` to the URL of the IRC5 robot controller.

If EGM is used, the EGM must be configured on the robot controller to point to the IP address of the
computer running the driver, on port 6510.

### Connection Info

- URL: `rr+tcp://localhost:59926?service=robot`
- Node Name: `experimental.abb_rws_robot.robot`
- Device Name: `abb_robot_rws`
- Service Name: `robot`
- Root Object Type: `experimental.abb_robot.rws.ABBRWSRobot`

## License

Apache 2.0

## Acknowledgment

This work was supported in part by Subaward No. ARM-TEC-21-02-F19 from the Advanced Robotics for Manufacturing ("ARM") Institute under Agreement Number W911NF-17-3-0004 sponsored by the Office of the Secretary of Defense. ARM Project Management was provided by Christopher Adams. The views and conclusions contained in this document are those of the authors and should not be interpreted as representing the official policies, either expressed or implied, of either ARM or the Office of the Secretary of Defense of the U.S. Government. The U.S. Government is authorized to reproduce and distribute reprints for Government purposes, notwithstanding any copyright notation herein.

This work was supported in part by the New York State Empire State Development Division of Science, Technology and Innovation (NYSTAR) under contract C160142. 

![](docs/figures/arm_logo.jpg) ![](docs/figures/nys_logo.jpg)



# actor_estop

ROS package for Emergency Stopping functionality on LTU ACTor vehicles. The system consists of two components: the core estop manager (`core_estop_manager.py`) and the edge estop manager (`edge_estop_manager.py`). The core system runs on the main computer, while the edge system runs on a remote device, such as a Raspberry Pi.

## Core Estop Manager (`core_estop_manager.py`)

- Sends heartbeats to the edge manager to verify its own health.
- Monitors heartbeats from the edge manager, if it times out for any reason, it will trigger a software estop (brakes).

## Edge Estop Manager (`edge_estop_manager.py`)

- Sends heartbeats to the core manager to verify its own health.
- Monitors heartbeats from the core manager, if it times out for any reason, it will trigger an estop.
- Monitors the physical button and wireless button loops (via GPIO) and triggers an estop/reset accordingly.
- Monitors trigger and reset topics and triggers an estop/reset accordingly.
- Triggers estop activation or reset via relay board wired to the Drive-By-Wire controller.

> Note: If the relay board or rpi is powered off or looses communication,
> (normally open, active off) hardware estop is triggered automatically.
> To disable hardware estop, it needs to be physically bypassed.

## Dependencies

- ROS Noetic
- [`dbw_polaris_msgs`](https://bitbucket.org/DataspeedInc/dbw_polaris_ros): ROS messages for controlling a DataSeed Drive-By-Wire system installed in ACTor.
- [`gpiozero`](https://gpiozero.readthedocs.io/en/latest/installing.html): Python GPIO library (install using `pip3 install -r requirements.txt` if not already installed in Raspberry Pi).

## Installation

1. Clone this repository into your ROS workspace's src folder:

    ```
    git clone <repository_url>
    ```

2. Build the ROS package once in the workspace:

    ```
    catkin build actor_estop
    ```

## Usage

1. Run the core estop manager on the main computer:

    ```
    rosrun actor_estop core_estop_manager.py
    ```

2. Run the edge estop manager on the remote device (e.g. Raspberry Pi):

    ```
    rosrun actor_estop edge_estop_manager.py
    ```

## Configuration

- Modify the GPIO pin configurations in the edge estop manager script (`edge_estop_manager.py`) to match your hardware setup.
- Adjust the heartbeat rate and timeout parameters in both scripts as needed for your system requirements.
- Setup autorun on boot (only on edge device) instructions: (coming soon)

## Contributing

Contributions are welcome! If you find any issues or have suggestions for improvements, please open an issue or submit a pull request on GitHub.

## License

This software is released under the [MIT License](LICENSE).

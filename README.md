# Reactive Turtlesim Project - Collision Avoidance

This project implements a reactive control system in ROS 2 (Python and C++) for two turtles in the `turtlesim` simulator. The main goal is to prevent collisions between the turtles and with the tank walls, acting as a safety filter over the raw motion commands provided by the user.


## System Architecture

The system consists of three ROS 2 nodes, each with a specific role:

| Node | Language | Role | Topic/Service |
| :--- | :--- | :--- | :--- |
| **`spawn.py`** | Python | Initialization of the second turtle. | Client `/spawn` |
| **`ui_node.cpp`** | C++ | Interface for desired motion commands. | Publisher to `/turtleX/raw_cmd_vel` |
| **`distance_node.cpp`** | C++ | Safety filter and collision prevention logic. | Subscriber/Publisher to command and pose topics. |


```bash
.
└── assignment1_rt
    ├── CMakeLists.txt
    ├── package.xml
    ├── scripts
    │   └── spawn.py
    └── src
        ├── distance_node.cpp
        └── ui_node.cpp
```

## 1. `spawn.py` (Turtle Spawner)

This Python client node is responsible for adding a second turtle to the simulator.

* **Action:** Calls the `/spawn` service of `turtlesim`.
* **Name:** Assigns the name **`turtle2`**.
* **Position:** Places the turtle at coordinates **(x=7.0, y=7.0)**.
* **Mechanism:** Uses an asynchronous call (`call_async`) to avoid blocking the node while waiting for the response.


## 2. `ui_node.cpp` (User Interface)

This C++ node provides a command-line interface for sending the **desired** (raw) movement commands to the turtles.

* **Input:** The user selects the turtle (`turtle1` or `turtle2`) and enters the desired linear velocity (`linear.x`) and angular velocity (`angular.z`).
* **Output Topic:** Publishes the command to the **`/turtleX/raw_cmd_vel`** topic.
* **Duration:** The command is maintained for **1 second**, after which the node automatically sends a zero-velocity command to stop the robot, prompting the user for a new input.


## 3. `distance_node.cpp` (Reactive Safety Control)

This C++ node is the brain of the safety system. It intercepts the raw commands and filters them before they reach the turtle's motors, ensuring safety.

### Control Flow

1.  **Subscriptions:**
    * Receives the current pose from `/turtleX/pose`.
    * Receives the desired command from `/turtleX/raw_cmd_vel`.
2.  **Publisher:**
    * Publishes the filtered command to the `/turtleX/cmd_vel` topic.
3.  **Frequency:** The control loop runs at **20 Hz** (every $50 \text{ ms}$).

### Safety Logic

The node implements the `handle_turtle` function for each robot, applying the following rules:

1.  **Command Limiting:**
    * Limits linear velocity to $\pm 5.0$.
    * Limits angular velocity to $\pm 5.0$.
2.  **Critical Zone Definition:**
    * **Collision:** The distance between the two turtles is $\leq 1.0$.
    * **Wall:** A turtle's position is $\leq 1.0$ or $\geq 10.0$ (tank boundary).
3.  **Collision Prevention (Safety Stop):**

    When a turtle is in a Critical Zone (`Collision` or `Wall`), its **linear velocity** command is **immediately blocked** (`cmd.linear.x = 0.0`), allowing only angular movement.

    To be able to receive a **linear velocity** command again (to exit the blocked state), the **Safety Prediction** condition must be met:
    
    * **Prediction:** If the turtle is safe OR if it's in a critical zone but is about to receive a movement command, the node calculates its future position after $50 \text{ ms}$ (`dt_`). The linear velocity command is **preventively blocked** ONLY if the calculated future position falls into a Critical Zone.

    In summary, linear velocity is only permitted if the turtle about to move does not displace to a position where:
    * The distance from the other turtle is $\leq 1.0$.
    * The distance from the wall is $\leq 1.0$.

#### How to run the simulation

1.  Open 4 different shell terminals.
2.  Initialize the ROS 2 environment (adjust the ROS distribution name if necessary):
    ```bash
    source /opt/ros/jazzy/setup.bash
    ```
3.  Build the workspace:
    ```bash
    colcon build
    ```
    ```bash
    source install/local_setup.sh
    ```
4. Run the nodes, the spawner, and the turtlesim simulator:
    ```bash
    ros2 run turtlesim turtlesim_node
    ```
    ```bash
    ros2 run assignment1_rt spawn.py
    ```
    ```bash
    ros2 run assignment1_rt distance
    ```
    ```bash
    ros2 run assignment1_rt ui
    ```


    # Robot Motion Project - Collision Avoidance

This project implements a ROS 2 control architecture for a mobile robot in a Gazebo simulation. The system acts as a safety layer between the user inputs and the robot actuators, utilizing LiDAR data to prevent collisions and dynamically adjust velocity based on obstacle proximity.

## System Architecture

The system is designed with a "man-in-the-middle" architecture where the safety node intercepts user commands before they reach the simulation.

| Node | Language | Role | Topic/Service |
| :--- | :--- | :--- | :--- |
| **`controller_node`** | C++ | User Interface for manual control and service requests. | **Pub:** `/cmd_vel_input`<br>**Client:** `get_avg_vel`, `set_threshold` |
| **`safety_node`** | C++ | Safety filter, LiDAR processing, and logic handling. | **Sub:** `/cmd_vel_input`, `/scan`<br>**Pub:** `/cmd_vel`, `/obstacle_info` |
| **`spawn_robot`** | Python | Launches Gazebo, spawns the URDF model, and bridges topics. | **Bridge:** `/cmd_vel`, `/scan`, `/odom` |

## 1. `controller_node` (User Interface)

This node runs in a separate terminal (via `xterm`) and provides a menu-driven interface for the user. It does **not** communicate directly with the robot; instead, it publishes to `/cmd_vel_input`, which is monitored by the Safety Node.

**Features:**
* **Velocity Command:** Accepts Linear ($x$) and Angular ($z$) inputs. The command is published for **5 seconds**, after which a stop command (0.0, 0.0) is automatically sent.
* **Average Velocity Service:** Requests the average velocity calculated by the Safety Node over the last set of valid commands.
* **Threshold Service:** Allows the user to dynamically update the safety stop distance (default: $0.5m$).

## 2. `safety_node` (Reactive Safety Control)

This node is the core of the collision avoidance system. It processes LaserScan data to detect obstacles and modifies the velocity commands received from the controller to ensure safety.

### Collision Avoidance Logic
The node subscribes to the LiDAR topic (`/scan`) and determines the minimum distance to obstacles in the **Front** and **Rear** sectors. The velocity $v$ sent to the robot is scaled based on the distance $d$ to the obstacle:

1.  **Safe Zone:** ($d > \text{Slowdown Dist}$)
    * The command is passed through unchanged: $v_{out} = v_{in}$.
2.  **Slowdown Zone:** ($\text{Stop Dist} < d < \text{Slowdown Dist}$)
    * The velocity is linearly scaled down to ensure a smooth deceleration:
3.  **Stop Zone:** ($d \le \text{Stop Dist}$)
    * The robot is forced to stop to prevent collision: $v_{out} = 0.0$.

### Services & Data Publishing
* **Obstacle Info:** Publishes a custom message `/obstacle_info` containing the closest obstacle distance and its direction (Front, Left, Right, Rear).
* **Average Velocity:** Calculates the mean of the last 5 distinct velocity commands and returns it via the `get_avg_vel` service.
* **Dynamic Threshold:** Provides the `set_threshold` service to update the stop distance at runtime.

## Custom Interfaces

The project utilizes custom Message and Service definitions:

| Type | Name | Content |
| :--- | :--- | :--- |
| **Message** | `ObstacleInfo` | `float32 min_distance`, `string direction`, `float32 threshold` |
| **Service** | `GetAvgVel` | **Req:** None <br> **Res:** `float32 avg_linear`, `float32 avg_angular` |
| **Service** | `SetThreshold` | **Req:** `float32 new_threshold` <br> **Res:** `bool success` |

## How to Run the Simulation

 **Launch the Project:**
    ```bash
    ros2 launch assignment2_rt project_launch.py
    ```

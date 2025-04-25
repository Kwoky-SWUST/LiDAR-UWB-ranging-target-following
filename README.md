
# LiDAR-UWB-ranging-target-following

This repository contains code for a project focused on target following using LiDAR and UWB ranging technology.

## Installation

### Prerequisites

*   ROS (Robot Operating System) - Installation instructions can be found at [http://wiki.ros.org/](http://wiki.ros.org/)
*   C++ compiler (e.g., g++)
*   CMake
*   Libraries:  (Install using `apt-get install` or similar package manager)
    *   `libserial` (for serial communication)
    *   Other dependencies may be required based on the specific ROS packages used.  Check the `CMakeLists.txt` files in the `src` directory for details.

### Build Instructions

1.  **Clone the repository:**

    ```bash
    git clone <repository_url>
    cd LiDAR-UWB-ranging-target-following
    ```

2.  **Create a ROS workspace (if you don't have one already):**

    ```bash
    mkdir -p catkin_ws/src
    cd catkin_ws/src
    catkin_init_workspace
    cd ..
    ```

3.  **Copy the repository into the `src` directory of your ROS workspace:**

    ```bash
    cp -r LiDAR-UWB-ranging-target-following/src .
    ```

4.  **Build the ROS package:**

    ```bash
    catkin_make
    ```

5.  **Source the ROS environment:**

    ```bash
    source devel/setup.bash
    ```

## Key Features

*   **LiDAR-based target detection and tracking:** Utilizes LiDAR sensor data for identifying and tracking target objects.
*   **UWB ranging integration:** Incorporates UWB (Ultra-Wideband) ranging data to enhance target localization accuracy.
*   **Target Following Control:** Implements control algorithms to enable a robot or system to follow a designated target.
*   **Simulation Environment:** Includes simulation capabilities for testing and development.
*   **Modular Design:** The code is organized into separate modules for LiDAR processing, UWB data handling, control algorithms, and simulation.

## Contribution Guidelines

We welcome contributions to this project!  Please follow these guidelines:

1.  **Fork the repository.**
2.  **Create a new branch for your feature or bug fix.**
3.  **Make your changes and commit them with clear, descriptive messages.**
4.  **Test your changes thoroughly.**
5.  **Submit a pull request.**

Please ensure your code adheres to the existing coding style and includes appropriate documentation.

## Troubleshooting

*   **Compilation Errors:**  Check that all required dependencies are installed and that the ROS environment is properly sourced.  Examine the `CMakeLists.txt` files for dependency information.
*   **Serial Port Issues:**  Verify that the serial port used for UWB communication is correctly configured and accessible.  Check permissions and baud rates.
*   **Simulation Problems:**  Ensure that the simulation environment is properly set up and that all necessary models and configurations are loaded.
*   **Target Tracking Instability:**  Tune the control parameters to optimize tracking performance.  Consider factors such as sensor noise and target dynamics.

## Contributors

*   Kwoky-SWUST
```
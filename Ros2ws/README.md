# ROS 2 Workspace and Package Building Guide

This guide explains the standard ROS 2 workspace folder structure, the `colcon` build tool, and how to create and build custom ROS 2 packages using C++ and Python.

## ROS 2 Workspace Folder Structure

A ROS 2 workspace is a directory containing all necessary files for building and running ROS 2 applications. The standard structure is:

- **`<workspace>/`**: The root directory of the workspace.
  - **`src/`**: Stores source code for ROS 2 packages, with each package in its own subdirectory.
  - **`build/`**: Contains intermediate build files generated during compilation.
  - **`install/`**: Holds compiled executables, libraries, and resources after a successful build.
  - **`log/`**: Stores logs for debugging build or runtime issues.

### Example Structure
```
my_ros2_ws/
├── src/
│   ├── package_1/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt (C++)
│   │   ├── setup.py (Python)
│   │   ├── src/ (C++ source files)
│   │   ├── package_1/ (Python module files)
│   ├── package_2/
│   │   ├── ...
├── build/
├── install/
├── log/
```

- **Purpose**:
  - `src/` keeps source code organized and separate from build artifacts.
  - `build/` and `install/` ensure clean separation of generated files.
  - `log/` aids in troubleshooting.

## Using `colcon` to Build a Workspace

`colcon` is the ROS 2 build tool that compiles packages in a workspace, handling dependencies and parallel builds.

### Basic Build Command
```bash
colcon build
```

- **Explanation**:
  - Run from the workspace root (e.g., `my_ros2_ws/`).
  - Builds all packages in `src/`, creating `build/` and `install/` directories.
  - After building, source the setup file to use the packages:
    ```bash
    source install/setup.bash
    ```

### Example
1. Navigate to the workspace:
   ```bash
   cd ~/my_ros2_ws
   ```
2. Build the workspace:
   ```bash
   colcon build
   ```
3. Source the setup file:
   ```bash
   source install/setup.bash
   ```

### Useful `colcon` Options
- Build a specific package:
  ```bash
  colcon build --packages-select package_1
  ```
  - Only builds `package_1`.
- Clean build artifacts:
  ```bash
  colcon build --cmake-clean
  ```
  - Clears previous build files.
- Parallel build:
  ```bash
  colcon build --parallel-workers 4
  ```
  - Uses 4 parallel jobs for faster builds.

## Building a Custom ROS 2 Package

Below are steps to create and build a custom ROS 2 package using C++ and Python.

### C++ Package
1. **Create the Package**:
   In the `src/` directory:
   ```bash
   cd ~/my_ros2_ws/src
   ros2 pkg create --build-type ament_cmake cpp_example
   ```
   - Creates a package with `package.xml` and `CMakeLists.txt`.

2. **Add C++ Code**:
   Create `cpp_example/src/talker.cpp`:
   ```cpp
   #include <rclcpp/rclcpp.hpp>
   int main(int argc, char *argv[]) {
       rclcpp::init(argc, argv);
       auto node = rclcpp::Node::make_shared("talker");
       RCLCPP_INFO(node->get_logger(), "Hello, ROS 2!");
       rclcpp::spin(node);
       rclcpp::shutdown();
       return 0;
   }
   ```

3. **Update `CMakeLists.txt`**:
   Add to `cpp_example/CMakeLists.txt`:
   ```cmake
   find_package(rclcpp REQUIRED)
   add_executable(talker src/talker.cpp)
   ament_target_dependencies(talker rclcpp)
   install(TARGETS talker DESTINATION lib/${PROJECT_NAME})
   ```
   - Defines the executable and installation.

4. **Update `package.xml`**:
   Add `<depend>rclcpp</depend>`.

5. **Build and Run**:
   ```bash
   cd ~/my_ros2_ws
   colcon build --packages-select cpp_example
   source install/setup.bash
   ros2 run cpp_example talker
   ```
   - Outputs "Hello, ROS 2!".

### Python Package
1. **Create the Package**:
   In the `src/` directory:
   ```bash
   cd ~/my_ros2_ws/src
   ros2 pkg create --build-type ament_python python_example
   ```
   - Creates a package with `package.xml` and `setup.py`.

2. **Add Python Code**:
   Create `python_example/python_example/talker.py`:
   ```python
   import rclpy
   from rclpy.node import Node
   class Talker(Node):
       def __init__(self):
           super().__init__('talker')
           self.get_logger().info('Hello, ROS 2!')
   def main():
       rclpy.init()
       node = Talker()
       rclpy.spin(node)
       rclpy.shutdown()
   if __name__ == '__main__':
       main()
   ```

3. **Update `setup.py`**:
   Add to `python_example/setup.py`:
   ```python
   entry_points={
       'console_scripts': [
           'talker = python_example.talker:main',
       ],
   }
   ```

4. **Update `package.xml`**:
   Add `<depend>rclpy</depend>`.

5. **Build and Run**:
   ```bash
   cd ~/my_ros2_ws
   colcon build --packages-select python_example
   source install/setup.bash
   ros2 run python_example talker
   ```
   - Outputs "Hello, ROS 2!".

### C++ vs. Python
- **C++ (`ament_cmake`)**: Uses `CMakeLists.txt` for compilation, faster execution, suited for performance-critical tasks.
- **Python (`ament_python`)**: Uses `setup.py`, easier to write and debug, ideal for prototyping.

## Troubleshooting
- **Missing Dependencies**: Check `package.xml` and install dependencies with `rosdep`.
- **Build Fails**: Run `colcon build --cmake-clean` to reset.
- **Node Not Found**: Ensure `source install/setup.bash` is run.

For more, see the [ROS 2 Documentation](https://docs.ros.org/en/rolling/index.html).
for tutorial see youtube

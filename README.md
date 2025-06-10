# ROS2_Study
# ROS 2 Installation
Follow the steps mentioned in above folder Readme.md to create ROS2 docker and run it in container after container is created proceed to next steps mentioned below
# ROS 2 Installation & Environment Setup

This README.md provides a brief overview of setting up the ROS 2 environment, focusing on making ROS 2 accessible via your terminal and integrating specific workspace paths.

## 1. Making ROS 2 Accessible and Adding Paths to .bashrc

To ensure ROS 2 commands are available in your terminal and to include your specific ROS 2 workspaces, you need to "source" the relevant setup files. This is typically done by adding source commands to your `~/.bashrc` file.

The `~/.bashrc` file is a script that runs every time you open a new terminal session. By adding the source commands here, your ROS 2 environment will be set up automatically.

### Steps:

1. Open your `~/.bashrc` file for editing. You can use a text editor like `gedit` or `nano`:

    ```bash
    gedit ~/.bashrc
    # Or, if gedit is not installed:
    # nano ~/.bashrc
    ```

2. Add the following lines to the end of your `~/.bashrc` file. These lines activate the ROS 2 Humble environment, enable `colcon` command-line autocompletion, and source your specific ROS workspace.

    ```bash
    # ROS 2 Humble setup
    source /opt/ros/humble/setup.bash

    # Colcon argcomplete setup (for build tools)
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

    # Your custom ROS 2 application workspace setup
    source ~/ROS_APP/ros_workspace/install/setup.bash
    ```

### Explanation of the lines:

- `source /opt/ros/humble/setup.bash`: This line initializes the core ROS 2 Humble environment. It sets up necessary environment variables like `ROS_DISTRO` and adds ROS 2 executables to your system's `PATH`.
- `source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash`: This command enables tab-completion for `colcon` commands, which is the primary build tool for ROS 2 workspaces.
- `source ~/ROS_APP/ros_workspace/install/setup.bash`: This line integrates your specific ROS 2 workspace (in this case, located at `~/ROS_APP/ros_workspace`) into the ROS 2 environment. This allows you to run nodes and use packages from your custom projects. Ensure the path `~/ROS_APP/ros_workspace/install/setup.bash` is correct for your setup.

3. Save the changes and close the text editor.

4. Apply the changes to your current terminal session. You have two options:

    - **Option A (Recommended)**: Open a new terminal. This is the simplest way, as new terminals automatically source `~/.bashrc`.
    - **Option B**: Manually source `~/.bashrc` in your current terminal:

        ```bash
        source ~/.bashrc
        ```

## Verification

After performing the steps above and opening a new terminal (or sourcing `~/.bashrc`), you can verify your setup by running a `ros2` command:

```bash
ros2 run demo_nodes_cpp talker
```

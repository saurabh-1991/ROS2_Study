# Running ROS2-Humble in a Docker Container

This guide walks you through building a Docker image for ROS2 Humble and running a container from that image, enabling graphical applications to be displayed on your host machine.

## Building the Docker Image

To build the Docker image, navigate to the root directory where your `Dockerfile` is located and execute the following command:

```bash
docker build -t ros_humble:latest .
```

This command builds an image named `ros_humble` with the tag `latest`.

## Enabling X Server Connections

Before running your Docker container, you need to allow connections to your host's X server so that graphical applications running inside the container can display their output on your machine.

You have two options:

### More Secure Option:

```bash
xhost +local:docker
```

This command specifically allows connections from the `docker` user on your local machine.

### Less Secure Option (use if the first doesn't work consistently):

```bash
xhost +
```

This command disables access control, allowing clients from any host to connect. Use this only if the more secure option fails.

You should see output similar to "localuser:docker being added to access control list" or "access control disabled, clients can connect from any host."

## Running the Docker Container

Once the Docker image is built and X server connections are enabled, you can run your ROS2 Humble container with the following command:

```bash
docker run -it \
    --name ros_humble_container \
    --network=host \
    --ipc=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    ros_humble:latest
```

### Explanation of Flags:

- `-it`: Allocates a pseudo-TTY and keeps STDIN open, allowing you to interact with the container.
- `--name ros_humble_container`: Assigns a name to your container for easier identification.
- `--network=host`: Connects the container to the host's network stack, allowing it to access network services directly.
- `--ipc=host`: Uses the host's IPC (Inter-Process Communication) namespace, which can be beneficial for some ROS 2 applications.
- `-e DISPLAY=$DISPLAY`: Passes your host's `DISPLAY` environment variable to the container, telling it where to send graphical output.
- `-v /tmp/.X11-unix:/tmp/.X11-unix`: Mounts the X11 Unix socket from your host into the container, enabling communication with the X server.
- `ros_humble:latest`: Specifies the image to use for creating the container.

After running this command, you will be inside your ROS2 Humble Docker container, ready to develop and run your ROS 2 applications with graphical support.

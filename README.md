# Fetch Picker Workspace

This repository provides starter code and samples for programming the Fetch mobile manipulator to pick requested items from densely packed shelves. It is used in the following University of Washington courses:

- [TECHIN 517: Robotics Lab II](https://sites.google.com/cs.washington.edu/techin517sp22/home)

Labs and other documentation are available on the [wiki](https://github.com/robotic-picker-sp22/fetch-picker/wiki).

## Project Structure

- `src/fetch-picker/` — Main project code and packages
- `build/`, `devel/`, `logs/` — Build and development artifacts (auto-generated)
- `.devcontainer/` — Development container setup for VSCode/Dev Containers

## Quick Start

### 1. Install Dependencies

Run the provided install script (for Ubuntu/ROS Noetic):

```bash
bash install_fetch_ws.sh
```

This will:
- Install system and ROS dependencies
- Clone the fetch-picker code
- Set up Node.js and web tools
- Build the workspace with `catkin build`

### 2. Build the Workspace

If you need to rebuild:

```bash
source /opt/ros/noetic/setup.bash
catkin build
```

### 3. Source the Workspace

Add these lines to your `~/.bashrc` (the script does this automatically):

```bash
source /opt/ros/noetic/setup.bash
source ~/fetch_ws/devel/setup.bash
```

### 4. Using Dev Containers (Optional)

If you use VSCode, you can open the workspace in a [Dev Container](https://code.visualstudio.com/docs/remote/containers) for a pre-configured development environment. See `.devcontainer/` for details.

## Additional Resources
- [Fetch Picker Wiki](https://github.com/robotic-picker-sp22/fetch-picker/wiki)
- [ROS Noetic Documentation](http://wiki.ros.org/noetic)

---

For any issues or questions, please refer to the course staff or the project wiki. 
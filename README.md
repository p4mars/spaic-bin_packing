# SPAIC Bin Packing

This repository contains the ROS 2 packages developed by our group for the SPAIC bin packing project.

## Repository Structure

This repository is intended to be cloned directly into the `src` folder of an external ROS 2 workspace.

```text
ros2_ws/
└── src/
    └── spaic-bin_packing/
        ├── package_1/
        ├── package_2/
        ├── package_3/
        ├── README.md
        └── .gitignore
```

Each group member can develop their own ROS 2 package directly under the root of this repository.

Please do **not** create another `ws/src` folder inside this repository.

## Setup

Create a ROS 2 workspace:

```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
```

Clone this repository into the `src` folder:

```bash
git clone https://github.com/p4mars/spaic-bin_packing.git
```

Go back to the workspace root:

```bash
cd ..
```

Build the workspace:

```bash
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

## Development Guidelines

Please follow these basic rules when contributing:

1. Put each ROS 2 package directly under the repository root.
2. Use clear and meaningful package names.
3. Do not put random scripts or temporary files in the root folder.
4. Add a short explanation for your package in this README or in your package folder.
5. Keep your code organized and documented.
6. Follow the Git/GitHub practices explained in Dave's lecture.

## Suggested Package Structure

A typical ROS 2 Python package may look like this:

```text
my_package/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
└── my_package/
    ├── __init__.py
    └── node.py
```

A typical ROS 2 C++ package may look like this:

```text
my_package/
├── package.xml
├── CMakeLists.txt
├── src/
└── include/
```

## Build and Run

After adding or modifying packages, rebuild the workspace from the workspace root:

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

To run a ROS 2 node:

```bash
ros2 run <package_name> <node_name>
```

## Package Overview

Please update this section when adding a new package.

| Package Name | Owner | Description |
|---|---|---|
| `mirte_driving` | Gui & Mati | Mapping + Navigation |
| `package_2` | Member 2 | Short description of this package |
| `package_3` | Member 3 | Short description of this package |

## Notes

- Keep the main branch clean and working.
- Pull the latest changes before starting your work.
- Commit with clear messages.
- Push your changes regularly.
- If you are unsure about the structure, discuss with the group before making large changes.

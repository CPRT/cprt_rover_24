# CPRT Rover 2024

This repository contains the workspace and source code for the CPRT Rover 2024 project. It is organized as a ROS 2 (Humble) workspace and includes packages for navigation, science sensors, camera integration, GPS, and more.

## Workspace Structure

- **src/**: Main source code for all packages (navigation, sensors, camera, GPS, etc.)
- **setup/**: Setup scripts for installing dependencies and configuring the environment
- **build.sh / build_science.sh**: Build scripts for the workspace and science packages
- **docker/**: Dockerfiles and instructions for containerized development (In Development)
- **.devcontainer/**: VSCode devcontainer configuration
- **.github/**: GitHub Actions and workflows

## Getting started

### Host (Only ubuntu 22.04)
See setup/README.md for which setup scripts to run

### Docker Method (New and still experimental):
Note: Still recommended to use ubuntu 22.04 even with devcontainer

#### Install Docker:
Window - [Link to download](https://docs.docker.com/desktop/setup/install/windows-install/)
Mac - [Link to download](https://docs.docker.com/desktop/setup/install/mac-install/)
Linux - [Link to download] (https://docs.docker.com/engine/install/)

#### Install VS code dev containers extension:
Use extension marketplace

#### Open Dev container in VS code

#### From inside the container run:
```
./make.sh
```

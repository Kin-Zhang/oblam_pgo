# OBLAM Assignment: Loop Closure and Pose-Graph Optimization
<!-- via Continuous-time Optimization -->

The course materials and instructions can be found at [KTH Canvas](https://canvas.kth.se/courses/40649) .

Please download pointcloud data [here](https://kth-my.sharepoint.com/:f:/g/personal/tmng_ug_kth_se/ErWpfrnkfQZJvrNUnw3Y-dEB0ljN-xF-FvTl8AztgkSx6A?e=ycLF8y), Then declare the path to the data in the launch file run_pgo.launch.

# Installation

## Dependencies

The software was developed on the following dependancies, Test System: [Ubuntu 20.04](https://releases.ubuntu.com/20.04/), [ROS Noetic full-desktop](http://wiki.ros.org/noetic/Installation)

Printer logger (glog, gflag are also dependencies on Ceres)

```bash
# maybe need sudo if you are running it in your desktop
curl -sL https://raw.githubusercontent.com/Kin-Zhang/Kin-Zhang/main/Dockerfiles/setup_lib.sh | bash
```

Solver: [Ceres 2.1.0](http://ceres-solver.org/installation.html) (do checkout the branch 2.1.0 after git clone)

```bash
git clone https://ceres-solver.googlesource.com/ceres-solver
git fetch --all --tags
git checkout tags/2.1.0
mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

The code was edited on VS Code with [#region folding add-on](https://marketplace.visualstudio.com/items?itemName=maptz.regionfolder) for tidier view.

## Build
Please install all dependencies first. Afterwards, create a ros workspace, clone the package to the workspace, and build by `catkin build` or `catkin_make`, for e.g.:

```
mkdir oblam/src
cd oblam/src
git clone https://github.com/brytsknguyen/oblam_pgo
cd ../..; catkin build
```
## Run

After finished the assignment part, run the launch

```bash
source devel/setup.zsh
roslaunch oblam_pgo run_pgo.launch
```



# Results
## Demo

Go to the function OptimmizePoseGraph() and create a ceres problem and solve it. If it works you should see a loop closure event like this.

<p align="center">
    <img src="docs/loop2.gif" alt="mcd ntu daytime 04" width="49%"/>
    <img src="docs/loop1.gif" alt="mcd ntu daytime 01" width="49%"/>
</p>
## Kin's results

LibArm_kinematics used by [Unity Reachy Simulator](https://github.com/pollen-robotics/reachy2021-unity-package)

To compile the lib
```
git clone --recursive git@github.com:pollen-robotics/Arm_kinematics.git
cd Arm_kinematics
mkdir build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
```
Copy the lib from the *build* directory to the *Packages/ReachySimulator/Plugins* of your Unity project.


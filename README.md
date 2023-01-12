# Developing an Isaac Sim extension
This repository contains example codes I tested for developing extensions for the Isaac Sim application on the Nvidia Omniverse platform.

## Test environment
|Items|Computer 1|Computer 2|
|---|---|---|
|Manufacturer|Lenovo|Prefab PC|
|Model|Legion-T7 (desktop)|Prefab PC (Desktop)|
|CPU|11th Gen Intel(R) Core(TM) i7-11700K @ 3.60GHz|9th Gen Intel(R) Core(TM) i9-9900KF @ 3.60GHz|
|GPU|Nvidia RTX 3080|Nvidia RTX 2080Ti|
|Operating system|Windows 11 Pro|Ubuntu 18.04 LTS|
|Package ver.|2022.1.1</br>2022.2.0|2022.1.1</br>2022.2.0|
|Etc.||Nvidia driver: 525.60.11</br>CUDA: 12.0|

## Example extensions
1. **Hello Extension**  
This example is configured to test the basics of building a custom extension as a plugin for the Isaac Sim application.  
![](/doc/hello-extension.png)  

2. **Hello Scene - IExt**  
This example implements setting a default stage using the omni.ext.IExt interface for the Isaac Sim. I made this example by referring to the BaseSample class to understand a basic pipeline to build a scene from an extension.  
![](/doc/hello-scene-iext.png)

3. **Hello Scene BaseSample**  
This example implements how to set a default stage using the BaseSample class for the Isaac Sim.

4. Hello Robot  
In this example, I tested how to add articulated robots, including industrial and collaborative robots.

5. Hello Task  
(to be added.)

6. Hello Sensor - Vision  
(to be added.)

7. Hello Sensor - Lidar?  
(to be added.)

8. (to be determined.)


## References
* NVIDIA OMNIVERSE™ DOCUMENTATION: https://docs.omniverse.nvidia.com/

## Thanks to
* Suyoung Park of IDIM @ Seoul National University

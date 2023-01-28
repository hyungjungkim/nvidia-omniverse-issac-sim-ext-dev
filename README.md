# Developing an Isaac Sim extension
Extensions are the core building block of Omniverse Kit-based applications. They are individually built application modules. All the tools used in Omniverse Isaac Sim are built as extensions.  
While studying Omniverse Isaac Sim, I found [three main workflows](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_intro_workflows.html#isaac-sim-workflows) when developing in Omniverse Isaac Sim: GUI, extensions, and standalone Python. Among these workflows, I think the Extensions are the most comprehensive way to experiment with changing various conditions and tasks. On the other hand, standalone Python would be useful when integrating with third-party algorithms such as deep reinforcement learning. Thus, most examples in this repository focus on developing extensions using prepared or imported USD models, not creating objects using the GUI.  
This repository contains examples I tested for developing an extension for the Isaac Sim application on the NVIDIA Omniverse platform. 
Although the details of python APIs and examples are well explained in the online documentation, it is difficult to read and test them one by one due to their vast contents, so here I will summarize the essentials for developing robotics and manufacturing process simulation applications using Isaac Sim and its Extension.

## What is Isaac Sim?
NVIDIA Omniverse™ Isaac Sim is a robotics simulation toolkit for the NVIDIA Omniverse™ platform. For more information, please visit the NVIDIA Omniverse Robotics documentation: https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html.
![](/doc/isaac_main_intro_from_nvidia.png)

---

## Test environment
|Items|Computer 1|Computer 2|
|---|---|---|
|Manufacturer|Lenovo|Prefab PC|
|Model|Legion-T7 (desktop)|Prefab PC (Desktop)|
|CPU|11th Gen Intel(R) Core(TM) i7-11700K @ 3.60GHz|9th Gen Intel(R) Core(TM) i9-9900KF @ 3.60GHz|
|GPU|NVIDIA RTX 3080|NVIDIA RTX 2080Ti|
|Operating system|Windows 11 Pro|Ubuntu 18.04 LTS|
|Package ver.|2022.1.1</br>2022.2.0|2022.1.1</br>2022.2.0|
|Driver ver.|527.56|525.60.11|
|CUDA ver.|12.0|12.0|

Please visit NVIDIA to check the driver requirements: https://developer.nvidia.com/omniverse/driver  

---

## How to use
### Prerequisites
* Intermediate knowledge of Python and asynchronous programming is required.
* Basic knowledge of robotics and the manufacturing process is also required but not mandatory.
* Please download and install Visual Studio Code before beginning the examples. https://code.visualstudio.com/download
* Please review Isaac Sim Interface and Isaac Sim Workflows before beginning the examples.  https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_intro_interface.html#isaac-sim-app-tutorial-intro-interface
https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_intro_workflows.html#isaac-sim-app-tutorial-intro-workflows
* In addition, please try a Hello World tutorial. I also started from this tutorial to learn Omniverse Isaac Sim.  https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_core_hello_world.html#isaac-sim-app-tutorial-core-hello-world

### Run the examples
1. Download examples from the source code folder [[src](/src/)].
2. Place the downloaded examples in a directory in the **Extension Search Paths** of the **Extensions manager** panel or create a new search path to the example directory in this panel.
    - Extensions Manager overview: https://docs.omniverse.nvidia.com/app_isaacsim/prod_extensions/ext_extension-manager.html 
    ![](/doc/extension-search-paths.png)
3. Isaac Sim's workflow is that the application runs asynchronously. It allows for **'hot reloading'**, so you can change the extension code while Omniverse Isaac Sim is running and then see the reflected changes in your application after saving the file without shutting down or restarting Omniverse Isaac Sim.

---

## Example extensions
1. **Hello Extension** [[code](/src/omni.isaac.hello_ext/)]   
* This example is configured to test the basics of building a custom extension as a plugin for the Isaac Sim application.  
![](/doc/hello-extension.png)  

2. **Hello Scene - IExt** [[code](/src/omni_isaac.hello_scene_iext/)]  
* This example implements setting a default stage using the omni.ext.IExt interface for the Isaac Sim. I made this example by referring to the BaseSample class to understand a basic pipeline to build a scene from an extension.  
    * **World** is the core class that support us to interact with the simulator in an easy and modular way. It handles many time-related events such as adding callbacks, stepping physics, resetting the scene, adding tasks, etc. The world contains an instance of a **Scene**. The Scene class manages simulation assets of interest in the USD Stage. It provides an easy API to add, manipulate, inspect, and reset different USD assets in the stage.

    ![](/doc/hello-scene-iext.png)

3. **Hello Scene - BaseSample** [[code](/src/omni_isaac.hello_scene_basesample/)]  
* This example implements setting a default stage using the BaseSample class for the Isaac Sim.  
    * The BaseSample and BaseSampleExtension classes contain a set of frequently used APIs that are designed to be used in robotics applications that can be inherited in Python and Omniverse Kit to start with the basic functionality needed for any Isaac Sim application, including loading, resetting, and clearing a world, as covered in most example Extensions.  

* In this example, I modified the menu name (in 'base_sample_extension.py') from 'Isaac Examples' to 'Isaac Ext Dev' to distinguish my example from the existing examples.  
![](/doc/hello-scene-basesample.png)

4. **Hello Object - primitive**  [[code](/src/omni_isaac.hello_object_primitive/)]
* This example contains how to add primitive Omniverse geometric objects (Create->Shapes in GUI menu) in the Scene of the World.
    * Through this example, I found a difference among the three primitive object classes of Capsule/Cone/Cuboid/Cylinder/Sphere. First, a DynamicObject class (e.g., DynamicCuboid, red one) is enabled for the Physics of both Collider and Rigid Body. Second, only a FixedObject class (e.g., FixedCylinder, green one) is allowed for the Physics of Collider. Last, a VisualObject class (e.g., VisualSphere, blue one) is not enabled any Physics properties compared to other object classes. This difference allows the movement and collision of the objects to appear differently.
    * A physics callback allows applying actions before each physics step. So, by adding a physics callback function, we can inspect information about objects in the World.

    ![](/doc/comparison-object-classes.gif)

5. **Hello Object - Asset Converter** [[code](/src/omni_isaac.hello_object_asset_converter/)]
* This example covers converting a non-USD format of engineering 3D models such as FBX and OBJ to the USD and adding the USD asset in the Scene of the World.
    * To convert the FBX and OBJ models, I utilized the Asset Converter, which is a native tool on Omniverse, asynchronously.
    * In this example, I implemented a custom UI frame, including buttons, a string field, and the file importer dialog.  
    ![](/doc/custom_ui_frame_ex.png)
    * There is also a native CAD importer for STEP and IGES files. Please visit the link to find more:  
    https://docs.omniverse.nvidia.com/app_isaacsim/prod_extensions/ext_cad-importer.html

    ![](/doc/hello-object-asset_converter.gif)

6. **Hello Robot** [[code](/src)]

7. Hello Task  
(to be added.)

8. Hello Sensor - Vision  
(to be added.)

9. Hello Sensor - Lidar?  
(to be added.)

10. (to be determined.)

---
## References
* NVIDIA OMNIVERSE™ documentation: https://docs.omniverse.nvidia.com/

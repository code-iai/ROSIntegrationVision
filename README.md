# ROSIntegrationVision for Unreal Engine 4
This plugin adds ROS Vision Support to your Unreal Engine 4 Project. 
A specialized Camera can measure rgb and depth data from your Unreal World and publishes it into a running ROS environment.
In order to use this plugin you also need to add the ROSIntegration Core Plugin (https://github.com/code-iai/ROSIntegration) to your project.

![RGBD Data in RVIZ](http://i.imgur.com/N45Pa28.png)


## Dependencies of this Plugin
This plugin depends on F16C Intrinsic Support (https://msdn.microsoft.com/de-de/library/hh977022.aspx), which should be included in newer CPU generations. This ensures the depth data to be converted quickly.
Important: If you use this Plugin on Linux, you need to enable F16C support manually in Unreal Engine 4 and recompile it.
To do this, open 
`PATH_TO_UNREAL/Engine/Source/Programs/UnrealBuildTool/Platform/Linux/LinuxToolChain.cs`, find the `GetCLArguments_Global` method and add `Result += " -mf16c";` in a suitable place. After that, recompile UE4.

## Usage
After installing this plugin and the core ROSIntegration plugin, you can load your UE4 project.
If the plugin is not enabled in the project yet, you can do this in the UE4Editor in Edit -> Plugins.

When the plugin is loaded correctly, a new `CameraActor` named `VisionActor` and a new `CameraComponent` named `VisionComponent` will be available to use.
Each one represents the RGBD camera.

In some cases (for example on Linux), it might be necessary to call the Generate Project Files action on UE4 in order to fetch the new header files for the plugin before your first compile. 

### Vision Component

Actor Attachment:

```c++
#include "ROSIntegrationVision/Public/VisionComponent.h"
...
UVisionComponent * vision = CreateDefaultSubobject<UVisionComponent>(TEXT("Vision"));
vision->SetupAttachment(RootComponent);
```

Fixed Camera Mode:

This disables tf publishing from within Unreal, and instead requires the links 
to be specified via URDF.  

```c++
vision->FixedCam = true;
```

Camera Parent Link Naming:

```c++
vision->ParentLink = "desired_link"
```

## Credits
Credits go to http://unrealcv.org/ and Thiemo Wiedemeyer, who laid out the rendering and data handling basics for this Plugin.

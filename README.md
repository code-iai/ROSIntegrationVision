# ROSIntegrationVision for Unreal Engine 4
This plugin adds ROS Vision Support to your Unreal Engine 4 Project. 
A specialized Camera can measure rgb and depth data from your Unreal World and publishes it into a running ROS environment.
In order to use this plugin you also need to add the ROSIntegration Core Plugin (https://github.com/code-iai/ROSIntegration) to your project.

![RGBD Data in RVIZ](http://i.imgur.com/N45Pa28.png){:class="img-responsive"}


## Dependencies of this Plugin
This plugin depends on F16C Intrinsic Support (https://msdn.microsoft.com/de-de/library/hh977022.aspx), which should be included in newer CPU generations. This ensures the depth data to be converted quickly.

## Credits
Credits go to http://unrealcv.org/ and Thiemo Wiedemeyer, who laid out the rendering and data handling basics for this Plugin.

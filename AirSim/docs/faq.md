
# FAQ

## General
#### Unreal editor is slow when it is not the active window

Go to Edit/Editor Preferences, select "All Settings" and type "CPU" in the search box. 
It should find the setting titled "Use Less CPU when in Background", and you want to uncheck this checkbox.

#### My mouse disappears

Yes, Unreal steals the mouse, and we don't draw one.  So to get your mouse back just use Alt+TAB to switch to a different window.

#### Where is setting file and how do I modify it?
AirSim will create empty settings file at `~/Documents/AirSim/settings.json`. You can view the available [settings options](settings.md). 

#### How do I arm my drone?
If your using simple_flight, your vehicle is already armed and ready to fly. For PX4 you can arm by holding both sticks on remote control down and to the center.

#### Where did my mouse pointer go?
Unreal steals the mouse so to get your mouse back use Alt+TAB to switch to a different window. To avoid this entirely, go to Project settings in Unreal Editor, go to Input tab and disable all settings for mouse capture.

#### Something went wrong. How do I debug?

First turn on C++ exceptions from the Exceptions window:

![exceptions](images/exceptions.png)

and copy the stack trace of all exceptions you see there during execution that look relevant (for example, there might be an initial 
exception from VSPerf140 that you can ignore) then paste these call stacks into a new AirSim github issue, thanks.

#### What do the colors mean in the Segmentation View ?

See [Camera Views](camera_views.md) for information on the camera views and how to change them.

#### Unreal 4.xx doesn't look as good as 4.yy

Unreal 4.15 added the ability for Foliage LOD dithering to be disabled on a case-by-case basis by unchecking the `Dithered LOD Transition` checkbox in the foliage materials. Note that all materials used on all LODs need to have the checkbox checked in order for dithered LOD transitions to work.  When checked the transition of generated foliage will be a lot smoother and will look better than 4.14.

#### Can I use an XBox controller to fly?
Yes, see [XBox controller](xbox_controller.md) for details.

#### Can I build a hexacopter with AirSim?

Definitely, see [how to build a hexacopter](hexacopter.md).

#### How do I use AirSim with multiple vehicles?
Here is [multi-vehicle setup guide](multi_vehicle.md).

#### What computer do you need?
It depends on how big your Unreal Environment is. The Blocks environment that comes with AirSim is very basic and works on typical laptops. The [Modular Neighborhood Pack](https://www.unrealengine.com/marketplace/modular-neighborhood-pack) that we use ourselves for research requires GPUs with at least 4GB of RAM. The [Open World environment](https://www.unrealengine.com/marketplace/open-world-demo-collection) needs GPU with 8GB RAM. Our typical development machines have 32GB of RAM and NVIDIA TitanX and a [fast hard drive](hard_drive.md).

#### How do I report issues?

It's good idea to include your configuration like below. If you can also include logs, that could also expedite investigation.

````
Operating System: Windows 10 64bit
CPU: Intel Core i7
GPU: Nvidia GTX 1080
RAM: 32 gb
Flight Controller: Pixhawk v2
Remote Control: Futaba
````

If you have modified the default `~/Document/AirSim/settings.json`, please include your
settings also.

If you are using PX4 then try to [capture log from MavLink or PX4](px4_logging.md).

File an issue through [GitHub Issues](https://github.com/microsoft/airsim/issues).

## Others

[Linux Build FAQ](build_linux.md#faq)
[Windows Build FAQ](build_windows.md#faq)
[PX4 Setup FAQ](px4_setup.md#faq)
[Remote Control FAQ](remote_control.md#faq)
[Unreal Blocks Environment FAQ](remote_control.md#faq)
[Unreal Custom Environment FAQ](remote_control.md#faq)
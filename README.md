# Visual Odometry Self Balance Bot
World's first visual odometry stabilized self balance robot!

What you'll get:
[![Preview](http://img.youtube.com/vi/DK2eS_DkTyM/0.jpg)](https://www.youtube.com/watch?v=DK2eS_DkTyM "A boring line follower")

## How to get started
- [Get the App](#get-the-app)
- [How to build the bot](#how-to-build-the-bot)
  - [Bill of Materials](#bill-of-materials)
  - [Parameters to start with](#parameters-to-start-with)
- [Phone mounting](#phone-mounting)
- [Troubleshooting](#troubleshooting)

## Get the App
Serial Sensor is free, you can download it from the [play store](https://play.google.com/store/apps/details?id=com.karl.serialsensor).
For using the camera based sensors, you need to run the calibration, therefore the app will tell you what you need to do.
When you're done with the calibration, select the line- and parameter sensor. Additionally you need to choose either USB or Bluetooth as an connection (see screenshots below).
| | | 
|:-------------------------:|:-------------------------:|
|<img width="1604" alt="sel_sens" src="https://raw.githubusercontent.com/SerialSensor/ProjMedia/master/Line/selected_sensors.png"> | <img width="1604" alt="sel_out" src="https://raw.githubusercontent.com/SerialSensor/ProjMedia/master/Line/selected_output.png"> |

## How to build the bot
Here's how you build the bot. Note: its the same setup like for the "fast" line follower. 
| | | |
|:-------------------------:|:-------------------------:|:-------------------------:|
|<img width="1604" alt="fast 1" src="https://raw.githubusercontent.com/SerialSensor/ProjMedia/master/Line/fast_1.jpg"> |  <img width="1604" alt="fast 2" src="https://raw.githubusercontent.com/SerialSensor/ProjMedia/master/Line/fast_2.jpg">|<img width="1604" alt="fast 0" src="https://raw.githubusercontent.com/SerialSensor/ProjMedia/master/Line/fast_0.jpg">|

### Bill of Materials
| What          | Example Link|
|:-------------:|:-------------:|
| Arduino Nano (or clone) | -|
| PCB 120x80mm  | [ebay](https://www.ebay.com/itm/Double-Side-Prototype-PCB-Tinned-Universal-Bread-board-8x12-cm-80x120-mm-FR4-DIY/271817506142?hash=item3f4995fd5e:g:0a0AAOSwstxVEm5v) |
| Ball Caster / Wheel caster   | [polulu](https://www.pololu.com/product/2692)|
| N20 DC motor 6V (300 - 500RPM) | [ebay](https://www.ebay.com/itm/GA12-N20-DC-3V-6V-12V-Micro-Electric-Gear-Motor-Speed-Reduction-Metal-Gearbox/402608204445?hash=item5dbd51aa9d:g:cFUAAOSw7yxf2HUC)     |
| Small battery 7.4V  | [Joom](https://www.joom.com/en/products/5cb9bb918b2c370101ba066b)     |
| JST plugs | [ebay](https://www.ebay.com/itm/10Pairs-Male-Female-10cm-JST-Connector-Plug-Cable-Line-for-RC-BEC-Lipo-Battery-s/313353616464?hash=item48f554bc50:g:ayIAAOSwB-1Y3Fwa)     |
| Female headers | [ebay](https://www.ebay.com/itm/16P-Female-Header-Pins-2-54mm-Pitch-PCB-Socket-Single-Row-Connector-Straight/223503325170?hash=item3409d617f2:g:EgsAAOSwD2NczByO)     |
| Motor brackets | [robu.in](https://robu.in/product/mounting-bracket-n20-micro-gear-motors/) |
| USB cable (you shouldn't use bluetooth here due to high latency) | [ebay](https://www.ebay.com/itm/USB-Type-c-to-Mini-USB-Cable-USB-C-Male-to-Mini-B-Male-Adapter-ConvertODUS/114238371143?hash=item1a99233547:g:YekAAOSwpUVcd5Eu) |
| Motor driver 8835| [pololu](https://www.pololu.com/product/2135) |
| Screw terminal block| [ebay](https://www.ebay.com/itm/10pcs-2-Poles-KF128-2-54mm-PCB-Universal-Screw-Terminal-Block-Gut-xm/184399768007?hash=item2aef1529c7:g:LoAAAOSw5E5avvGm) |
|Magnetic car phone mount| [aliexpress](https://www.aliexpress.com/item/4001141818390.html?spm=a2g0o.productlist.0.0.3332259ajzEPxO&algo_pvid=eb210c7e-4940-40af-9eb8-64d94017fdbb&algo_expid=eb210c7e-4940-40af-9eb8-64d94017fdbb-6&btsid=2100bdca16137527752533041e69f6&ws_ab_test=searchweb0_0,searchweb201602_,searchweb201603_) or [print it](https://www.thingiverse.com/thing:4763685) |
| electrical tape for the line |-|

### Parameters to start with
With the provided sbController.py script, you can send some default values to your phone. Therefore put your PC's and your Phone's ip inside the script (somewhere around line 16). Inside the Serial Sensor App enable only the parameter sensor and in the parameter sensor's detail view you need to check "forward parameters from network" and "save parameters from network". This first will make sure that your arduino gets parameters forwarded in real time, while the latter saves everything what you have received. As a last thing you need to enable the Network connnection inside the app und put your PC's ip address in there.

Now you can run the script, start the app and start sending (play button). Press 'd' on your pc to send default values to your phone. 

## Phone mounting
* Make always sure to adapt the mounting height of your phone within the calibration menu, if you want the estimated distances to make any sense. 
* The tilt angle of the phone is automatically accounted for, nothing to do here.

## Troubleshooting
* When using a camera based sensor, always make sure that there is nothing of the bot visible to the camera!
* Remove Serial Sensor from battery optimizations inside the Android settings
* For other problems, check the app internal help.

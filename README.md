# M5STACK_TAB5_USB_HOST_UVC

## Overview

<table>
<tr><td valign="top">

This is part of my bigger project but some might find it useful. This example basically takes preview of camera and displays it on m5stack tab5 display. My camera as UVC DEVICE is esp32p4 with OV5647 camera, so basically these are two esp32p4 boards talking to eachother. Currently only mjpeg receiving is enabled. Further resolutions and formats need testing.

</td><td width="200" valign="top">
  <img src="/examples/display/doc/pic1.jpg">
  <img src="/examples/display/doc/pic2.jpg">
</td></tr>
</table>

## Build and Flash

This example is based on ESP-BSP for m5stack tab5 with lvgl disabled (although it still is present in componenets) and uvc_host from ESP-IDF. Build on ESP-IDF v5.5.3

## Known issues and limitations

Currently no further testing for different resolutions than 320x240 @15fps, mjpeg, number of urbs and urb sizes needs to be adjusted for different resolutions.

Checkboard artifact is from uvc device due to PPA, tearing is different issue - already solved.

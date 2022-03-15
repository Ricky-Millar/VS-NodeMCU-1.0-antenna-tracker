# VS-NodeMCU-1.0-antenna-tracker

This project is in early stages

I am trying to get an antenna-tracker to follow my drone based on its gps coordinates through mavlink.
This is the first I have seen MAVlink used in a TCP client but overall its working 90% at the moment

I'm using an esp8266 pluged into 2 cheap servos and will eventualy include some form of magnetomiter or imu to as a reference angle for the tracker.

I designed this servo mount to work with the servos I have at home, I'll include a case for the elecronics once I've finalised what hardware I'll use. https://www.thingiverse.com/thing:5279876


The code is all over the place at the moment, any questions let me know! I sure had a hard time working out how to get mavlink going through tcp. a lot of the librarys that say they can do it appear not to.

Current state of tracker -> https://photos.app.goo.gl/4SRZVt3ofeJafCrg7 ( sorry the video is kinda crusty)
In this video my drone is running betaflight, with just GPS, no magnetomiter or barometer, so the altitude measurments are way out, and at close range the tracker is having a hard time. Once I'm out of Covid isolation I will try to do some longer range videos.




# TIP FOR BETAFLIGHT:
## If you want to get real in the deep end...

If you are running betaflight firmware, The rate that gps is transmitted is locked at 2hz, there is no way to change this with mavlink. to fix this you have to make a custom build of betaflight. I use the docker method:
https://hub.docker.com/r/betaflight/betaflight-build

https://www.youtube.com/watch?v=nr5SqZvpMRI&ab_channel=UAVTech

Before building custom hex files, within "src\main\telemetry\mavlink.c" you should change:
 [MAV_DATA_STREAM_POSITION] = 20
this will set the data stream to 20hz,

## Configuring U-Blox gps modules
this next part is very experimental and might cause problems I havent found yet. but I am also playing aroudn with changing the data-rate between my ublox gps module and the FC. 
to do this go into "src\main\io\gps.c"
then find:
ubloxSetNavRate(0xC8, 1, 1)
the "0xC8" is 200, for 200ms between transmissions. I have set this to 50 (0x32).  (this is probably overkill but I want to see if anything goes wrong...)

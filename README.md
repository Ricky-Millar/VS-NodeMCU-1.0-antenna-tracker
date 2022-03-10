# VS-NodeMCU-1.0-antenna-tracker

This project is in early stages

I am trying to get an antenna-tracker to follow my drone based on its gps coordinates through mavlink.
This is the first I have seen MAVlink used in a TCP client but overall its working 90% at the moment

I'm using an esp8266 pluged into 2 cheap servos and will eventualy include some form of magnetomiter or imu to as a reference angle for the tracker.

I designed thsi servo mount to work with the servos I have at home, I'll include a case for the elecronics once I've finalised what hardware I'll use. https://www.thingiverse.com/thing:5279876


The code is all over the place at the moment, any questions let me know! I sure had a hard time working out how to get mavlink going through tcp. a lot of the librarys that say they can do it appear not to.

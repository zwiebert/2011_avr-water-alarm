avrdude -c avrisp -p m8 -P '\\.\COM4' -b 19200 -U 'flash:w:default/water_alarm.hex:i' 

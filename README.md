# inkyphat-esphome
A component and sample configuration for the red/black/white 2.13 in inkyphat from pimoroni for raspberry pi

It is derived from a combination of the waveshare_epaper display driver which forms part of the esphome standard component library and some reverse-engineering of the inkyphat python library for raspberry pi. 

The chip is unique to this display and based on the ssd1608 display but with support for 3 colours using a different LUT from the
standard one used by wavesshare

It takes almost a minute to do an update ( no reliable support for partial updates ) and there is a lot of potential for timeouts
and other nasties, so the provided example config uses some scripts to try and prevent too many attempts to update the
display before it's ready and batch up updates received in the meantime.

That said, this technology is not suitable for a real-time display. If what you display changes often, 
you will spend a lot of time watching the display refresh and not much seeing the data. 

My use-case involves getting data reminders about the day and updating hourly. The chip goes to sleep for most of the time.

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

For reference, the wiring of the inkyphat is at https://pinout.xyz/pinout/inky_phat 
The pins are on a 40 (20x2) pin female connector designed to merge with a raspberry pi and the numbers are the same as on a pi.
Remember you will be looking at the female connector from the 'wrong' side compared to looking down on a raspberry pi. 
You need to connect all the below pins to appropriate ones on the esp board.

Pin 1 - 3.3v
Pin 2 - 5v
Pin 3 - i2c sda
Pin 5 - i2c scl
Pin 6 - ground
Pin 11 busy
Pin 13 reset
Pin 15 data/command
Pin 19 spi MOSI
Pin 23 SPI SCLK
Pin 24 Chip select

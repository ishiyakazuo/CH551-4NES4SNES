# CH551-4NES4SNES
4NES4SNES from Raphnet (Original: https://www.raphnet.net/electronique/4nes4snes/index_en.php) ported to WCH CH551G microcontroller
NOTE: I didn't realize the original was on Github when I started this, which is why this isn't a fork... original code is up at https://github.com/raphnet/4nes4snes

# Why?
Many reasons:
- CH551 is insanely cheap.  The chip is less than USD$0.25 in small quantities.
- It supports hardware 12Mbps USB (no V-USB hackery).
- It has hardware support for 4 interrupt IN endpoints, and can be configured to look like 4 different controllers to all sorts of devices.
  - This allows Linux to see 4NES4SNES as 4 controllers without a kernel patch!  (Tested in Linux Mint 19 with a different USB VID/PID)
- This is an example of how to support multiple gamepads (and can easily be dumbed down to support 1 or 2 gamepads).  Someday, I might parameterize this further to make it more obvious how things need to change to adjust for fewer controllers.

# How?
I started with the following:
- CH552/551 datasheet in Chinese, which I liberally Google Translated
- The USB HID example on http://atcnetz.blogspot.com/p/downloads.html
- The USB Composite Device example at https://github.com/rikka0w0/CH55x_USB_CompositeDevice
- And of course, Raphnet's 4NES4SNES v1.5 code (available at https://www.raphnet.net/electronique/4nes4snes/index_en.php)
The main code is based largely on the USB HID example, with a number of tweaks made (actually, a LOT of tweaks made), as the example only had two of the four non-control endpoints used.  Figuring out the quirks to make endpoint #4 work as the others was important, as it doesn't have its own DMA address directly (it's always endpoint #0 + 0x40), and it doesn't support the auto-DATA0/DATA1 toggle that the other endpoints do.
I decided early on that this was an all-or-nothing thing.  I wanted 4 controllers, and I wanted them to work in Linux without a kernel patch.  I figured that if push came to shove, I'd program this thing to act as a USB Hub and emulate accesses to 4 virtual downstream devices, but thankfully, that doesn't seem to be necessary.
This code uses the VID/PID I obtained from OpenMoko a number of years ago for my 5NES5SNES variant in 4 controller/12 button mode.  Because of this, I also enabled my 12 button mode here.

# Wait, 12 buttons?  But an SNES controller only has 8...
You're right!  But I originally built my 4NES4SNES variant to support 12 buttons because I made my own "SNES" controllers (two 4021 shift registers cascaded), which allows me to access the additional 4 buttons.  You can use actual SNES controllers with this version as well, but buttons 9-12 will never fire.  (It's sort of the same as using NES controllers with it -- you'll only ever see 4 of the buttons fire, even though more than 4 show up to the OS.)

# Current Status
- USB: Done!
- NES/SNES controller interface: Untested, but probably likely to work (logic is identical to the original 4NES4SNES, I just redefined the macros that access the physical pins).  I do plan to test this in the near future.

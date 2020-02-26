# CH551-4NES4SNES (Plus 4Play for PSX and PS2 controllers)
- Based originally on 4NES4SNES from Raphnet (Original: https://www.raphnet.net/electronique/4nes4snes/index_en.php) and ported to WCH CH551G microcontroller (with somewhat different behavior -- see below).
- Newest code has preliminary support for PlayStation controllers -- only one type of controller can be used at a time, and each one requires their own build.
  - In other words, the code can support either PSX controllers or SNES controllers, but not both.

## NOTE:
I didn't realize the original was on Github when I started this, which is why this isn't a fork... original code is up at https://github.com/raphnet/4nes4snes

# Pinout / "Schematic" (SNES version)
                   Pin 1 
    (Latch)    P3.2 - o     - 3.3V (connect to GND via 0.1uF)
    (Data #1)  P1.4 -       - 5.0V (connect to USB VBUS, connect to GND via 0.1uF)
    (Data #2)  P1.5 -       - GND  (to USB GND)
    (Data #3)  P1.6 -       - USB D-
    (Data #4)  P1.7 -       - USB D+
              Reset -       - P3.4
    (Multitap) P3.1 -       - P3.3 (Clock)
    (LED)      P3.0 -       - P1.1
- It should be noted that this is pretty much the entire schematic.  You only need to connect two 0.1uF decoupling capacitors (one between 5.0V and GND, and one between 3.3V and GND), 
- If you want to see the blinky LED (which is only really needed for debugging to ensure that the chip is programmed and running), you'll need an LED and resistor (1K resistor is probably suitable).
- Logic is +5V like the original design.
- To reprogram the chip at some point (for example, if the flashing doesn't go right and you need to retry with a different tool...), connect 3.3V to USB D+ (pins 16 and 12) via 10~20K resistor when connecting the USB to your host machine.
  - I'll just note here that if you end up with a CH551 with bootloader 2.3.1 (the latest as of this writing), you may want to grab one of the tools in my repos, or the original programming tool from WCH's website, because most of the tools out there do not work with CH551, only CH552.  (This code should also work just fine with CH552, though CH551 is about 30% cheaper and will work equally well, as even the CH551 is only about half full in terms of flash and RAM.)

# Pinout / "Schematic" (PSX version)
                   Pin 1 
    (Command) P3.2 - o     - 3.3V (connect to GND via 0.1uF)
    (ATT #1)  P1.4 -       - 5.0V (connect to USB VBUS, connect to GND via 0.1uF)
    (ATT #2)  P1.5 -       - GND  (to USB GND)
    (ATT #3)  P1.6 -       - USB D-
    (ATT #4)  P1.7 -       - USB D+
             Reset -       - P3.4
    (Data)    P3.1 -       - P3.3 (Clock)
    (LED)     P3.0 -       - P1.1 (ACK)
- For this version, this assumes that whatever PlayStation controllers are being used are okay with +5V logic throughout.  I believe it may be safer to add a +3.3V regulator and tie both pins 15 and 16 to it (with appropriate decoupling).

# Why?
Many reasons:
- CH551 is insanely cheap.  The chip is less than USD$0.25 in small quantities.
- Way fewer parts!  Only 2 decoupling caps and a USB connector, makes it way easier to build!
- It supports hardware 12Mbps USB (no V-USB hackery at 1.5Mbps).  This means faster response times, as the poll interval can be set as low as 1ms!
- It has hardware support for 4 interrupt IN endpoints, and can be configured to look like 4 different controllers to all sorts of devices.
  - This allows Linux to see 4NES4SNES as 4 controllers without a kernel patch!  (Tested in Linux Mint 19 with a different USB VID/PID.  Latest code still works with Windows as well without issue.)
- This is an example of how to support multiple gamepads (and can easily be dumbed down to support 1 or 2 gamepads).  I've parameterized this code to make it more obvious how things need to change to adjust for fewer controllers.  (4 is the maximum.)

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
- It works!  SNES controller support tested on actual hardware as of 2020/02/09.
- NOTE: If downloaded prior to 2020/02/16, there was an issue with Windows PCs detecting it properly.  New version fixes this.
- PlayStation controller digital inputs work, but the analog sticks and buttons do not yet work properly.  (Nothing works when NUM_AXES is defined to be 6 or 8 -- define NUM_AXES as 2 for digital only mode.)

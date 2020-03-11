# HW design notes:
- For PlayStation use, populate all components, short SJ1 and short pins 2 and 3 of SJ2.
- For SNES use, do not populate the regulator, or the 4.7uF capacitors.  Short pins 1 and 2 of SJ2, do not short SJ1.  SV5 is not used, and SV6 only goes to the controller port connected to the multitap (which I believe is the first port).

# Things I might go back and fix
- The pins aren't aligned to a 0.1" layout.  I use a 0.1" protoboard for lining up and holding header pins in place while soldering, so this was something I regretted when I soldered up the first batch.
  - Update: I did fix this on the mini variant, so if you want to connect up 0.1" header pins, rather than the screw terminals, I'd recommend going with that version from now on.

# Things I did go back and fix since rev. 1
- Jumpers for selecting VCC are much more simplified now.

# Other comments:
- TVS diode is optional.  The TVS diode I use is only about $0.02 from LCSC.
- The 22 ohm resistors on the USB lines are just for safety.  Solder-blobbing across the pads also works just fine on multiple setups I've tried.
- The 10K resistor (which can be populated with anything in the 10K-20K range) only needs to be populated if you want to be able to reprogram the board.  Shorting J1 pins 1 and 2 together will cause USB D+ to be pulled to 3.3V via this resistor -- only short it when you plug in the board, and remove the jumper as soon as connected to the PC.  (You may want to connect up a momentary pushbutton across J1 and just hold it as you plug in, and let go.)
- The 5mm screw terminal ports are useful for stripping and wiring some cut up SNES extension cables into.  SNES connectors are fairly scarce, compared to PSX ones.
- Want cheap PSX controller connectors?  Type "disgree" into eBay.  (Don't ask how I figured this out... long story.)  You can often find them 2 for US$0.99 (or less if you buy more than one pair)  Typing "disgree" into Aliexpress may also work, if eBay supply dries up.
- Two pins of the USB port are connected to GND.  I'd advise not populating the pin corresponding to the ID pin if you're using a micro-B or mini-B USB breakout board.  I did it this way so that GND is going to be in the right place for a pin, regardless of whether you use a 4 pin or a 5 pin breakout, but I'm not sure if tying the ID line to GND will do something unexpected.

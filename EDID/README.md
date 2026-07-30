# EDID for SHARP LQ5AW136 in COMAND 2.0 / 2.5

  

It is perfectly integrated into the COMAND 2 display for use via RGBs TV input


<p align="center">
  <img src="https://github.com/niz93/Open-TV-COMAND/blob/main/pic/CP_on_COMAND.jpg" alt="CP on COMAND" width="70%" />
</p>

  

## Why 230p and 460i

  

The LQ5AW136 has a native 320x234 resolution, but in COMAND 2 frame it cuts the visible vertical resolution by ~4 pixels, so only 230 pixels are used.

  

The original TV tuner uses 460 interlaced resolution, but this causes flickering, so it is preferable to use 230 progressive resolution.

  

However, the 230p uses too low a pixel clock and not all video adapters can work with this resolution. Read the solutions below.

  

## H-V to S synk

  

Use ONLY AND logic for mixing H and V synk

  

Another mix method will cause offset images on the display, they work, but you will need to adjust the image yourself.

  

VGA - COMAND scheme:

  <p align="center">
  <img src="https://github.com/niz93/Open-TV-COMAND/blob/main/pic/hv-s.jpg" alt="VGA - COMAND scheme" width="70%" />
</p>
  

You need:

  

0) HDMI-VGA adapters

1) VGA D-SUB 15HDP Famale connector (DS1035-15F)

2) D-SUB 9P housing (DS1047-09-M2L-N)

3) 10kOm Resistor (any hole-mounted resistor)

4) NPN transistor (2n3904)

5) A0345457528 (connector for COMAND) (if you have a phone or CD changer installed, it is already in the car, otherwise you need to buy)

6) 5 pieces Nano MQS (2-1703930-1)

  
  

## Nuances of choice HDMI-VGA adapters

  

EDID profiles for VGA panels whose pixel clock is below HDMI's 25 MHz minimum, which means they cannot be driven over HDMI as-is.

  

Some VGA adapters can work without problems, but only if the video core supports it. But some may not work as is, even if the video core provides frame generation.

  

on some adapters it is possible to work without additional manipulations with the correct EDID:

  

1) AG6200 - works well 230p and 460i without path

2) MS9291/MS9292 - wokrs only with path

3) ... if you know the adapters that are working and not working with the patch, write to me.

  
  

## Path for vc4 KMS Raspberry Pi

  

For Raspberry Pi, you can multiply the frequency by pixels, read more https://github.com/f-io/LIVI/tree/main/assets/displays

  
  

## Change EDID

  

1) There is an EEPROM inside the adapter, but as a rule they are protected from overwriting, however, you can try using EDID-DisplayID-Writer https://www.monitortests.com/forum/Thread-EDID-DisplayID-Writer

  

2) You can change EDID with path (read above)

  

3) You can replace the EDID yourself

  
`sudo mkdir -p /lib/firmware/edid`
`cd /lib/firmware/edid`
`sudo wget *link EDID file*`
`sudo nano /boot/firmware/cmdline.txt`
  

insert in end string:

  

`drm.edid_firmware=HDMI-A-1:edid/Mercedes_COMAND2_LQ5AW136_320_230p_60Hz.bin,HDMI-A-2:edid/Mercedes_COMAND2_LQ5AW136_640_460i_60Hz.bin`

  

save, reboot

  
  

4) You can instel external i2c eeprom an write EDID with EDID-DisplayID-Writer

   <p align="center">
  <img src="https://github.com/niz93/Open-TV-COMAND/blob/main/pic/i2c_hdmi.jpg" alt="external i2c eeprom" width="70%" />
</p>
  

## Profiles

| File | Display | Native timing | On the wire (with path) |
|------|---------|---------------|-------------|
| `Mercedes_COMAND2_LQ5AW136_320_230p_60Hz.bin` | Mercedes COMAND 2, Sharp LQ5AW136 400x234 | 320x230p 60Hz @ 6.30 MHz | 4x to 25.2 MHz |
| `Mercedes_COMAND2_LQ5AW136_640_460i_30Hz.bin` | Mercedes COMAND 2, Sharp LQ5AW136 400x234 | 640x460i 30Hz @ 12.71 MHz | 2x to 25.42 MHz |
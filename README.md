
# Open TV for COMAND 2.0 or 2.5

This emulator TV tuner for Mercedes (Bosch) COMAND 2.0 or 2.5


## What is it

This emulator work over [MKS CANable V1.0](https://github.com/makerbase-mks/CANable-MKS), initializes TV mode and forwards the pressing of buttons on the panel to usb via HID keyboard

### How to use?

1) Install the BOOT jumper on CANable 1.0 
2) Open [DfuSeDemo ](https://www.st.com/en/development-tools/stsw-stm32080.html) (DFU USB device firmware upgrade)
3) Select in "firmwire" folder .dfu file
4) Connect CANable 1.0 via USB
5) Upgrade CANable 1.0
4) Disconnect CANable 1.0 
6) Remove BOOT jumper
7) Connect CAN from Comand and plug USB in PC
8) Use it! Press keyboard on comand for action on PC


### Table bends key COMAND - HID

| COMAND CAN key | HID key D-pad mode (default)*| HID key numeric mode* | Long press function** |
|----------------|:---------------------------:|:--------------------:|:-------------------:|
| 0 | KEY_KP0    |  KEY_KP0  |
| 1 | KEY_KP1    |  KEY_KP1  |
| 2 | KEY_DOWN   |  KEY_KP2  |
| 3 | KEY_KP3    |  KEY_KP3  |
| 4 | KEY_LEFT   |  KEY_KP4  |
| 5 | KEY_ENTER  |  KEY_KP5  | toogle key mode |
| 6 | KEY_RIGHT  |  KEY_KP6  |
| 7 | KEY_KP7    |  KEY_KP7  |
| 8 | KEY_UP     |  KEY_KP8  |
| 9 | KEY_KP9    |  KEY_KP9  |
| * | KEY_KPASTERISK |  KEY_KPASTERISK  | open sound setting for TV |
| # | KEY_KPDOT      |  KEY_KPDOT       |
| Back     | KEY_B |  KEY_B  |
| Forward  | KEY_N |  KEY_N  |
| Back Steering Wheel Key     | KEY_B |  KEY_B  |
| Forward Steering Wheel Key  | KEY_N |  KEY_N  |
| Press encoder | KEY_ENTER |  KEY_ENTER  |
| RET  | KEY_BACKSPACE |  KEY_BACKSPACE  |
| Encoder + rotary  | KEY_RIGHT |  KEY_RIGHT  |
| Encoder - rotary  | KEY_LEFT |  KEY_LEFT  |
| Open TV mode***     | KEY_F1 |  KEY_F1  |
| Close TV mode***    | KEY_F2 |  KEY_F2  |
| On background sound | KEY_F1 |  KEY_F1  |
| Mute                | KEY_F2 |  KEY_F2  |
| UNMute (same key)   | KEY_F1 |  KEY_F1  |
| Power off  | KEY_F5 |  KEY_F5  |

*hex HID code see in Open TV COMAND/Core/Inc/usb_hid_keys.h

**Press and hold the button for more than 5 seconds

***Pressing the TV button and any other



## Support

For support, email 193niz@gmail.com or  [drive2](www.drive2.ru/users/niz/) 


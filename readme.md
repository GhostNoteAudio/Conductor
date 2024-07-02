# Ghost Note Audio Conductor

![logo](logo.svg "Logo")

This repository contains the firmware for the Conductor MIDI controller, as well as the online programmer.

[**Open Mark II Editor**](https://ghostnoteaudio.github.io/Conductor/EditorMk2.html) - [**Open Classic Editor**](https://ghostnoteaudio.github.io/Conductor/Editor.html)


<p xmlns:cc="http://creativecommons.org/ns#" xmlns:dct="http://purl.org/dc/terms/"><a property="dct:title" rel="cc:attributionURL" href="https://ghostnoteaudio.uk/products/conductor">Conductor</a> by <a rel="cc:attributionURL dct:creator" property="cc:attributionName" href="https://ghostnoteaudio.uk/">Ghost Note Audio</a> is licensed under <a href="https://creativecommons.org/licenses/by-nc-sa/4.0/?ref=chooser-v1" target="_blank" rel="license noopener noreferrer" style="display:inline-block;">CC BY-NC-SA 4.0<img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/cc.svg?ref=chooser-v1" alt=""><img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/by.svg?ref=chooser-v1" alt=""><img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/nc.svg?ref=chooser-v1" alt=""><img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/sa.svg?ref=chooser-v1" alt=""></a></p>

## Instructions for developers

### Setup

You will need to install the following items:

* Arduino IDE (version 2.2 or later)
* Arduino Pico board support
  * https://arduino-pico.readthedocs.io/en/latest/install.html
* MIDI Library by Francois Best and lathoub
  * https://www.arduino.cc/reference/en/libraries/midi-library/
  * Use the library manager in the IDE to install it.

Note: The project uses the Adafruit TinyUSB Library. This is included in the Pi Pico Arduino IDE board setup, select this from Tools->USB Stack. *DO NOT* install this library separately. If you already have it installed in your IDE you may need to remove it, otherwise you get errors complaining about duplicated code.

### Build and upload the firmware

0. Open the Firmware-V3.ino file in Arduino IDE
1. Connect the Conductor unit to your computer via USB
2. Choose the Pi Pico as the target board
    * Tools -> Board -> Raspberry Pi Pico/RP2040 -> Raspberry Pi Pico
3. Configure your settings to match the following screenshot (most likely they are all identical by default, except the USB Stack)
![settings](docs/settings.png "Settings")
4. Choose the connected board in the dropdown menu<br/>
![board select](docs/boardselect.png "Board Select")<br/>
Note that if you have multiple serial port devices or Pi Pico controllers connected, it may not be obvious which is which. You may just have to try until you find the right one :)
5. Hit Upload
6. (sometimes necessary) Disconnect and reconnect the controller from the computer - this allows the USB stack to reset and present itself as a MIDI class compliant device


### FAQ

**Q:** My controller is unresponsive following a failed upload. What do I do?

**A:** You need to reset the Pi Pico using the on-board reset button. Disconnect the USB cable, press and hold the button, then re-apply power / connect the cable. This should let the Pico boot into its bootloader mode.

**Q:** How do I enter bootloader mode on the Mark II model?

**A:** The page switch on the front panel acts as a reset button during startup. Press and hold the page button down while connecting the USB-C cable, and the device will enter bootloader mode, and present itself as a flash drive on your computer. You can then
copy the .UF2 firmware file to the flash drive, and the device will reset and load the new firmware.

**Q:** How do I enter bootloder mode on the Conductor Classic?

**A:** You need to open the unit up. Remove the four silver socket screws on the front panel using a hex/allen key. You can then remove the top panel and circuit board from the enclosure. There are small tabs in each corner which prevent the circuit board from lifting out directly. You must lift the lower side out first. It's easiest to lift one corner, and once the circuit board is free, the whole thing will come out. Be patient and careful :) Once you have the device open, press and hold the reset button on the Raspberry Pi Pico board while powering up the unit.
The device will enter bootloader mode, and present itself as a flash drive on your computer. You can then
copy the .UF2 firmware file to the flash drive, and the device will reset and load the new firmware.

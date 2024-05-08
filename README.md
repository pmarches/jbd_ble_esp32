Use a ESP32 as a bridge between venus GX serialBattery and BLE BMS based on the JBD protocol.

The victron octoGX has a pretty bad bluetooth harsware module. SerialBattery (https://github.com/Louisvdw/dbus-serialbattery) has support for bluetooth connected BMS that speak the JBD serial protocol. However, I found this setup consumes a lot of CPU and is unreliable.

This project uses a common ESP32 to connect BLE BMS that speak the JBD protocol to SerialBattery. 

Reference:
https://gitlab.com/Overkill-Solar-LLC/overkill-solar-bms-tools/-/blob/master/JBD_REGISTER_MAP.md
https://github.com/kolins-cz/Smart-BMS-Bluetooth-ESP32/blob/master/BMS_process_data.ino
https://wiki.jmehan.com/download/attachments/59114595/JBD%20Protocol%20English%20version.pdf?version=1&modificationDate=1650716897000&api=v2

ISSUES:
- Can't connect twice to get the logs. nc BLEBMS-4A4D60.local 85 just hangs, no output.
- Cell values 0,1,2 stay zero
- Sometimes, it only shows data from one battery. Does not deal with BLE disconnects I think
- Percentage is wrong after a few hours of operation. Restart of python serial-battery does not improve things. Reboot of ESP32 fixes the issue for a while.

TODO:
- Improve log messages. Know to the context: BATTERY1, BATTERY2, GX comm, etc..
- Actually read EEPROM settings from batteries. How to compute them?
- Having trouble with the serial battery protocol. It might be different than the JBD protocol??

HEX Protocol
----
https://www.victronenergy.com/upload/documents/BMV-7xx-HEX-Protocol.pdf

Testing:
socat -x TCP-LISTEN:100 PTY,raw,link=/dev/ttyS10
/opt/victronenergy/vedirect-interface/start-vedirect.sh ttyS10 PRODUCT=builtin-vedirect

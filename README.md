Use a ESP32 as a bridge between venus GX serialBattery and BLE BMS based on the JBD protocol.

The victron octoGX has a pretty bad bluetooth harsware module. SerialBattery (https://github.com/Louisvdw/dbus-serialbattery) has support for bluetooth connected BMS that speak the JBD serial protocol. However, I found this setup consumes a lot of CPU and is unreliable.

This project uses a common ESP32 to connect BLE BMS that speak the JBD protocol to SerialBattery. 

Reference:
https://gitlab.com/Overkill-Solar-LLC/overkill-solar-bms-tools/-/blob/master/JBD_REGISTER_MAP.md
https://github.com/kolins-cz/Smart-BMS-Bluetooth-ESP32/blob/master/BMS_process_data.ino
https://wiki.jmehan.com/download/attachments/59114595/JBD%20Protocol%20English%20version.pdf?version=1&modificationDate=1650716897000&api=v2

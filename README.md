Use a ESP32 as a bridge between venus GX serialBattery and BLE BMS based on the JBD protocol.

The victron octoGX has a pretty bad bluetooth harsware module. SerialBattery (https://github.com/Louisvdw/dbus-serialbattery) has support for bluetooth connected BMS that speak the JBD serial protocol. However, I found this setup consumes a lot of CPU and is unreliable.

This project uses a common ESP32 to connect BLE BMS that speak the JBD protocol to SerialBattery. 

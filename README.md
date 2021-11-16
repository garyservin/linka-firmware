# linka-firmware

ESP8266 firmware for [AireLibre](https://github.com/melizeche/AireLibre)

## Installation

You can use [ PlatformIO ](https://platformio.org/platformio-ide) or [ Arduino IDE ](https://www.arduino.cc/en/software) to upload the firmware

### PlatformIO

Install [ PlatformIO ](https://platformio.org/platformio-ide) and...

#### ... in Visual Studio Code

- Open the project with vscode having the platformio plugin installed
- Click on the right arrow icon on the platformio toolbar

![Right Arrow Icon on Platformio Toolbar in Visual Studio Code](/doc/img/vscode_pio_upload.png)

#### ... or in your terminal

```bash
platformio run -t upload
````

#### ... if you want to upload via OTA, add the following lines to the `platformio.ini`

and the code will compile and upload to your connected device (it should be detected automatically)

```bash
upload_protocol = espota
upload_port = <IP_ADDRESS_OF_YOUR_SENSOR>
````

and upload just like the previous method.

### Arduino IDE

Install [ Arduino IDE ](https://www.arduino.cc/en/software) and...

- Open **File -> Preferences** and copy the following urls on the __"Additional Boards Manager URLs"__ input text

```
https://arduino.esp8266.com/stable/package_esp8266com_index.json,https://dl.espressif.com/dl/package_esp32_index.json
```

![Additional Boards URLs on arduino IDE preferences](/doc/img/arduino_ide_aditional_boards.png)

and click on **Ok**

- Install the esp8266 boards family on **Tools -> Boards Manager**

![Installation of esp8266 boards on arduino IDE](/doc/img/arduino_ide_board_installation.png)

- Select the corresponding esp8266 board on **Tools -> Board:** for example the **"LOLIN(WEMOS) D1 R2 & mini"** 

![Board selection in arduino Tools menu](/doc/img/arduino_ide_board_selection.png)

- Install these libraries by clicking on the **Tools -> Manage Libraries...**

  * WifiConnect Lite
  * PubSubClient (by Nick O'Leary)
  * ArduinoJson (important! select a version previous to 6.0.0, 5.13.5 i.e.)

- Select the correct port on the **Tools -> Port:** menu

![Port selection in arduino Tools menu](/doc/img/arduino_ide_select_port.png)

- Upload the sketch using the arrow button or by selecting the **Sketch -> Upload** menu

![Upload sketch in arduino Tools menu](/doc/img/arduino_ide_upload_sketch.png)

## Enjoy!


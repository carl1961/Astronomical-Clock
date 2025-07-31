# Astronomical Clock By illusionmanager
https://www.instructables.com/Astronomical-Clock-1/  

Read and follow illusionmanager instrutions

This project is 150% size of original

## Features (Optional)

- **Easy WiFi Configuration:** No need to hardcode WiFi credentials.
- **Automatic Reconnection:** The device automatically tries to connect to the last known WiFi network on startup.
- **Display** No need to connected to computer to see progress.
## Hardware Requirements

- ESP32-C3 Super mini Development Board

## Pinout

- ESP32-C3 Super mini

 ESP32-C3 Super Mini Pin
 
(*) or Optional

VCC	5V       PIN 5

GND	         PIN 6

VCC	3V3      PIN 7

Pin 4 *      PIN 8

Pin 3 *      PIN 9

Pin 2 *      PIN 10

Pin 1 *      PIN 20

Pin 0 *      PIN 21


TMC2209

originl setup

EN             VM 5V

MS1            GND

MS2            M2B  orange wire stepper motor 28BYJ-48

SRD            M2A  pink wire stepper motor 28BYJ-48

RX  PIN 9      M1A  yellow wire stepper motor 28BYJ-48

TX             M1B  blue wire stepper motor 28BYJ-48

STEP PIN 6     VIO 5V

DIR  PIN 5     GND


I had to reverse motor, reverse wires on stepper motor 28BYJ-48


EN             VM 5V

MS1            GND

MS2            M2B  blue wire stepper motor 28BYJ-48

SRD            M2A  yellow wire stepper motor 28BYJ-48

RX  PIN 9      M1A  pink wire stepper motor 28BYJ-48

TX             M1B  orange wire stepper motor 28BYJ-48

STEP PIN 6     VIO 5V

DIR  PIN 5     GND



- ESP32 C3 with ILI9341 display (Optional)

TFT Display	   ESP32-C3 Super Mini Pin

VCC				      	5V

GND		    			GND

CS				     	Pin 3

DC			      	Pin 1

RST			    		Pin 0

SCK					    Pin 2

SDI (MOSI)  Pin 4

LED VCC				3V3

## Installation & Usage

1.  **Install Arduino IDE and ESP32 Core:** Make sure you have the Arduino IDE installed and the ESP32 board support package added.
    Arduino IDE, go to File/Preferences and add "https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_dev_index.json" 
2.  **Install Libraries:** Open the Library Manager in the Arduino IDE and install the following library:
    -   `WiFiManager` by tzapu
3.  **Configure Pins (Optional):** If you are using different GPIO pins.
4.  **Upload the code:** Connect your ESP32 board to your computer, select the correct board and port in the Arduino IDE, and upload the sketch.

## AP Option

1.  **Initial Setup:**
    -   On the first boot, or if a WiFi connection fails, the ESP32 starts a WiFi Access Point with the SSID **"AstroClockAP"** and the password **"12345678"**.
    -   An LED will blink to indicate it's in configuration mode.
2.  **Configuration:**
    -   Connect to the "AstroClockAP" WiFi network from your computer or smartphone.
    -   Once connected, a captive portal should automatically open in your web browser. If not, open a browser and navigate to `192.168.4.1`.
    -   Select your home WiFi network (SSID) from the list, enter the password, and click save.
3.  **Connection:**
    -   The ESP32 will then try to connect to the network you configured.
    -   If successful, the "Connected" LED will light up.
    -   The WiFi credentials are saved in non-volatile memory and will be used for future connections.
4.  **Resetting WiFi:**
    -   To clear the saved WiFi credentials, press and hold the "Reset WiFi" button. You can do this either when the device is booting up or while it is running. The device will restart and enter configuration mode again.

## Dependencies
For Optinal Wifi AP
-   [WiFiManager](https://github.com/tzapu/WiFiManager) 
-   [Setting  TZ](https://github.com/esp8266/Arduino/blob/master/cores/esp8266/TZ.h)
-   
For Optional Display
-   [Adafruit_ILI9341](https://github.com/tzapu/WiFiManager) 
-   [Adafruit_GFX](https://github.com/tzapu/WiFiManager) 

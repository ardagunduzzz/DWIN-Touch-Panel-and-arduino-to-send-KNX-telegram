DWIN Touch Panel Documentation ENG
This document is a technical guide explaining how to use the DWIN touch panel, Arduino Mega 2560, and the KNX communication protocol in an integrated system. The system allows the user to send values selected on the touch screen to a building automation system via the KNX protocol.
To use the DWIN screen, first download the DGUS Studio software. Create a new project, and immediately generate a custom font guide by selecting “0#word bank generating”. Afterward, go to the software's file location, sort the files by "date modified", and you’ll see a file named “0_DWIN_ASC.HZK”. Move this file into the DWIN_SET folder inside your project directory.
Before doing this, download the pages you’ll use on the touch panel in .png format. You can use simple tools like Paint or more advanced software to design your interface. The images must be in .png format — otherwise, DGUS won’t recognize them.
Once your font guide is ready, in the DGUS welcome section, click on DWIN ICL generation to load your interface images, animations, or icons. Important points to keep in mind:
•	Images must be saved in folder 32.
•	Icons should be saved in folders 40 to 63 (e.g., 40, 41, ..., 63). If DGUS doesn’t recognize the icons, you probably used an incorrect or out-of-range folder number.
•	The images and icons will be saved in files like 32.icl inside your project folder.
•	All file names must start with numbers like 00, 01, 02... for both images and animated icons; otherwise, DGUS cannot configure them.
After loading your interface images into DGUS, you can begin editing the visuals and adding elements such as touch modules or RTC displays. To add a touch function, use the Basic Touch Module and drag it to the desired screen location. If your screen doesn’t include an RTC battery, use address 0x0010; otherwise, use 0x009C. Then, under Page Switching, define which page should be opened when that touch area is pressed.
Let’s assume you're on a "Fan" page and you have a fan animation icon with "+" and "–" buttons. You can configure these using the Incremental Adjustment module and define the behavior, limit values, and whether it should increment or decrement. The most important part here is the VP address — for example, you can assign it to 0x1200. This address allows Arduino to interact with the touch module. You also need to use the Data Variable Display to show the changing value, and this should use the same VP address. Keep note of all these VP addresses, as elements like temperature, fan, and curtain controls will communicate based on them.
There is also something called SP (System Pointer), which refers to internal system parameters. These are not of concern to engineers — we only deal with VP addresses.
If your screen does not include an internal RTC battery, you can program Arduino to update the time every second via serial communication. If your screen includes a battery-powered RTC, this step is unnecessary.
After assigning all VP addresses and setting up the touch modules, go to File > Save, then click Generation. Next, select Config Generator. A new window will appear — in my case, it was the T5L module. Make all necessary settings here. If you're using CRC, toggle it on or off depending on your preference.
🚨 One very important setting is:
Set “Touch-sensitive variable changes update” to Auto.
If you don’t do this, the screen won’t send updates to the processor when a VP variable is changed. For example, if you increase the fan speed, a command must be sent to the Arduino. If Auto is not enabled, this update won’t happen.
By default, the baud rate is 115200, but I recommend setting it to 9600 for best performance with Serial Monitor.
Once all settings are complete, click NEW CFG in the bottom-right corner, go to your project folder, and save the config file as T5LCFG.CFG inside the DWIN_SET folder.
Now your folder is ready to be transferred to the screen. You’ll need a microSD card with a maximum of 16GB. Insert the SD card into your computer and copy only the DWIN_SET folder onto it. After copying, safely eject the SD card.
Power off your DWIN screen, insert the SD card, then power the screen back on. You will see a blue screen with installation messages. Wait until the installation is complete. Then power off and back on again — the new interface will be successfully loaded.
Arduino – DWIN Communication
In my setup, I used Serial2 on the Arduino Mega 2560 to establish communication with the DWIN screen using UART (serial protocol). The data packet format is as follows:
• Header1: 0x5A
• Header2: 0xA5
• Data Length: e.g., 0x05
• Command Byte: 0x82 (write), 0x83 (read)
• VP Address: 2 bytes (e.g., 0x12 0x00)
• Data: 2 bytes (e.g., 0x00 0x03)
When a read command is sent, the screen responds as follows:
0x5A 0xA5 0x06 0x83 <VP Address High> <VP Address Low> <Data High> <Data Low>
Each component inside the DWIN panel has its own VP address that allows Arduino to recognize and handle the related data.
Example VP Addresses:
•	Fan Speed: 0x1200
•	AC Temperature: 0x1400
•	Curtain: 0x1500
•	Dimmer: 0x1600
•	Notification: 0x1000
Arduino Code Functionality
The Arduino code performs two main tasks:
1.	It communicates with the DWIN screen to read user-defined values from VP addresses.
2.	It converts those values into valid KNX telegrams and sends them over the KNX bus. 
Required Libraries:

#include <Arduino.h>
#include <KnxDevice.h>
#include <Cli.h>
After establishing communication between Arduino and the DWIN panel, it’s essential to send the updated screen data as KNX telegrams. These libraries allow that.
⚠️ Note: This code only works in Arduino IDE. Other software development environments cannot compile it or load the necessary libraries.
________________________________________
Final Remarks
This system enables user inputs from the DWIN screen to be transmitted into a KNX network. It can be expanded to control lighting, temperature, curtains, and fans for larger-scale automation projects. While designing the system:
•	Ensure VP addresses do not conflict.
•	Use ETS software on the KNX side to correctly map and link addresses.
Below is a sample code block for your reference.
Good luck and happy building!

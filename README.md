# Stunning-Octo-Barnacle
SOB is a personal approach to creating a DMX controller for live show production
The main components are:

 ## Microcontroler
The first prototypes was made on the STMF103B8T6 but since it lacks of Timers, oversampling, FMC and memory now its time to make a few step ups and settle in the STM32H723ZG (Can be H723VG) to fullfill all the needs of the project.

 ## Motorized Faders
 The first prototype have 10 motorized faders that must function as what i call "Dmx Channel Control Array"
 fisically enumered from 0 to 9 and in program depends on the active page. In order to make this possible
 these are the requirements
 
- PID control
	- Individual PWM control
 	- Individual PWM sintonization
 	- Individual H-Bridge Control (L293D && MCP23017)
 		
- Page system
 	- DMX Channels page
 	- Chase/scene page
 	- Quick "Channel link"
 		
 ## DMX Universe
 It is fundamental to have at least 1 DMX universe output, that can be modified by the "DCCA", The DMX packet is created modifying
 a basic UART and passing it throuh a 6N137(to isolate and protect the MCU) en then a MAX485 to match the DMX standar. 
 
- Blackout
- Fade in/out
- Clear
- Scenes
- Chases
 	
 ## 7 inch touch tft display (SSD1963/XPT2046)
 The most effective (and easy) way to make a interface work would probably be the use of LVGL since his high compativility with HAL and premade stuff.
 The most importante function is create "Channel links" 
 
- GUI
- Fast touch utility (Link channels,Page select)
 	
 ## Encoders
 Basic  User control, works great for pan and tilt and a primary way to page change/select
- Quick Channel link (for focus, pan and tilt)
- Fast page selector
 	
 ## Buttons
Buttons are necessesary to give Quick access to functions.
- Blackout
- Clear
- Start record
- Save step
- Save Scenes
- Save Chase
- Delete 
- Call Scene / Chase
Currently working on: 10 motorized Faders, so basically its all smoke until i have something solid.
 

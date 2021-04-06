# Bicycle immobilizer
## General info
This project uses simple electromechanical lock, AVR ATmega 32a, RFID module NXP RC 522 and a small LCD screen to let you secure your bicycle with RFID card of your choosing.

## Technologies
Project was created with Atmel Studio 7.0.

## Sources
This code is partially based on library created by [@miguelbalboa](https://github.com/miguelbalboa/rfid) (release v.1.4.8).
It was created with partial involvement of one more person for university purposes.

## How it works
After successful setup the user is supposed to put their RFID card of choice near the RFID module in order to save it as the only acceptable card for the system. If saving is succesful, an appriopriate confirmation pops on the LCD screen. If the user puts their card near the module again, the lock will close and a confirmation will appear on the screen. If the process will be repeated, the lock would open accompanied by the message on the screen. If an unsaved card will be put near the module, an error message would appear on the screen and the state of the lock wouldn't change.

Microcontroller and RFID module communicate through SPI (Serial Peripheral Interface). Microcontroller and LCD screen communicate through IIC (Inter-Integrated Circuit)/TWI (Two-Wire Interface).

Working prototype can be seen [here](https://youtu.be/Nk5CqEz8z4s) and [here](https://youtu.be/4OXVpa1z-Do).

To do:
* figure out security issues
* find a way to use portable battery




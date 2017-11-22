EMV Reader Example for KYT-79xx
=======

The KYT-7916 Euro-Mastercard-Visa (EMV) reader can read magnetic strips as well as interface to the 'chip' in most modern credit cards. It is approximately $150-$250. It **can not** duplicate or modify the chip. 

This EMV reader is designed to work with ATMs. I needed it for some research into chipped cards so I hope others can benefit from it.

The reader has a 5 pin cable with the following connections to a microcontroller:  

* Red: 5V
* White: TX out from EMV reader
* Grey: RX in to EMV reader
* Blue: GND
* Yellow: GND (GND pins are connected together so pick one)

Power: 5V with ~ 1.5A when the solenoid ejects a card. Use an external 5V power supply with the Arduino or else the system will hang when a card is ejected.
  
Baud is 19200, 8N1. However the EMV reader expects RS232 not TTL serial. Luckily, softwareSerial library has option to invert signal and we are able to 'fake' RS232!


Interestingly: BCC = [Block Check Character](https://en.wikipedia.org/wiki/Block_check_character). Basically it's a CRC.

License Information
-------------------

The firmware is released under the [Beerware license](http://en.wikipedia.org/wiki/Beerware).
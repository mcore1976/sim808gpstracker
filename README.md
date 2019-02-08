# sim808gpstracker
DIY cheap GPS motorbike/car tracker based on  ATMEGA 328P (arduino uno chip) and SIM808 module from China (includes GPS and GNSS function). The total cost is below 20USD ( as in 2019 ) and positioning accuracy is ~1-5 meters ( tested in Europe location)

The device when called by mobile phone polls info from GPS module ( if can fix to sattelites - tries several minutes to fix) or when not available polls cell-id info from nearest 2G cell and  using GPRS  query Google servers for GPS location of that 2G cell. Collected location information is send back as text message to your phone as Google Map link. I have tried to keep the code as simple as possible and conserve battery power so functionality is rather limited... However...
The software can be customized to provide location in realtime to some HTTP POST /FTP server (there is short tutorial here https://www.raviyp.com/embedded/194-sim900-gprs-http-at-commands?start=1 ) - it is up to you to expand the code. 

The part list is (with the cost as in 2019):

a) SIM808 based board BK-SIM808 (10-12 USD on Aliexpress )
 - search for "www.amd-global.com" boards BK-SIM808 or equivalent...
   https://cdn.instructables.com/ORIG/FAO/80RU/IXLALERK/FAO80RUIXLALERK.pdf)
   it may also work with boards SKU405361-SIM808 (see description below for source code options)
   
b) GPS (passive) antenna with IPEX connector matching BK-SIM808 board - 2 USD

c) GSM antenna with IPEX connector matching BK-SIM808 board - 1 USD

d) ATMEGA 328P (arduino uno) - 2 USD

e) 3x 1N4007 (1 USD) - to convert 5V from powerbank to 3.3V for ATMEGA328P VCC ( only for BK-808 board and others that require TTL 3.3V logic)

f) 2x 1000uF / 16V capacitor ( 0.5 USD) - connect to VCC & GND of SIM808 board 
   AND to existing 100uF (parallel) on the SIM808 board

g) 100nF / 12V (or higher)  capacitor (0.2 USD) - connect to VCC & GND of SIM808 board

h) universal PCB, pins & connector (2 USD)

i) USB Powerbank 5V to make it work...

CONNECTIONS TO BE MADE :

1) SIM808 RXD (BK-SIM808 pin R) to ATMEGA328 TXD PIN #3,
2) SIM808 TXD (BK-SIM808 pin T) to ATMEGA328 RXD PIN #2
3) SIM808 DTR (BK-SIM808 pin S : SLEEP PIN) to ATMEGA328 PC5 PIN #28
4) SIM808 GND (BK-SIM808 pin G ) : to powerbank GND 
5) SIM808 VCC (BK-SIM808 pin V)  : to powerbank +5V VCC
6) SIM808 PWRKEY (BK-SIM808 pin K - left unused - it is internally bound to GND, however when breaking this connection it can be used to switch on/off whole SIM808 board)

OPTIONAL) if SIM808 RI/RING available - connect to ATMEGA328P INT0 pin #4 

7) Capacitor 1000uF between +5V and GND of powerbank 

8) put 3x 1N40007 diodes IN SERIAL between 5V VCC and ATMEGA328P VCC PIN #7 - ATMEGA must be powered from ~3.3V to adopt TTL logic of outputs TXD/RXD of SIM808 (BK-SIM808) board

9) put 100nF capacitor between ATMEGA328P VCC pin #7 and ATMEGA328P GND pin #8 & PIN#22

10) connect GPS passive antenna and GSM antenna to BK-SIM808 board. Probably it can work with active GPS antenna (but you would need to add another resistor for pullup antenna input to VCC - decribed here https://www.raviyp.com/embedded/205-sim808-gps-active-antenna-unable-to-acquire-fix-solution )

11) The AND-GLOBAL BK-SIM808 board has TO SMALL electrolytic capacitor (only 100uF). You have to solder/add another big capacitor (I have used 2200uF, but it can be 1000uF ) in parallel to make this board work correctly. Otherwise it will continously restart itself while trying to register to the 2G network.

To upload program code to the chip using cheapest USBASP programmer (less than 2 USD on eBay/Aliexpress) 
look at this page : http://www.learningaboutelectronics.com/Articles/Program-AVR-chip-using-a-USBASP-with-10-pin-cable.php

The script attached in repository ( "compileatmega" ) can be used to upload data to the chip if you have Linux machine with following packages : "avr-gcc", "avr-libc" and "avrdude". For example in Ubuntu download these packages using command : "sudo apt-get install avr-gcc" , "sudo apt-get install avr-libc",  "sudo apt-get install avrdude"  and you are ready to go. Then you can run the script "compileatmega" to upload via AVRDUDE & USBASP to ATMEGA328P

In the code you have to put correct APN, USERNAME and PASSWORD of GPRS access from your Mobile Network Operator before compiling - replace word "internet" with correct words for your MNO :

constchar SAPBR2[] PROGMEM = {"AT+SAPBR=3,1,\"APN\",\"internet\"\r\n"}; // Put your mobile operator APN name here

constchar SAPBR3[] PROGMEM = {"AT+SAPBR=3,1,\"USER\",\"internet\"\r\n"}; // Put your mobile operator APN username here

constchar SAPBR4[] PROGMEM = {"AT+SAPBR=3,1,\"PWD\",\"internet\"\r\n"}; // Put your mobile operator APN password here

SOURCE FILE OPTIONS :

"main.c"  (+ compilation script "compileatmega") - source file for SIM808 boards WITH DTR/SLEEP PIN exposed as BK-808 board. To use this file you will have to attach ATMEGA PC5 PIN #28 to SIM808 board DTR/SLEEP pin. 

"main2.c"  (+ compilation script "compileatmega2")  - source file for SIM808 boards WIHOUT DTR/SLEEP PIN exposed. To use this file you DO NOT connect ATMEGA PC5 pin to DTR SIM808.  Example of such board is this module SKU405361-SIM808 http://files.banggood.com/2016/06/SKU405361-SIM808.rar . These boards are also sold here : https://www.electrodragon.com/product/sim808-dev-board-gsmgprsgps-replacing-sim908/
Also pay attentiopn to type of TTL logic the board uses. If you want to use this board and 5V TTL logic do not put 1N4007 Diodes to ATMEGA328P. If you want to use .3V TTL logic on it, you will probably need to connect 3.3V from ATMEGA VCC (after 3x !N4007 Diode drop it from 5V) to VMCU PIN of SIM808 board to switch it to 3.3V TTL mode. It is in the boards manual.


The solution has low power consumption because it is utilizing SLEEP MODE on SIM808 module and switches on GPS only when needed.
I have found that on the board BK-SIM808 it is better to get rid of PWR LED (cut off)  because it is taking few mA of current thus unnecessary increasing power consumption - keep that in mind. Generally speaking SIM808 board is not so  power efficient as SIM800L because contains GPS/GNSS block.

The ATMEGA328P must be active all the time because my BK-SIM808 board does not have SIM808 RING/RI pin exposed (which can be used to wake up ATMEGA via hardware interrupt). In this design DTR pin of SIM808 module is used to sleepmode manipulation (when  coming out of sleepmode the DTR pin must be held LOW for at least 50msec). 
Measured power consumption for whole gps tracker is 14mA when SIM808 is in sleepmode with PWR LED on, but by getting PWR LED out the current lowers to 8mA.

When SIM808 module sends SMS/GPRS data it it may draw a lot of current ( up to 2A ) in short peaks so it is crucial to use good cables and thick copper lines for GND and VCC on PCB. This is the main issue people face when dealing with SIMCOM modules. The voltage may additionaly drop during this situation so that is why such big capacitor is in use. 

The tracker as designed will  be powered  from USB 5V Powerbank - it is good to use the cheapest USB powerbanks that do not have current sensor. Remember that GPS tracker will draw LOW current ( lower than 14mA). Some advanced powerbanks tend to switch off USB 5V when they find out that there is very little current consumed. If you have Powerbank with signalling LED then I suggest to get rid of this LED to reduce power drain on Powerbank.
By adding LM1804/LM317/LM7805 circuit you may adopt it to power from +12V car battery.

You can see how it works here : https://www.youtube.com/watch?v=8R99t0O52GI


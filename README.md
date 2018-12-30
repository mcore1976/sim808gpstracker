# sim808gpstracker
DIY ultra cheap GPS bike/car tracker based on  ATMEGA 328P (arduino uno chip) and SIM808 module from China (includes GPS and GNSS function). The total cost is below 20USD ( as in 2018 )

The device when called by mobile phone polls info from GPS module or when not available polls cell-id info from nearest 2G cell, uses GPRS to query Google servers for GPS location of that cell and sends back text message to your phone with current location link to Google map with timestamp.

The part list is (with the cost as in 2018):

    SIM808 (12 USD on Aliexpress search for "www.amd-global.com" boards BK-SIM808 https://cdn.instructables.com/ORIG/FAO/80RU/IXLALERK/FAO80RUIXLALERK.pdf)
    GPS antenna with UNC connector - 2 USD
    GSM antenna with UNC connector - 1 USD
    ATMEGA 328P (arduino uno) - 2 USD
    3x 1N4007 (1 USD) - to convert 5V from powerbank to 3.3V for ATMEGA328P VCC
    2x 1000uF / 16V capacitor ( 0.5 USD) - when powered from 3xAA battery pack only 1 capacitor is needed
    100nF capacitor (0.2 USD)
    universal PCB, pins & connector (2 USD)

To upload program code to the chip using cheapest USBASP programmer (less than 2 USD on eBay/Aliexpress) look at this page : http://www.learningaboutelectronics.com/Articles/Program-AVR-chip-using-a-USBASP-with-10-pin-cable.php

The script attached in repository ( "compileatmega" or "compileattiny" ) can be used to upload data to the chip if you have Linux machine with following packages : "avr-gcc" and "avrdude". For example in Ubuntu download these packages using command : "sudo apt-get install avr-gcc" and "sudo apt-get install avrdude" and you are ready to go.

The code is written in avr-gcc and was uploaded via USBASP. 

The solution has low power consumption because it is utilizing SLEEP MODE on SIM808 module and switches on GPS only when needed.

When SIM808 module sends SMS/GPRS data it it may draw a lot of current ( up to 2A ) in short peaks so it is crucial to use good cables and thick copper lines for GND and VCC on PCB. This is the main issue people face when dealing with SIMCOM modules. The voltage may additionaly drop during this situation so that is why such big capacitor is in use. 

The tracker can be powered  from USB 5V Powerbanks - it is good to use the cheapest USB powerbanks that do not have current sensor. Remember that GPS tracker will draw LOW current ( lower than 8mA). Some advanced powerbanks tend to switch off USB 5V when they find out that there is very little current consumed. If you have Powerbank with signalling LED then I suggest to get rid of this LED to reduce power drain on Powerbank.


You can see how it works : here : https://www.youtube.com/watch?v=t7mvomytDq4 and here : https://www.youtube.com/watch?v=546j1f_qA50


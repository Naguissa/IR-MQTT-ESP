# IR-MQTT-ESP, IR multiple repeater using MQTT for ESP8266 and ESP32 microcontrollers #

This is an Arduino sketch created in order to connect several devices and be able to retransmit IR signals among all devices using my ForoElectro.Net MQTT server.

You need one MQTT account: https://www.foroelectro.net/arduino/en/mqtt-doc

You can register it for free.



## Compilation ##

You can compile the sketch using Arduino IDE with ESP8266 or ESP32 support.


## Sketch upload ##

This sketch uses SPIFFS in order to store configurations. You need to set a minimal SPIFFS space when uploading. It only needs few kbytes.

Configuration is not encrypted, so take it in mind.


## Usage ##

Once uploaded it will be a new WiFi open network available, called IR-MQTT-AP.

Connect to that AP, navigate to 192.168.4.1, set correct values and save.

After resetting itself it will be connected to your network.

If you compile it with debug enabled you will see a lot of info, as its IP, via serial monitor.


## Hardware ##

You need one IR emitter (940nm) and one IR receiver. Also, an generic NPN transistor (BC547, 2N3904 or A42 are good examples).

IR receiver: https://s.click.aliexpress.com/e/_dUhAFXX
IR emitter: https://s.click.aliexpress.com/e/_dXUhbhx
Transistor (pack): https://s.click.aliexpress.com/e/_dVthBh7
Resistor, 10 to 100 Ohm (pack): https://s.click.aliexpress.com/e/_AUPOeB


In order to IR emitter work correctly you need the NPN transistor as stated here: https://github.com/crankyoldgit/IRremoteESP8266/wiki#ir-sending

By default:
 - IR demulator is connected to GPIO14, D5 on NodeMCU.
 - The base of transistor driving IR emitter is connected to GPIO4, D2 on NodeMCU.

You can change it using following variables on the code:
 - IR demulator: kRecvPin
 - IR emitter (base of transistor): kIrLed


![schematic](https://github.com/naguissa/IR-MQTT-ESP/raw/master/img/schematic.png)


## Notes ##

Included PubSubClient library in order to incrase max packet size.

Library page: https://pubsubclient.knolleary.net/
Changed define: MQTT_MAX_PACKET_SIZE
See more info here: https://pubsubclient.knolleary.net/api.html#configoptions


## Config file (in SPIFFS and when accessing via web) ##

ssid = Your-WiFi-SSID
password = Your-WiFi-password
ApiKey = ApiKey-from-https://www.foroelectro.net/arduino/en/mqtt-doc
email = Email-registered-with-ApiKey
clientName = A-unique-name-for-the-device
topic = A-name-for-your-devices-group---ie-home


## Who do I talk to? ##

 * [Naguissa](https://github.com/Naguissa)
 * https://www.foroelectro.net/proyectos-personales-f23/repetidores-de-infrarrojos-usando-esp-y-mqtt-t465.html
 * https://www.naguissa.com


## Contribute ##

Any code contribution, report or comment are always welcome. Don't hesitate to use GitHub for that.


 * You can sponsor this project using GitHub's Sponsor button: https://github.com/Naguissa/IR-MQTT-ESP
 * You can make a donation via PayPal: https://paypal.me/foroelectro


Thanks for your support.


Contributors hall of fame: https://www.foroelectro.net/hall-of-fame-f32/contributors-contribuyentes-t271.html

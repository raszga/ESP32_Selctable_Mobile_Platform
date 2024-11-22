# ESP32_Selctable_Mobile_Platform
An application to ESP32 udp wifi protocol using a wifi Joystick acting also as access point.
The folder configuration contains in the library folder also the ESP32Mobile (.cpp, .h) which are the common files to be used by different platform configurations.
The Joystick uses also an ESP32 to read 5 analog inputs and buttons and pack them in a comma separated string and broadcast them to the receivers (server).
The stp files for the Joystick can be found at :
<https://grabcad.com/library/basic-wifi-joystick-using-esp32-1>
I configured  the joystick as gated joysticks so that the software will receive the correct input. 
The mechanical hardware for the platforms are .. some old toys .. in my case.:)

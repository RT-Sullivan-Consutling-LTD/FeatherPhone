# FeatherPhone on Adafruit Feather 32u4 FONA 
## Rotary phone powered by Arduino


## Circuit Connection
I use a simple connection to the rotary mechinism. In my case, the pink wire on my dial is the pulse pin. When dialing a number, the pink wire is intermittently connected to the orange wire. So...
1. Connect Pink wire to pin 2
2. Connect Orange wire to ground

Hook Detection:  
The phone hook is a simple switch which is normally open. 
1. Connect one hook wire to pin3
2. Connect other hook wire to ground

```
     Rotary Dial                              Arduino
                      /---------------------- readyPin w/pulllup
  /- ready switch (NO) 
 /-- pulse switch (NC) 
 \                    \---------------------- pulsePin w/pullup
  \------------------------------------------ GND
```

```
readyPin  pulsePin  state
HIGH      n/a       default (waiting)
LOW       LOW       ready to dial / for first pulse
LOW       HIGH      pulse received (number = 1)
LOW       LOW       ready for next pulse
LOW       HIGH      pulse received (number = 2)
LOW       ...       (repeat)
HIGH      n/a       rotation complete, count recorded
```
## Hardware
Adafruit Feather 32u4 FONA  
An 'all-in-one' Arduino-compatible + audio/sms/data capable cellular with built in USB and battery charging. Its an Adafruit Feather 32u4 with a FONA800 module.

Rotary Telephone  
I bought one from ebay. Perfect working order as they are actually designed to last FOREVER! Mine is a BT model from the 70's. They are cheap and fun to take apart. 

## Building
The project is in VSCode using the PlatformIO plugin. RoataryDial is the only additional library needed. 

## ToDo
- Implement dialing detection  
- Circuit to power original bell ringer  

## See also
Platform: [Adafruit Feather 32u4 FONA](https://learn.adafruit.com/adafruit-feather-32u4-fona)  
Library: [RotaryDial by Tomas Mudrunka](https://github.com/Harvie/RotaryDial)  
Phone: [Roatary Telephone](https://www.ebay.co.uk/sch/i.html?_from=R40&_nkw=rotary+telephone&_sacat=0&rt=nc&Featured%2520Refinements=GPO%2520Telephone&_dcat=985)  
Wikipedia: [Rotary dial](http://en.wikipedia.org/wiki/Rotary_dial)  

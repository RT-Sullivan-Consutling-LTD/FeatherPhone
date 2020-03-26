# FeatherPhone
## Rotary phone powered by Arduino

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

## Circuit Connection
I use a simple connection to the rotary mechinism. In my case, the pink wire on my dial is the pulse pin. When dialing a number, the pink wire is intermittently connected to the XXXXX wire. So...
1. Connect Pink wire to pin 2
2. Connect XXXXX wire to ground

Hook Detection:  
The phone hook is a simple switch which is normally open. 
1. Connect one hook wire to pin3
2. Connect other hook wire to Ground

## See also
Wikipedia: [Rotary dial](http://en.wikipedia.org/wiki/Rotary_dial)  
Library: [RotaryDial by Tomas Mudrunka](https://github.com/Harvie/RotaryDial)



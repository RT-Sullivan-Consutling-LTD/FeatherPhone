# FeatherPhone
Rotary phone powered by Arduino


     Rotary Dial                              Arduino
                      /---------------------- readyPin
  /- ready switch (NO) -- pull-up resistor -- VCC
 /-- pulse switch (NC) -- pull-up resistor -/
 \                    \---------------------- pulsePin
  \------------------------------------------ GND


readyPin  pulsePin  state
HIGH      n/a       default (waiting)
LOW       LOW       ready to dial / for first pulse
LOW       HIGH      pulse received (number = 1)
LOW       LOW       ready for next pulse
LOW       HIGH      pulse received (number = 2)
LOW       ...       (repeat)
HIGH      n/a       rotation complete, count recorded

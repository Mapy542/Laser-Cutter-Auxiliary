# IO Modes

## None (00x) Opt Selection

Acts as a touchless limit switch (Open Drain Mode)
Does care about focal length setting

If PC5(Input pulldown)

Measured Dist <= [Lens Target] + Adjustment: Pull Down PC6
Else: Tri-State PC6

Performs time avg? over 5 samples to reduce noise

PC6 5V tolerant, max sink 20mA

## Opt 0 (10x) Selection

3 bit Bidirectional Comms (Communicate to Base Station)
Does not care about focal length setting

PC3: Clk (3.3 only, output from TOF only)
PC4: Data (3.3 only, output from TOF only)
PC5: Data (5V tolerant, bidirectional)
PC6: Data (5V tolerant, bidirectional)

Target Transfer every 50ms:

-   8 bit health byte
-   16 bit distance in mm
-   8 bit xor checksum
-   8 bit end byte (0xAA)
-   1 clk delay to set TOF into tri-state
-   4 bit command from base station

# Focal Length

## Opt 2 (xx0) 2" Lens

-   Focal Length: 50.8mm
-   Custom adjustment set in firmware only

## Opt 2 (xx1) 4" Lens

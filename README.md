# nanoAmmeter


<img src="https://github.com/joeltroughton/nanoAmmeter/raw/master/pcb_image.png" alt="nanoAmmeter" width="200"/>

nanoAmmeter is a small research device designed to measure the voltage applied to, and the current flowing through biological glucose sensors on the order of 10s to 100s of nanoamps.

As glucose is detected by the sensor, its resistance changes. nanoAmmeter measures the current flow through the sensor for later analysis.



## Function

The nanoAmmeter is inserted between the biosensor's negative electrode and ground. The voltage drop across a 100kOhm resistor is amplified by a gain of 101 by a very low offset opamp (Analog LTC2055, ± 3 μV) and finally read by a 16-bit ADC (TI ADS1115). With a FSR of 4.096V, this gives us a theoretical current resolution of 12.4 pA. 

The voltage applied to the positive side of the biosensor is buffered to the ADC with unity gain for a theoretic voltage resolution of 125 μV.

The ADC is read by a STM32F072 MCU at 128 samples per second where it is fed into a finite impulse response, low pass filter and exported via USB. 

## Acknowledgments
Thanks to https://github.com/pms67 for the FIR circular buffer implementation

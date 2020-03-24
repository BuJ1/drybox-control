# drybox-control

A short arduino code to control a ventilator, a temperature sensor, a display and a 4-20 mA current loop
readout to display a humidity value for a glovebox with dry air atmosphere.

## Used Electronic Components

Description                             | Exact Name                            | Manufacturer
----------------------------------------|---------------------------------------|-----------------------
Case 1550EBK                            | Product Number                        | Hammond Electronics
3D printed Case                         | -                                     | PI Heidelberg
Push-Button                             | Standard 7mm round Push Button        | -
Rocker Switch                           | Standard I/O switch                   | -
Temperature Sensor                      | LM 35 DZ                              | Texas Instruments
Arduino Nano                            | Nano V3.0 ATmega 328 Board CH340      | From China
TFT Touch Display                       | 2,4" 240x320 TFT LCD Display ILI9341  | From China
9V Power Supply                         | 9 V DC / 0,6 A/ 54807                 | Goobay
24V (5A) Power Supply                   | GST120A                               | Mean Well
Mains Adapter                           | H05VV-F1.5/3                          | -
Voltage Regulator                       | TS2940                                | TSC
4-20mA Current Loop Extension Board     | 932-MIKROE-1387                       | MikroE
Control Line Cable (5m)                 | SMART 108 3G x 0.55mm Grey            | LAPP
Power Supply Cable (5m)                 | SMART 108 2 x 1.50mm Grey             | LAPP
3 pole plug female                      | SP1312 / S3 (SP13)                    | Weipu
3 pole plug male                        | SP1310 / P3 I (SP13)                  | Weipu
2 pole plug female                      | SP2112/ S2 (SP21)                     | Weipu
2 pole plug male                        | SP2110 / S2 (SP21)                    | Weipu
Diode                                   | P600D P600 200 V 6 A                  | Diotec Semiconductor

## Helpful Tutorials

Without the matching click adapter from MikroE I do not recommend to use the 4-20mA current loop board since
the programming is quite annoying and the result in my case rather inaccurate. To still make it work I followed
the tutorial in
    <https://electronza.com/4-20ma-current-loop-arduino-receiver/>
where I did the calibration using a multimeter and the humidity sensor that needed to be read out.
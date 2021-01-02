EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:USB_B_Micro J1
U 1 1 5F67BB6D
P 1400 2900
F 0 "J1" H 1457 3367 50  0000 C CNN
F 1 "USB_B_Micro" H 1457 3276 50  0000 C CNN
F 2 "digikey-footprints:USB_Micro_AB_Female_0475890001_DP_PWR_GND" H 1550 2850 50  0001 C CNN
F 3 "~" H 1550 2850 50  0001 C CNN
	1    1400 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 3350 1400 3300
$Comp
L power:GND #PWR0101
U 1 1 5F67F747
P 1400 3350
F 0 "#PWR0101" H 1400 3100 50  0001 C CNN
F 1 "GND" H 1405 3177 50  0000 C CNN
F 2 "" H 1400 3350 50  0001 C CNN
F 3 "" H 1400 3350 50  0001 C CNN
	1    1400 3350
	1    0    0    -1  
$EndComp
$Comp
L avr-arduino:FTDI_Header J2
U 1 1 5F681939
P 3550 2750
F 0 "J2" H 3370 2658 50  0000 R CNN
F 1 "FTDI_Header" H 3370 2749 50  0000 R CNN
F 2 "avr-arduino:FTDI_Header" H 3550 3150 50  0001 C CNN
F 3 "https://www.sparkfun.com/products/9716" H 3500 2750 50  0001 C CNN
	1    3550 2750
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0102
U 1 1 5F6850CC
P 3450 3050
F 0 "#PWR0102" H 3450 2900 50  0001 C CNN
F 1 "+3.3V" H 3465 3223 50  0000 C CNN
F 2 "" H 3450 3050 50  0001 C CNN
F 3 "" H 3450 3050 50  0001 C CNN
	1    3450 3050
	-1   0    0    1   
$EndComp
$Comp
L power:+3.3V #PWR0103
U 1 1 5F68586F
P 4600 3500
F 0 "#PWR0103" H 4600 3350 50  0001 C CNN
F 1 "+3.3V" H 4615 3673 50  0000 C CNN
F 2 "" H 4600 3500 50  0001 C CNN
F 3 "" H 4600 3500 50  0001 C CNN
	1    4600 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 3500 4600 3500
$Comp
L power:GND #PWR0104
U 1 1 5F686DB3
P 3250 3050
F 0 "#PWR0104" H 3250 2800 50  0001 C CNN
F 1 "GND" H 3255 2877 50  0000 C CNN
F 2 "" H 3250 3050 50  0001 C CNN
F 3 "" H 3250 3050 50  0001 C CNN
	1    3250 3050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5F687209
P 2300 3750
F 0 "#PWR0105" H 2300 3500 50  0001 C CNN
F 1 "GND" V 2350 3600 50  0000 C CNN
F 2 "" H 2300 3750 50  0001 C CNN
F 3 "" H 2300 3750 50  0001 C CNN
	1    2300 3750
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 5F68C201
P 5650 4100
F 0 "#PWR0106" H 5650 3850 50  0001 C CNN
F 1 "GND" H 5655 3927 50  0000 C CNN
F 2 "" H 5650 4100 50  0001 C CNN
F 3 "" H 5650 4100 50  0001 C CNN
	1    5650 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 4100 5650 3900
Wire Wire Line
	5650 3900 5600 3900
Wire Wire Line
	4700 4000 4900 4000
Connection ~ 4700 3500
$Comp
L Device:R R4
U 1 1 5F68FFA4
P 5100 3650
F 0 "R4" H 5000 3700 50  0000 L CNN
F 1 "R" H 5000 3600 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 5030 3650 50  0001 C CNN
F 3 "~" H 5100 3650 50  0001 C CNN
	1    5100 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 3900 5100 3800
Connection ~ 5100 3900
Wire Wire Line
	5100 3900 5200 3900
$Comp
L power:GND #PWR0107
U 1 1 5F69179D
P 1250 2000
F 0 "#PWR0107" H 1250 1750 50  0001 C CNN
F 1 "GND" H 1255 1827 50  0000 C CNN
F 2 "" H 1250 2000 50  0001 C CNN
F 3 "" H 1250 2000 50  0001 C CNN
	1    1250 2000
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0108
U 1 1 5F691A9F
P 1550 2000
F 0 "#PWR0108" H 1550 1850 50  0001 C CNN
F 1 "+3.3V" H 1565 2173 50  0000 C CNN
F 2 "" H 1550 2000 50  0001 C CNN
F 3 "" H 1550 2000 50  0001 C CNN
	1    1550 2000
	-1   0    0    1   
$EndComp
$Comp
L Device:C C2
U 1 1 5F692175
P 1400 1800
F 0 "C2" V 1300 1800 50  0000 C CNN
F 1 "470uF" V 1550 1800 50  0000 C CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.00mm" H 1438 1650 50  0001 C CNN
F 3 "~" H 1400 1800 50  0001 C CNN
	1    1400 1800
	0    1    1    0   
$EndComp
Wire Wire Line
	1250 2000 1250 1800
Wire Wire Line
	1250 1800 1250 1450
Connection ~ 1250 1800
Wire Wire Line
	1550 2000 1550 1800
Wire Wire Line
	1550 1800 1550 1450
Connection ~ 1550 1800
Wire Wire Line
	2750 4000 2800 4000
$Comp
L ADS1115:ADS1115 U3
U 1 1 5F6B29E2
P 5600 1800
F 0 "U3" H 5969 1450 50  0000 L CNN
F 1 "ADS1115" H 5969 1359 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x10_P2.54mm_Vertical" H 5600 1800 50  0001 C CNN
F 3 "" H 5600 1800 50  0001 C CNN
	1    5600 1800
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR0110
U 1 1 5F6B3D4C
P 1800 2700
F 0 "#PWR0110" H 1800 2550 50  0001 C CNN
F 1 "VBUS" V 1815 2827 50  0000 L CNN
F 2 "" H 1800 2700 50  0001 C CNN
F 3 "" H 1800 2700 50  0001 C CNN
	1    1800 2700
	0    1    1    0   
$EndComp
Wire Wire Line
	5450 1700 5450 1750
Wire Wire Line
	5450 1750 5550 1750
$Comp
L power:GND #PWR0112
U 1 1 5F6B6E94
P 5300 1850
F 0 "#PWR0112" H 5300 1600 50  0001 C CNN
F 1 "GND" V 5305 1722 50  0000 R CNN
F 2 "" H 5300 1850 50  0001 C CNN
F 3 "" H 5300 1850 50  0001 C CNN
	1    5300 1850
	0    1    1    0   
$EndComp
Wire Wire Line
	5300 1850 5450 1850
Text GLabel 2750 4000 0    50   Input ~ 0
SCL
Text GLabel 2350 4300 3    50   Input ~ 0
SDA
Text GLabel 5400 1950 0    50   Input ~ 0
SCL
Text GLabel 5400 2050 0    50   Input ~ 0
SDA
Wire Wire Line
	5400 1950 5550 1950
Wire Wire Line
	5400 2050 5550 2050
Wire Wire Line
	5550 2150 5450 2150
Wire Wire Line
	5450 2150 5450 1850
Connection ~ 5450 1850
Wire Wire Line
	5450 1850 5550 1850
Wire Wire Line
	5350 2350 5450 2350
Wire Wire Line
	5350 2550 5400 2550
Wire Wire Line
	5450 2550 5450 2450
Wire Wire Line
	5450 2450 5550 2450
$Comp
L Connector:Conn_01x02_Female J7
U 1 1 5F694DC9
P 5550 3050
F 0 "J7" V 5396 3098 50  0000 L CNN
F 1 "Conn_01x02_Female" V 5487 3098 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 5550 3050 50  0001 C CNN
F 3 "~" H 5550 3050 50  0001 C CNN
	1    5550 3050
	0    1    1    0   
$EndComp
Wire Wire Line
	5550 2650 5550 2850
Wire Wire Line
	5550 2550 5500 2550
Wire Wire Line
	5500 2550 5500 2850
Wire Wire Line
	5500 2850 5450 2850
$Comp
L Connector:Conn_01x02_Female J6
U 1 1 5F6955DB
P 5200 3250
F 0 "J6" V 5046 3298 50  0000 L CNN
F 1 "Conn_01x02_Female" V 5137 3298 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 5200 3250 50  0001 C CNN
F 3 "~" H 5200 3250 50  0001 C CNN
	1    5200 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	5450 2350 5450 2200
Wire Wire Line
	5450 2200 4850 2200
Wire Wire Line
	4850 2200 4850 3050
Wire Wire Line
	4850 3050 5100 3050
Connection ~ 5450 2350
Wire Wire Line
	5450 2350 5550 2350
Wire Wire Line
	5200 3050 5200 2700
Wire Wire Line
	5200 2700 5400 2700
Wire Wire Line
	5400 2700 5400 2550
Connection ~ 5400 2550
Wire Wire Line
	5400 2550 5450 2550
$Comp
L Connector:Conn_01x04_Female J4
U 1 1 5F69D5E8
P 3600 3900
F 0 "J4" H 3628 3876 50  0000 L CNN
F 1 "0" H 3600 4000 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 3600 3900 50  0001 C CNN
F 3 "~" H 3600 3900 50  0001 C CNN
	1    3600 3900
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Female J5
U 1 1 5F69E66D
P 3850 4000
F 0 "J5" H 3742 3575 50  0000 C CNN
F 1 "0" H 3900 3800 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 3850 4000 50  0001 C CNN
F 3 "~" H 3850 4000 50  0001 C CNN
	1    3850 4000
	-1   0    0    1   
$EndComp
$Comp
L JACK_TRS:AudioJack3_GroundPinNumbers J3
U 1 1 5F695121
P 5150 2450
F 0 "J3" H 5132 2775 50  0000 C CNN
F 1 "AudioJack3_GroundPinNumbers" H 5132 2684 50  0000 C CNN
F 2 "audio35mm:Tayda_3.5mm_stereo_TRS_jack_A-853" H 5150 2450 50  0001 C CNN
F 3 "~" H 5150 2450 50  0001 C CNN
	1    5150 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 3700 2300 3750
Wire Wire Line
	2700 3800 2700 3900
Wire Wire Line
	2700 3900 2800 3900
Wire Wire Line
	2350 3900 2700 3900
Connection ~ 2700 3900
Wire Wire Line
	4700 3500 4900 3500
Wire Wire Line
	4900 3500 4900 4000
Connection ~ 4900 3500
Wire Wire Line
	4900 3500 5100 3500
$Comp
L Device:CP C1
U 1 1 5F75345F
P 1400 1450
F 0 "C1" V 1250 1450 50  0000 C CNN
F 1 "0.1uF" V 1500 1450 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D3.0mm_W1.6mm_P2.50mm" H 1438 1300 50  0001 C CNN
F 3 "~" H 1400 1450 50  0001 C CNN
	1    1400 1450
	0    1    1    0   
$EndComp
$Comp
L Device:CP C3
U 1 1 5F7553AD
P 2000 1150
F 0 "C3" V 1850 1150 50  0000 C CNN
F 1 "0.33uF" V 2150 1150 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D3.0mm_W1.6mm_P2.50mm" H 2038 1000 50  0001 C CNN
F 3 "~" H 2000 1150 50  0001 C CNN
	1    2000 1150
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 5F757012
P 1850 1300
F 0 "#PWR0109" H 1850 1050 50  0001 C CNN
F 1 "GND" H 1855 1127 50  0000 C CNN
F 2 "" H 1850 1300 50  0001 C CNN
F 3 "" H 1850 1300 50  0001 C CNN
	1    1850 1300
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR0113
U 1 1 5F7576FC
P 2150 1300
F 0 "#PWR0113" H 2150 1150 50  0001 C CNN
F 1 "VBUS" H 2165 1473 50  0000 C CNN
F 2 "" H 2150 1300 50  0001 C CNN
F 3 "" H 2150 1300 50  0001 C CNN
	1    2150 1300
	-1   0    0    1   
$EndComp
Wire Wire Line
	1850 1150 1850 1300
Wire Wire Line
	2150 1150 2150 1300
$Comp
L power:GND #PWR0114
U 1 1 5F75ED44
P 2700 4150
F 0 "#PWR0114" H 2700 3900 50  0001 C CNN
F 1 "GND" H 2705 3977 50  0000 C CNN
F 2 "" H 2700 4150 50  0001 C CNN
F 3 "" H 2700 4150 50  0001 C CNN
	1    2700 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 4100 2700 4150
$Comp
L ADS1115:SW_SPST_SM_DP SW2
U 1 1 5F76AB89
P 5400 3900
F 0 "SW2" H 5400 3850 50  0000 C CNN
F 1 "SW_SPST_SM_DP" H 6000 4000 50  0000 C CNN
F 2 "eec:E-Switch-TL3342F160QG-0" H 5400 3900 50  0001 C CNN
F 3 "~" H 5400 3900 50  0001 C CNN
	1    5400 3900
	1    0    0    -1  
$EndComp
$Comp
L ADS1115:SW_SPST_SM_DP SW3
U 1 1 5F76E993
P 2500 3800
F 0 "SW3" H 2500 3850 50  0000 C CNN
F 1 "SW_SPST_SM_DP" H 2400 3950 50  0000 C CNN
F 2 "eec:E-Switch-TL3342F160QG-0" H 2500 3800 50  0001 C CNN
F 3 "~" H 2500 3800 50  0001 C CNN
	1    2500 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 3800 2700 3700
Connection ~ 2700 3800
Wire Wire Line
	5600 3900 5600 3800
Connection ~ 5600 3900
Wire Wire Line
	5200 3800 5200 3900
Connection ~ 5200 3900
$Comp
L power:+3.3V #PWR0111
U 1 1 5F785201
P 5450 1700
F 0 "#PWR0111" H 5450 1550 50  0001 C CNN
F 1 "+3.3V" H 5465 1873 50  0000 C CNN
F 2 "" H 5450 1700 50  0001 C CNN
F 3 "" H 5450 1700 50  0001 C CNN
	1    5450 1700
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR0117
U 1 1 5F787AC4
P 5850 800
F 0 "#PWR0117" H 5850 650 50  0001 C CNN
F 1 "VBUS" V 5865 927 50  0000 L CNN
F 2 "" H 5850 800 50  0001 C CNN
F 3 "" H 5850 800 50  0001 C CNN
	1    5850 800 
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5850 800  6000 800 
$Comp
L power:GND #PWR0118
U 1 1 5F79B4F5
P 8200 1350
F 0 "#PWR0118" H 8200 1100 50  0001 C CNN
F 1 "GND" H 8205 1177 50  0000 C CNN
F 2 "" H 8200 1350 50  0001 C CNN
F 3 "" H 8200 1350 50  0001 C CNN
	1    8200 1350
	1    0    0    -1  
$EndComp
Connection ~ 2300 3750
Wire Wire Line
	2300 3750 2300 3800
$Comp
L UA78M05CDCY:UA78M05CDCY U1
U 1 1 5F7AC9E7
P 7350 1450
F 0 "U1" H 7350 1820 50  0000 C CNN
F 1 "UA78M05CDCY" H 7350 1729 50  0000 C CNN
F 2 "UA78M05CDCY:SOT230P700X180-4N" H 7350 1450 50  0001 L BNN
F 3 "" H 7350 1450 50  0001 C CNN
	1    7350 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 1350 8200 1350
Wire Wire Line
	3450 3050 3450 2950
Wire Wire Line
	1700 2700 1800 2700
Wire Wire Line
	3250 3050 3250 2950
$Comp
L power:VBUS #PWR0119
U 1 1 5F7CF7D9
P 6500 1350
F 0 "#PWR0119" H 6500 1200 50  0001 C CNN
F 1 "VBUS" V 6515 1477 50  0000 L CNN
F 2 "" H 6500 1350 50  0001 C CNN
F 3 "" H 6500 1350 50  0001 C CNN
	1    6500 1350
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0120
U 1 1 5F7CFF2C
P 6500 1450
F 0 "#PWR0120" H 6500 1300 50  0001 C CNN
F 1 "+3.3V" V 6515 1578 50  0000 L CNN
F 2 "" H 6500 1450 50  0001 C CNN
F 3 "" H 6500 1450 50  0001 C CNN
	1    6500 1450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6500 1450 6650 1450
Wire Wire Line
	6500 1350 6650 1350
$Comp
L ESP8266:ESP-01v090 U2
U 1 1 5F67B525
P 3750 3950
F 0 "U2" H 3750 3435 50  0000 C CNN
F 1 "ESP-01v090" H 3750 3526 50  0000 C CNN
F 2 "ESP8266:ESP-01" H 3750 3950 50  0001 C CNN
F 3 "http://l0l.org.uk/2014/12/esp8266-modules-hardware-guide-gotta-catch-em-all/" H 3750 3950 50  0001 C CNN
	1    3750 3950
	-1   0    0    1   
$EndComp
Wire Wire Line
	4050 3800 4700 3800
Connection ~ 4700 3800
Wire Wire Line
	4700 3900 5100 3900
Wire Wire Line
	4050 4100 4700 4100
Wire Wire Line
	4050 4000 4700 4000
Connection ~ 4700 4000
Wire Wire Line
	4700 3500 4700 3800
Wire Wire Line
	4050 3900 4700 3900
Connection ~ 4700 3900
Wire Wire Line
	2350 3900 2350 4300
Wire Wire Line
	2700 4100 2800 4100
Connection ~ 2800 4100
Wire Wire Line
	2800 4100 3400 4100
Wire Wire Line
	2800 4000 3400 4000
Connection ~ 2800 4000
Wire Wire Line
	2800 3900 3400 3900
Connection ~ 2800 3900
Wire Wire Line
	2800 3350 2800 3800
Wire Wire Line
	2800 3800 3400 3800
Connection ~ 2800 3800
Wire Wire Line
	2800 3350 3550 3350
Wire Wire Line
	3550 3350 3550 2950
Wire Wire Line
	3650 2950 3650 3250
Wire Wire Line
	3650 3250 4750 3250
Wire Wire Line
	4750 3250 4750 4100
Wire Wire Line
	4750 4100 4700 4100
Connection ~ 4700 4100
$Comp
L Connector:TestPoint TP1
U 1 1 5F834784
P 6000 800
F 0 "TP1" H 6058 918 50  0000 L CNN
F 1 "TestPoint" H 6050 850 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x01_P2.54mm_Vertical" H 6200 800 50  0001 C CNN
F 3 "~" H 6200 800 50  0001 C CNN
	1    6000 800 
	1    0    0    -1  
$EndComp
$EndSCHEMATC
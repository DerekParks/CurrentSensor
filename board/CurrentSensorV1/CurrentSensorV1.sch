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
L power:GND #PWR0101
U 1 1 5F67F747
P 850 4300
F 0 "#PWR0101" H 850 4050 50  0001 C CNN
F 1 "GND" H 855 4127 50  0000 C CNN
F 2 "" H 850 4300 50  0001 C CNN
F 3 "" H 850 4300 50  0001 C CNN
	1    850  4300
	1    0    0    -1  
$EndComp
$Comp
L avr-arduino:FTDI_Header J2
U 1 1 5F681939
P 5700 2400
F 0 "J2" H 5520 2308 50  0000 R CNN
F 1 "FTDI_Header" V 5350 2600 50  0000 R CNN
F 2 "avr-arduino:FTDI_Header" H 5700 2800 50  0001 C CNN
F 3 "https://www.sparkfun.com/products/9716" H 5650 2400 50  0001 C CNN
	1    5700 2400
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0102
U 1 1 5F6850CC
P 5600 2700
F 0 "#PWR0102" H 5600 2550 50  0001 C CNN
F 1 "+3.3V" H 5615 2873 50  0000 C CNN
F 2 "" H 5600 2700 50  0001 C CNN
F 3 "" H 5600 2700 50  0001 C CNN
	1    5600 2700
	-1   0    0    1   
$EndComp
$Comp
L power:+3.3V #PWR0103
U 1 1 5F68586F
P 4600 2750
F 0 "#PWR0103" H 4600 2600 50  0001 C CNN
F 1 "+3.3V" H 4615 2923 50  0000 C CNN
F 2 "" H 4600 2750 50  0001 C CNN
F 3 "" H 4600 2750 50  0001 C CNN
	1    4600 2750
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5F686DB3
P 5400 2700
F 0 "#PWR0104" H 5400 2450 50  0001 C CNN
F 1 "GND" H 5405 2527 50  0000 C CNN
F 2 "" H 5400 2700 50  0001 C CNN
F 3 "" H 5400 2700 50  0001 C CNN
	1    5400 2700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5F687209
P 3450 4950
F 0 "#PWR0105" H 3450 4700 50  0001 C CNN
F 1 "GND" V 3500 4800 50  0000 C CNN
F 2 "" H 3450 4950 50  0001 C CNN
F 3 "" H 3450 4950 50  0001 C CNN
	1    3450 4950
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 5F69179D
P 1250 2750
F 0 "#PWR0107" H 1250 2500 50  0001 C CNN
F 1 "GND" H 1255 2577 50  0000 C CNN
F 2 "" H 1250 2750 50  0001 C CNN
F 3 "" H 1250 2750 50  0001 C CNN
	1    1250 2750
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0108
U 1 1 5F691A9F
P 1550 2750
F 0 "#PWR0108" H 1550 2600 50  0001 C CNN
F 1 "+3.3V" H 1565 2923 50  0000 C CNN
F 2 "" H 1550 2750 50  0001 C CNN
F 3 "" H 1550 2750 50  0001 C CNN
	1    1550 2750
	-1   0    0    1   
$EndComp
$Comp
L power:VBUS #PWR0110
U 1 1 5F6B3D4C
P 1400 3550
F 0 "#PWR0110" H 1400 3400 50  0001 C CNN
F 1 "VBUS" V 1415 3677 50  0000 L CNN
F 2 "" H 1400 3550 50  0001 C CNN
F 3 "" H 1400 3550 50  0001 C CNN
	1    1400 3550
	0    1    1    0   
$EndComp
Text GLabel 2550 3000 0    50   Input ~ 0
SCL
Text GLabel 2550 2900 0    50   Input ~ 0
SDA
Wire Wire Line
	5400 1000 5400 900 
Wire Wire Line
	6900 1000 6900 1200
$Comp
L Connector:Conn_01x02_Female J6
U 1 1 5F6955DB
P 5150 1700
F 0 "J6" V 4996 1748 50  0000 L CNN
F 1 "Conn_01x02_Female" V 5087 1748 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 5150 1700 50  0001 C CNN
F 3 "~" H 5150 1700 50  0001 C CNN
	1    5150 1700
	0    1    1    0   
$EndComp
Wire Wire Line
	5150 1150 5350 1150
Connection ~ 5350 1000
Wire Wire Line
	5350 1000 5400 1000
Wire Wire Line
	3450 4900 3450 4950
Wire Wire Line
	3850 5000 3850 5100
$Comp
L power:GND #PWR0109
U 1 1 5F757012
P 1250 1400
F 0 "#PWR0109" H 1250 1150 50  0001 C CNN
F 1 "GND" H 1255 1227 50  0000 C CNN
F 2 "" H 1250 1400 50  0001 C CNN
F 3 "" H 1250 1400 50  0001 C CNN
	1    1250 1400
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR0113
U 1 1 5F7576FC
P 1550 1400
F 0 "#PWR0113" H 1550 1250 50  0001 C CNN
F 1 "VBUS" H 1565 1573 50  0000 C CNN
F 2 "" H 1550 1400 50  0001 C CNN
F 3 "" H 1550 1400 50  0001 C CNN
	1    1550 1400
	-1   0    0    1   
$EndComp
Wire Wire Line
	1250 850  1250 1000
Wire Wire Line
	1550 850  1550 1000
$Comp
L power:GND #PWR0114
U 1 1 5F75ED44
P 2500 3150
F 0 "#PWR0114" H 2500 2900 50  0001 C CNN
F 1 "GND" H 2505 2977 50  0000 C CNN
F 2 "" H 2500 3150 50  0001 C CNN
F 3 "" H 2500 3150 50  0001 C CNN
	1    2500 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 3100 2500 3150
$Comp
L ADS1115:SW_SPST_SM_DP FLASH1
U 1 1 5F76E993
P 3650 5000
F 0 "FLASH1" H 3650 5050 50  0000 C CNN
F 1 "SW_SPST_SM_DP" H 3550 5150 50  0000 C CNN
F 2 "eec:E-Switch-TL3342F160QG-0" H 3650 5000 50  0001 C CNN
F 3 "~" H 3650 5000 50  0001 C CNN
F 4 "C530667" H 3650 5000 50  0001 C CNN "LCSC"
	1    3650 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 5000 3850 4900
Connection ~ 3850 5000
$Comp
L power:+3.3V #PWR0111
U 1 1 5F785201
P 2700 1000
F 0 "#PWR0111" H 2700 850 50  0001 C CNN
F 1 "+3.3V" H 2715 1173 50  0000 C CNN
F 2 "" H 2700 1000 50  0001 C CNN
F 3 "" H 2700 1000 50  0001 C CNN
	1    2700 1000
	-1   0    0    1   
$EndComp
$Comp
L power:VBUS #PWR0117
U 1 1 5F787AC4
P 3200 950
F 0 "#PWR0117" H 3200 800 50  0001 C CNN
F 1 "VBUS" V 3215 1077 50  0000 L CNN
F 2 "" H 3200 950 50  0001 C CNN
F 3 "" H 3200 950 50  0001 C CNN
	1    3200 950 
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 5F79B4F5
P 2800 2150
F 0 "#PWR0118" H 2800 1900 50  0001 C CNN
F 1 "GND" H 2805 1977 50  0000 C CNN
F 2 "" H 2800 2150 50  0001 C CNN
F 3 "" H 2800 2150 50  0001 C CNN
	1    2800 2150
	1    0    0    -1  
$EndComp
Connection ~ 3450 4950
Wire Wire Line
	3450 4950 3450 5000
Wire Wire Line
	2800 2000 2800 2150
Wire Wire Line
	5600 2700 5600 2600
Wire Wire Line
	5400 2700 5400 2600
$Comp
L power:VBUS #PWR0119
U 1 1 5F7CF7D9
P 2350 1700
F 0 "#PWR0119" H 2350 1550 50  0001 C CNN
F 1 "VBUS" V 2365 1827 50  0000 L CNN
F 2 "" H 2350 1700 50  0001 C CNN
F 3 "" H 2350 1700 50  0001 C CNN
	1    2350 1700
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0120
U 1 1 5F7CFF2C
P 3250 1700
F 0 "#PWR0120" H 3250 1550 50  0001 C CNN
F 1 "+3.3V" V 3265 1828 50  0000 L CNN
F 2 "" H 3250 1700 50  0001 C CNN
F 3 "" H 3250 1700 50  0001 C CNN
	1    3250 1700
	0    1    1    0   
$EndComp
Wire Wire Line
	3250 1700 3100 1700
Wire Wire Line
	2350 1700 2500 1700
Wire Wire Line
	5700 3000 5700 2600
$Comp
L Connector:TestPoint TP1
U 1 1 5F834784
P 3200 800
F 0 "TP1" H 3258 918 50  0000 L CNN
F 1 "TestPoint" H 3250 850 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x01_P2.54mm_Vertical" H 3400 800 50  0001 C CNN
F 3 "~" H 3400 800 50  0001 C CNN
	1    3200 800 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 603763A0
P 6150 2450
F 0 "#PWR0112" H 6150 2200 50  0001 C CNN
F 1 "GND" H 6155 2277 50  0000 C CNN
F 2 "" H 6150 2450 50  0001 C CNN
F 3 "" H 6150 2450 50  0001 C CNN
	1    6150 2450
	-1   0    0    1   
$EndComp
$Comp
L Device:R R5
U 1 1 603774D2
P 6150 2650
F 0 "R5" H 6150 2850 50  0000 L CNN
F 1 "680R" V 6150 2550 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6080 2650 50  0001 C CNN
F 3 "~" H 6150 2650 50  0001 C CNN
F 4 "C23228" H 6150 2650 50  0001 C CNN "LCSC"
	1    6150 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 2450 6150 2500
Wire Wire Line
	5900 2600 5900 2800
Wire Wire Line
	5900 2800 6150 2800
$Comp
L Device:C C6
U 1 1 6037C06A
P 6450 2800
F 0 "C6" V 6198 2800 50  0000 C CNN
F 1 "2.2uF" V 6289 2800 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6488 2650 50  0001 C CNN
F 3 "~" H 6450 2800 50  0001 C CNN
F 4 "C23630" V 6450 2800 50  0001 C CNN "LCSC"
	1    6450 2800
	0    1    1    0   
$EndComp
Wire Wire Line
	6150 2800 6300 2800
Connection ~ 6150 2800
$Comp
L power:+3.3V #PWR0115
U 1 1 6037FDE9
P 6700 2300
F 0 "#PWR0115" H 6700 2150 50  0001 C CNN
F 1 "+3.3V" H 6715 2473 50  0000 C CNN
F 2 "" H 6700 2300 50  0001 C CNN
F 3 "" H 6700 2300 50  0001 C CNN
	1    6700 2300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 6038053A
P 6700 2550
F 0 "R6" H 6550 2650 50  0000 L CNN
F 1 "10K" V 6700 2500 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6630 2550 50  0001 C CNN
F 3 "~" H 6700 2550 50  0001 C CNN
F 4 "C23192" H 6700 2550 50  0001 C CNN "LCSC"
	1    6700 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 2300 6700 2350
Wire Wire Line
	6600 2800 6700 2800
Wire Wire Line
	6700 2800 6700 2700
Wire Wire Line
	6700 2800 6850 2800
Connection ~ 6700 2800
$Comp
L ADS1115:ADS1015_IC U1
U 1 1 603797BB
P 1350 5450
F 0 "U1" H 1450 5765 50  0000 C CNN
F 1 "ADS1015_IC" H 1450 5674 50  0000 C CNN
F 2 "Package_SO:VSSOP-10_3x3mm_P0.5mm" H 1350 5450 50  0001 C CNN
F 3 "" H 1350 5450 50  0001 C CNN
F 4 "C193969" H 1350 5450 50  0001 C CNN "LCSC"
F 5 "0;0;270" H 1350 5450 50  0001 C CNN "JLCPCB_CORRECTION"
	1    1350 5450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 6037AC9B
P 1900 5200
F 0 "C5" H 2015 5246 50  0000 L CNN
F 1 "1uF" H 2015 5155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 1938 5050 50  0001 C CNN
F 3 "~" H 1900 5200 50  0001 C CNN
F 4 "C52923" H 1900 5200 50  0001 C CNN "LCSC"
	1    1900 5200
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0116
U 1 1 6037D07E
P 2200 5500
F 0 "#PWR0116" H 2200 5350 50  0001 C CNN
F 1 "+3.3V" V 2215 5628 50  0000 L CNN
F 2 "" H 2200 5500 50  0001 C CNN
F 3 "" H 2200 5500 50  0001 C CNN
	1    2200 5500
	0    1    1    0   
$EndComp
Wire Wire Line
	1800 5500 1900 5500
Wire Wire Line
	1900 5500 1900 5350
Connection ~ 1900 5500
$Comp
L power:GND #PWR0121
U 1 1 6038225E
P 1900 4850
F 0 "#PWR0121" H 1900 4600 50  0001 C CNN
F 1 "GND" H 1905 4677 50  0000 C CNN
F 2 "" H 1900 4850 50  0001 C CNN
F 3 "" H 1900 4850 50  0001 C CNN
	1    1900 4850
	-1   0    0    1   
$EndComp
Wire Wire Line
	1900 4850 1900 5050
$Comp
L Device:R R2
U 1 1 603858DC
P 2250 5800
F 0 "R2" V 2200 6000 50  0000 C CNN
F 1 "10K" V 2250 5800 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2180 5800 50  0001 C CNN
F 3 "~" H 2250 5800 50  0001 C CNN
F 4 "C23192" V 2250 5800 50  0001 C CNN "LCSC"
	1    2250 5800
	0    1    1    0   
$EndComp
$Comp
L Device:R R1
U 1 1 60385015
P 2250 5700
F 0 "R1" V 2200 5900 50  0000 C CNN
F 1 "10K" V 2250 5700 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2180 5700 50  0001 C CNN
F 3 "~" H 2250 5700 50  0001 C CNN
F 4 "C23192" V 2250 5700 50  0001 C CNN "LCSC"
	1    2250 5700
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0122
U 1 1 6039C899
P 2550 5700
F 0 "#PWR0122" H 2550 5550 50  0001 C CNN
F 1 "+3.3V" V 2565 5828 50  0000 L CNN
F 2 "" H 2550 5700 50  0001 C CNN
F 3 "" H 2550 5700 50  0001 C CNN
	1    2550 5700
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0123
U 1 1 6039D0F2
P 2550 5800
F 0 "#PWR0123" H 2550 5650 50  0001 C CNN
F 1 "+3.3V" V 2565 5928 50  0000 L CNN
F 2 "" H 2550 5800 50  0001 C CNN
F 3 "" H 2550 5800 50  0001 C CNN
	1    2550 5800
	0    1    1    0   
$EndComp
Wire Wire Line
	2400 5700 2550 5700
Wire Wire Line
	2400 5800 2550 5800
$Comp
L power:GND #PWR0124
U 1 1 603A27AC
P 2000 6100
F 0 "#PWR0124" H 2000 5850 50  0001 C CNN
F 1 "GND" H 2005 5927 50  0000 C CNN
F 2 "" H 2000 6100 50  0001 C CNN
F 3 "" H 2000 6100 50  0001 C CNN
	1    2000 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 6100 2000 6100
Wire Wire Line
	1900 5500 2200 5500
Text Label 1850 5700 0    50   ~ 0
SCL
Wire Wire Line
	1800 5700 2100 5700
Text Label 1850 5800 0    50   ~ 0
SDA
Wire Wire Line
	1800 5800 2100 5800
Text Label 1850 5900 0    50   ~ 0
ADDR
Text Label 1850 6000 0    50   ~ 0
ALERT
$Comp
L Device:R R3
U 1 1 603CACB2
P 2250 6000
F 0 "R3" V 2200 6200 50  0000 C CNN
F 1 "10K" V 2250 6000 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2180 6000 50  0001 C CNN
F 3 "~" H 2250 6000 50  0001 C CNN
F 4 "C23192" V 2250 6000 50  0001 C CNN "LCSC"
	1    2250 6000
	0    1    1    0   
$EndComp
Wire Wire Line
	2400 6000 2550 6000
Wire Wire Line
	1800 6000 2100 6000
$Comp
L Connector:Conn_01x02_Female J7
U 1 1 5F694DC9
P 6900 1400
F 0 "J7" V 6746 1448 50  0000 L CNN
F 1 "Conn_01x02_Female" V 6837 1448 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 6900 1400 50  0001 C CNN
F 3 "~" H 6900 1400 50  0001 C CNN
	1    6900 1400
	0    1    1    0   
$EndComp
Wire Wire Line
	5150 1500 5150 1150
Wire Wire Line
	5350 1150 5350 1000
Text GLabel 5550 800  2    50   Input ~ 0
AIN0
Text GLabel 5550 900  2    50   Input ~ 0
AIN1
Wire Wire Line
	5400 900  5550 900 
Text GLabel 1050 5600 0    50   Input ~ 0
AIN0
Wire Wire Line
	1050 5600 1100 5600
Text GLabel 1050 5700 0    50   Input ~ 0
AIN1
Wire Wire Line
	1050 5700 1100 5700
Text GLabel 1050 5800 0    50   Input ~ 0
AIN2
Text GLabel 1050 5900 0    50   Input ~ 0
AIN3
Wire Wire Line
	1050 5800 1100 5800
Wire Wire Line
	1050 5900 1100 5900
Text GLabel 7000 1000 2    50   Input ~ 0
AIN3
Text GLabel 7000 800  2    50   Input ~ 0
AIN2
Wire Wire Line
	6900 1000 7000 1000
Connection ~ 6900 1000
$Comp
L power:GND #PWR0125
U 1 1 604346D4
P 2200 5900
F 0 "#PWR0125" H 2200 5650 50  0001 C CNN
F 1 "GND" V 2205 5772 50  0000 R CNN
F 2 "" H 2200 5900 50  0001 C CNN
F 3 "" H 2200 5900 50  0001 C CNN
	1    2200 5900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1800 5900 2200 5900
$Comp
L Connector:TestPoint TP3
U 1 1 60438278
P 3350 5900
F 0 "TP3" V 3304 6088 50  0000 L CNN
F 1 "TestPoint" V 3395 6088 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x01_P2.54mm_Vertical" H 3550 5900 50  0001 C CNN
F 3 "~" H 3550 5900 50  0001 C CNN
	1    3350 5900
	0    1    1    0   
$EndComp
Text GLabel 3250 5900 0    50   Input ~ 0
ADDR
Wire Wire Line
	3250 5900 3350 5900
Wire Wire Line
	5500 800  5500 1400
Wire Wire Line
	5500 1400 5050 1400
Wire Wire Line
	5050 1400 5050 1500
Connection ~ 5500 800 
Wire Wire Line
	5500 800  5550 800 
Wire Wire Line
	3200 800  3200 950 
$Comp
L Connector:TestPoint TP2
U 1 1 6047E08F
P 2700 800
F 0 "TP2" H 2758 918 50  0000 L CNN
F 1 "TestPoint" H 2750 850 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x01_P2.54mm_Vertical" H 2900 800 50  0001 C CNN
F 3 "~" H 2900 800 50  0001 C CNN
	1    2700 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 800  2700 1000
$Comp
L Device:Ferrite_Bead FB2
U 1 1 604A71D6
P 1550 1150
F 0 "FB2" H 1687 1196 50  0000 L CNN
F 1 "Ferrite_Bead" H 1687 1105 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1480 1150 50  0001 C CNN
F 3 "~" H 1550 1150 50  0001 C CNN
F 4 "C21517" H 1550 1150 50  0001 C CNN "LCSC"
	1    1550 1150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0126
U 1 1 604AA4F9
P 2550 6000
F 0 "#PWR0126" H 2550 5750 50  0001 C CNN
F 1 "GND" V 2555 5872 50  0000 R CNN
F 2 "" H 2550 6000 50  0001 C CNN
F 3 "" H 2550 6000 50  0001 C CNN
	1    2550 6000
	0    -1   -1   0   
$EndComp
$Comp
L Device:Ferrite_Bead FB1
U 1 1 604D6C33
P 1250 1150
F 0 "FB1" H 850 1200 50  0000 L CNN
F 1 "Ferrite_Bead" H 700 1100 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1180 1150 50  0001 C CNN
F 3 "~" H 1250 1150 50  0001 C CNN
F 4 "C21517" H 1250 1150 50  0001 C CNN "LCSC"
	1    1250 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 1300 1550 1400
Wire Wire Line
	1250 1300 1250 1400
Text GLabel 6950 2800 2    50   Input ~ 0
RST
Wire Wire Line
	4600 4800 4600 4850
Connection ~ 5000 4900
Wire Wire Line
	5000 4900 5000 4800
$Comp
L ADS1115:SW_SPST_SM_DP RST1
U 1 1 5F76AB89
P 4800 4900
F 0 "RST1" H 4800 4850 50  0000 C CNN
F 1 "SW_SPST_SM_DP" H 5400 5000 50  0000 C CNN
F 2 "eec:E-Switch-TL3342F160QG-0" H 4800 4900 50  0001 C CNN
F 3 "~" H 4800 4900 50  0001 C CNN
F 4 "C530667" H 4800 4900 50  0001 C CNN "LCSC"
	1    4800 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 4900 5000 4900
Wire Wire Line
	5050 5100 5050 4900
$Comp
L power:GND #PWR0106
U 1 1 5F68C201
P 5050 5100
F 0 "#PWR0106" H 5050 4850 50  0001 C CNN
F 1 "GND" H 5055 4927 50  0000 C CNN
F 2 "" H 5050 5100 50  0001 C CNN
F 3 "" H 5050 5100 50  0001 C CNN
	1    5050 5100
	1    0    0    -1  
$EndComp
Text GLabel 4850 2900 2    50   Input ~ 0
RST
$Comp
L power:+3.3V #PWR0128
U 1 1 6057E04E
P 4800 3000
F 0 "#PWR0128" H 4800 2850 50  0001 C CNN
F 1 "+3.3V" V 4815 3128 50  0000 L CNN
F 2 "" H 4800 3000 50  0001 C CNN
F 3 "" H 4800 3000 50  0001 C CNN
	1    4800 3000
	0    1    1    0   
$EndComp
Text GLabel 3850 5100 3    50   Input ~ 0
SDA
Text GLabel 2550 2800 0    50   Input ~ 0
FTDI_TX
Text GLabel 5700 3000 3    50   Input ~ 0
FTDI_TX
Text GLabel 5800 3000 3    50   Input ~ 0
FTDI_RX
Wire Wire Line
	5800 2600 5800 3000
Text GLabel 4550 3100 2    50   Input ~ 0
FTDI_RX
Wire Wire Line
	4500 2800 4500 2750
Wire Wire Line
	4500 2750 4600 2750
Wire Wire Line
	1250 2250 1250 1900
Connection ~ 1250 2250
Wire Wire Line
	1550 2250 1550 1900
Connection ~ 1550 2250
$Comp
L Device:C C2
U 1 1 5F692175
P 1400 2250
F 0 "C2" V 1300 2250 50  0000 C CNN
F 1 "100uF" V 1550 2250 50  0000 C CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-3528-12_Kemet-T" H 1438 2100 50  0001 C CNN
F 3 "~" H 1400 2250 50  0001 C CNN
F 4 "C16133" V 1400 2250 50  0001 C CNN "LCSC"
F 5 "0;0;180" V 1400 2250 50  0001 C CNN "JLCPCB_CORRECTION"
	1    1400 2250
	0    1    1    0   
$EndComp
$Comp
L Device:CP C4
U 1 1 6060DF74
P 1400 2650
F 0 "C4" V 1250 2650 50  0000 C CNN
F 1 "1000uF" V 1500 2650 50  0000 C CNN
F 2 "Capacitor_SMD:C_Elec_8x10.2" H 1438 2500 50  0001 C CNN
F 3 "~" H 1400 2650 50  0001 C CNN
F 4 "C99833" V 1400 2650 50  0001 C CNN "LCSC"
F 5 "0;0;180" V 1400 2650 50  0001 C CNN "JLCPCB_CORRECTION"
	1    1400 2650
	0    1    1    0   
$EndComp
Wire Wire Line
	1250 2250 1250 2650
Wire Wire Line
	1250 2650 1250 2750
Connection ~ 1250 2650
Wire Wire Line
	1550 2750 1550 2650
Wire Wire Line
	1550 2250 1550 2650
Connection ~ 1550 2650
$Comp
L Device:D D1
U 1 1 60642C1C
P 6850 2550
F 0 "D1" V 6804 2630 50  0000 L CNN
F 1 "D" V 6895 2630 50  0000 L CNN
F 2 "Diode_SMD:D_SOD-123" H 6850 2550 50  0001 C CNN
F 3 "~" H 6850 2550 50  0001 C CNN
F 4 "C81598" V 6850 2550 50  0001 C CNN "LCSC"
	1    6850 2550
	0    1    1    0   
$EndComp
Wire Wire Line
	6850 2800 6850 2700
Connection ~ 6850 2800
Wire Wire Line
	6850 2800 6950 2800
Wire Wire Line
	6850 2400 6850 2350
Wire Wire Line
	6850 2350 6700 2350
Connection ~ 6700 2350
Wire Wire Line
	6700 2350 6700 2400
$Comp
L Regulator_Linear:AMS1117-3.3 U3
U 1 1 60649B4D
P 2800 1700
F 0 "U3" H 2800 1942 50  0000 C CNN
F 1 "AMS1117-3.3" H 2800 1851 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 2800 1900 50  0001 C CNN
F 3 "http://www.advanced-monolithic.com/pdf/ds1117.pdf" H 2900 1450 50  0001 C CNN
F 4 "C6186" H 2800 1700 50  0001 C CNN "LCSC"
F 5 "0;0;180" H 2800 1700 50  0001 C CNN "JLCPCB_CORRECTION"
	1    2800 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 60673198
P 1400 1900
F 0 "C3" V 1148 1900 50  0000 C CNN
F 1 "1uF" V 1239 1900 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 1438 1750 50  0001 C CNN
F 3 "~" H 1400 1900 50  0001 C CNN
F 4 "C52923" H 1400 1900 50  0001 C CNN "LCSC"
	1    1400 1900
	0    1    1    0   
$EndComp
$Comp
L Device:C C1
U 1 1 606769FE
P 1400 850
F 0 "C1" V 1148 850 50  0000 C CNN
F 1 "1uF" V 1239 850 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 1438 700 50  0001 C CNN
F 3 "~" H 1400 850 50  0001 C CNN
F 4 "C52923" H 1400 850 50  0001 C CNN "LCSC"
	1    1400 850 
	0    1    1    0   
$EndComp
Connection ~ 6750 800 
Wire Wire Line
	6750 800  7000 800 
Wire Wire Line
	6800 1200 6800 1100
Wire Wire Line
	6800 1100 7300 1100
Wire Wire Line
	7300 1100 7300 700 
Wire Wire Line
	7300 700  6750 700 
Wire Wire Line
	6750 700  6750 800 
Wire Wire Line
	5300 1000 5350 1000
Wire Wire Line
	5300 800  5500 800 
$Comp
L JACK_TRS:AudioJack3_GroundPinNumbers J3
U 1 1 5F695121
P 5100 900
F 0 "J3" H 5082 1225 50  0000 C CNN
F 1 "AudioJack3_GroundPinNumbers" H 5082 1134 50  0000 C CNN
F 2 "audio35mm:Tayda_3.5mm_stereo_TRS_jack_A-853" H 5100 900 50  0001 C CNN
F 3 "~" H 5100 900 50  0001 C CNN
	1    5100 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 1000 6900 1000
$Comp
L JACK_TRS:AudioJack3_GroundPinNumbers J8
U 1 1 603F4C0C
P 6550 900
F 0 "J8" H 6532 1225 50  0000 C CNN
F 1 "AudioJack3_GroundPinNumbers" H 6532 1134 50  0000 C CNN
F 2 "audio35mm:Tayda_3.5mm_stereo_TRS_jack_A-853" H 6550 900 50  0001 C CNN
F 3 "~" H 6550 900 50  0001 C CNN
	1    6550 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 2800 2600 2800
Wire Wire Line
	2550 2900 2600 2900
Wire Wire Line
	4500 2900 4850 2900
Wire Wire Line
	2550 3000 2600 3000
Wire Wire Line
	4500 3000 4800 3000
Wire Wire Line
	2500 3100 2600 3100
Wire Wire Line
	4550 3100 4500 3100
$Comp
L ESP8266:ESP-01v090 U2
U 1 1 5F67B525
P 3550 2950
F 0 "U2" H 3550 2435 50  0000 C CNN
F 1 "ESP-01v090" H 3550 2526 50  0000 C CNN
F 2 "ESP8266:ESP-01" H 3550 2950 50  0001 C CNN
F 3 "http://l0l.org.uk/2014/12/esp8266-modules-hardware-guide-gotta-catch-em-all/" H 3550 2950 50  0001 C CNN
	1    3550 2950
	-1   0    0    1   
$EndComp
Text GLabel 4450 4800 1    50   Input ~ 0
RST
Wire Wire Line
	4450 4800 4450 4850
Wire Wire Line
	4450 4850 4600 4850
Connection ~ 4600 4850
Wire Wire Line
	4600 4850 4600 4900
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 607BDB4C
P 5850 3650
F 0 "H1" H 5950 3699 50  0000 L CNN
F 1 "MountingHole_Pad" H 5950 3608 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.5mm_Pad" H 5850 3650 50  0001 C CNN
F 3 "~" H 5850 3650 50  0001 C CNN
	1    5850 3650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0127
U 1 1 607BF8E0
P 5850 3900
F 0 "#PWR0127" H 5850 3650 50  0001 C CNN
F 1 "GND" H 5855 3727 50  0000 C CNN
F 2 "" H 5850 3900 50  0001 C CNN
F 3 "" H 5850 3900 50  0001 C CNN
	1    5850 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 3750 5850 3900
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 607C4343
P 6050 3900
F 0 "H2" H 6150 3949 50  0000 L CNN
F 1 "MountingHole_Pad" H 6150 3858 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.5mm_Pad" H 6050 3900 50  0001 C CNN
F 3 "~" H 6050 3900 50  0001 C CNN
	1    6050 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0129
U 1 1 607C4349
P 6050 4150
F 0 "#PWR0129" H 6050 3900 50  0001 C CNN
F 1 "GND" H 6055 3977 50  0000 C CNN
F 2 "" H 6050 4150 50  0001 C CNN
F 3 "" H 6050 4150 50  0001 C CNN
	1    6050 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 4000 6050 4150
$Comp
L Mechanical:MountingHole_Pad H3
U 1 1 607C71AC
P 6250 4150
F 0 "H3" H 6350 4199 50  0000 L CNN
F 1 "MountingHole_Pad" H 6350 4108 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.5mm_Pad" H 6250 4150 50  0001 C CNN
F 3 "~" H 6250 4150 50  0001 C CNN
	1    6250 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0130
U 1 1 607C71B2
P 6250 4400
F 0 "#PWR0130" H 6250 4150 50  0001 C CNN
F 1 "GND" H 6255 4227 50  0000 C CNN
F 2 "" H 6250 4400 50  0001 C CNN
F 3 "" H 6250 4400 50  0001 C CNN
	1    6250 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 4250 6250 4400
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 607CA084
P 6400 4400
F 0 "H4" H 6500 4449 50  0000 L CNN
F 1 "MountingHole_Pad" H 6500 4358 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.5mm_Pad" H 6400 4400 50  0001 C CNN
F 3 "~" H 6400 4400 50  0001 C CNN
	1    6400 4400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0131
U 1 1 607CA08A
P 6400 4650
F 0 "#PWR0131" H 6400 4400 50  0001 C CNN
F 1 "GND" H 6405 4477 50  0000 C CNN
F 2 "" H 6400 4650 50  0001 C CNN
F 3 "" H 6400 4650 50  0001 C CNN
	1    6400 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 4500 6400 4650
$Comp
L Connector:USB_B_Micro J1
U 1 1 603D3CE7
P 950 3750
F 0 "J1" H 1007 4217 50  0000 C CNN
F 1 "USB_B_Micro" H 1007 4126 50  0000 C CNN
F 2 "digikey-footprints:USB_Micro_AB_Female_0475890001_DP_PWR_GND" H 1100 3700 50  0001 C CNN
F 3 "~" H 1100 3700 50  0001 C CNN
	1    950  3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 3550 1400 3550
Wire Wire Line
	850  4150 850  4200
Wire Wire Line
	950  4150 950  4200
Wire Wire Line
	950  4200 850  4200
Connection ~ 850  4200
Wire Wire Line
	850  4200 850  4300
$EndSCHEMATC

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
Text GLabel 10550 7800 2    50   Input ~ 0
Throttle
$Comp
L Device:Q_PMOS_GDS Q1
U 1 1 6067746C
P 1500 6900
F 0 "Q1" H 1704 6946 50  0000 L CNN
F 1 "IRF1010N" H 1704 6855 50  0000 L CNN
F 2 "Package_TO_SOT_THT_Electeam:TO-220-3_Horizontal_TabDown" H 1700 7000 50  0001 C CNN
F 3 "~" H 1500 6900 50  0001 C CNN
	1    1500 6900
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_PMOS_GDS Q2
U 1 1 6067945D
P 1500 7400
F 0 "Q2" H 1704 7446 50  0000 L CNN
F 1 "IRF1010N" H 1704 7355 50  0000 L CNN
F 2 "Package_TO_SOT_THT_Electeam:TO-220-3_Horizontal_TabUp" H 1700 7500 50  0001 C CNN
F 3 "~" H 1500 7400 50  0001 C CNN
	1    1500 7400
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_PMOS_GDS Q3
U 1 1 60679A4E
P 3600 6900
F 0 "Q3" H 3804 6946 50  0000 L CNN
F 1 "IRF1010N" H 3804 6855 50  0000 L CNN
F 2 "Package_TO_SOT_THT_Electeam:TO-220-3_Horizontal_TabDown" H 3800 7000 50  0001 C CNN
F 3 "~" H 3600 6900 50  0001 C CNN
	1    3600 6900
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_PMOS_GDS Q4
U 1 1 6067A63E
P 3600 7400
F 0 "Q4" H 3804 7446 50  0000 L CNN
F 1 "IRF1010N" H 3804 7355 50  0000 L CNN
F 2 "Package_TO_SOT_THT_Electeam:TO-220-3_Horizontal_TabUp" H 3800 7500 50  0001 C CNN
F 3 "~" H 3600 7400 50  0001 C CNN
	1    3600 7400
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_PMOS_GDS Q5
U 1 1 6067ADF0
P 5650 6950
F 0 "Q5" H 5854 6996 50  0000 L CNN
F 1 "IRF1010N" H 5854 6905 50  0000 L CNN
F 2 "Package_TO_SOT_THT_Electeam:TO-220-3_Horizontal_TabDown" H 5850 7050 50  0001 C CNN
F 3 "~" H 5650 6950 50  0001 C CNN
	1    5650 6950
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_PMOS_GDS Q6
U 1 1 6067B8DE
P 5650 7450
F 0 "Q6" H 5854 7496 50  0000 L CNN
F 1 "IRF1010N" H 5854 7405 50  0000 L CNN
F 2 "Package_TO_SOT_THT_Electeam:TO-220-3_Horizontal_TabUp" H 5850 7550 50  0001 C CNN
F 3 "~" H 5650 7450 50  0001 C CNN
	1    5650 7450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 6900 3400 6900
Wire Wire Line
	1100 6900 1300 6900
Wire Wire Line
	1100 7400 1300 7400
Wire Wire Line
	1600 7100 1600 7150
Wire Wire Line
	3150 7400 3400 7400
Wire Wire Line
	5200 6950 5450 6950
Wire Wire Line
	5200 7450 5450 7450
Wire Wire Line
	5950 5350 5500 5350
Wire Wire Line
	4000 5400 3550 5400
Wire Wire Line
	1950 5350 1500 5350
$Comp
L power:GND #PWR026
U 1 1 606BCA6C
P 1200 6250
F 0 "#PWR026" H 1200 6000 50  0001 C CNN
F 1 "GND" H 1205 6077 50  0000 C CNN
F 2 "" H 1200 6250 50  0001 C CNN
F 3 "" H 1200 6250 50  0001 C CNN
	1    1200 6250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR028
U 1 1 606C580A
P 5200 6250
F 0 "#PWR028" H 5200 6000 50  0001 C CNN
F 1 "GND" H 5205 6077 50  0000 C CNN
F 2 "" H 5200 6250 50  0001 C CNN
F 3 "" H 5200 6250 50  0001 C CNN
	1    5200 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 7100 3700 7150
Wire Wire Line
	5750 7150 5750 7200
Wire Wire Line
	5500 5850 5550 5850
Wire Wire Line
	1600 5450 1500 5450
Wire Wire Line
	800  6900 650  6900
Wire Wire Line
	1600 5950 1500 5950
Wire Wire Line
	800  7400 650  7400
Wire Wire Line
	3650 5500 3550 5500
Wire Wire Line
	3650 6000 3550 6000
Wire Wire Line
	2750 6900 2850 6900
Wire Wire Line
	2750 7400 2850 7400
Wire Wire Line
	5500 5450 5600 5450
Wire Wire Line
	5500 5950 5600 5950
Wire Wire Line
	4800 6950 4900 6950
Wire Wire Line
	4800 7450 4900 7450
Wire Wire Line
	1600 6600 1600 6700
Wire Wire Line
	1600 7600 1600 7700
Wire Wire Line
	3700 6550 3700 6700
Wire Wire Line
	3700 7600 3700 7700
Wire Wire Line
	5750 6650 5750 6750
Wire Wire Line
	5750 7650 5750 7750
Wire Wire Line
	3650 5900 3600 5900
Text GLabel 1750 7150 2    50   Input ~ 0
PH1
Text GLabel 3850 7150 2    50   Input ~ 0
PH2
Text GLabel 5900 7200 2    50   Input ~ 0
PH3
Wire Wire Line
	1750 7150 1600 7150
Connection ~ 1600 7150
Wire Wire Line
	1600 7150 1600 7200
Wire Wire Line
	3850 7150 3700 7150
Connection ~ 3700 7150
Wire Wire Line
	3700 7150 3700 7200
Wire Wire Line
	5900 7200 5750 7200
Connection ~ 5750 7200
Wire Wire Line
	5750 7200 5750 7250
Wire Wire Line
	2850 5700 2950 5700
Wire Wire Line
	2850 5800 2950 5800
Wire Wire Line
	800  5650 900  5650
Wire Wire Line
	800  5750 900  5750
Wire Wire Line
	4800 5650 4900 5650
Wire Wire Line
	4800 5750 4900 5750
Text GLabel 3700 7700 2    50   Input ~ 0
Current_Sensor
Text GLabel 5750 7750 2    50   Input ~ 0
Current_Sensor
Text GLabel 1600 7700 2    50   Input ~ 0
Current_Sensor
$Comp
L pspice:DIODE D1
U 1 1 6066AC3A
P 1950 5050
F 0 "D1" V 1996 4922 50  0000 R CNN
F 1 "1N4004" V 1905 4922 50  0000 R CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 1950 5050 50  0001 C CNN
F 3 "~" H 1950 5050 50  0001 C CNN
	1    1950 5050
	0    1    1    0   
$EndComp
$Comp
L pspice:DIODE D2
U 1 1 6066F442
P 4000 5100
F 0 "D2" V 4046 4972 50  0000 R CNN
F 1 "1N4004" V 3955 4972 50  0000 R CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 4000 5100 50  0001 C CNN
F 3 "~" H 4000 5100 50  0001 C CNN
	1    4000 5100
	0    1    1    0   
$EndComp
$Comp
L pspice:DIODE D3
U 1 1 6066FEAB
P 5950 5050
F 0 "D3" V 5996 4922 50  0000 R CNN
F 1 "1N4004" V 5905 4922 50  0000 R CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 5950 5050 50  0001 C CNN
F 3 "~" H 5950 5050 50  0001 C CNN
	1    5950 5050
	0    1    1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 606765CA
P 5050 7450
F 0 "R8" V 4950 7450 50  0000 C CNN
F 1 "100" V 5050 7450 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4980 7450 50  0001 C CNN
F 3 "~" H 5050 7450 50  0001 C CNN
	1    5050 7450
	0    1    1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 6067585C
P 5050 6950
F 0 "R7" V 4950 6950 50  0000 C CNN
F 1 "100" V 5050 6950 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4980 6950 50  0001 C CNN
F 3 "~" H 5050 6950 50  0001 C CNN
	1    5050 6950
	0    1    1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 6067511F
P 3000 7400
F 0 "R6" V 2900 7400 50  0000 C CNN
F 1 "100" V 3000 7400 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2930 7400 50  0001 C CNN
F 3 "~" H 3000 7400 50  0001 C CNN
	1    3000 7400
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 606747BF
P 3000 6900
F 0 "R5" V 2900 6900 50  0000 C CNN
F 1 "100" V 3000 6900 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2930 6900 50  0001 C CNN
F 3 "~" H 3000 6900 50  0001 C CNN
	1    3000 6900
	0    1    1    0   
$EndComp
$Comp
L Device:R R10
U 1 1 60672386
P 950 7400
F 0 "R10" V 850 7400 50  0000 C CNN
F 1 "100" V 950 7400 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 880 7400 50  0001 C CNN
F 3 "~" H 950 7400 50  0001 C CNN
	1    950  7400
	0    1    1    0   
$EndComp
$Comp
L Device:R R9
U 1 1 60670FD6
P 950 6900
F 0 "R9" V 850 6900 50  0000 C CNN
F 1 "100" V 950 6900 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 880 6900 50  0001 C CNN
F 3 "~" H 950 6900 50  0001 C CNN
	1    950  6900
	0    1    1    0   
$EndComp
Wire Wire Line
	5950 5850 5950 5750
$Comp
L Device:CP C4
U 1 1 6066A248
P 5950 5600
F 0 "C4" H 6068 5646 50  0000 L CNN
F 1 "2.2uF" H 6068 5555 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 5988 5450 50  0001 C CNN
F 3 "~" H 5950 5600 50  0001 C CNN
	1    5950 5600
	1    0    0    -1  
$EndComp
Text GLabel 4800 7450 0    50   Input ~ 0
LO3
Text GLabel 4800 6950 0    50   Input ~ 0
HO3
Text GLabel 5950 5850 2    50   Input ~ 0
VS3
Text GLabel 5600 5950 2    50   Input ~ 0
LO3
Text GLabel 5600 5850 2    50   Input ~ 0
VS3
Text GLabel 5600 5450 2    50   Input ~ 0
HO3
Text GLabel 4800 5750 0    50   Input ~ 0
LIN3
Text GLabel 4800 5650 0    50   Input ~ 0
HIN3
Wire Wire Line
	4000 5900 4000 5800
$Comp
L Device:CP C3
U 1 1 60669B72
P 4000 5650
F 0 "C3" H 4118 5696 50  0000 L CNN
F 1 "2.2uF" H 4118 5605 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 4038 5500 50  0001 C CNN
F 3 "~" H 4000 5650 50  0001 C CNN
	1    4000 5650
	1    0    0    -1  
$EndComp
Text GLabel 2750 7400 0    50   Input ~ 0
LO2
Text GLabel 2750 6900 0    50   Input ~ 0
HO2
Text GLabel 4000 5900 2    50   Input ~ 0
VS2
Text GLabel 3650 6000 2    50   Input ~ 0
LO2
Text GLabel 3650 5900 2    50   Input ~ 0
VS2
Text GLabel 3650 5500 2    50   Input ~ 0
HO2
Text GLabel 2850 5800 0    50   Input ~ 0
LIN2
Text GLabel 2850 5700 0    50   Input ~ 0
HIN2
Wire Wire Line
	1950 5700 1950 5800
$Comp
L Device:CP C2
U 1 1 606683E1
P 1950 5550
F 0 "C2" H 2068 5596 50  0000 L CNN
F 1 "2.2uF" H 2068 5505 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 1988 5400 50  0001 C CNN
F 3 "~" H 1950 5550 50  0001 C CNN
	1    1950 5550
	1    0    0    -1  
$EndComp
Text GLabel 1950 5800 2    50   Input ~ 0
VS1
Text GLabel 650  7400 0    50   Input ~ 0
LO1
Text GLabel 650  6900 0    50   Input ~ 0
HO1
Text GLabel 1600 5950 2    50   Input ~ 0
LO1
Text GLabel 1600 5850 2    50   Input ~ 0
VS1
Text GLabel 1600 5450 2    50   Input ~ 0
HO1
Text GLabel 800  5750 0    50   Input ~ 0
LIN1
Text GLabel 800  5650 0    50   Input ~ 0
HIN1
Text GLabel 1600 5750 2    50   Input ~ 0
PH1
Wire Wire Line
	1600 5850 1550 5850
Wire Wire Line
	1600 5750 1550 5750
Wire Wire Line
	1550 5750 1550 5850
Connection ~ 1550 5850
Wire Wire Line
	1550 5850 1500 5850
Text GLabel 3650 5800 2    50   Input ~ 0
PH2
Wire Wire Line
	3650 5800 3600 5800
Wire Wire Line
	3600 5800 3600 5900
Connection ~ 3600 5900
Wire Wire Line
	3600 5900 3550 5900
Text GLabel 5600 5750 2    50   Input ~ 0
PH3
Wire Wire Line
	5600 5750 5550 5750
Wire Wire Line
	5550 5750 5550 5850
Connection ~ 5550 5850
Wire Wire Line
	5550 5850 5600 5850
$Comp
L Driver_FET:IR2103 U8
U 1 1 60823C9D
P 1200 5650
F 0 "U8" H 1200 6331 50  0000 C CNN
F 1 "IR2103" H 1200 6240 50  0000 C CNN
F 2 "Package_SO:SO-8_3.9x4.9mm_P1.27mm" H 1200 5650 50  0001 C CIN
F 3 "https://www.infineon.com/dgdl/ir2103.pdf?fileId=5546d462533600a4015355c7b54b166f" H 1200 5650 50  0001 C CNN
	1    1200 5650
	1    0    0    -1  
$EndComp
$Comp
L Driver_FET:IR2103 U9
U 1 1 60825783
P 3250 5700
F 0 "U9" H 3250 6381 50  0000 C CNN
F 1 "IR2103" H 3250 6290 50  0000 C CNN
F 2 "Package_SO:SO-8_3.9x4.9mm_P1.27mm" H 3250 5700 50  0001 C CIN
F 3 "https://www.infineon.com/dgdl/ir2103.pdf?fileId=5546d462533600a4015355c7b54b166f" H 3250 5700 50  0001 C CNN
	1    3250 5700
	1    0    0    -1  
$EndComp
$Comp
L Driver_FET:IR2103 U4
U 1 1 6092DBC3
P 5200 5650
F 0 "U4" H 5200 6331 50  0000 C CNN
F 1 "IR2103" H 5200 6240 50  0000 C CNN
F 2 "Package_SO:SO-8_3.9x4.9mm_P1.27mm" H 5200 5650 50  0001 C CIN
F 3 "https://www.infineon.com/dgdl/ir2103.pdf?fileId=5546d462533600a4015355c7b54b166f" H 5200 5650 50  0001 C CNN
	1    5200 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 4850 1200 5150
Wire Wire Line
	1950 4850 1200 4850
Wire Wire Line
	1950 5350 1950 5250
Wire Wire Line
	3250 4900 3250 5200
Wire Wire Line
	3250 4900 4000 4900
Wire Wire Line
	4000 5300 4000 5400
Wire Wire Line
	1200 6150 1200 6250
$Comp
L power:GND #PWR0108
U 1 1 6088A078
P 3250 6300
F 0 "#PWR0108" H 3250 6050 50  0001 C CNN
F 1 "GND" H 3255 6127 50  0000 C CNN
F 2 "" H 3250 6300 50  0001 C CNN
F 3 "" H 3250 6300 50  0001 C CNN
	1    3250 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 6200 3250 6300
Wire Wire Line
	5200 6150 5200 6250
Wire Wire Line
	5950 4850 5200 4850
Connection ~ 5200 4850
Wire Wire Line
	5200 4850 5200 5150
Wire Wire Line
	5950 5350 5950 5250
Text GLabel 1600 6600 2    50   Input ~ 0
36V-Power
Text GLabel 3700 6550 2    50   Input ~ 0
36V-Power
Text GLabel 5750 6650 2    50   Input ~ 0
36V-Power
Wire Wire Line
	5200 4750 5200 4850
Connection ~ 3250 4900
Wire Wire Line
	1200 4850 1200 4700
Connection ~ 1200 4850
$Comp
L power:+12V #PWR0115
U 1 1 608462B9
P 1200 4700
F 0 "#PWR0115" H 1200 4550 50  0001 C CNN
F 1 "+12V" H 1215 4873 50  0000 C CNN
F 2 "" H 1200 4700 50  0001 C CNN
F 3 "" H 1200 4700 50  0001 C CNN
	1    1200 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 4900 3250 4750
$Comp
L power:+12V #PWR0116
U 1 1 6084A421
P 3250 4750
F 0 "#PWR0116" H 3250 4600 50  0001 C CNN
F 1 "+12V" H 3265 4923 50  0000 C CNN
F 2 "" H 3250 4750 50  0001 C CNN
F 3 "" H 3250 4750 50  0001 C CNN
	1    3250 4750
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0117
U 1 1 6084AD2A
P 5200 4750
F 0 "#PWR0117" H 5200 4600 50  0001 C CNN
F 1 "+12V" H 5215 4923 50  0000 C CNN
F 2 "" H 5200 4750 50  0001 C CNN
F 3 "" H 5200 4750 50  0001 C CNN
	1    5200 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 5400 1950 5350
Connection ~ 1950 5350
Wire Wire Line
	4000 5500 4000 5400
Connection ~ 4000 5400
Wire Wire Line
	5950 5450 5950 5350
Connection ~ 5950 5350
Text Notes 500  450  0    98   ~ 0
Power System\n
Wire Wire Line
	800  1200 950  1200
Wire Wire Line
	1250 1200 1350 1200
Wire Wire Line
	2600 2350 2600 2850
Wire Wire Line
	2600 3050 2250 3050
Connection ~ 2600 3050
Wire Wire Line
	2900 3050 2600 3050
Wire Wire Line
	2600 2850 2250 2850
Connection ~ 2600 2850
Wire Wire Line
	2900 2850 2600 2850
$Comp
L power:+12V #PWR0120
U 1 1 61AD3724
P 2600 2350
F 0 "#PWR0120" H 2600 2200 50  0001 C CNN
F 1 "+12V" H 2615 2523 50  0000 C CNN
F 2 "" H 2600 2350 50  0001 C CNN
F 3 "" H 2600 2350 50  0001 C CNN
	1    2600 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4275 2425 3950 2425
Connection ~ 4275 2425
Wire Wire Line
	4275 2425 4275 2350
$Comp
L power:+5V #PWR0121
U 1 1 618BBD75
P 4275 2350
F 0 "#PWR0121" H 4275 2200 50  0001 C CNN
F 1 "+5V" H 4290 2523 50  0000 C CNN
F 2 "" H 4275 2350 50  0001 C CNN
F 3 "" H 4275 2350 50  0001 C CNN
	1    4275 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 2425 3950 2850
Wire Wire Line
	4550 2425 4550 2500
Wire Wire Line
	4550 2425 4275 2425
Wire Wire Line
	5650 2350 5650 2950
$Comp
L power:-5V #PWR0137
U 1 1 608A9160
P 5650 2350
F 0 "#PWR0137" H 5650 2450 50  0001 C CNN
F 1 "-5V" H 5665 2523 50  0000 C CNN
F 2 "" H 5650 2350 50  0001 C CNN
F 3 "" H 5650 2350 50  0001 C CNN
	1    5650 2350
	1    0    0    -1  
$EndComp
Connection ~ 5650 2950
$Comp
L Converter_DCDC_Electeam:LM2596SOURCE_Pin2 U3
U 1 1 60F9ED12
P 3300 2950
F 0 "U3" H 3300 3315 50  0000 C CNN
F 1 "LM2596SOURCE_Pin2" H 3300 3224 50  0000 C CNN
F 2 "Package_TO_SOT_THT_Electeam:LM2596DCSourcePin2_Electeam" H 3300 2950 50  0001 C CNN
F 3 "" H 3300 2950 50  0001 C CNN
	1    3300 2950
	1    0    0    -1  
$EndComp
$Comp
L Diode:1N4148 D8
U 1 1 608B1EB7
P 5400 2950
F 0 "D8" H 5400 3167 50  0000 C CNN
F 1 "1N4148" H 5400 3076 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 5400 2775 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/1N4148_1N4448.pdf" H 5400 2950 50  0001 C CNN
	1    5400 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 2950 4400 2950
Connection ~ 4550 2950
Wire Wire Line
	4750 2950 4550 2950
Wire Wire Line
	5100 2950 5250 2950
Connection ~ 5100 2950
Wire Wire Line
	5100 3100 5100 2950
Wire Wire Line
	4950 2950 5100 2950
$Comp
L Device:R R18
U 1 1 60A368A0
P 4550 2650
F 0 "R18" H 4620 2696 50  0000 L CNN
F 1 "220" V 4550 2550 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4480 2650 50  0001 C CNN
F 3 "~" H 4550 2650 50  0001 C CNN
	1    4550 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C5
U 1 1 60A5B403
P 4850 2950
F 0 "C5" H 4700 2850 50  0000 C CNN
F 1 "0.47uF" H 4650 3000 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D7.5mm_W5.0mm_P10.00mm" H 4850 2950 50  0001 C CNN
F 3 "~" H 4850 2950 50  0001 C CNN
	1    4850 2950
	0    1    1    0   
$EndComp
$Comp
L Diode:1N4148 D6
U 1 1 60ABA1D4
P 4250 2950
F 0 "D6" H 4250 3167 50  0000 C CNN
F 1 "1N4148" H 4250 3076 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 4250 2775 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/1N4148_1N4448.pdf" H 4250 2950 50  0001 C CNN
	1    4250 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 2950 4550 2800
Wire Wire Line
	5650 3400 5650 3500
$Comp
L power:GND #PWR0129
U 1 1 60B222A1
P 5650 3500
F 0 "#PWR0129" H 5650 3250 50  0001 C CNN
F 1 "GND" H 5655 3327 50  0000 C CNN
F 2 "" H 5650 3500 50  0001 C CNN
F 3 "" H 5650 3500 50  0001 C CNN
	1    5650 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 3400 5100 3500
$Comp
L power:GND #PWR0128
U 1 1 60B0F797
P 5100 3500
F 0 "#PWR0128" H 5100 3250 50  0001 C CNN
F 1 "GND" H 5105 3327 50  0000 C CNN
F 2 "" H 5100 3500 50  0001 C CNN
F 3 "" H 5100 3500 50  0001 C CNN
	1    5100 3500
	1    0    0    -1  
$EndComp
$Comp
L Diode:1N4148 D7
U 1 1 60AFD8C6
P 5100 3250
F 0 "D7" H 5100 3467 50  0000 C CNN
F 1 "1N4148" H 5100 3376 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 5100 3075 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/1N4148_1N4448.pdf" H 5100 3250 50  0001 C CNN
	1    5100 3250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3700 2950 4100 2950
Wire Wire Line
	5650 2950 5650 3100
Wire Wire Line
	5550 2950 5650 2950
$Comp
L Device:CP C6
U 1 1 60A351B9
P 5650 3250
F 0 "C6" H 5532 3204 50  0000 R CNN
F 1 "47uF" H 5532 3295 50  0000 R CNN
F 2 "Capacitor_THT:CP_Radial_D8.0mm_P3.80mm" H 5688 3100 50  0001 C CNN
F 3 "~" H 5650 3250 50  0001 C CNN
	1    5650 3250
	-1   0    0    1   
$EndComp
Wire Wire Line
	3700 2850 3950 2850
Wire Wire Line
	3950 3050 3700 3050
$Comp
L power:GND #PWR0125
U 1 1 609C613E
P 3950 3500
F 0 "#PWR0125" H 3950 3250 50  0001 C CNN
F 1 "GND" H 3955 3327 50  0000 C CNN
F 2 "" H 3950 3500 50  0001 C CNN
F 3 "" H 3950 3500 50  0001 C CNN
	1    3950 3500
	1    0    0    -1  
$EndComp
$Comp
L Diode:1N47xxA D5
U 1 1 608B0788
P 900 2750
F 0 "D5" V 850 2500 50  0000 L CNN
F 1 "1N4728A" V 950 2350 50  0000 L CNN
F 2 "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal" H 900 2575 50  0001 C CNN
F 3 "https://www.vishay.com/docs/85816/1n4728a.pdf" H 900 2750 50  0001 C CNN
	1    900  2750
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 608CE5F4
P 1250 3450
F 0 "#PWR0118" H 1250 3200 50  0001 C CNN
F 1 "GND" H 1255 3277 50  0000 C CNN
F 2 "" H 1250 3450 50  0001 C CNN
F 3 "" H 1250 3450 50  0001 C CNN
	1    1250 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 3050 1450 3050
$Comp
L Device:R R17
U 1 1 6092FC51
P 900 3150
F 0 "R17" H 970 3196 50  0000 L CNN
F 1 "5.2k" V 900 3050 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 830 3150 50  0001 C CNN
F 3 "~" H 900 3150 50  0001 C CNN
	1    900  3150
	1    0    0    -1  
$EndComp
$Comp
L power:+36V #PWR0123
U 1 1 60972CCF
P 900 2300
F 0 "#PWR0123" H 900 2150 50  0001 C CNN
F 1 "+36V" H 915 2473 50  0000 C CNN
F 2 "" H 900 2300 50  0001 C CNN
F 3 "" H 900 2300 50  0001 C CNN
	1    900  2300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0124
U 1 1 609B2986
P 900 3450
F 0 "#PWR0124" H 900 3200 50  0001 C CNN
F 1 "GND" H 905 3277 50  0000 C CNN
F 2 "" H 900 3450 50  0001 C CNN
F 3 "" H 900 3450 50  0001 C CNN
	1    900  3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	900  3450 900  3300
$Comp
L Converter_DCDC_Electeam:LM2596SOURCE U5
U 1 1 60F35E56
P 1850 2950
F 0 "U5" H 1850 3315 50  0000 C CNN
F 1 "LM2596SOURCE" H 1850 3224 50  0000 C CNN
F 2 "Package_TO_SOT_THT_Electeam:LM2596DCSource_Electeam" H 1850 2950 50  0001 C CNN
F 3 "" H 1850 2950 50  0001 C CNN
	1    1850 2950
	1    0    0    -1  
$EndComp
$Comp
L Diode:1N47xxA D4
U 1 1 608AF42D
P 900 2450
F 0 "D4" V 854 2530 50  0000 L CNN
F 1 "1N4728A" V 945 2530 50  0000 L CNN
F 2 "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal" H 900 2275 50  0001 C CNN
F 3 "https://www.vishay.com/docs/85816/1n4728a.pdf" H 900 2450 50  0001 C CNN
	1    900  2450
	0    1    1    0   
$EndComp
Wire Wire Line
	900  3000 900  2950
Wire Wire Line
	1450 2850 1150 2850
Wire Wire Line
	1150 2850 1150 2950
Wire Wire Line
	1150 2950 900  2950
Connection ~ 900  2950
Wire Wire Line
	900  2950 900  2900
$Comp
L power:GND #PWR0119
U 1 1 608CF78C
P 2600 3500
F 0 "#PWR0119" H 2600 3250 50  0001 C CNN
F 1 "GND" H 2605 3327 50  0000 C CNN
F 2 "" H 2600 3500 50  0001 C CNN
F 3 "" H 2600 3500 50  0001 C CNN
	1    2600 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 1650 2700 1500
Wire Wire Line
	2700 1050 2700 1200
Wire Wire Line
	2250 900  2250 1050
Text GLabel 2250 900  2    50   Input ~ 0
36V-Power
Wire Wire Line
	2250 1800 2250 1650
$Comp
L power:GND #PWR0101
U 1 1 608A0585
P 2250 1800
F 0 "#PWR0101" H 2250 1550 50  0001 C CNN
F 1 "GND" H 2255 1627 50  0000 C CNN
F 2 "" H 2250 1800 50  0001 C CNN
F 3 "" H 2250 1800 50  0001 C CNN
	1    2250 1800
	1    0    0    -1  
$EndComp
Connection ~ 2250 1650
Wire Wire Line
	2700 1650 2250 1650
Wire Wire Line
	2250 1650 2250 1500
Wire Wire Line
	1750 1650 2250 1650
Wire Wire Line
	1750 1500 1750 1650
Connection ~ 2250 1050
Wire Wire Line
	2700 1050 2250 1050
Wire Wire Line
	2350 1450 2350 1300
Wire Wire Line
	2250 1050 2250 1200
Wire Wire Line
	1750 1050 2250 1050
Wire Wire Line
	1750 1200 1750 1050
$Comp
L Device:CP C12
U 1 1 60DBBBDB
P 2700 1350
F 0 "C12" H 2818 1396 50  0000 L CNN
F 1 "220uF" H 2818 1305 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_10x14.3" H 2738 1200 50  0001 C CNN
F 3 "~" H 2700 1350 50  0001 C CNN
	1    2700 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C11
U 1 1 60DBB821
P 2250 1350
F 0 "C11" H 2368 1396 50  0000 L CNN
F 1 "220uF" H 2368 1305 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_10x14.3" H 2288 1200 50  0001 C CNN
F 3 "~" H 2250 1350 50  0001 C CNN
	1    2250 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C10
U 1 1 60B587DD
P 1750 1350
F 0 "C10" H 1868 1396 50  0000 L CNN
F 1 "220uF" H 1868 1305 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_10x14.3" H 1868 1259 50  0001 L CNN
F 3 "~" H 1750 1350 50  0001 C CNN
	1    1750 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 1400 4550 1400
Wire Wire Line
	4250 1600 4250 1400
Wire Wire Line
	4350 1600 4250 1600
Text GLabel 4350 1600 2    50   Input ~ 0
OPC_1
Wire Wire Line
	5000 1750 5100 1750
Connection ~ 5000 1750
Wire Wire Line
	5000 1900 5000 1750
Connection ~ 5100 1750
Wire Wire Line
	4850 1750 5000 1750
$Comp
L power:GND #PWR0109
U 1 1 618C8BBA
P 5000 1900
F 0 "#PWR0109" H 5000 1650 50  0001 C CNN
F 1 "GND" H 5005 1727 50  0000 C CNN
F 2 "" H 5000 1900 50  0001 C CNN
F 3 "" H 5000 1900 50  0001 C CNN
	1    5000 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 1600 4850 1750
Wire Wire Line
	5100 1750 5500 1750
Wire Wire Line
	5100 1350 5100 1750
Wire Wire Line
	5500 900  5500 950 
Connection ~ 5500 900 
Wire Wire Line
	5100 900  5500 900 
Wire Wire Line
	5100 950  5100 900 
Wire Wire Line
	4250 950  4850 950 
Wire Wire Line
	4850 1200 4850 950 
Wire Wire Line
	5500 850  5500 900 
Connection ~ 4250 950 
Wire Wire Line
	4250 950  4250 1000
Wire Wire Line
	4250 900  4250 950 
$Comp
L pspice:DIODE D9
U 1 1 6101C203
P 5100 1150
F 0 "D9" V 5200 1400 50  0000 R CNN
F 1 "1N4004" V 5100 1550 50  0000 R CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 5100 1150 50  0001 C CNN
F 3 "~" H 5100 1150 50  0001 C CNN
	1    5100 1150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5800 850  5800 950 
$Comp
L Relay:RAYEX-L90B K1
U 1 1 6114BA59
P 5700 1350
F 0 "K1" H 6030 1396 50  0000 L CNN
F 1 "RAYEX-L90B" H 6030 1305 50  0000 L CNN
F 2 "Relay_THT:Relay_SPST_RAYEX-L90B" H 6050 1300 50  0001 L CNN
F 3 "https://a3.sofastcdn.com/attachment/7jioKBjnRiiSrjrjknRiwS77gwbf3zmp/L90-SERIES.pdf" H 6200 1200 50  0001 L CNN
	1    5700 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R25
U 1 1 61021042
P 4250 1150
F 0 "R25" H 4320 1196 50  0000 L CNN
F 1 "2k" V 4250 1100 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4180 1150 50  0001 C CNN
F 3 "~" H 4250 1150 50  0001 C CNN
	1    4250 1150
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:PN2222A Q7
U 1 1 6101E23D
P 4750 1400
F 0 "Q7" H 4940 1446 50  0000 L CNN
F 1 "PN2222A" H 4940 1355 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 4950 1325 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/PN2222-D.PDF" H 4750 1400 50  0001 L CNN
	1    4750 1400
	1    0    0    -1  
$EndComp
Connection ~ 4250 1400
Wire Wire Line
	4250 1300 4250 1400
$Comp
L power:+12V #PWR0134
U 1 1 6106A128
P 5500 850
F 0 "#PWR0134" H 5500 700 50  0001 C CNN
F 1 "+12V" H 5515 1023 50  0000 C CNN
F 2 "" H 5500 850 50  0001 C CNN
F 3 "" H 5500 850 50  0001 C CNN
	1    5500 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 1750 5900 1950
$Comp
L power:+36V #PWR03
U 1 1 607E0827
P 5800 850
F 0 "#PWR03" H 5800 700 50  0001 C CNN
F 1 "+36V" H 5815 1023 50  0000 C CNN
F 2 "" H 5800 850 50  0001 C CNN
F 3 "" H 5800 850 50  0001 C CNN
	1    5800 850 
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0110
U 1 1 60E85274
P 4250 900
F 0 "#PWR0110" H 4250 750 50  0001 C CNN
F 1 "+12V" H 4265 1073 50  0000 C CNN
F 2 "" H 4250 900 50  0001 C CNN
F 3 "" H 4250 900 50  0001 C CNN
	1    4250 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 1950 5950 1950
Text GLabel 5950 1950 2    50   Input ~ 0
36V-Power
$Comp
L Device:Opamp_Dual_Generic U2
U 3 1 60CF4CB9
P 3550 1250
F 0 "U2" H 3508 1296 50  0000 L CNN
F 1 "Opamp_Dual_Generic" H 3508 1205 50  0000 L CNN
F 2 "Package_DIP:DIP-8_W7.62mm_LongPads" H 3550 1250 50  0001 C CNN
F 3 "~" H 3550 1250 50  0001 C CNN
	3    3550 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 950  3450 900 
Wire Wire Line
	3450 1550 3450 1650
$Comp
L power:GND #PWR0131
U 1 1 60CCCE5C
P 3450 1650
F 0 "#PWR0131" H 3450 1400 50  0001 C CNN
F 1 "GND" H 3455 1477 50  0000 C CNN
F 2 "" H 3450 1650 50  0001 C CNN
F 3 "" H 3450 1650 50  0001 C CNN
	1    3450 1650
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0130
U 1 1 60CCBCC7
P 3450 900
F 0 "#PWR0130" H 3450 750 50  0001 C CNN
F 1 "+5V" H 3465 1073 50  0000 C CNN
F 2 "" H 3450 900 50  0001 C CNN
F 3 "" H 3450 900 50  0001 C CNN
	1    3450 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 1550 1200 1500
Wire Wire Line
	1350 1200 1350 1150
$Comp
L Device:Fuse F1
U 1 1 6082709D
P 1100 1200
F 0 "F1" V 903 1200 50  0000 C CNN
F 1 "Fuse" V 994 1200 50  0000 C CNN
F 2 "Package_TO_SOT_THT_Electeam:Portafusible" V 1030 1200 50  0001 C CNN
F 3 "~" H 1100 1200 50  0001 C CNN
	1    1100 1200
	0    1    1    0   
$EndComp
$Comp
L power:+36V #PWR07
U 1 1 6069FCB5
P 1350 1150
F 0 "#PWR07" H 1350 1000 50  0001 C CNN
F 1 "+36V" H 1365 1323 50  0000 C CNN
F 2 "" H 1350 1150 50  0001 C CNN
F 3 "" H 1350 1150 50  0001 C CNN
	1    1350 1150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 6069EAE5
P 1200 1550
F 0 "#PWR08" H 1200 1300 50  0001 C CNN
F 1 "GND" H 1205 1377 50  0000 C CNN
F 2 "" H 1200 1550 50  0001 C CNN
F 3 "" H 1200 1550 50  0001 C CNN
	1    1200 1550
	1    0    0    -1  
$EndComp
Text Notes 7050 400  0    98   ~ 0
Sensors
Wire Wire Line
	10750 2150 10750 2200
$Comp
L power:GND #PWR0105
U 1 1 60892AC0
P 10750 2200
F 0 "#PWR0105" H 10750 1950 50  0001 C CNN
F 1 "GND" H 10755 2027 50  0000 C CNN
F 2 "" H 10750 2200 50  0001 C CNN
F 3 "" H 10750 2200 50  0001 C CNN
	1    10750 2200
	1    0    0    -1  
$EndComp
Connection ~ 10750 1750
Wire Wire Line
	10750 1750 10900 1750
Wire Wire Line
	10600 1750 10750 1750
Wire Wire Line
	10750 1750 10750 1950
Wire Wire Line
	10200 1650 10200 1750
Wire Wire Line
	10200 1300 10200 1350
$Comp
L Device:R R2
U 1 1 6083BE0A
P 10200 1500
F 0 "R2" H 10270 1546 50  0000 L CNN
F 1 "1k" V 10200 1450 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10130 1500 50  0001 C CNN
F 3 "~" H 10200 1500 50  0001 C CNN
	1    10200 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	10100 1750 10100 1800
Wire Wire Line
	10300 1750 10200 1750
$Comp
L Device:C_Small C1
U 1 1 6082364C
P 10750 2050
F 0 "C1" H 10600 1950 50  0000 C CNN
F 1 "1uF" H 10550 2100 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D7.5mm_W5.0mm_P10.00mm" H 10750 2050 50  0001 C CNN
F 3 "~" H 10750 2050 50  0001 C CNN
	1    10750 2050
	-1   0    0    1   
$EndComp
$Comp
L Device:R R4
U 1 1 608210BA
P 10450 1750
F 0 "R4" V 10350 1750 50  0000 C CNN
F 1 "10k" V 10450 1750 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10380 1750 50  0001 C CNN
F 3 "~" H 10450 1750 50  0001 C CNN
	1    10450 1750
	0    1    1    0   
$EndComp
Wire Wire Line
	10200 1000 10200 950 
Wire Wire Line
	10100 2200 10100 2100
$Comp
L power:GND #PWR010
U 1 1 60811C0F
P 10100 2200
F 0 "#PWR010" H 10100 1950 50  0001 C CNN
F 1 "GND" H 10105 2027 50  0000 C CNN
F 2 "" H 10100 2200 50  0001 C CNN
F 3 "" H 10100 2200 50  0001 C CNN
	1    10100 2200
	1    0    0    -1  
$EndComp
$Comp
L power:+36V #PWR09
U 1 1 60811134
P 10200 950
F 0 "#PWR09" H 10200 800 50  0001 C CNN
F 1 "+36V" H 10215 1123 50  0000 C CNN
F 2 "" H 10200 950 50  0001 C CNN
F 3 "" H 10200 950 50  0001 C CNN
	1    10200 950 
	1    0    0    -1  
$EndComp
Text GLabel 10900 1750 2    50   Input ~ 0
Voltage_Sensor
$Comp
L Device:R R3
U 1 1 60804559
P 10100 1950
F 0 "R3" H 10170 1996 50  0000 L CNN
F 1 "1k" V 10100 1900 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10030 1950 50  0001 C CNN
F 3 "~" H 10100 1950 50  0001 C CNN
	1    10100 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 60803003
P 10200 1150
F 0 "R1" H 10270 1196 50  0000 L CNN
F 1 "12k" V 10200 1050 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10130 1150 50  0001 C CNN
F 3 "~" H 10200 1150 50  0001 C CNN
	1    10200 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 1100 8050 1100
Wire Wire Line
	7850 1150 7850 1100
Wire Wire Line
	7850 1500 7850 1450
Wire Wire Line
	7850 2450 7850 2650
Connection ~ 7850 2450
Wire Wire Line
	7850 2450 7900 2450
Wire Wire Line
	7750 2450 7850 2450
Wire Wire Line
	7750 2250 7900 2250
Wire Wire Line
	7200 2500 7200 2450
Connection ~ 8900 2350
Wire Wire Line
	9000 2350 8900 2350
Wire Wire Line
	8900 2350 8800 2350
Wire Wire Line
	8900 2650 8900 2350
Wire Wire Line
	7850 2650 8900 2650
$Comp
L Device:Opamp_Dual_Generic U2
U 2 1 60CF35EA
P 8200 2350
F 0 "U2" H 8200 2717 50  0000 C CNN
F 1 "Opamp_Dual_Generic" H 8200 2626 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm_LongPads" H 8200 2350 50  0001 C CNN
F 3 "~" H 8200 2350 50  0001 C CNN
	2    8200 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:Opamp_Dual_Generic U2
U 1 1 60CF1C51
P 8400 1000
F 0 "U2" H 8400 1367 50  0000 C CNN
F 1 "Opamp_Dual_Generic" H 8400 1276 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm_LongPads" H 8400 1000 50  0001 C CNN
F 3 "~" H 8400 1000 50  0001 C CNN
	1    8400 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 2350 9300 2350
$Comp
L Device:R R27
U 1 1 6140BDEB
P 9150 2350
F 0 "R27" V 9050 2350 50  0000 C CNN
F 1 "2.5k" V 9150 2350 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9080 2350 50  0001 C CNN
F 3 "~" H 9150 2350 50  0001 C CNN
	1    9150 2350
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R26
U 1 1 613AAA2C
P 9050 1000
F 0 "R26" V 8950 1000 50  0000 C CNN
F 1 "2.5k" V 9050 1000 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8980 1000 50  0001 C CNN
F 3 "~" H 9050 1000 50  0001 C CNN
	1    9050 1000
	0    -1   -1   0   
$EndComp
Text GLabel 9250 1000 2    50   Input ~ 0
Current_Sensor_IN
Wire Wire Line
	7450 2450 7200 2450
$Comp
L power:GND #PWR0133
U 1 1 60E305C9
P 7200 2500
F 0 "#PWR0133" H 7200 2250 50  0001 C CNN
F 1 "GND" H 7205 2327 50  0000 C CNN
F 2 "" H 7200 2500 50  0001 C CNN
F 3 "" H 7200 2500 50  0001 C CNN
	1    7200 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 2250 7450 2250
Text GLabel 7300 2250 0    50   Input ~ 0
LM35-RAW
$Comp
L power:GND #PWR0132
U 1 1 60D78945
P 7850 1500
F 0 "#PWR0132" H 7850 1250 50  0001 C CNN
F 1 "GND" H 7855 1327 50  0000 C CNN
F 2 "" H 7850 1500 50  0001 C CNN
F 3 "" H 7850 1500 50  0001 C CNN
	1    7850 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R24
U 1 1 60D0886E
P 8650 2350
F 0 "R24" V 8550 2350 50  0000 C CNN
F 1 "75k" V 8650 2350 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8580 2350 50  0001 C CNN
F 3 "~" H 8650 2350 50  0001 C CNN
	1    8650 2350
	0    1    1    0   
$EndComp
$Comp
L Device:R R22
U 1 1 60D07DB0
P 7600 2450
F 0 "R22" V 7500 2450 50  0000 C CNN
F 1 "15k" V 7600 2450 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7530 2450 50  0001 C CNN
F 3 "~" H 7600 2450 50  0001 C CNN
	1    7600 2450
	0    1    1    0   
$EndComp
$Comp
L Device:R R21
U 1 1 60D0679E
P 7600 2250
F 0 "R21" V 7500 2250 50  0000 C CNN
F 1 "15k" V 7600 2250 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7530 2250 50  0001 C CNN
F 3 "~" H 7600 2250 50  0001 C CNN
	1    7600 2250
	0    1    1    0   
$EndComp
$Comp
L Device:R R19
U 1 1 60D05754
P 7800 900
F 0 "R19" V 7700 900 50  0000 C CNN
F 1 "4.7k" V 7800 900 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7730 900 50  0001 C CNN
F 3 "~" H 7800 900 50  0001 C CNN
	1    7800 900 
	0    1    1    0   
$EndComp
$Comp
L Device:R R20
U 1 1 60D04B90
P 7850 1300
F 0 "R20" V 7750 1300 50  0000 C CNN
F 1 "15k" V 7850 1300 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7780 1300 50  0001 C CNN
F 3 "~" H 7850 1300 50  0001 C CNN
	1    7850 1300
	-1   0    0    1   
$EndComp
$Comp
L Device:R R23
U 1 1 60D03771
P 8800 1250
F 0 "R23" V 8700 1250 50  0000 C CNN
F 1 "150k" V 8800 1250 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8730 1250 50  0001 C CNN
F 3 "~" H 8800 1250 50  0001 C CNN
	1    8800 1250
	-1   0    0    1   
$EndComp
Wire Wire Line
	7400 1100 7400 900 
$Comp
L power:GND #PWR0104
U 1 1 6084D0EA
P 7400 1500
F 0 "#PWR0104" H 7400 1250 50  0001 C CNN
F 1 "GND" H 7405 1327 50  0000 C CNN
F 2 "" H 7400 1500 50  0001 C CNN
F 3 "" H 7400 1500 50  0001 C CNN
	1    7400 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C13
U 1 1 60885D9B
P 7400 1200
F 0 "C13" V 7171 1200 50  0000 C CNN
F 1 "0.1uF" V 7262 1200 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D7.5mm_W5.0mm_P10.00mm" H 7400 1200 50  0001 C CNN
F 3 "~" H 7400 1200 50  0001 C CNN
	1    7400 1200
	-1   0    0    1   
$EndComp
$Comp
L Device:R R16
U 1 1 608850A5
P 7150 900
F 0 "R16" V 7050 900 50  0000 C CNN
F 1 "10k" V 7150 900 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7080 900 50  0001 C CNN
F 3 "~" H 7150 900 50  0001 C CNN
	1    7150 900 
	0    1    1    0   
$EndComp
Text GLabel 6850 550  2    50   Input ~ 0
Current_Sensor
$Comp
L power:GND #PWR040
U 1 1 607F52F7
P 6850 1500
F 0 "#PWR040" H 6850 1250 50  0001 C CNN
F 1 "GND" H 6855 1327 50  0000 C CNN
F 2 "" H 6850 1500 50  0001 C CNN
F 3 "" H 6850 1500 50  0001 C CNN
	1    6850 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R11
U 1 1 607F2CED
P 6850 1150
F 0 "R11" H 6920 1196 50  0000 L CNN
F 1 "0.1" V 6850 1100 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 6780 1150 50  0001 C CNN
F 3 "~" H 6850 1150 50  0001 C CNN
	1    6850 1150
	1    0    0    -1  
$EndComp
Text GLabel 9500 2350 2    50   Input ~ 0
LM35
Wire Notes Line
	150  150  6550 150 
Wire Wire Line
	800  1200 800  1400
Wire Wire Line
	800  1400 550  1400
$Comp
L Connector:Conn_01x02_Male J1
U 1 1 6069C405
P 350 1400
F 0 "J1" H 458 1581 50  0000 C CNN
F 1 "Battery Supply" H 458 1490 50  0000 C CNN
F 2 "Connector_Phoenix_GMSTB:PhoenixContact_GMSTBA_2,5_2-G_1x02_P7.50mm_Horizontal" H 350 1400 50  0001 C CNN
F 3 "~" H 350 1400 50  0001 C CNN
	1    350  1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	550  1500 1200 1500
$Comp
L Connector:Conn_01x03_Male J9
U 1 1 6069B167
P 9050 6750
F 0 "J9" H 9158 7031 50  0000 C CNN
F 1 "BLDC Motor" H 9158 6940 50  0000 C CNN
F 2 "Connector_Phoenix_GMSTB:PhoenixContact_GMSTBA_2,5_3-G-7,62_1x03_P7.62mm_Horizontal" H 9050 6750 50  0001 C CNN
F 3 "~" H 9050 6750 50  0001 C CNN
	1    9050 6750
	1    0    0    -1  
$EndComp
Text GLabel 9350 6650 2    50   Input ~ 0
PH1
Text GLabel 9350 6750 2    50   Input ~ 0
PH2
Text GLabel 9350 6850 2    50   Input ~ 0
PH3
Wire Wire Line
	9250 6650 9350 6650
Wire Wire Line
	9250 6750 9350 6750
Wire Wire Line
	9250 6850 9350 6850
Wire Wire Line
	7950 7150 7950 7100
Connection ~ 7950 7100
Wire Wire Line
	8250 7100 7950 7100
Wire Wire Line
	8250 7050 8250 7100
Wire Wire Line
	7950 7100 7950 7050
Wire Wire Line
	7650 7100 7950 7100
Wire Wire Line
	7650 7050 7650 7100
$Comp
L power:GND #PWR0106
U 1 1 60A7EDB4
P 7950 7150
F 0 "#PWR0106" H 7950 6900 50  0001 C CNN
F 1 "GND" H 7955 6977 50  0000 C CNN
F 2 "" H 7950 7150 50  0001 C CNN
F 3 "" H 7950 7150 50  0001 C CNN
	1    7950 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 6700 8400 6700
Wire Wire Line
	7950 6600 8400 6600
Wire Wire Line
	8400 6500 7650 6500
Wire Wire Line
	8250 6450 8250 6700
Wire Wire Line
	7950 6450 7950 6600
Connection ~ 7650 6500
Wire Wire Line
	7650 6450 7650 6500
Connection ~ 8250 6700
Wire Wire Line
	8250 6750 8250 6700
Wire Wire Line
	7200 6700 8250 6700
Connection ~ 7950 6600
Wire Wire Line
	7950 6750 7950 6600
Wire Wire Line
	7200 6600 7950 6600
Wire Wire Line
	7650 6500 7650 6750
Wire Wire Line
	7950 6100 7950 6050
Wire Wire Line
	7950 6100 8250 6100
Connection ~ 7950 6100
Wire Wire Line
	7950 6150 7950 6100
Wire Wire Line
	7650 6100 7950 6100
Wire Wire Line
	8250 6100 8250 6150
Wire Wire Line
	7650 6150 7650 6100
$Comp
L Device:C C9
U 1 1 609A4DB4
P 8250 6900
F 0 "C9" H 8365 6946 50  0000 L CNN
F 1 "0.1uF" H 8300 6800 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.7mm_W2.5mm_P5.00mm" H 8288 6750 50  0001 C CNN
F 3 "~" H 8250 6900 50  0001 C CNN
	1    8250 6900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 609A47BD
P 7950 6900
F 0 "C8" H 8065 6946 50  0000 L CNN
F 1 "0.1uF" H 8000 6800 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.7mm_W2.5mm_P5.00mm" H 7988 6750 50  0001 C CNN
F 3 "~" H 7950 6900 50  0001 C CNN
	1    7950 6900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 609A2D7A
P 7650 6900
F 0 "C7" H 7765 6946 50  0000 L CNN
F 1 "0.1uF" H 7650 6800 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.7mm_W2.5mm_P5.00mm" H 7688 6750 50  0001 C CNN
F 3 "~" H 7650 6900 50  0001 C CNN
	1    7650 6900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 6500 7650 6500
$Comp
L Device:R R15
U 1 1 60965F59
P 8250 6300
F 0 "R15" H 8320 6346 50  0000 L CNN
F 1 "1.5k" V 8250 6200 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8180 6300 50  0001 C CNN
F 3 "~" H 8250 6300 50  0001 C CNN
	1    8250 6300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R14
U 1 1 6095C5B7
P 7950 6300
F 0 "R14" H 8020 6346 50  0000 L CNN
F 1 "1.5k" V 7950 6200 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7880 6300 50  0001 C CNN
F 3 "~" H 7950 6300 50  0001 C CNN
	1    7950 6300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R13
U 1 1 6095A9F2
P 7650 6300
F 0 "R13" H 7720 6346 50  0000 L CNN
F 1 "1.5k" V 7650 6200 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 7580 6300 50  0001 C CNN
F 3 "~" H 7650 6300 50  0001 C CNN
	1    7650 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 6800 7450 6800
Wire Wire Line
	7200 6400 7450 6400
$Comp
L power:+5V #PWR021
U 1 1 6086B325
P 7450 6400
F 0 "#PWR021" H 7450 6250 50  0001 C CNN
F 1 "+5V" H 7465 6573 50  0000 C CNN
F 2 "" H 7450 6400 50  0001 C CNN
F 3 "" H 7450 6400 50  0001 C CNN
	1    7450 6400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR022
U 1 1 6086A5AE
P 7450 6800
F 0 "#PWR022" H 7450 6550 50  0001 C CNN
F 1 "GND" H 7455 6627 50  0000 C CNN
F 2 "" H 7450 6800 50  0001 C CNN
F 3 "" H 7450 6800 50  0001 C CNN
	1    7450 6800
	1    0    0    -1  
$EndComp
Text GLabel 8400 6700 2    50   Input ~ 0
HALL-C
Text GLabel 8400 6600 2    50   Input ~ 0
HALL-B
Text GLabel 8400 6500 2    50   Input ~ 0
HALL-A
$Comp
L Connector:Conn_01x05_Male J7
U 1 1 60865592
P 7000 6600
F 0 "J7" H 7108 6981 50  0000 C CNN
F 1 "Hall Sensors" H 7108 6890 50  0000 C CNN
F 2 "Connector_Molex:Molex_KK-254_AE-6410-05A_1x05_P2.54mm_Vertical" H 7000 6600 50  0001 C CNN
F 3 "~" H 7000 6600 50  0001 C CNN
	1    7000 6600
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0107
U 1 1 60EDA81B
P 7950 6050
F 0 "#PWR0107" H 7950 5900 50  0001 C CNN
F 1 "+5V" H 7965 6223 50  0000 C CNN
F 2 "" H 7950 6050 50  0001 C CNN
F 3 "" H 7950 6050 50  0001 C CNN
	1    7950 6050
	1    0    0    -1  
$EndComp
Text Notes 7050 5750 0    98   ~ 0
Connectors\n
$Comp
L Connector:Conn_01x03_Male J6
U 1 1 6084BBBC
P 7700 7800
F 0 "J6" H 7808 8081 50  0000 C CNN
F 1 "LM35" H 7808 7990 50  0000 C CNN
F 2 "Connector_Molex:Molex_KK-254_AE-6410-03A_1x03_P2.54mm_Vertical" H 7700 7800 50  0001 C CNN
F 3 "~" H 7700 7800 50  0001 C CNN
	1    7700 7800
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR017
U 1 1 6084CF1A
P 8150 7700
F 0 "#PWR017" H 8150 7550 50  0001 C CNN
F 1 "+5V" H 8165 7873 50  0000 C CNN
F 2 "" H 8150 7700 50  0001 C CNN
F 3 "" H 8150 7700 50  0001 C CNN
	1    8150 7700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 6084DB07
P 8150 7900
F 0 "#PWR018" H 8150 7650 50  0001 C CNN
F 1 "GND" H 8155 7727 50  0000 C CNN
F 2 "" H 8150 7900 50  0001 C CNN
F 3 "" H 8150 7900 50  0001 C CNN
	1    8150 7900
	1    0    0    -1  
$EndComp
Text GLabel 8150 7800 2    50   Input ~ 0
LM35-RAW
Wire Wire Line
	7900 7800 8150 7800
Wire Wire Line
	7900 7900 8150 7900
Wire Wire Line
	7900 7700 8150 7700
Text GLabel 9850 7800 0    50   Input ~ 0
Throttle_Raw
$Comp
L Device:R R29
U 1 1 6094A225
P 10200 7800
F 0 "R29" V 10100 7800 50  0000 C CNN
F 1 "2.5k" V 10200 7800 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10130 7800 50  0001 C CNN
F 3 "~" H 10200 7800 50  0001 C CNN
	1    10200 7800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	10050 7800 9850 7800
Wire Wire Line
	10550 7800 10350 7800
$Comp
L power:GND #PWR020
U 1 1 6068DD1E
P 10500 6000
F 0 "#PWR020" H 10500 5750 50  0001 C CNN
F 1 "GND" H 10505 5827 50  0000 C CNN
F 2 "" H 10500 6000 50  0001 C CNN
F 3 "" H 10500 6000 50  0001 C CNN
	1    10500 6000
	-1   0    0    1   
$EndComp
Text GLabel 11150 6450 2    50   Input ~ 0
Brake
$Comp
L Device:R R12
U 1 1 6097CC14
P 11050 6150
F 0 "R12" H 11120 6196 50  0000 L CNN
F 1 "1.5k" V 11050 6050 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10980 6150 50  0001 C CNN
F 3 "~" H 11050 6150 50  0001 C CNN
	1    11050 6150
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0111
U 1 1 6099D286
P 11050 5950
F 0 "#PWR0111" H 11050 5800 50  0001 C CNN
F 1 "+3V3" H 11065 6123 50  0000 C CNN
F 2 "" H 11050 5950 50  0001 C CNN
F 3 "" H 11050 5950 50  0001 C CNN
	1    11050 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	11050 6300 11050 6450
Wire Wire Line
	11050 6450 11150 6450
Connection ~ 11050 6450
$Comp
L power:GND #PWR0112
U 1 1 609BC705
P 10950 6650
F 0 "#PWR0112" H 10950 6400 50  0001 C CNN
F 1 "GND" H 10955 6477 50  0000 C CNN
F 2 "" H 10950 6650 50  0001 C CNN
F 3 "" H 10950 6650 50  0001 C CNN
	1    10950 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	11050 5950 11050 6000
Wire Wire Line
	10500 6150 10500 6000
$Comp
L power:+5V #PWR0113
U 1 1 60B56613
P 10850 6350
F 0 "#PWR0113" H 10850 6200 50  0001 C CNN
F 1 "+5V" H 10865 6523 50  0000 C CNN
F 2 "" H 10850 6350 50  0001 C CNN
F 3 "" H 10850 6350 50  0001 C CNN
	1    10850 6350
	1    0    0    -1  
$EndComp
Text GLabel 10250 7050 2    50   Input ~ 0
OPC_1
Wire Wire Line
	10150 7050 10250 7050
Wire Wire Line
	10150 7150 10250 7150
Wire Wire Line
	10150 6550 10300 6550
Wire Wire Line
	10150 6250 10250 6250
Wire Wire Line
	10150 6150 10500 6150
Text GLabel 10250 6750 2    50   Input ~ 0
RX-LCD
Text GLabel 10250 6850 2    50   Input ~ 0
TX-LCD
$Comp
L Connector:Conn_01x11_Male J2
U 1 1 61AA9310
P 9950 6650
F 0 "J2" H 10058 7331 50  0000 C CNN
F 1 "Conn_01x11_Male" H 10058 7240 50  0000 C CNN
F 2 "Connector_Molex:Molex_KK-254_AE-6410-11A_1x11_P2.54mm_Vertical" H 9950 6650 50  0001 C CNN
F 3 "~" H 9950 6650 50  0001 C CNN
	1    9950 6650
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR032
U 1 1 607AEA21
P 10700 6950
F 0 "#PWR032" H 10700 6800 50  0001 C CNN
F 1 "+5V" H 10715 7123 50  0000 C CNN
F 2 "" H 10700 6950 50  0001 C CNN
F 3 "" H 10700 6950 50  0001 C CNN
	1    10700 6950
	1    0    0    -1  
$EndComp
Wire Wire Line
	10150 6650 10300 6650
Wire Wire Line
	10300 6650 10300 6600
Wire Wire Line
	10150 6950 10700 6950
Wire Wire Line
	10150 6850 10250 6850
Wire Wire Line
	10150 6750 10250 6750
Wire Wire Line
	10950 6650 10950 6600
Wire Wire Line
	10950 6600 10300 6600
Connection ~ 10300 6600
Wire Wire Line
	10300 6600 10300 6550
$Comp
L power:GND #PWR0136
U 1 1 61EDC5E5
P 10250 7150
F 0 "#PWR0136" H 10250 6900 50  0001 C CNN
F 1 "GND" H 10255 6977 50  0000 C CNN
F 2 "" H 10250 7150 50  0001 C CNN
F 3 "" H 10250 7150 50  0001 C CNN
	1    10250 7150
	1    0    0    -1  
$EndComp
Text GLabel 10250 6250 2    50   Input ~ 0
Throttle_Raw
Wire Wire Line
	10150 6350 10850 6350
Wire Wire Line
	10150 6450 11050 6450
Wire Notes Line
	6700 8150 6700 5500
Wire Wire Line
	8450 3850 8250 3850
Wire Wire Line
	8250 4150 8450 4150
Text Notes 7000 3300 0    98   ~ 0
ESP32\n
Wire Wire Line
	8250 4050 8450 4050
Wire Wire Line
	8250 3950 8450 3950
$Comp
L power:+12V #PWR0102
U 1 1 613151FB
P 7850 5050
F 0 "#PWR0102" H 7850 4900 50  0001 C CNN
F 1 "+12V" H 7865 5223 50  0000 C CNN
F 2 "" H 7850 5050 50  0001 C CNN
F 3 "" H 7850 5050 50  0001 C CNN
	1    7850 5050
	1    0    0    -1  
$EndComp
NoConn ~ 10000 4650
Wire Wire Line
	7850 5050 7850 5100
Wire Wire Line
	8400 4950 8450 4950
Wire Wire Line
	8400 5100 8400 4950
Wire Wire Line
	7850 5100 8400 5100
$Comp
L controller_bldc-rescue:ESP32-DEVKITC-32D-ESP32-DEVKITC-32D U1
U 1 1 6081E288
P 9250 4250
F 0 "U1" H 9225 5417 50  0000 C CNN
F 1 "ESP32-DEVKITC-32D" H 9225 5326 50  0000 C CNN
F 2 "ESP32-DEVKITC-32D_Electeam:MODULE_ESP32-DEVKITC-32D" H 9200 4250 50  0001 L BNN
F 3 "" H 9250 4250 50  0001 L BNN
F 4 "4" H 9250 4250 50  0001 L BNN "PARTREV"
F 5 "Espressif Systems" H 9200 4150 50  0001 L BNN "MANUFACTURER"
	1    9250 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 4750 8450 4750
Wire Wire Line
	8450 4850 8200 4850
Wire Wire Line
	8250 4250 8450 4250
Wire Wire Line
	8250 4350 8450 4350
Wire Wire Line
	8250 4450 8450 4450
$Comp
L power:GND #PWR0114
U 1 1 60880F4C
P 8200 4850
F 0 "#PWR0114" H 8200 4600 50  0001 C CNN
F 1 "GND" H 8205 4677 50  0000 C CNN
F 2 "" H 8200 4850 50  0001 C CNN
F 3 "" H 8200 4850 50  0001 C CNN
	1    8200 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	10550 4950 10000 4950
Wire Wire Line
	10250 4850 10000 4850
Wire Wire Line
	10200 4050 10000 4050
Wire Wire Line
	10200 4250 10000 4250
Text GLabel 10200 4250 2    50   Input ~ 0
HIN3
Wire Wire Line
	10200 4350 10000 4350
Wire Wire Line
	10200 4750 10000 4750
Text GLabel 10200 4750 2    50   Input ~ 0
HIN1
Wire Wire Line
	10000 4450 10200 4450
Text GLabel 10200 4450 2    50   Input ~ 0
HIN2
Text GLabel 10200 4150 2    50   Input ~ 0
LIN3
Wire Wire Line
	10000 4150 10200 4150
$Comp
L power:+3.3V #PWR0103
U 1 1 60900CF9
P 10550 4950
F 0 "#PWR0103" H 10550 4800 50  0001 C CNN
F 1 "+3.3V" H 10565 5123 50  0000 C CNN
F 2 "" H 10550 4950 50  0001 C CNN
F 3 "" H 10550 4950 50  0001 C CNN
	1    10550 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 3750 10200 3750
Wire Wire Line
	10000 3850 10200 3850
Wire Wire Line
	10000 4550 10200 4550
Text GLabel 10200 4050 2    50   Input ~ 0
Brake
Text GLabel 8250 4050 0    50   Input ~ 0
LM35
Text GLabel 8250 4150 0    50   Input ~ 0
Voltage_Sensor
Text GLabel 8250 3950 0    50   Input ~ 0
Current_Sensor_IN
Text GLabel 8250 3850 0    50   Input ~ 0
Throttle
Text GLabel 10200 4350 2    50   Input ~ 0
LIN2
Text GLabel 10200 4550 2    50   Input ~ 0
LIN1
Text GLabel 8250 4450 0    50   Input ~ 0
HALL-C
Text GLabel 8250 4350 0    50   Input ~ 0
HALL-B
Text GLabel 8250 4250 0    50   Input ~ 0
HALL-A
Text GLabel 10200 3850 2    50   Input ~ 0
TX-LCD
Text GLabel 10200 3750 2    50   Input ~ 0
RX-LCD
$Comp
L power:GND #PWR016
U 1 1 606881A3
P 10250 4850
F 0 "#PWR016" H 10250 4600 50  0001 C CNN
F 1 "GND" H 10255 4677 50  0000 C CNN
F 2 "" H 10250 4850 50  0001 C CNN
F 3 "" H 10250 4850 50  0001 C CNN
	1    10250 4850
	1    0    0    -1  
$EndComp
Wire Notes Line
	150  8150 6550 8150
Wire Notes Line
	6550 4200 6550 8150
Wire Notes Line
	150  4200 150  8150
Wire Notes Line
	150  4200 6550 4200
Text Notes 450  4450 0    98   ~ 0
Commutation and Motor\n
Wire Notes Line
	6700 150  6700 2850
Wire Notes Line
	6550 4050 150  4050
Wire Notes Line
	150  150  150  4050
Wire Notes Line
	6550 150  6550 4050
Wire Notes Line
	11550 150  11550 2850
Wire Notes Line
	6700 150  11550 150 
Wire Notes Line
	6700 2850 11550 2850
Wire Notes Line
	11550 5500 11550 8150
Wire Notes Line
	6700 5500 11550 5500
Wire Notes Line
	6700 8150 11550 8150
Wire Notes Line
	11550 3000 11550 5350
Wire Notes Line
	6700 5350 6700 3000
Wire Notes Line
	6700 5350 11550 5350
Wire Notes Line
	6700 3000 11550 3000
Wire Wire Line
	7400 1300 7400 1500
Wire Wire Line
	6850 1300 6850 1500
Wire Wire Line
	6850 550  6850 900 
Connection ~ 10200 1750
Wire Wire Line
	10200 1750 10100 1750
Wire Wire Line
	8050 1100 8050 1450
Wire Wire Line
	8050 1450 8800 1450
Wire Wire Line
	8800 1450 8800 1400
Connection ~ 8050 1100
Wire Wire Line
	8050 1100 8100 1100
Wire Wire Line
	8800 1100 8800 1000
Wire Wire Line
	8800 1000 8700 1000
Wire Wire Line
	8800 1000 8900 1000
Connection ~ 8800 1000
Wire Wire Line
	9200 1000 9250 1000
Wire Wire Line
	7950 900  8100 900 
Wire Wire Line
	7650 900  7400 900 
Connection ~ 6850 900 
Wire Wire Line
	6850 900  6850 1000
Wire Wire Line
	7400 900  7300 900 
Connection ~ 7400 900 
Wire Wire Line
	6850 900  7000 900 
$Comp
L power:+5V #PWR0122
U 1 1 62B299D0
P 6150 2350
F 0 "#PWR0122" H 6150 2200 50  0001 C CNN
F 1 "+5V" H 6165 2523 50  0000 C CNN
F 2 "" H 6150 2350 50  0001 C CNN
F 3 "" H 6150 2350 50  0001 C CNN
	1    6150 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R28
U 1 1 62B2AA51
P 6150 2950
F 0 "R28" H 6220 2996 50  0000 L CNN
F 1 "220" V 6150 2850 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 6080 2950 50  0001 C CNN
F 3 "~" H 6150 2950 50  0001 C CNN
	1    6150 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 3050 2600 3500
Wire Wire Line
	1250 3050 1250 3450
Wire Wire Line
	3950 3050 3950 3500
$Comp
L power:GND #PWR0126
U 1 1 62B97047
P 6150 3450
F 0 "#PWR0126" H 6150 3200 50  0001 C CNN
F 1 "GND" H 6155 3277 50  0000 C CNN
F 2 "" H 6150 3450 50  0001 C CNN
F 3 "" H 6150 3450 50  0001 C CNN
	1    6150 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 3450 6150 3100
Wire Wire Line
	6150 2800 6150 2350
$EndSCHEMATC

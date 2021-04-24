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
L Device:Q_PMOS_GDS Q1
U 1 1 6067746C
P 10750 2300
F 0 "Q1" H 10954 2346 50  0000 L CNN
F 1 "IRF1010N" H 10954 2255 50  0000 L CNN
F 2 "Package_TO_SOT_THT_Electeam:TO-220-3_Horizontal_TabDown" H 10950 2400 50  0001 C CNN
F 3 "~" H 10750 2300 50  0001 C CNN
	1    10750 2300
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_PMOS_GDS Q2
U 1 1 6067945D
P 10750 2800
F 0 "Q2" H 10954 2846 50  0000 L CNN
F 1 "IRF1010N" H 10954 2755 50  0000 L CNN
F 2 "Package_TO_SOT_THT_Electeam:TO-220-3_Horizontal_TabUp" H 10950 2900 50  0001 C CNN
F 3 "~" H 10750 2800 50  0001 C CNN
	1    10750 2800
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_PMOS_GDS Q3
U 1 1 60679A4E
P 10700 4500
F 0 "Q3" H 10904 4546 50  0000 L CNN
F 1 "IRF1010N" H 10904 4455 50  0000 L CNN
F 2 "Package_TO_SOT_THT_Electeam:TO-220-3_Horizontal_TabDown" H 10900 4600 50  0001 C CNN
F 3 "~" H 10700 4500 50  0001 C CNN
	1    10700 4500
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_PMOS_GDS Q4
U 1 1 6067A63E
P 10700 5000
F 0 "Q4" H 10904 5046 50  0000 L CNN
F 1 "IRF1010N" H 10904 4955 50  0000 L CNN
F 2 "Package_TO_SOT_THT_Electeam:TO-220-3_Horizontal_TabUp" H 10900 5100 50  0001 C CNN
F 3 "~" H 10700 5000 50  0001 C CNN
	1    10700 5000
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_PMOS_GDS Q5
U 1 1 6067ADF0
P 10650 6750
F 0 "Q5" H 10854 6796 50  0000 L CNN
F 1 "IRF1010N" H 10854 6705 50  0000 L CNN
F 2 "Package_TO_SOT_THT_Electeam:TO-220-3_Horizontal_TabDown" H 10850 6850 50  0001 C CNN
F 3 "~" H 10650 6750 50  0001 C CNN
	1    10650 6750
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_PMOS_GDS Q6
U 1 1 6067B8DE
P 10650 7250
F 0 "Q6" H 10854 7296 50  0000 L CNN
F 1 "IRF1010N" H 10854 7205 50  0000 L CNN
F 2 "Package_TO_SOT_THT_Electeam:TO-220-3_Horizontal_TabUp" H 10850 7350 50  0001 C CNN
F 3 "~" H 10650 7250 50  0001 C CNN
	1    10650 7250
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LM317_TO-220 U2
U 1 1 6067E65E
P 1250 3250
F 0 "U2" H 1250 3492 50  0000 C CNN
F 1 "LM317_TO-220" H 1250 3401 50  0000 C CNN
F 2 "Package_TO_SOT_THT_Electeam:TO-220-3_Horizontal_TabDown" H 1250 3500 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/lm317.pdf" H 1250 3250 50  0001 C CNN
	1    1250 3250
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR012
U 1 1 60684086
P 1800 3250
F 0 "#PWR012" H 1800 3100 50  0001 C CNN
F 1 "+12V" H 1815 3423 50  0000 C CNN
F 2 "" H 1800 3250 50  0001 C CNN
F 3 "" H 1800 3250 50  0001 C CNN
	1    1800 3250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 6068663B
P 1250 3650
F 0 "#PWR05" H 1250 3400 50  0001 C CNN
F 1 "GND" H 1255 3477 50  0000 C CNN
F 2 "" H 1250 3650 50  0001 C CNN
F 3 "" H 1250 3650 50  0001 C CNN
	1    1250 3650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 6068DD1E
P 7950 900
F 0 "#PWR020" H 7950 650 50  0001 C CNN
F 1 "GND" H 7955 727 50  0000 C CNN
F 2 "" H 7950 900 50  0001 C CNN
F 3 "" H 7950 900 50  0001 C CNN
	1    7950 900 
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J8
U 1 1 606978CD
P 5150 550
F 0 "J8" H 5258 831 50  0000 C CNN
F 1 "Nextion Nx3224T028" H 5258 740 50  0000 C CNN
F 2 "Connector_Molex:Molex_KK-254_AE-6410-04A_1x04_P2.54mm_Vertical" H 5150 550 50  0001 C CNN
F 3 "~" H 5150 550 50  0001 C CNN
	1    5150 550 
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Male J9
U 1 1 6069B167
P 6550 650
F 0 "J9" H 6658 931 50  0000 C CNN
F 1 "BLDC Motor" H 6658 840 50  0000 C CNN
F 2 "Connector_Phoenix_GMSTB:PhoenixContact_GMSTBA_2,5_3-G-7,62_1x03_P7.62mm_Horizontal" H 6550 650 50  0001 C CNN
F 3 "~" H 6550 650 50  0001 C CNN
	1    6550 650 
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male J1
U 1 1 6069C405
P 350 500
F 0 "J1" H 458 681 50  0000 C CNN
F 1 "Battery Supply" H 458 590 50  0000 C CNN
F 2 "Connector_Phoenix_GMSTB:PhoenixContact_GMSTBA_2,5_2-G_1x02_P7.50mm_Horizontal" H 350 500 50  0001 C CNN
F 3 "~" H 350 500 50  0001 C CNN
	1    350  500 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 6069EAE5
P 1800 650
F 0 "#PWR08" H 1800 400 50  0001 C CNN
F 1 "GND" H 1805 477 50  0000 C CNN
F 2 "" H 1800 650 50  0001 C CNN
F 3 "" H 1800 650 50  0001 C CNN
	1    1800 650 
	1    0    0    -1  
$EndComp
$Comp
L power:+36V #PWR07
U 1 1 6069FCB5
P 1800 450
F 0 "#PWR07" H 1800 300 50  0001 C CNN
F 1 "+36V" H 1815 623 50  0000 C CNN
F 2 "" H 1800 450 50  0001 C CNN
F 3 "" H 1800 450 50  0001 C CNN
	1    1800 450 
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male J4
U 1 1 606A121C
P 900 6750
F 0 "J4" H 1008 6931 50  0000 C CNN
F 1 "Brake" H 1008 6840 50  0000 C CNN
F 2 "Connector_Molex:Molex_KK-254_AE-6410-02A_1x02_P2.54mm_Vertical" H 900 6750 50  0001 C CNN
F 3 "~" H 900 6750 50  0001 C CNN
	1    900  6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 4500 10500 4500
Wire Wire Line
	10350 2300 10550 2300
Wire Wire Line
	10350 2800 10550 2800
Wire Wire Line
	10850 2500 10850 2550
Wire Wire Line
	10250 5000 10500 5000
Wire Wire Line
	10200 6750 10450 6750
Wire Wire Line
	10200 7250 10450 7250
Wire Wire Line
	750  3250 950  3250
Wire Wire Line
	1250 3550 1250 3650
Wire Wire Line
	8450 6700 8000 6700
Wire Wire Line
	8500 4450 8050 4450
Wire Wire Line
	8600 2300 8150 2300
$Comp
L power:GND #PWR026
U 1 1 606BCA6C
P 7850 3200
F 0 "#PWR026" H 7850 2950 50  0001 C CNN
F 1 "GND" H 7855 3027 50  0000 C CNN
F 2 "" H 7850 3200 50  0001 C CNN
F 3 "" H 7850 3200 50  0001 C CNN
	1    7850 3200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR028
U 1 1 606C580A
P 7700 7600
F 0 "#PWR028" H 7700 7350 50  0001 C CNN
F 1 "GND" H 7705 7427 50  0000 C CNN
F 2 "" H 7700 7600 50  0001 C CNN
F 3 "" H 7700 7600 50  0001 C CNN
	1    7700 7600
	1    0    0    -1  
$EndComp
Wire Wire Line
	10800 4700 10800 4750
Wire Wire Line
	10750 6950 10750 7000
Wire Wire Line
	8000 7200 8050 7200
Wire Wire Line
	8250 2400 8150 2400
Wire Wire Line
	10050 2300 9900 2300
Wire Wire Line
	8250 2900 8150 2900
Wire Wire Line
	10050 2800 9900 2800
Wire Wire Line
	8150 4550 8050 4550
Wire Wire Line
	8150 5050 8050 5050
Wire Wire Line
	9850 4500 9950 4500
Wire Wire Line
	9850 5000 9950 5000
Wire Wire Line
	8000 6800 8100 6800
Wire Wire Line
	8000 7300 8100 7300
Wire Wire Line
	9800 6750 9900 6750
Wire Wire Line
	9800 7250 9900 7250
Wire Wire Line
	10850 2000 10850 2100
Wire Wire Line
	10850 3000 10850 3100
Wire Wire Line
	10800 4150 10800 4300
Wire Wire Line
	10800 5200 10800 5300
Wire Wire Line
	10750 6450 10750 6550
Wire Wire Line
	10750 7450 10750 7550
Wire Wire Line
	8150 4950 8100 4950
Text GLabel 6850 550  2    50   Input ~ 0
PH1
Text GLabel 6850 650  2    50   Input ~ 0
PH2
Text GLabel 6850 750  2    50   Input ~ 0
PH3
Wire Wire Line
	6750 550  6850 550 
Wire Wire Line
	6750 650  6850 650 
Wire Wire Line
	6750 750  6850 750 
Text GLabel 11000 2550 2    50   Input ~ 0
PH1
Text GLabel 10950 4750 2    50   Input ~ 0
PH2
Text GLabel 10900 7000 2    50   Input ~ 0
PH3
Wire Wire Line
	11000 2550 10850 2550
Connection ~ 10850 2550
Wire Wire Line
	10850 2550 10850 2600
Wire Wire Line
	10950 4750 10800 4750
Connection ~ 10800 4750
Wire Wire Line
	10800 4750 10800 4800
Wire Wire Line
	10900 7000 10750 7000
Connection ~ 10750 7000
Wire Wire Line
	10750 7000 10750 7050
Wire Wire Line
	7350 4750 7450 4750
Wire Wire Line
	7350 4850 7450 4850
Wire Wire Line
	7450 2600 7550 2600
Wire Wire Line
	7450 2700 7550 2700
Wire Wire Line
	7300 7000 7400 7000
Wire Wire Line
	7300 7100 7400 7100
$Comp
L power:+5V #PWR032
U 1 1 607AEA21
P 5750 450
F 0 "#PWR032" H 5750 300 50  0001 C CNN
F 1 "+5V" H 5765 623 50  0000 C CNN
F 2 "" H 5750 450 50  0001 C CNN
F 3 "" H 5750 450 50  0001 C CNN
	1    5750 450 
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 450  5750 450 
Text GLabel 5550 550  2    50   Input ~ 0
TX-LCD
Text GLabel 5550 650  2    50   Input ~ 0
RX-LCD
$Comp
L power:GND #PWR033
U 1 1 607B3EF3
P 5750 750
F 0 "#PWR033" H 5750 500 50  0001 C CNN
F 1 "GND" H 5755 577 50  0000 C CNN
F 2 "" H 5750 750 50  0001 C CNN
F 3 "" H 5750 750 50  0001 C CNN
	1    5750 750 
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 750  5750 750 
Wire Wire Line
	5350 650  5550 650 
Wire Wire Line
	5350 550  5550 550 
$Comp
L Connector:Conn_01x02_Male J2
U 1 1 606A4893
P 750 1450
F 0 "J2" V 600 1400 50  0000 C CNN
F 1 "Emergency Button OUT" V 700 1400 50  0000 C CNN
F 2 "Connector_Phoenix_GMSTB:PhoenixContact_GMSTBA_2,5_2-G_1x02_P7.50mm_Horizontal" H 750 1450 50  0001 C CNN
F 3 "~" H 750 1450 50  0001 C CNN
	1    750  1450
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x02_Male J3
U 1 1 607D7C4D
P 1700 1450
F 0 "J3" V 1550 1400 50  0000 C CNN
F 1 "Emergency Button IN" V 1650 1400 50  0000 C CNN
F 2 "Connector_Phoenix_GMSTB:PhoenixContact_GMSTBA_2,5_2-G_1x02_P7.50mm_Horizontal" H 1700 1450 50  0001 C CNN
F 3 "~" H 1700 1450 50  0001 C CNN
	1    1700 1450
	0    1    1    0   
$EndComp
$Comp
L power:+36V #PWR03
U 1 1 607E0827
P 650 1800
F 0 "#PWR03" H 650 1650 50  0001 C CNN
F 1 "+36V" H 665 1973 50  0000 C CNN
F 2 "" H 650 1800 50  0001 C CNN
F 3 "" H 650 1800 50  0001 C CNN
	1    650  1800
	-1   0    0    1   
$EndComp
Wire Wire Line
	650  1800 650  1650
Wire Wire Line
	750  1650 1600 1650
Text GLabel 2800 2900 2    50   Input ~ 0
36V-Power
Text GLabel 10800 5300 2    50   Input ~ 0
Current_Sensor
Text GLabel 10750 7550 2    50   Input ~ 0
Current_Sensor
$Comp
L Device:R R1
U 1 1 60803003
P 2850 6300
F 0 "R1" H 2920 6346 50  0000 L CNN
F 1 "12k" H 2920 6255 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2780 6300 50  0001 C CNN
F 3 "~" H 2850 6300 50  0001 C CNN
	1    2850 6300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 60804559
P 2850 7100
F 0 "R3" H 2920 7146 50  0000 L CNN
F 1 "1k" H 2920 7055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2780 7100 50  0001 C CNN
F 3 "~" H 2850 7100 50  0001 C CNN
	1    2850 7100
	1    0    0    -1  
$EndComp
Text GLabel 3650 6900 2    50   Input ~ 0
Voltage_Sensor
$Comp
L power:+36V #PWR09
U 1 1 60811134
P 2850 6100
F 0 "#PWR09" H 2850 5950 50  0001 C CNN
F 1 "+36V" H 2865 6273 50  0000 C CNN
F 2 "" H 2850 6100 50  0001 C CNN
F 3 "" H 2850 6100 50  0001 C CNN
	1    2850 6100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 60811C0F
P 2850 7350
F 0 "#PWR010" H 2850 7100 50  0001 C CNN
F 1 "GND" H 2855 7177 50  0000 C CNN
F 2 "" H 2850 7350 50  0001 C CNN
F 3 "" H 2850 7350 50  0001 C CNN
	1    2850 7350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 7350 2850 7250
Wire Wire Line
	2850 6150 2850 6100
$Comp
L Device:R R4
U 1 1 608210BA
P 3200 6900
F 0 "R4" V 2993 6900 50  0000 C CNN
F 1 "10k" V 3084 6900 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3130 6900 50  0001 C CNN
F 3 "~" H 3200 6900 50  0001 C CNN
	1    3200 6900
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C1
U 1 1 6082364C
P 3500 7200
F 0 "C1" H 3350 7100 50  0000 C CNN
F 1 "0.1uF" H 3300 7250 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D7.5mm_W5.0mm_P10.00mm" H 3500 7200 50  0001 C CNN
F 3 "~" H 3500 7200 50  0001 C CNN
	1    3500 7200
	-1   0    0    1   
$EndComp
Wire Wire Line
	3050 6900 2850 6900
Connection ~ 2850 6900
Wire Wire Line
	2850 6900 2850 6950
$Comp
L Device:R R2
U 1 1 6083BE0A
P 2850 6650
F 0 "R2" H 2920 6696 50  0000 L CNN
F 1 "1k" H 2920 6605 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2780 6650 50  0001 C CNN
F 3 "~" H 2850 6650 50  0001 C CNN
	1    2850 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 6450 2850 6500
Wire Wire Line
	2850 6800 2850 6900
$Comp
L Connector:Conn_01x03_Male J6
U 1 1 6084BBBC
P 3900 600
F 0 "J6" H 4008 881 50  0000 C CNN
F 1 "LM35" H 4008 790 50  0000 C CNN
F 2 "Connector_Molex:Molex_KK-254_AE-6410-03A_1x03_P2.54mm_Vertical" H 3900 600 50  0001 C CNN
F 3 "~" H 3900 600 50  0001 C CNN
	1    3900 600 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR017
U 1 1 6084CF1A
P 4350 500
F 0 "#PWR017" H 4350 350 50  0001 C CNN
F 1 "+5V" H 4365 673 50  0000 C CNN
F 2 "" H 4350 500 50  0001 C CNN
F 3 "" H 4350 500 50  0001 C CNN
	1    4350 500 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 6084DB07
P 4350 700
F 0 "#PWR018" H 4350 450 50  0001 C CNN
F 1 "GND" H 4355 527 50  0000 C CNN
F 2 "" H 4350 700 50  0001 C CNN
F 3 "" H 4350 700 50  0001 C CNN
	1    4350 700 
	1    0    0    -1  
$EndComp
Text GLabel 4350 600  2    50   Input ~ 0
LM35
Wire Wire Line
	4100 600  4350 600 
Wire Wire Line
	4100 700  4350 700 
Wire Wire Line
	4100 500  4350 500 
Text GLabel 10850 3100 2    50   Input ~ 0
Current_Sensor
$Comp
L power:GND #PWR016
U 1 1 606881A3
P 6150 4800
F 0 "#PWR016" H 6150 4550 50  0001 C CNN
F 1 "GND" H 6155 4627 50  0000 C CNN
F 2 "" H 6150 4800 50  0001 C CNN
F 3 "" H 6150 4800 50  0001 C CNN
	1    6150 4800
	1    0    0    -1  
$EndComp
Text GLabel 7700 650  0    50   Input ~ 0
Throttle
$Comp
L pspice:DIODE D1
U 1 1 6066AC3A
P 8600 2000
F 0 "D1" V 8646 1872 50  0000 R CNN
F 1 "1N4004" V 8555 1872 50  0000 R CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 8600 2000 50  0001 C CNN
F 3 "~" H 8600 2000 50  0001 C CNN
	1    8600 2000
	0    1    1    0   
$EndComp
$Comp
L pspice:DIODE D2
U 1 1 6066F442
P 8500 4150
F 0 "D2" V 8546 4022 50  0000 R CNN
F 1 "1N4004" V 8455 4022 50  0000 R CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 8500 4150 50  0001 C CNN
F 3 "~" H 8500 4150 50  0001 C CNN
	1    8500 4150
	0    1    1    0   
$EndComp
$Comp
L pspice:DIODE D3
U 1 1 6066FEAB
P 8450 6400
F 0 "D3" V 8496 6272 50  0000 R CNN
F 1 "1N4004" V 8405 6272 50  0000 R CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 8450 6400 50  0001 C CNN
F 3 "~" H 8450 6400 50  0001 C CNN
	1    8450 6400
	0    1    1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 606765CA
P 10050 7250
F 0 "R8" V 9843 7250 50  0000 C CNN
F 1 "100" V 9934 7250 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9980 7250 50  0001 C CNN
F 3 "~" H 10050 7250 50  0001 C CNN
	1    10050 7250
	0    1    1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 6067585C
P 10050 6750
F 0 "R7" V 9843 6750 50  0000 C CNN
F 1 "100" V 9934 6750 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9980 6750 50  0001 C CNN
F 3 "~" H 10050 6750 50  0001 C CNN
	1    10050 6750
	0    1    1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 6067511F
P 10100 5000
F 0 "R6" V 9893 5000 50  0000 C CNN
F 1 "100" V 9984 5000 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10030 5000 50  0001 C CNN
F 3 "~" H 10100 5000 50  0001 C CNN
	1    10100 5000
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 606747BF
P 10100 4500
F 0 "R5" V 9893 4500 50  0000 C CNN
F 1 "100" V 9984 4500 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10030 4500 50  0001 C CNN
F 3 "~" H 10100 4500 50  0001 C CNN
	1    10100 4500
	0    1    1    0   
$EndComp
$Comp
L Device:R R10
U 1 1 60672386
P 10200 2800
F 0 "R10" V 9993 2800 50  0000 C CNN
F 1 "100" V 10084 2800 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10130 2800 50  0001 C CNN
F 3 "~" H 10200 2800 50  0001 C CNN
	1    10200 2800
	0    1    1    0   
$EndComp
$Comp
L Device:R R9
U 1 1 60670FD6
P 10200 2300
F 0 "R9" V 9993 2300 50  0000 C CNN
F 1 "100" V 10084 2300 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10130 2300 50  0001 C CNN
F 3 "~" H 10200 2300 50  0001 C CNN
	1    10200 2300
	0    1    1    0   
$EndComp
Wire Wire Line
	9050 7150 9050 7050
$Comp
L Device:CP C4
U 1 1 6066A248
P 9050 6900
F 0 "C4" H 9168 6946 50  0000 L CNN
F 1 "2.2uF" H 9168 6855 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 9088 6750 50  0001 C CNN
F 3 "~" H 9050 6900 50  0001 C CNN
	1    9050 6900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 6750 9050 6700
Connection ~ 8450 6700
Wire Wire Line
	9050 6700 8450 6700
Text GLabel 9800 7250 0    50   Input ~ 0
LO3
Text GLabel 9800 6750 0    50   Input ~ 0
HO3
Text GLabel 9050 7150 2    50   Input ~ 0
VS3
Text GLabel 8100 7300 2    50   Input ~ 0
LO3
Text GLabel 8100 7200 2    50   Input ~ 0
VS3
Text GLabel 8100 6800 2    50   Input ~ 0
HO3
Text GLabel 7300 7100 0    50   Input ~ 0
LIN3
Text GLabel 7300 7000 0    50   Input ~ 0
HIN3
Connection ~ 8500 4450
Wire Wire Line
	9100 4450 8500 4450
Wire Wire Line
	9100 4550 9100 4450
Wire Wire Line
	9100 4950 9100 4850
$Comp
L Device:CP C3
U 1 1 60669B72
P 9100 4700
F 0 "C3" H 9218 4746 50  0000 L CNN
F 1 "2.2uF" H 9218 4655 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 9138 4550 50  0001 C CNN
F 3 "~" H 9100 4700 50  0001 C CNN
	1    9100 4700
	1    0    0    -1  
$EndComp
Text GLabel 9850 5000 0    50   Input ~ 0
LO2
Text GLabel 9850 4500 0    50   Input ~ 0
HO2
Text GLabel 9100 4950 2    50   Input ~ 0
VS2
Text GLabel 8150 5050 2    50   Input ~ 0
LO2
Text GLabel 8150 4950 2    50   Input ~ 0
VS2
Text GLabel 8150 4550 2    50   Input ~ 0
HO2
Text GLabel 7350 4850 0    50   Input ~ 0
LIN2
Text GLabel 7350 4750 0    50   Input ~ 0
HIN2
Connection ~ 8600 2300
Wire Wire Line
	9200 2300 8600 2300
Wire Wire Line
	9200 2350 9200 2300
Wire Wire Line
	9200 2650 9200 2750
$Comp
L Device:CP C2
U 1 1 606683E1
P 9200 2500
F 0 "C2" H 9318 2546 50  0000 L CNN
F 1 "2.2uF" H 9318 2455 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 9238 2350 50  0001 C CNN
F 3 "~" H 9200 2500 50  0001 C CNN
	1    9200 2500
	1    0    0    -1  
$EndComp
Text GLabel 9200 2750 2    50   Input ~ 0
VS1
Text GLabel 9900 2800 0    50   Input ~ 0
LO1
Text GLabel 9900 2300 0    50   Input ~ 0
HO1
Text GLabel 8250 2900 2    50   Input ~ 0
LO1
Text GLabel 8250 2800 2    50   Input ~ 0
VS1
Text GLabel 8250 2400 2    50   Input ~ 0
HO1
Text GLabel 7450 2700 0    50   Input ~ 0
LIN1
Text GLabel 7450 2600 0    50   Input ~ 0
HIN1
Text GLabel 1500 6750 2    50   Input ~ 0
Brake
Text GLabel 6100 3700 2    50   Input ~ 0
RX-LCD
Text GLabel 6100 3800 2    50   Input ~ 0
TX-LCD
Text GLabel 4150 4200 0    50   Input ~ 0
HALL-A
Text GLabel 4150 4300 0    50   Input ~ 0
HALL-B
Text GLabel 4150 4400 0    50   Input ~ 0
HALL-C
Text GLabel 6100 4500 2    50   Input ~ 0
LIN1
Text GLabel 6100 4300 2    50   Input ~ 0
LIN2
Text GLabel 4150 4500 0    50   Input ~ 0
Throttle
Text GLabel 4150 4600 0    50   Input ~ 0
Current_Sensor_IN
Text GLabel 4150 4700 0    50   Input ~ 0
Voltage_Sensor
Text GLabel 6100 4000 2    50   Input ~ 0
LM35
Text GLabel 6100 3900 2    50   Input ~ 0
Brake
Text GLabel 8250 2700 2    50   Input ~ 0
PH1
Wire Wire Line
	8250 2800 8200 2800
Wire Wire Line
	8250 2700 8200 2700
Wire Wire Line
	8200 2700 8200 2800
Connection ~ 8200 2800
Wire Wire Line
	8200 2800 8150 2800
Text GLabel 8150 4850 2    50   Input ~ 0
PH2
Wire Wire Line
	8150 4850 8100 4850
Wire Wire Line
	8100 4850 8100 4950
Connection ~ 8100 4950
Wire Wire Line
	8100 4950 8050 4950
Text GLabel 8100 7100 2    50   Input ~ 0
PH3
Wire Wire Line
	8100 7100 8050 7100
Wire Wire Line
	8050 7100 8050 7200
Connection ~ 8050 7200
Wire Wire Line
	8050 7200 8100 7200
$Comp
L Driver_FET:IR2103 U8
U 1 1 60823C9D
P 7850 2600
F 0 "U8" H 7850 3281 50  0000 C CNN
F 1 "IR2103" H 7850 3190 50  0000 C CNN
F 2 "Package_SO:SO-8_3.9x4.9mm_P1.27mm" H 7850 2600 50  0001 C CIN
F 3 "https://www.infineon.com/dgdl/ir2103.pdf?fileId=5546d462533600a4015355c7b54b166f" H 7850 2600 50  0001 C CNN
	1    7850 2600
	1    0    0    -1  
$EndComp
$Comp
L Driver_FET:IR2103 U9
U 1 1 60825783
P 7750 4750
F 0 "U9" H 7750 5431 50  0000 C CNN
F 1 "IR2103" H 7750 5340 50  0000 C CNN
F 2 "Package_SO:SO-8_3.9x4.9mm_P1.27mm" H 7750 4750 50  0001 C CIN
F 3 "https://www.infineon.com/dgdl/ir2103.pdf?fileId=5546d462533600a4015355c7b54b166f" H 7750 4750 50  0001 C CNN
	1    7750 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:Fuse F1
U 1 1 6082709D
P 900 500
F 0 "F1" V 703 500 50  0000 C CNN
F 1 "Fuse" V 794 500 50  0000 C CNN
F 2 "Package_TO_SOT_THT_Electeam:Portafusible" V 830 500 50  0001 C CNN
F 3 "~" H 900 500 50  0001 C CNN
	1    900  500 
	0    1    1    0   
$EndComp
Wire Wire Line
	550  500  750  500 
$Comp
L Device:CP C5
U 1 1 60843E87
P 1700 3500
F 0 "C5" H 1818 3546 50  0000 L CNN
F 1 "33uF" H 1818 3455 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D8.0mm_P3.50mm" H 1738 3350 50  0001 C CNN
F 3 "~" H 1700 3500 50  0001 C CNN
	1    1700 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 608BF738
P 1700 3650
F 0 "#PWR0101" H 1700 3400 50  0001 C CNN
F 1 "GND" H 1705 3477 50  0000 C CNN
F 2 "" H 1700 3650 50  0001 C CNN
F 3 "" H 1700 3650 50  0001 C CNN
	1    1700 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 3250 1700 3250
Wire Wire Line
	1700 3350 1700 3250
Connection ~ 1700 3250
Wire Wire Line
	1700 3250 1800 3250
Wire Wire Line
	5900 4500 6100 4500
Wire Wire Line
	5900 3800 6100 3800
Wire Wire Line
	5900 3700 6100 3700
$Comp
L power:+3.3V #PWR0103
U 1 1 60900CF9
P 6450 4900
F 0 "#PWR0103" H 6450 4750 50  0001 C CNN
F 1 "+3.3V" H 6465 5073 50  0000 C CNN
F 2 "" H 6450 4900 50  0001 C CNN
F 3 "" H 6450 4900 50  0001 C CNN
	1    6450 4900
	1    0    0    -1  
$EndComp
$Comp
L power:+36V #PWR0105
U 1 1 6092CE52
P 750 3250
F 0 "#PWR0105" H 750 3100 50  0001 C CNN
F 1 "+36V" H 765 3423 50  0000 C CNN
F 2 "" H 750 3250 50  0001 C CNN
F 3 "" H 750 3250 50  0001 C CNN
	1    750  3250
	1    0    0    -1  
$EndComp
$Comp
L Driver_FET:IR2103 U4
U 1 1 6092DBC3
P 7700 7000
F 0 "U4" H 7700 7681 50  0000 C CNN
F 1 "IR2103" H 7700 7590 50  0000 C CNN
F 2 "Package_SO:SO-8_3.9x4.9mm_P1.27mm" H 7700 7000 50  0001 C CIN
F 3 "https://www.infineon.com/dgdl/ir2103.pdf?fileId=5546d462533600a4015355c7b54b166f" H 7700 7000 50  0001 C CNN
	1    7700 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 1350 10000 1300
Connection ~ 10000 1300
Wire Wire Line
	10300 1300 10000 1300
Wire Wire Line
	10300 1250 10300 1300
Wire Wire Line
	10000 1300 10000 1250
Wire Wire Line
	9700 1300 10000 1300
Wire Wire Line
	9700 1250 9700 1300
$Comp
L power:GND #PWR0106
U 1 1 60A7EDB4
P 10000 1350
F 0 "#PWR0106" H 10000 1100 50  0001 C CNN
F 1 "GND" H 10005 1177 50  0000 C CNN
F 2 "" H 10000 1350 50  0001 C CNN
F 3 "" H 10000 1350 50  0001 C CNN
	1    10000 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 900  10450 900 
Wire Wire Line
	10000 800  10450 800 
Wire Wire Line
	10450 700  9700 700 
Wire Wire Line
	10300 650  10300 900 
Wire Wire Line
	10000 650  10000 800 
Connection ~ 9700 700 
Wire Wire Line
	9700 650  9700 700 
Connection ~ 10300 900 
Wire Wire Line
	10300 950  10300 900 
Wire Wire Line
	9250 900  10300 900 
Connection ~ 10000 800 
Wire Wire Line
	10000 950  10000 800 
Wire Wire Line
	9250 800  10000 800 
Wire Wire Line
	9700 700  9700 950 
Wire Wire Line
	10000 300  10000 250 
Wire Wire Line
	10000 300  10300 300 
Connection ~ 10000 300 
Wire Wire Line
	10000 350  10000 300 
Wire Wire Line
	9700 300  10000 300 
Wire Wire Line
	10300 300  10300 350 
Wire Wire Line
	9700 350  9700 300 
$Comp
L Device:C C9
U 1 1 609A4DB4
P 10300 1100
F 0 "C9" H 10415 1146 50  0000 L CNN
F 1 "0.1uF" H 10350 1000 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.7mm_W2.5mm_P5.00mm" H 10338 950 50  0001 C CNN
F 3 "~" H 10300 1100 50  0001 C CNN
	1    10300 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 609A47BD
P 10000 1100
F 0 "C8" H 10115 1146 50  0000 L CNN
F 1 "0.1uF" H 10050 1000 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.7mm_W2.5mm_P5.00mm" H 10038 950 50  0001 C CNN
F 3 "~" H 10000 1100 50  0001 C CNN
	1    10000 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 609A2D7A
P 9700 1100
F 0 "C7" H 9815 1146 50  0000 L CNN
F 1 "0.1uF" H 9700 1000 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.7mm_W2.5mm_P5.00mm" H 9738 950 50  0001 C CNN
F 3 "~" H 9700 1100 50  0001 C CNN
	1    9700 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 700  9700 700 
$Comp
L Device:R R15
U 1 1 60965F59
P 10300 500
F 0 "R15" H 10370 546 50  0000 L CNN
F 1 "1.5k" H 10370 455 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10230 500 50  0001 C CNN
F 3 "~" H 10300 500 50  0001 C CNN
	1    10300 500 
	1    0    0    -1  
$EndComp
$Comp
L Device:R R14
U 1 1 6095C5B7
P 10000 500
F 0 "R14" H 10070 546 50  0000 L CNN
F 1 "1.5k" H 10070 455 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9930 500 50  0001 C CNN
F 3 "~" H 10000 500 50  0001 C CNN
	1    10000 500 
	1    0    0    -1  
$EndComp
$Comp
L Device:R R13
U 1 1 6095A9F2
P 9700 500
F 0 "R13" H 9770 546 50  0000 L CNN
F 1 "1.5k" H 9770 455 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9630 500 50  0001 C CNN
F 3 "~" H 9700 500 50  0001 C CNN
	1    9700 500 
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 1000 9500 1000
Wire Wire Line
	9250 600  9500 600 
$Comp
L power:+5V #PWR021
U 1 1 6086B325
P 9500 600
F 0 "#PWR021" H 9500 450 50  0001 C CNN
F 1 "+5V" H 9515 773 50  0000 C CNN
F 2 "" H 9500 600 50  0001 C CNN
F 3 "" H 9500 600 50  0001 C CNN
	1    9500 600 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR022
U 1 1 6086A5AE
P 9500 1000
F 0 "#PWR022" H 9500 750 50  0001 C CNN
F 1 "GND" H 9505 827 50  0000 C CNN
F 2 "" H 9500 1000 50  0001 C CNN
F 3 "" H 9500 1000 50  0001 C CNN
	1    9500 1000
	1    0    0    -1  
$EndComp
Text GLabel 10450 900  2    50   Input ~ 0
HALL-C
Text GLabel 10450 800  2    50   Input ~ 0
HALL-B
Text GLabel 10450 700  2    50   Input ~ 0
HALL-A
$Comp
L Connector:Conn_01x05_Male J7
U 1 1 60865592
P 9050 800
F 0 "J7" H 9158 1181 50  0000 C CNN
F 1 "Hall Sensors" H 9158 1090 50  0000 C CNN
F 2 "Connector_Molex:Molex_KK-254_AE-6410-05A_1x05_P2.54mm_Vertical" H 9050 800 50  0001 C CNN
F 3 "~" H 9050 800 50  0001 C CNN
	1    9050 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 1800 7850 2100
Wire Wire Line
	8600 1800 7850 1800
Wire Wire Line
	8600 2300 8600 2200
Wire Wire Line
	7750 3950 7750 4250
Wire Wire Line
	7750 3950 8500 3950
Wire Wire Line
	8500 4350 8500 4450
Wire Wire Line
	7850 3100 7850 3200
$Comp
L power:GND #PWR0108
U 1 1 6088A078
P 7750 5350
F 0 "#PWR0108" H 7750 5100 50  0001 C CNN
F 1 "GND" H 7755 5177 50  0000 C CNN
F 2 "" H 7750 5350 50  0001 C CNN
F 3 "" H 7750 5350 50  0001 C CNN
	1    7750 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 5250 7750 5350
Wire Wire Line
	7700 7500 7700 7600
Wire Wire Line
	8450 6200 7700 6200
Connection ~ 7700 6200
Wire Wire Line
	7700 6200 7700 6500
Wire Wire Line
	8450 6700 8450 6600
$Comp
L Relay:ADW11 K1
U 1 1 608F8601
P 2550 2400
F 0 "K1" H 2980 2446 50  0000 L CNN
F 1 "ADW11" H 2980 2355 50  0000 L CNN
F 2 "Relay_THT:Relay_1P1T_NO_10x24x18.8mm_Panasonic_ADW11xxxxW_THT" H 3875 2350 50  0001 C CNN
F 3 "https://www.panasonic-electric-works.com/pew/es/downloads/ds_dw_hl_en.pdf" H 2550 2400 50  0001 C CNN
	1    2550 2400
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_SPST SW1
U 1 1 60902E70
P 1300 500
F 0 "SW1" H 1300 735 50  0000 C CNN
F 1 "ON_OFF" H 1300 644 50  0000 C CNN
F 2 "Connector_Molex:Molex_KK-254_AE-6410-02A_1x02_P2.54mm_Vertical" H 1300 500 50  0001 C CNN
F 3 "~" H 1300 500 50  0001 C CNN
	1    1300 500 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 500  1050 500 
Wire Wire Line
	1700 1650 2850 1650
$Comp
L Switch:SW_Push SW2
U 1 1 6094A449
P 2050 2000
F 0 "SW2" H 2050 2285 50  0000 C CNN
F 1 "OPC" H 2050 2194 50  0000 C CNN
F 2 "Connector_Molex:Molex_KK-254_AE-6410-02A_1x02_P2.54mm_Vertical" H 2050 2200 50  0001 C CNN
F 3 "~" H 2050 2200 50  0001 C CNN
	1    2050 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 2000 2350 2000
Wire Wire Line
	2350 2000 2350 2100
Wire Wire Line
	2850 1650 2850 2100
$Comp
L power:GND #PWR0109
U 1 1 60967BAE
P 2350 2850
F 0 "#PWR0109" H 2350 2600 50  0001 C CNN
F 1 "GND" H 2355 2677 50  0000 C CNN
F 2 "" H 2350 2850 50  0001 C CNN
F 3 "" H 2350 2850 50  0001 C CNN
	1    2350 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 2850 2350 2700
Wire Wire Line
	1700 1950 1700 2000
Wire Wire Line
	1700 2000 1850 2000
$Comp
L Device:R R12
U 1 1 6097CC14
P 1400 6450
F 0 "R12" H 1470 6496 50  0000 L CNN
F 1 "R" H 1470 6405 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1330 6450 50  0001 C CNN
F 3 "~" H 1400 6450 50  0001 C CNN
	1    1400 6450
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0111
U 1 1 6099D286
P 1400 6250
F 0 "#PWR0111" H 1400 6100 50  0001 C CNN
F 1 "+3V3" H 1415 6423 50  0000 C CNN
F 2 "" H 1400 6250 50  0001 C CNN
F 3 "" H 1400 6250 50  0001 C CNN
	1    1400 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 6600 1400 6750
Wire Wire Line
	1400 6750 1500 6750
Wire Wire Line
	1100 6750 1400 6750
Connection ~ 1400 6750
$Comp
L power:GND #PWR0112
U 1 1 609BC705
P 1400 6900
F 0 "#PWR0112" H 1400 6650 50  0001 C CNN
F 1 "GND" H 1405 6727 50  0000 C CNN
F 2 "" H 1400 6900 50  0001 C CNN
F 3 "" H 1400 6900 50  0001 C CNN
	1    1400 6900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 6850 1400 6850
Wire Wire Line
	1400 6850 1400 6900
Wire Wire Line
	1400 6250 1400 6300
Wire Wire Line
	1700 4650 1800 4650
Connection ~ 1700 4650
Wire Wire Line
	1700 4700 1700 4650
Wire Wire Line
	1700 5000 1700 5050
$Comp
L power:GND #PWR0104
U 1 1 6090D217
P 1700 5050
F 0 "#PWR0104" H 1700 4800 50  0001 C CNN
F 1 "GND" H 1705 4877 50  0000 C CNN
F 2 "" H 1700 5050 50  0001 C CNN
F 3 "" H 1700 5050 50  0001 C CNN
	1    1700 5050
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C6
U 1 1 6090BD63
P 1700 4850
F 0 "C6" H 1818 4896 50  0000 L CNN
F 1 "100uF" H 1818 4805 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 1738 4700 50  0001 C CNN
F 3 "~" H 1700 4850 50  0001 C CNN
	1    1700 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 5050 1250 4950
Wire Wire Line
	1550 4650 1700 4650
Wire Wire Line
	950  4650 750  4650
$Comp
L Regulator_Linear:LM7805_TO220 U3
U 1 1 60798537
P 1250 4650
F 0 "U3" H 1250 4892 50  0000 C CNN
F 1 "LM7805_TO220" H 1250 4801 50  0000 C CNN
F 2 "Package_TO_SOT_THT_Electeam:TO-220-3_Horizontal_TabDown" H 1250 4875 50  0001 C CIN
F 3 "https://www.onsemi.cn/PowerSolutions/document/MC7800-D.PDF" H 1250 4600 50  0001 C CNN
	1    1250 4650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 60797457
P 1250 5050
F 0 "#PWR06" H 1250 4800 50  0001 C CNN
F 1 "GND" H 1255 4877 50  0000 C CNN
F 2 "" H 1250 5050 50  0001 C CNN
F 3 "" H 1250 5050 50  0001 C CNN
	1    1250 5050
	1    0    0    -1  
$EndComp
$Comp
L power:+36V #PWR02
U 1 1 607965E6
P 750 4650
F 0 "#PWR02" H 750 4500 50  0001 C CNN
F 1 "+36V" H 765 4823 50  0000 C CNN
F 2 "" H 750 4650 50  0001 C CNN
F 3 "" H 750 4650 50  0001 C CNN
	1    750  4650
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR013
U 1 1 6079552E
P 1800 4650
F 0 "#PWR013" H 1800 4500 50  0001 C CNN
F 1 "+5V" H 1815 4823 50  0000 C CNN
F 2 "" H 1800 4650 50  0001 C CNN
F 3 "" H 1800 4650 50  0001 C CNN
	1    1800 4650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R11
U 1 1 607F2CED
P 4950 6950
F 0 "R11" H 5020 6996 50  0000 L CNN
F 1 "0.1" H 5020 6905 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4880 6950 50  0001 C CNN
F 3 "~" H 4950 6950 50  0001 C CNN
	1    4950 6950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 7100 4950 7200
$Comp
L power:GND #PWR040
U 1 1 607F52F7
P 4950 7200
F 0 "#PWR040" H 4950 6950 50  0001 C CNN
F 1 "GND" H 4955 7027 50  0000 C CNN
F 2 "" H 4950 7200 50  0001 C CNN
F 3 "" H 4950 7200 50  0001 C CNN
	1    4950 7200
	1    0    0    -1  
$EndComp
Text GLabel 4950 6350 2    50   Input ~ 0
Current_Sensor
Wire Wire Line
	2750 2900 2800 2900
Wire Wire Line
	2750 2700 2750 2900
$Comp
L Connector:Conn_01x03_Male J5
U 1 1 60B3505B
P 8150 650
F 0 "J5" H 8122 582 50  0000 R CNN
F 1 "Throttle" H 8122 673 50  0000 R CNN
F 2 "Connector_Molex:Molex_KK-254_AE-6410-03A_1x03_P2.54mm_Vertical" H 8150 650 50  0001 C CNN
F 3 "~" H 8150 650 50  0001 C CNN
	1    8150 650 
	-1   0    0    1   
$EndComp
Wire Wire Line
	7950 750  7950 900 
Wire Wire Line
	7700 650  7950 650 
Wire Wire Line
	7950 400  7950 550 
$Comp
L power:+5V #PWR0113
U 1 1 60B56613
P 7950 400
F 0 "#PWR0113" H 7950 250 50  0001 C CNN
F 1 "+5V" H 7965 573 50  0000 C CNN
F 2 "" H 7950 400 50  0001 C CNN
F 3 "" H 7950 400 50  0001 C CNN
	1    7950 400 
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C10
U 1 1 60B587DD
P 2300 600
F 0 "C10" H 2418 646 50  0000 L CNN
F 1 "220uF" H 2418 555 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_10x14.3" H 2418 509 50  0001 L CNN
F 3 "~" H 2300 600 50  0001 C CNN
	1    2300 600 
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 4100 6100 4100
Text GLabel 6100 4100 2    50   Input ~ 0
LIN3
Text GLabel 6100 4400 2    50   Input ~ 0
HIN2
Wire Wire Line
	5900 4400 6100 4400
Text GLabel 6100 4700 2    50   Input ~ 0
HIN1
Wire Wire Line
	6100 4700 5900 4700
Wire Wire Line
	6100 4300 5900 4300
Text GLabel 6100 4200 2    50   Input ~ 0
HIN3
Wire Wire Line
	6100 4200 5900 4200
Wire Wire Line
	6100 4000 5900 4000
Wire Wire Line
	6100 3900 5900 3900
Wire Wire Line
	6150 4800 5900 4800
$Comp
L power:+5V #PWR0102
U 1 1 60D5293D
P 3750 5000
F 0 "#PWR0102" H 3750 4850 50  0001 C CNN
F 1 "+5V" H 3765 5173 50  0000 C CNN
F 2 "" H 3750 5000 50  0001 C CNN
F 3 "" H 3750 5000 50  0001 C CNN
	1    3750 5000
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C11
U 1 1 60DBB821
P 2800 600
F 0 "C11" H 2918 646 50  0000 L CNN
F 1 "220uF" H 2918 555 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_10x14.3" H 2838 450 50  0001 C CNN
F 3 "~" H 2800 600 50  0001 C CNN
	1    2800 600 
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C12
U 1 1 60DBBBDB
P 3250 600
F 0 "C12" H 3368 646 50  0000 L CNN
F 1 "220uF" H 3368 555 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_10x14.3" H 3288 450 50  0001 C CNN
F 3 "~" H 3250 600 50  0001 C CNN
	1    3250 600 
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0110
U 1 1 60E85274
P 1700 1950
F 0 "#PWR0110" H 1700 1800 50  0001 C CNN
F 1 "+12V" H 1715 2123 50  0000 C CNN
F 2 "" H 1700 1950 50  0001 C CNN
F 3 "" H 1700 1950 50  0001 C CNN
	1    1700 1950
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0107
U 1 1 60EDA81B
P 10000 250
F 0 "#PWR0107" H 10000 100 50  0001 C CNN
F 1 "+5V" H 10015 423 50  0000 C CNN
F 2 "" H 10000 250 50  0001 C CNN
F 3 "" H 10000 250 50  0001 C CNN
	1    10000 250 
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 4900 5900 4900
$Comp
L power:GND #PWR0114
U 1 1 60880F4C
P 4100 4800
F 0 "#PWR0114" H 4100 4550 50  0001 C CNN
F 1 "GND" H 4105 4627 50  0000 C CNN
F 2 "" H 4100 4800 50  0001 C CNN
F 3 "" H 4100 4800 50  0001 C CNN
	1    4100 4800
	1    0    0    -1  
$EndComp
Text GLabel 10850 2000 2    50   Input ~ 0
36V-Power
Text GLabel 10800 4150 2    50   Input ~ 0
36V-Power
Text GLabel 10750 6450 2    50   Input ~ 0
36V-Power
Wire Wire Line
	7700 6100 7700 6200
Connection ~ 7750 3950
Wire Wire Line
	7850 1800 7850 1650
Connection ~ 7850 1800
Wire Wire Line
	4150 4400 4350 4400
Wire Wire Line
	4150 4300 4350 4300
Wire Wire Line
	4150 4200 4350 4200
Wire Wire Line
	4350 4800 4100 4800
Wire Wire Line
	4150 4700 4350 4700
Wire Wire Line
	4350 4600 4150 4600
Wire Wire Line
	4150 4500 4350 4500
$Comp
L controller_bldc-rescue:ESP32-DEVKITC-32D-ESP32-DEVKITC-32D U1
U 1 1 6081E288
P 5150 4200
F 0 "U1" H 5125 5367 50  0000 C CNN
F 1 "ESP32-DEVKITC-32D" H 5125 5276 50  0000 C CNN
F 2 "ESP32-DEVKITC-32D_Electeam:MODULE_ESP32-DEVKITC-32D" H 5100 4200 50  0001 L BNN
F 3 "" H 5150 4200 50  0001 L BNN
F 4 "4" H 5150 4200 50  0001 L BNN "PARTREV"
F 5 "Espressif Systems" H 5100 4100 50  0001 L BNN "MANUFACTURER"
	1    5150 4200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R16
U 1 1 608850A5
P 5400 6700
F 0 "R16" V 5193 6700 50  0000 C CNN
F 1 "10k" V 5284 6700 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5330 6700 50  0001 C CNN
F 3 "~" H 5400 6700 50  0001 C CNN
	1    5400 6700
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C13
U 1 1 60885D9B
P 5950 7000
F 0 "C13" V 5721 7000 50  0000 C CNN
F 1 "0.1uF" V 5812 7000 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D7.5mm_W5.0mm_P10.00mm" H 5950 7000 50  0001 C CNN
F 3 "~" H 5950 7000 50  0001 C CNN
	1    5950 7000
	-1   0    0    1   
$EndComp
Wire Wire Line
	4950 6350 4950 6700
Connection ~ 4950 6700
Wire Wire Line
	4950 6700 4950 6800
Wire Wire Line
	4950 6700 5250 6700
Text GLabel 6250 6700 2    50   Input ~ 0
Current_Sensor_IN
$Comp
L power:+12V #PWR0115
U 1 1 608462B9
P 7850 1650
F 0 "#PWR0115" H 7850 1500 50  0001 C CNN
F 1 "+12V" H 7865 1823 50  0000 C CNN
F 2 "" H 7850 1650 50  0001 C CNN
F 3 "" H 7850 1650 50  0001 C CNN
	1    7850 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 3950 7750 3800
$Comp
L power:+12V #PWR0116
U 1 1 6084A421
P 7750 3800
F 0 "#PWR0116" H 7750 3650 50  0001 C CNN
F 1 "+12V" H 7765 3973 50  0000 C CNN
F 2 "" H 7750 3800 50  0001 C CNN
F 3 "" H 7750 3800 50  0001 C CNN
	1    7750 3800
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0117
U 1 1 6084AD2A
P 7700 6100
F 0 "#PWR0117" H 7700 5950 50  0001 C CNN
F 1 "+12V" H 7715 6273 50  0000 C CNN
F 2 "" H 7700 6100 50  0001 C CNN
F 3 "" H 7700 6100 50  0001 C CNN
	1    7700 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 5050 4300 5050
Wire Wire Line
	4300 5050 4300 4900
Wire Wire Line
	4300 4900 4350 4900
Wire Wire Line
	3750 5000 3750 5050
NoConn ~ 5900 4600
Wire Wire Line
	1500 500  1800 500 
Wire Wire Line
	1800 500  1800 450 
Wire Wire Line
	1800 650  1800 600 
Wire Wire Line
	1800 600  550  600 
Wire Wire Line
	2300 450  2300 300 
Wire Wire Line
	2300 300  2800 300 
Wire Wire Line
	2800 300  2800 450 
Wire Wire Line
	3250 450  3250 300 
Wire Wire Line
	3250 300  2800 300 
Connection ~ 2800 300 
Wire Wire Line
	2300 750  2300 900 
Wire Wire Line
	2300 900  2800 900 
Wire Wire Line
	2800 900  2800 750 
Wire Wire Line
	3250 750  3250 900 
Wire Wire Line
	3250 900  2800 900 
Connection ~ 2800 900 
$Comp
L power:GND #PWR?
U 1 1 608A0585
P 2800 1050
F 0 "#PWR?" H 2800 800 50  0001 C CNN
F 1 "GND" H 2805 877 50  0000 C CNN
F 2 "" H 2800 1050 50  0001 C CNN
F 3 "" H 2800 1050 50  0001 C CNN
	1    2800 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 1050 2800 900 
Text GLabel 2800 150  2    50   Input ~ 0
36V-Power
Wire Wire Line
	2800 150  2800 300 
$Comp
L power:GND #PWR?
U 1 1 6084D0EA
P 5950 7200
F 0 "#PWR?" H 5950 6950 50  0001 C CNN
F 1 "GND" H 5955 7027 50  0000 C CNN
F 2 "" H 5950 7200 50  0001 C CNN
F 3 "" H 5950 7200 50  0001 C CNN
	1    5950 7200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 7100 5950 7200
Wire Wire Line
	5950 6900 5950 6700
Wire Wire Line
	5550 6700 5950 6700
Wire Wire Line
	5950 6700 6250 6700
Connection ~ 5950 6700
Wire Wire Line
	3500 6900 3500 7100
Wire Wire Line
	3350 6900 3500 6900
Wire Wire Line
	3500 6900 3650 6900
Connection ~ 3500 6900
$Comp
L power:GND #PWR?
U 1 1 60892AC0
P 3500 7350
F 0 "#PWR?" H 3500 7100 50  0001 C CNN
F 1 "GND" H 3505 7177 50  0000 C CNN
F 2 "" H 3500 7350 50  0001 C CNN
F 3 "" H 3500 7350 50  0001 C CNN
	1    3500 7350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 7300 3500 7350
$EndSCHEMATC

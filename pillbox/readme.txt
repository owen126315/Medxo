---------------------------------------------------------------------------------
Pin assignmaent of MCU
---------------------------------------------------------------------------------
A0 = battery V+
A1 = LDR
D0 = softwareSerial TX
D1 = softwareSerial RX
D2 = Green LED low bit
D3 = Green LED high bit
D4 = S1
D5 = S2
D6 = S3
D7 = S4
D8 = Green LED enable
D9 = Red LED 1
D10 = Red LED 2
D11 = Red LED 3
D12 = Red LED 4
D13 = vibrator
----------------------------------------------------------------------------------

----------------------------------------------------------------------------------
Pin assignmaent of Sensor Board
----------------------------------------------------------------------------------
1 = GND
2 = Red LED 4
3 = Red LED 3
4 = Red LED 2
5 = Red LED 1
6 = Green LED enable
7 = Green LED low bit
8 = Green LED high bit
9 = S1
10 = S2
11 = S3
12 = S4
13 = VCC
----------------------------------------------------------------------------------

if new sim card is used, change:

const char GPRS_APN[]  
const char GPRS_LOGIN[] 
const char GPRS_PASSWORD[]

 

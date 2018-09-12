 
// libraries
#include <MKRGSM.h>
#include "Timer.h"

const char PINNUMBER[]     = "";
// APN data
const char GPRS_APN[]      = "SmarTone";
const char GPRS_LOGIN[]    = "";
const char GPRS_PASSWORD[] = "";

// initialize the library instance
GSMClient client;
GPRS gprs;
GSM gsmAccess;
Timer timer1, timer2, timer3;
//  timer1: internal clock 
//  timer2: red led flashing controll
//  timer3: green led flashing controll

// URL, path and port (for example: arduino.cc)
char myserver[] = "medpot.com.hk";
char mypath[] = "/api/patient/10/alarm.json?key=KEY10";
int myport = 80; // port 80 is the default for HTTP
char rx_buffer[3000];

int current_time;
int next_to_open;
int schedule[4];
int green_led_event, red_led_event[4], timer3_event;
char overtime_wav[] = {0x7e,0x07,0xa1,0x30,0x30,0x30,0x30,0x68,0xef};
char early_wav[] = {0x7e,0x07,0xa1,0x30,0x30,0x30,0x32,0x6a,0xef};
char wrong_wav[] = {0x7e,0x07,0xa1,0x30,0x30,0x30,0x31,0x69,0xef};
char change_wav[] = {0x7e,0x07,0xa1,0x30,0x30,0x30,0x33,0x6b,0xef};
int cover_status[4] = {0,0,0,0};
int battery_ADC;
int ldr_ADC;

void setup() 
{
  Serial.begin(9600);
  Serial1.begin(9600);
  //while (!Serial);
  while (!Serial1);
  delay(1000);
  Serial.println("Start");
  timer1.every(60000, update_current_time);
  timer3_event = timer3.every(10000, change_slot);
  pinMode(2, OUTPUT);       //GREEN LED CONTROL LOW PIN 
  pinMode(3, OUTPUT);       //GREEN LED CONTROL HIGH PIN
  pinMode(4, INPUT_PULLUP); //COVER 1
  pinMode(5, INPUT_PULLUP); //COVER 2
  pinMode(6, INPUT_PULLUP); //COVER 3
  pinMode(7, INPUT_PULLUP); //COVER 4
  pinMode(8, OUTPUT);       //GREEN LED ENABLE 
  pinMode(9, OUTPUT);       //COVER 1 RED LED 
  pinMode(10, OUTPUT);      //COVER 2 RED LED
  pinMode(11, OUTPUT);      //COVER 3 RED LED
  pinMode(12, OUTPUT);      //COVER 4 RED LED
  pinMode(13, OUTPUT);      //VIBRATOR
  /*
  attachInterrupt(digitalPinToInterrupt(4), cover_status_changed_4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(5), cover_status_changed_5, CHANGE);
  attachInterrupt(digitalPinToInterrupt(6), cover_status_changed_6, CHANGE);
  attachInterrupt(digitalPinToInterrupt(7), cover_status_changed_7, CHANGE);
  */
  digitalWrite(8,LOW);
  gsm_begin();
  next_to_open = 0;
  
  if(connect_to_server(myserver, mypath, myport))
  {
    int str_len = read_respone(); 
    print_rx_data(str_len);
  }
  schedule[0] = 15*60+59;
  Serial.print("Current Time:");
  Serial.print(current_time/60);
  Serial.print(":");
  Serial.println(current_time%60);
  Serial.print("schedule:");
  for(int i=0;i<4;i++)
  {
    Serial.print(schedule[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void loop() 
{
  timer1.update();
  timer2.update(); 
  int tem[4];
  battery_ADC = analogRead(A0);
  ldr_ADC = analogRead(A1);
  if(digitalRead(4) != cover_status[0])
     cover_status_changed_4();

  if(digitalRead(5) != cover_status[1])
     cover_status_changed_5();   
 
  if(digitalRead(6) != cover_status[2])
     cover_status_changed_6();

  if(digitalRead(7) != cover_status[3])
     cover_status_changed_7();     
  
}

void gsm_begin()
{
  boolean connected = false;
  while (!connected) 
  {
    if ((gsmAccess.begin(PINNUMBER) == GSM_READY) && (gprs.attachGPRS(GPRS_APN, GPRS_LOGIN, GPRS_PASSWORD) == GPRS_READY)) 
      connected = true;
     else 
     {
      Serial.println("Not connected");
      delay(1000);
     }
  }
  Serial.println("connecting...");  
}

boolean connect_to_server(char server[], char path[], int port)
{ 
  int server_connected = 0;
  if (client.connect(server, port)) 
  {
    Serial.println("connected");
    server_connected = 1;
    // Make a HTTP request:
    
    client.print("GET ");
    client.print(path);
    client.println(" HTTP/1.1");
    client.print("Host: ");
    client.println(server);
    client.println("Connection: close");
    client.println();
    
  }
  else 
    Serial.println("connection failed");
  return server_connected;
}

int read_respone()  //return string length
{
  char temp[2];
  int string_length = 0;
  //boolean found_data = false;
  while(client.connected())
  {  
    if (client.available()) 
    {
      rx_buffer[string_length++] = client.read();
      /*
      if(temp[0] == '{' && !found_data)
        found_data = true;
      if(found_data)
      {  
        //Serial.print(temp[0]);
        rx_buffer[string_length++] = temp[0];
      }
      */
      /*
      if(temp[0] != '\n')
      {
        rx_buffer[string_length++] = temp[0];
        temp[1] = 0;
      }
      else
      { 
        if(temp[1] != '\n')
        {
         temp[1] = '\n';
         rx_buffer[string_length++] = temp[0];
        }  
        else
        {
          client.stop();
          client.flush();
          break;  
        }      
      }
      */ 
    }
    if (!client.connected()) 
    {
      Serial.println();
      Serial.println("disconnecting.");
      client.stop();
    }
  }
  get_time_and_schedule();  
  return string_length;
}

void get_time_and_schedule()
{
   int i =0;
   int tem;
   while(rx_buffer[i++]!='\n');
   i+=23;
   tem = (rx_buffer[i++]&0x0F)*10*60;
   tem += (rx_buffer[i]&0x0F)*60;
   i+=2;
   tem += (rx_buffer[i++]&0x0F)*10;
   tem += rx_buffer[i]&0x0F;
   tem += 60*8;
   if(tem>24*60)
    tem -= 24*60; 
   current_time = tem;
   for(int j=0; j<10; j++)
   {
    while(rx_buffer[i++]!='\n');
   }
   i+=558;
   
   for(int j=0;j<3;j++)
   {
    String temp="";
    while(rx_buffer[i]!=',')
    {
     temp+=rx_buffer[i++]; 
    }
    schedule[j] = temp.toInt();
    i++; 
   }
   String temp="";  
   while(rx_buffer[i]!=']')
   {
    temp+=rx_buffer[i++]; 
   }
   schedule[3] = temp.toInt(); 
   
}

void print_rx_data(int str_length)
{
  for(int i =0; i<str_length; i++)
    Serial.print(rx_buffer[i]);
  for(int i =0; i<2500; i++)
    rx_buffer[i] = 0;
}

void cover_status_changed_4()
{
  cover_status[0] = digitalRead(4);
  Serial.print("D4 =");
  Serial.println(cover_status[0]);
  send_switch_data_to_server();
  if(cover_status[0])
  {
    int status = check_time_and_cover(0);
    led_sound_respond(0, status, 9);
  }
    
  else
  {
    timer2.stop(red_led_event[0]);
    digitalWrite(9,LOW);
  }      
      
}

void cover_status_changed_5()
{
  cover_status[1] = digitalRead(5);
  Serial.print("D5 =");
  Serial.println(cover_status[1]);
  send_switch_data_to_server();
  if(cover_status[1])
  {
    int status = check_time_and_cover(1);
    led_sound_respond(1, status, 10);
  }
  
  else
  {
    timer2.stop(red_led_event[1]);
    digitalWrite(10,LOW);
  }
  
}

void cover_status_changed_6()
{
  cover_status[2] = digitalRead(6);
  Serial.print("D6 =");
  Serial.println(cover_status[2]);
  send_switch_data_to_server(); 
  if(cover_status[2])
  {
    int status = check_time_and_cover(2);
    led_sound_respond(2, status, 11);
  }
  
  else
  {
    timer2.stop(red_led_event[2]);
    digitalWrite(11,LOW);
  }
  
}

void cover_status_changed_7()
{
  cover_status[3] = digitalRead(7);
  Serial.print("D7 =");
  Serial.println(cover_status[3]);
  send_switch_data_to_server();  
  if(cover_status[3])
  {
    int status = check_time_and_cover(3);
    led_sound_respond(3, status, 12);
  }
  
  else
  {
    timer2.stop(red_led_event[3]);
    digitalWrite(12,LOW);
  }
  
}

int check_time_and_cover(int cover_num)
{
  if(current_time<schedule[next_to_open]-1)  
    return 2;
  
  else if(next_to_open != cover_num)  
    return 1;
    
  else
    return 0;
}

void update_current_time()
{
  current_time++;
  Serial.print(current_time/60);
  Serial.print(":");
  Serial.println(current_time%60);
  if(current_time>schedule[next_to_open]+1)   //overtime
  {
    Serial.println("over time");
    Serial1.write(overtime_wav,sizeof(overtime_wav));
    switch(next_to_open)
    {
      case 0:
      {
        digitalWrite(2,LOW);
        digitalWrite(3,LOW);
        break;
      }
      
      case 1:
      {
        digitalWrite(2,HIGH);
        digitalWrite(3,LOW);
        break;
      }
      
      case 2:
      {
        digitalWrite(2,LOW);
        digitalWrite(3,HIGH);
        break;
      }     
      
      case 3:
      {
        digitalWrite(2,HIGH);
        digitalWrite(3,HIGH);
      }           
    }
    green_led_event = timer2.oscillate(8, 1000, LOW);

  }
}

void led_sound_respond(int cover_num, int val, int red_led_pin)
{
  switch(val)
  {
    case 0:   //correct
    {
      timer2.stop(green_led_event);
      digitalWrite(8,LOW);
      if(cover_num<3)
      {
        Serial.print("next to be open ");
        Serial.println(next_to_open);  
      }
      else
      {
        timer3.update();
        Serial.println("change all slots");
        while(analogRead(A1)<18);
        timer3.stop(timer3_event);
        send_switch_data_to_server();
        Serial.println("changed");
      }
      
      next_to_open++;  
      break;
    }

    case 1:   //wrong cover
    {
      Serial.println("Wrong cover");
      red_led_event[cover_num] = timer2.oscillate(red_led_pin, 1000, HIGH);
      Serial1.write(wrong_wav,sizeof(wrong_wav));
      break;
    }

    case 2:   //too early
    {
      Serial.println("too early");
      red_led_event[cover_num] = timer2.oscillate(red_led_pin, 1000, HIGH);
      Serial1.write(early_wav,sizeof(early_wav));
      break;
    }
  }
}

bool send_switch_data_to_server()
{
  int percentage = (battery_ADC - 775)*100/248;
  if(percentage < 0)
    percentage = 0;
  Serial.print("percentage:");
  Serial.println(percentage);
  
  int plastic_slot;
  if(ldr_ADC > 18)
    plastic_slot = 0;
  else
    plastic_slot = 1;
  Serial.print("plastic slot extisence:");
  Serial.println(plastic_slot);
  char server[] = "medpot.com.hk";
  char path[] = "/api/patient/10/send_switch.json?key=KEY10";
  int port = 80;
  String datajson = "{\"data\":[{\"s1\":"+ String(cover_status[0]) +",\"s2\":"+ String(cover_status[1]) +",\"s3\":"+ String(cover_status[2]) +",\"s4\":"+ String(cover_status[3]) +",\"s5\":"+ String(percentage) +",\"s6\":"+ String(plastic_slot)+ ",\"at\":1530094746}]}";
  int server_connected = 0;
  if (client.connect(server, port)) 
  {
    Serial.println("connected");
    server_connected = 1;
    // Make a HTTP request:
    
    client.print("POST ");
    client.print(path);
    client.println(" HTTP/1.1");
    client.println("Content-Type: application/json; charset=utf-8");
    client.print("Host: ");
    client.println(server);
    client.println("Connection: close");
    client.print("Content-Length: ");
    client.println(datajson.length());
    client.println();
    client.print(datajson);
  }
  else 
    Serial.println("connection failed");
  return server_connected;
}

void change_slot()
{
    Serial1.write(change_wav,sizeof(change_wav));  
}


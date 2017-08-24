/*
  FreeRTOS.org V5.0.4 - Copyright (C) 2003-2008 Richard Barry.

  This file is part of the FreeRTOS.org distribution.

  FreeRTOS.org is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  FreeRTOS.org is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with FreeRTOS.org; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

  A special exception to the GPL can be applied should you wish to distribute
  a combined work that includes FreeRTOS.org, without being obliged to provide
  the source code for any proprietary components.  See the licensing section
  of http://www.FreeRTOS.org for full details of how and when the exception
  can be applied.

    ***************************************************************************
    ***************************************************************************
    *                                                                         *
      SAVE TIME AND MONEY!  We can port FreeRTOS.org to your own hardware,
      and even write all or part of your application on your behalf.
      See http://www.OpenRTOS.com for details of the services we provide to
      expedite your project.
    *                                                                         *
    ***************************************************************************
    ***************************************************************************

  Please ensure to read the configuration and relevant port sections of the
  online documentation.

  http://www.FreeRTOS.org - Documentation, latest information, license and
  contact details.

  http://www.SafeRTOS.com - A version that is certified for use in safety
  critical systems.

  http://www.OpenRTOS.com - Commercial support, development, porting,
  licensing and training services.
*/

/* FreeRTOS.org includes. */
#include "FreeRTOS_ARM.h"
//#include "task.h"
//#include "semphr.h"
//#include "portasm.h"
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
/* Demo includes. */
#include "basic_io_arm.h"

/* Compiler includes. */
//#include <dos.h>

/* The tasks to be created. */
static void vHandlerTask( void *pvParameters );
static void vPeriodicTask( void *pvParameters );
static void vClockTask( void *pvParameters );
static void vSyncTask( void *pvParameters );

byte ledState = 0;
/* The service routine for the interrupt.  This is the interrupt that the task
  will be synchronized with. */
static void vExampleInterruptHandler( void );

/*BEGIN OF SYNC VARS*/
signed long TS1, TS2, TS3, TS4;                   //time variables of the slave
signed long TM1 = 0, TM2 = 0, TM3 = 0, TM4 = 0;   //time variable of the master
signed long T1 = 0, T2 = 0, T3 = 0, T4 = 0;   //time variable of the master
signed long comm_delay = 0;                       //contains the calculated communication delay
long offset = 0;                                  //contains the actual offset of the clocks
int i = 0;                                        //Counts the step of the synchronization
byte command = B000;                              //TEST COMAND.
int ii = 1;                                       //counts the number of iterations in the sync
unsigned long aux = 4261148655ul;    //Saves the time stamp to send (TM1,TM2,TM3,TM4)
byte a1, a2, a3, a4;                 //Each byte of the time stamp to send


const byte ASK = B001;
const byte SYNC = B010;
const byte FOLLOWUP = B011;
const byte DELAYREQ = B100;
const byte DELAYRESP = B101;
int portMaster = 2000;                                    //This is the port for synchronization


/*-----------------------------------------------------------*/

/* Declare a variable of type SemaphoreHandle_t.  This is used to reference the
  semaphore that is used to synchronize a task with an interrupt. */
SemaphoreHandle_t xBinarySemaphore;

// pins to generate interrupts - they must be connected
const uint8_t inputPin = 2;
const uint8_t outputPin = 3;

unsigned long local_time_stamp = 0;

/*BEGIN OF ADC VARS*/
const int DTRDY = 9;      //Data ready pin of the ADC (ads1247)
const int ADS_CS = 8;     //SPI Control pin of the ADC
int regarray[14];         //This array saves the actual configuration of the ADC
/*BEGIN OF ADC VARS*/

/*BEGIN OF NETWORK VARS*/
const int CS = 10;                                         //W5500 SPI control pin
int master_port = 2032;                                    //This is the port for synchronization
int localPort = 2034;
byte mac_local[] = { 0x90, 0xA2, 0xDA, 0x10, 0x39, 0x6A }; //MAC of the master device
byte ip_pc[] = { 192, 168, 255, 4};                        //IP of the DATA collector
EthernetUDP Udp;                                           //Variable that holds the UDP instance
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,
char  ReplyBuffer[] = "acknowledged";       // a string to send back
const int PTP_PACKET_SIZE = 5; // NTP time stamp is in the first 48 bytes of the message
char  datagramSync[PTP_PACKET_SIZE];
char  ptpdatagramSend[PTP_PACKET_SIZE];       // a string to send back
char  ptpdatagramRecieve[PTP_PACKET_SIZE];       // a string to send back
EthernetClient client_pc;                                  //Contains the socket for comunicating with
byte ipMaster[] = { 192, 168, 255, 2};                    //IP of the master clock

/*END OF NETWORK VARS*/

/*BEGIN OF DAQ VARS*/
int dataind1 = 0;         //index of the data buffer 1
int dataind2 = 0;         //index of the data buffer 2
String msg2 = "";         //second buffer
int msg_rdy = 0;          //this flag indicates if the first fubber is ready
int msg2_rdy = 0;         //same as before for the second one
int msgflag = 0;          //this flag indicates wich buffer is being filled, 0 for msg, 1 for msg2
/*END OF DAQ VARS*/

int clockPin = 3;                   // Pin wich outputs the clock frequency
bool toogle_flag = 0;                // 1 ON, 0 OFF


int counterx = 0;

struct measure {
  long x;
  long y;
  long z;
};
long timeaftertime = 0;

/*BEGIN OF toogle()*/
/*This function toogles the clock pin ON and OFF*/
void toogle() {
  if (toogle_flag == 1) {
    digitalWrite(clockPin, LOW);
    toogle_flag = 0;
  }
  else if (toogle_flag == 0) {
    digitalWrite(clockPin, HIGH);
    toogle_flag = 1;
  }
}
/*END OF toogle()*/


/*BEGIN OF ads_reset()*/
/*This function sends the reset command to the ads1247*/
void ads_reset() {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADS_CS, LOW);
  SPI.transfer(0x07);
  delay(10);
  digitalWrite(ADS_CS, HIGH);
  SPI.endTransaction();
}
/*END OF ads_reset()*/

/*BEGIN OF ads_get_reg_value()*/
/*This function reads one register from the ads1247*/
signed long ads_get_reg_value(int reg) { //falta verificar que sean 4 bites
  int response;
  int command = 0x2;
  command <<= 4;
  command |= reg;          //Se genera el comando de lectura
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADS_CS, LOW);
  SPI.transfer(command);   //Se comunica el comando
  SPI.transfer(0x00);      //cantidad de bytes a leer - 1
  response = SPI.transfer(0xFF);
  digitalWrite(ADS_CS, HIGH);
  SPI.endTransaction();
  return response;
}
/*END OF ads_get_reg_value()*/

/*BEGIN OF ads_get_all_regs()*/
/*Get all register values of the ads1247 and saves it on regarray*/
void ads_get_all_regs() {
  for (int i = 0; i <= 14; i++)
    regarray[i] = ads_get_reg_value(i); //SE DEBEN MEZCLAR ESTAS FUNCIONES!?!?!?!
}
/*END OF ads_get_all_regs()*/

/*BEGIN OF ads_print_regarray()*/
/*Prints on Serial the regarray*/
void ads_print_regarray() {//SE DEBEN MEZCLAR ESTAS FUNCIONES!?!?!?!
  String msj;
  for (int i = 0; i <= 13; i++) {
    msj = "0x0" + String(i, HEX) + " -> " + String(regarray[i], HEX);
    Serial.println(msj);
  }
}//search for streaming library
/*END OF ads_print_regarray()*/

/*BEGIN OF ads_write_reg()*/
/*asign the value <value> to the register <reg> of the ads1247*/
void ads_write_reg(int reg, int value) { // falta verificar los tamaños de los imputs
  int command = 0x4;
  command <<= 4;
  command |= reg;          //Se genera el comando de escritura
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ADS_CS, LOW);
  SPI.transfer(command);   //Se comunica el comando
  SPI.transfer(0x00);      //cantidad de bytes a escribir - 1
  SPI.transfer(value);
  digitalWrite(ADS_CS, HIGH);
  SPI.endTransaction();
}
/*END OF ads_write_reg()*/

int packetSize;

int syncFlag = 0;
int askFlag2 = 0;
/*BEGIN of ads_read_once()*/
/*READS one value from the ads1247 DEPRECATED!!!!! DONT USE THIS!! */
long ads_read_once() {
  while (1) {
    if (digitalRead(DTRDY) == LOW) {
      signed long reading = 0x0;
      SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE1));
      digitalWrite(ADS_CS, LOW);   //CS negative logic
      SPI.transfer(0x13);          //Rdata once
      reading |= SPI.transfer(0xFF);// Get the first byte
      reading <<= 8;                // add bytes to total reading
      reading |= SPI.transfer(0xFF);// Get the second byte
      reading <<= 8;
      reading |= SPI.transfer(0xFF);// and so on...
      digitalWrite(ADS_CS, HIGH);
      SPI.endTransaction();
      return reading;
    }
  }
}
/*END of ads_read_once()*/

/*BEGIN OF givemeThemicros()*/
/*Returns the value of the clock in microseconds*/
/*UNUSED*/
unsigned long givemeThemicros (unsigned long value) {
  return (value * 20);
}
/*END OF givemeThemicros()*/

//en que version se usa asi?
//xQueueHandle xQueue;
QueueHandle_t xQueue;
void setup( void )
{
  Serial.begin(115200);
  SPI.begin();

  Serial.println("DHCP request sended");
  Ethernet.begin(mac_local);
  Serial.print("The ip adress is: ");
  Serial.println(Ethernet.localIP());

  Serial.println("tratando de conectarse al PC en la direccion ");
  Serial.println(ip_pc[3]);
  if (client_pc.connect(ip_pc, 2034)) {
    Serial.println("connected");
  }
  else {
    Serial.println("DRAMA MY MAN, plz reboot");
    while (HIGH) {
      Serial.println("DRAMA MY MAN, plz reboot");
      delay(1000);
    }
  }  Udp.begin(localPort);


  pinMode(ADS_CS, OUTPUT);
  digitalWrite(ADS_CS, HIGH);
  pinMode(DTRDY, INPUT);
  Serial.println();
  SPI.begin();
  Serial.println("RESET TO POWERUP VALUES");
  ads_reset();
  ads_get_all_regs();
  ads_print_regarray();
  Serial.println("APPLY CONFIGURATION TO ADC");
  ads_write_reg(0x0, 0x08);
  ads_write_reg(0x2, 0x00);
  ads_write_reg(0x3, 0x08);  //1000 SPS
  ads_write_reg(0x4, 0x00);  //OFFSET 18
  ads_write_reg(0x5, 0x00);  //OFFSET 30
  ads_write_reg(0x6, 0x00);  //OFFSET 4D
  ads_get_all_regs();
  ads_print_regarray();
  Serial.println("_____________________________________");
  /* Before a semaphore is used it must be explicitly created.  In this example
    a binary semaphore is created. */
  xQueue = xQueueCreate(5, sizeof(signed long));
  Serial.println("_____________________________________");




  /* Check the semaphore was created successfully. */
  if ((xQueue != NULL))
  {

    xTaskCreate( vPeriodicTask, "Periodic", 200, NULL, 1, NULL );
    xTaskCreate( vClockTask, "Clock", 200, NULL, 4, NULL );
    xTaskCreate( vSyncTask, "Clock", 200, NULL, 3, NULL );

    /* Install the interrupt handler. */
    pinMode(clockPin, OUTPUT);

    Serial.println("start the shieeet");

    /* Start the scheduler so the created tasks start executing. */
    vTaskStartScheduler();
  }

  /* If all is well we will never reach here as the scheduler will now be
    running the tasks.  If we do reach here then it is likely that there was
    insufficient heap memory available for a resource to be created. */
  for ( ;; );
  //  return 0;
}



static void vSyncTask( void *pvParameters )

{
  const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;
  for ( ;; )
  {
    while (syncFlag == 0) {
      if (syncFlag == 0) {
        if (askFlag2 == 0)
        {
          askFlag2 = 1;
          memset(ptpdatagramSend, 0, PTP_PACKET_SIZE);
          ptpdatagramSend[0] = ASK;
          ptpdatagramSend[1] = 1;
          ptpdatagramSend[2] = 1;
          ptpdatagramSend[3] = 1;
          ptpdatagramSend[4] = 1;
          //sendig ASK
          vPrintString("Sendig ASK");
          Udp.beginPacket(ipMaster, portMaster);
          Udp.write(ptpdatagramSend);
          Udp.endPacket();
        }
        packetSize = Udp.parsePacket();
        if (packetSize) {
          // read the packet into ptpdatagramRecieve
          memset(ptpdatagramRecieve, 0, PTP_PACKET_SIZE);
          Udp.read(ptpdatagramRecieve, PTP_PACKET_SIZE);
          if (int(ptpdatagramRecieve[0]) == SYNC) {
            T2 = timeaftertime; // T2
            vPrintString("Recieved SYNC");
          }
          else if (int(ptpdatagramRecieve[0]) == FOLLOWUP) {
            T3 = local_time_stamp;
            a1 = ~ptpdatagramRecieve[1] ;
            a2 = ~ptpdatagramRecieve[2] ;
            a3 = ~ptpdatagramRecieve[3] ;
            a4 = ~ptpdatagramRecieve[4];
            T1 = a4;
            T1 <<= 8;
            T1 |= a3;
            T1 <<= 8;
            T1 |= a2;
            T1 <<= 8;
            T1 |= a1;
            timeaftertime -= T2 - T1;
            /*ACA SE DEBE GUARDAR EL T1*/
            vPrintString("Recieved FOLLOWUP");
            vPrintString("T1: ");
            vPrintString((char *)T1);
            memset(ptpdatagramSend, 0, PTP_PACKET_SIZE);
            ptpdatagramSend[0] = DELAYREQ;
            ptpdatagramSend[1] = 1;
            ptpdatagramSend[2] = 1;
            ptpdatagramSend[3] = 1;
            ptpdatagramSend[4] = 1;
            Udp.beginPacket(ipMaster, portMaster);
            Udp.write(ptpdatagramSend);
            Udp.endPacket();
            vPrintString("Sended DELAYREQ");
          }
          else if (int(ptpdatagramRecieve[0]) == DELAYRESP) {
            T3 = timeaftertime;
            a1 = ~ptpdatagramRecieve[1] ;
            a2 = ~ptpdatagramRecieve[2] ;
            a3 = ~ptpdatagramRecieve[3] ;
            a4 = ~ptpdatagramRecieve[4];
            T4 = a4;
            T4 <<= 8;
            T4 |= a3;
            T4 <<= 8;
            T4 |= a2;
            T4 <<= 8;
            T4 |= a1;
            /*ACA SE DEBE GUARDAR EL T1*/
            vPrintString("Recieved DELAYRESP");
            vPrintString("T4: ");
            vPrintString((char *)T4);
            //ACTUALIZAMOS EL RELOJ
            timeaftertime -= ((T1 - T2) + (T3 - T4)) / 2;
            syncFlag = 1;
            askFlag2 = 0;
            while (1)taskYIELD();
          }
        }
        vTaskDelay((10L * configTICK_RATE_HZ) / 1000L);
      }
    }
  }
}

static void vPeriodicTask( void *pvParameters )
{
  String msg = "";
  signed long x;
  portBASE_TYPE xStatus;
  const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;

  /* As per most tasks, this task is implemented within an infinite loop. */
  for ( ;; )
  {
    /* This task is just used to 'simulate' an interrupt.  This is done by
      periodically generating a software interrupt. */
    if ( uxQueueMessagesWaiting( xQueue ) != 0 )
    {
      //   vPrintString( "Queue should have been empty!\n" );
    }
    xStatus = xQueueReceive( xQueue, &x, xTicksToWait );

    // Turn LED on.

    //Serial.print(x);

    //vPrintString( "a\r\n" );
    //taskYIELD();
    if ( xStatus == pdPASS )
    {

      /* Data was successfully received from the queue, print out the received
        value. */
      //vPrintStringAndNumber( "Received = ", m2.x );
      msg = "";
      msg += String(timeaftertime);
      msg += String(";");
      msg += String(x, HEX);
      msg += String("\n");
      client_pc.print(msg);
    }
    else
    {
      /* Data was not received from the queue even after waiting for 100ms.
        This must be an error as the sending tasks are free running and will be
        continuously writing to the queue. */
      vPrintString( "Could not receive from the queue.\n" );
    }

    vTaskDelay((4L * configTICK_RATE_HZ) / 1000L);

  }
}
/*-----------------------------------------------------------*/
static void vClockTask( void *pvParameters )
{
  signed long x;     //Contain the values of the sensor at each sample
  portBASE_TYPE xStatus;
  /* As per most tasks, this task is implemented within an infinite loop. */
  for ( ;; )
  {
    /* This task is just used to 'simulate' an interrupt.  This is done by
      periodically generating a software interrupt. */
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE1));
    digitalWrite(ADS_CS, LOW);   //CS negative logic
    SPI.transfer(0x13);          //Rdata once
    x = 0;
    x |= SPI.transfer(0xFF);// Get the first byte
    x <<= 8;                // add bytes to total reading
    x |= SPI.transfer(0xFF);// Get the second byte
    x <<= 8;
    x |= SPI.transfer(0xFF);// and so on...
    digitalWrite(ADS_CS, HIGH);
    SPI.endTransaction();
    xStatus = xQueueSendToBack(xQueue, &x, 0);
    ++timeaftertime;
    toogle();
    vTaskDelay((4 * configTICK_RATE_HZ) / 1000L);
  }
}

void loop() {}

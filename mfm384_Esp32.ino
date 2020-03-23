/*

  RS485_HalfDuplex.pde - example using ModbusMaster library to communicate
  with EPSolar LS2024B controller using a half-duplex RS485 transceiver.

  This example is tested against an EPSolar LS2024B solar charge controller.
  See here for protocol specs:
  http://www.solar-elektro.cz/data/dokumenty/1733_modbus_protocol.pdf

  Library:: ModbusMaster
  Author:: Marius Kintel <marius at kintel dot net>

  Copyright:: 2009-2016 Doc Walker

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

*/

#include <ModbusMaster.h>
//#include <SoftwareSerial.h>


/*!
  We're using a MAX485-compatible RS485 Transceiver.
  Rx/Tx is hooked up to the hardware serial port at 'Serial'.
  The Data Enable and Receiver Enable pins are hooked up as follows:
*/
#define MAX485_DE      2
#define MAX485_RE_NEG  2


// set up a new serial object
//SoftwareSerial mySerial(rxPin, txPin);


// instantiate ModbusMaster object
ModbusMaster node;

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void setup()
{
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Modbus communication runs at 115200 baud
  Serial.begin(9600);
  Serial2.begin(9600);
 // mySerial.begin(9600);

  // Modbus slave ID 1
  node.begin(1, Serial2);
  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

bool state = true;

void loop()
{


  uint8_t result;
  uint16_t data[6];
  uint32_t tempVariable = 0;
  float finalFloatValue = 0;
  // Toggle the coil at address 0x0002 (Manual Load Control)
  //result = node.writeSingleCoil(0x0002, state);
  //state = !state;
  // Read 16 registers starting at 0x3100)
  result = node.readInputRegisters(0x00, 2);
  if (result == node.ku8MBSuccess)
  {
    for(int i =0;i < 2; i++)
    {
      data[i] =  node.getResponseBuffer(i);
    }

    tempVariable = data[1]| 0x0000;
    tempVariable = data[0] | (tempVariable << 16); 

    finalFloatValue = *(float*)&tempVariable;
    Serial.print("Voltage V1N: ");
    Serial.println(finalFloatValue);
    node.clearResponseBuffer();


   result = node.readInputRegisters(0x38, 2);
  if (result == node.ku8MBSuccess)
  {
    for(int i =2,j=0;i < 4,j<2; i++,j++)
    {
      data[i] =  node.getResponseBuffer(j);
    }

    tempVariable = data[3]| 0x0000;
    tempVariable = data[2] | (tempVariable << 16); 

    finalFloatValue = *(float*)&tempVariable;
    Serial.print("Frequency o/p : ");
    Serial.println(finalFloatValue);
  }    /*.print("Vload: ");
    Serial.println(node.getResponseBuffer(0xC0)/100.0f);
    Serial.print("Pload: ");
    Serial.println((node.getResponseBuffer(0x0D) +
                    node.getResponseBuffer(0x0E) << 16)/100.0f);*/
  }

  delay(1000);
}

/*
  Interface to a KYT-7916 EMV card reader
  Nathan Seidle / SparkFun Electronics
  November 12th, 2017

  This sketch shows how to issue various commands to the KYT-7916 EMV card reader. 
  Most of the major functions are written and working however I have not written the 
  functions to do the ICC Direct Control. This is the main role of an EMV reader. I
  just don't have enough knowhow or examples of how ICC interactions work to truly
  test the direct control.
  
  The reader has a 5 pin cable with the following connections to a microcontroller:  
  Red: 5V
  White: TX out from EMV reader
  Grey: RX in to EMV reader
  Blue: GND
  Yellow: GND (GND pins are connected together so pick one)

  Power: 5V with ~ 1.5A when the solenoid ejects a card. Use an external 5V power supply with the Arduino
  or else the system will hang when a card is ejected.
  
  Baud is 19200, 8N1
  However the EMV reader expects RS232 not TTL serial. Luckily, softwareSerial library has option to
  invert signal and we are able to 'fake' RS232!

  Interestingly: BCC = Block Check Character https://en.wikipedia.org/wiki/Block_check_character
  Basically it's a CRC

*/

#include <SoftwareSerial.h>

#define CONTROL_SOH 0x01 //Start of header
#define CONTROL_STX 0x02 //Start of text
#define CONTROL_ETX 0x03 //End of text
#define CONTROL_EOT 0x04 //End of Transmission
#define CONTROL_ENQ 0x05 //Enquiry
#define CONTROL_ACK 0x06 //ACK
#define CONTROL_NAK 0x15 //NAK
#define CONTROL_CAN 0x18 //Cancel

#define NEGATIVE_RESPONSE_NOT_DEFINED         1
#define NEGATIVE_RESPONSE_NO_CARD             2
#define NEGATIVE_RESPONSE_CARD_FAIL           3
#define NEGATIVE_RESPONSE_CARD_JAM            4
#define NEGATIVE_RESPONSE_DATA_FAIL           5
#define NEGATIVE_RESPONSE_TIME_OUT            6
#define NEGATIVE_RESPONSE_MS_BLANK_ERROR      8
#define NEGATIVE_RESPONSE_MS_PREAMBLE_ERROR   9
#define NEGATIVE_RESPONSE_MS_PARITY_ERROR     10
#define NEGATIVE_RESPONSE_MS_POSTAMBLE_ERROR  11
#define NEGATIVE_RESPONSE_MS_LRC_ERROR        12
#define NEGATIVE_RESPONSE_ICC_CONTACT_ERROR   14
#define NEGATIVE_RESPONSE_ICC_CONTROL_ERROR   15
#define NEGATIVE_RESPONSE_COMMAND_CANCEL      16
#define NEGATIVE_RESPONSE_EEPROM_ERROR        18

#define SPOT_STX 0
#define SPOT_LEN_H 1
#define SPOT_LEN_L 2
#define SPOT_RESPONSE_TYPE 3
#define SPOT_STAT 4
#define SPOT_DATA_START 5
#define SPOT_ST1 5
#define SPOT_ST2 6
#define SPOT_ETX (dataLength + 3)
#define SPOT_BCC (dataLength + 4)

#define getResponseLength() (responseData[SPOT_LEN_H] << 8 | responseData[SPOT_LEN_L])

//White wire is TX from reader and RX on Arduino, Gray is RX into reader and TX from Arduino
SoftwareSerial emv(2, 3, true); //True tells library to invert signal and allows us to fake RS232 communication

byte responseData[128]; //The track data can be up to 116 bytes (perhaps more?)

void setup()
{
  Serial.begin(115200);
  emv.begin(19200);
}

void loop()
{
  Serial.println();
  Serial.println(F("EMV controller"));
  Serial.println(F("S)tatus"));
  Serial.println(F("E)ject"));
  Serial.println(F("M)ag stripe"));
  Serial.println(F("I)CC Reset"));
  Serial.print(F(">"));

  while (Serial.available() == false) ; //Wait for user input

  byte incoming = tolower(Serial.read());

  if (incoming == 's')
  {
    if (getStatus() == true) printResponse();
    else Serial.println(F("Get Status Failed"));
  }
  else if (incoming == 'e')
  {
    Serial.println(F("Eject card"));
    if (ejectCard() == true) printResponse();
    else Serial.println(F("Eject card failed"));
  }
  else if (incoming == 'm')
  {
    Serial.println(F("Mag Stripe Data:"));
    if (readMagStripeData() == true) printMagStripeData();
    else Serial.println(F("Mag strip read failed"));
  }
  else if (incoming == 'i')
  {
    Serial.println(F("ICC Reset and ATR:"));
    if (iccReset() == true) printATR();
    else Serial.println(F("ICC reset failed"));
  }
  else
  {
    Serial.print(F("Unknown command: "));
    Serial.print(incoming);
  }
}

//Pretty print the ICC ATR
void printATR()
{
  int dataLength = getResponseLength();
  Serial.println();

  Serial.print(F("ATR: 0x "));
  for (int x = SPOT_DATA_START ; x < SPOT_ETX ; x++)
  {
    if(responseData[x] < 0x10) Serial.print("0");
    Serial.print(responseData[x], HEX);
    Serial.print(" ");
  }
  Serial.println();

}

//Pretty prints the mag stripe data
void printMagStripeData()
{
  int dataLength = getResponseLength();
  Serial.println();

  Serial.print(F("Track 1: "));
  byte trackNumber = 2;
  for (int x = SPOT_DATA_START ; x < SPOT_ETX ; x++)
  {
    if (responseData[x] == 0x00) //Look for track delimiter
    {
      Serial.println();
      Serial.print(F("Track "));
      Serial.print(trackNumber++);
      Serial.print(F(": "));
    }
    else
    {
      Serial.write(responseData[x]);
    }
  }
  Serial.println();
}

//Read the magnetic data
//Returns true if responseData is correctly filled
boolean readMagStripeData()
{
  //Make sure we have mag data available
  if (getStatus() == false || responseData[SPOT_RESPONSE_TYPE] != 'P')
  {
    Serial.println(F("Mag stripe failed to read. Please check connection to reader."));
    return (false);
  }
  if (responseData[SPOT_STAT] & (1 << 4) == 0)
  {
    Serial.println(F("No mag stripe data available. Please insert card."));
    return (false);
  }

  sendCommand('M');
  return (receiveFrame()); //Get response
}

//If a card is present, eject it
boolean ejectCard()
{
  sendCommand('E');
  return (receiveFrame()); //Get response
}

//Gets the current status of the reader
//Returns true if response was clean
boolean getStatus()
{
  sendCommand('S');
  return (receiveFrame()); //Get response
}

//Gets the current status of the reader
//Returns true if response was clean
boolean iccReset()
{
  sendCommand('R');
  return (receiveFrame()); //Get response
}

//If we need to send a command with no data, this does it for us
//Send array of one byte
boolean sendCommand(byte command)
{
  byte data[1] = {command};
  sendFrame(data, sizeof(data));
}

//Turns the status byte or negative response into human readable
void printResponse()
{
  //Serial.print("Res: ");
  //Serial.println(response);

  if (responseData[SPOT_RESPONSE_TYPE] == 'P')
  {
    byte positiveResponse = responseData[SPOT_STAT];
    Serial.println(F("Positive Response / Unit Status:"));
    if (positiveResponse & (1 << 7)) Serial.println(F("Rear detect"));
    if (positiveResponse & (1 << 6)) Serial.println(F("Front detect"));
    if (positiveResponse & (1 << 5)) Serial.println(F("IC Reset On"));
    if (positiveResponse & (1 << 4)) Serial.println(F("Is M/S Data"));
    if (positiveResponse & (1 << 3)) Serial.println(F("M/S Forward Read"));
    if (positiveResponse & (1 << 1)) Serial.println(F("Is SAM2"));
    if (positiveResponse & (1 << 0)) Serial.println(F("Is SAM1"));
    if (positiveResponse == 0) Serial.println(F("No status"));
  }
  else if (responseData[SPOT_RESPONSE_TYPE] == 'N')
  {
    byte ST1 = responseData[SPOT_ST1] - '0';
    byte ST2 = responseData[SPOT_ST2] - '0';
    byte negativeResponse = (ST1 * 10 + ST2);

    //Negative response codes
    if (negativeResponse == NEGATIVE_RESPONSE_NOT_DEFINED) Serial.println(F("Command Not Defined"));
    else if (negativeResponse == NEGATIVE_RESPONSE_NO_CARD) Serial.println(F("No Card"));
    else if (negativeResponse == NEGATIVE_RESPONSE_CARD_FAIL) Serial.println(F("Card Fail"));
    else if (negativeResponse == NEGATIVE_RESPONSE_CARD_JAM) Serial.println(F("Card Jam"));
    else if (negativeResponse == NEGATIVE_RESPONSE_DATA_FAIL) Serial.println(F("Data Fail"));
    else if (negativeResponse == NEGATIVE_RESPONSE_TIME_OUT) Serial.println(F("Time Out"));
    else if (negativeResponse == NEGATIVE_RESPONSE_MS_BLANK_ERROR) Serial.println(F("M/S Blank Error"));
    else if (negativeResponse == NEGATIVE_RESPONSE_MS_PREAMBLE_ERROR) Serial.println(F("M/S Preamble Error"));
    else if (negativeResponse == NEGATIVE_RESPONSE_MS_PARITY_ERROR) Serial.println(F("M/S Parity Error"));
    else if (negativeResponse == NEGATIVE_RESPONSE_MS_POSTAMBLE_ERROR) Serial.println(F("M/S Postamble Error"));
    else if (negativeResponse == NEGATIVE_RESPONSE_MS_LRC_ERROR) Serial.println(F("M/S LRC Error"));
    else if (negativeResponse == NEGATIVE_RESPONSE_ICC_CONTACT_ERROR) Serial.println(F("IC Card Contact Error"));
    else if (negativeResponse == NEGATIVE_RESPONSE_ICC_CONTROL_ERROR) Serial.println(F("IC Card Control Error"));
    else if (negativeResponse == NEGATIVE_RESPONSE_COMMAND_CANCEL) Serial.println(F("Command Cancel"));
    else if (negativeResponse == NEGATIVE_RESPONSE_EEPROM_ERROR) Serial.println(F("EEPROM Error"));
    else Serial.println(F("Unknown error"));
  }
  else
  {
    Serial.println(F("Unknown response type"));
  }
}

//Receives a frame reponse from the EMV reader and stores it into a global array responseData[]
//Returns false if STX or ETX are missing, or BCC is corrupt
//Returns true if responseData is clean
//responseData is ('P' + STAT + Data) or ('N' + ST1 + ST2)
boolean receiveFrame()
{
  //Some responses take up to 250ms
  byte counter = 0;
  while (emv.available() == false)
  {
    if (counter++ > 250) return (false);
    delay(1);
  }

  //Suck in data until either we get the Len_H+L number of bytes or timeout
  int dataLength = 128; //By default, go big
  int spot = 0;
  for ( ; spot < sizeof(responseData) ; spot++)
  {
    //Watch for timeout
    byte counter = 0;
    while (emv.available() == false)
    {
      if (counter++ > 100)
      {
        Serial.println(F("Response timed out"));
        return (false); //EMV reader timed out before completing response
      }
      delay(1);
    }

    responseData[spot] = emv.read(); //Add this data to the global array

    if (spot == SPOT_BCC) break; //We have received all we are going to

    //Parse data length when it comes in
    if (spot == 3)
    {
      dataLength = getResponseLength();
    }
  }

  if (spot == sizeof(responseData))
  {
    Serial.println(F("Overflow: Increase responseData size."));
    return (false);
  }

  //Error check response
  if (responseData[SPOT_STX] != CONTROL_STX || responseData[SPOT_ETX] != CONTROL_ETX)
  {
    Serial.println("STX or ETX is missing");
    return (false);
  }

  //Calculate BCC
  byte calculatedBCC = 0;
  for (int x = 1 ; x < SPOT_BCC ; x++)
  {
    calculatedBCC ^= responseData[x];
  }

  if (calculatedBCC != responseData[SPOT_BCC])
  {
    Serial.println("BCC incorrect");
    return (false);
  }

  return (true);
}

//Sends a frame containing header bytes, and calculates the BCC
void sendFrame(byte *data, int lengthOfData)
{
  //Clear incoming buffer
  while (emv.available())
    emv.read();

  //Calculate BCC starting with last byte (always CONTROL_ETX)
  byte BCC = CONTROL_ETX;
  //Add length bytes
  BCC ^= (byte)(lengthOfData >> 8);
  BCC ^= (byte)(lengthOfData & 0xFF);

  //Add data bytes
  for (int x = 0 ; x < lengthOfData ; x++)
    BCC ^= data[x];

  //Send frame
  emv.write(CONTROL_STX);
  emv.write(byte(lengthOfData >> 8));
  emv.write(byte(lengthOfData & 0xFF));

  for (int x = 0 ; x < lengthOfData ; x++)
    emv.write(data[x]);

  emv.write(CONTROL_ETX);
  emv.write(BCC);
}


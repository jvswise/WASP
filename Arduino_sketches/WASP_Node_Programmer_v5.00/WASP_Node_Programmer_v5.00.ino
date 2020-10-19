/*******************************************************************************
 * Wireless Addressable Strings of Pixels (WASP) wireless programmer for slaves
 * 
 * Wireless programming gateway node for WASP slave nodes.
 * 
 * This sketch implements a wireless programming gateway to support wireless
 * upgrades to WASP slave nodes. (Note that the slave nodes must include
 * the add-on 4 MiB (or greater) Flash memory chip.) It is an intermediary
 * between a programmer running on a PC and the WASP slaves. The PC-based
 * programmer is typically going to be the Moteino Wireless "OTA" Programmer
 * (WirelessProgramming.exe) application written by LowPowerLab. LowPowerLab
 * has also included some Python scriptware that could be used in Windows,
 * Linux, or on a MAC; however, I haven't tested the scriptware.
 * 
 * To wirelessly program the WASP slave node:
 *   IMPORTANT: Ensure that both the slave node and its new hex loadfile
 *              contain the appropriate wireless programming protocol code
 *              (e.g. CheckForWirelessHEX()). Otherwise you won't be able to
 *              upload the hex file now or after upgrade, respectively.
 *   - Connect the PC to the gateway's serial COM port.
 *   - The programmer gateway's serial port baud rate must be set to 115200.
 *   - Using the Arduino IDE, open the new sketch to be compiled for upload.
 *       - Select menu item Sketch > Export compiled Binary
 *       - The resulting .hex files (one of them includes "with_bootloader" in
 *         its name--ignore that one) will be located in the sketch's folder
 *         (along with its .ino Arduino source code file).
 *   - Startup the WirelessProgramming.exe Windows programmer application. A
 *     copy is located under folder
 *       .../sketchbook/Xmas_Pixel_Lighting/Wireless_Programming_Support_Files/
 *       
 *       - Click the "Browse Hex file" button and choose the exported .hex file.
 *       - Select the gateway's COM port and WASP slave node ID 77.
 *       - Click the "Start!" button.
 *       - Because the gateway outputs help text for someone opening the
 *         console, the first "Start!" may fail. Simply retry.
 *       
 * NOTES:
 *   - The WASP slaves must have LowPowerLab's DualOptiboot bootloader
 *     installed; they are sold with the bootloader already installed. Without
 *     that bootloader, the slave cannot load a new sketch from the add-on
 *     Flash memory.
 *   - The programming gateway node requires neither the add-on Flash memory,
 *     nor the custon Optiboot.
 *   - This gateway code can run on any Moteino board that has an RFM69 radio.
 * 
 *
 * History
 * =======
 * Version  Date     Author         Description
 * -------  -------- -------------- -------------------------------------
 * 0.01     18-11-23 J.van Schouwen Initial creation.
 * 1.00     18-11-23 J.van Schouwen Prototype v0.01 released as v1.00.
 * 5.00     19-03-29 J.van Schouwen Re-release of v1.00 as v5.00
 ******************************************************************************/

//Ruler
//345678901234567890123456789012345678901234567890123456789012345678901234567890

#include <RFM69.h>    //get it here: https://github.com/lowpowerlab/RFM69
#include <RFM69_ATC.h>//get it here: https://github.com/lowpowerlab/RFM69
#include <RFM69_OTA.h>//get it here: https://github.com/lowpowerlab/RFM69
#include <SPIFlash.h> //get it here: https://www.github.com/lowpowerlab/spiflash
#include <SPI.h>      //included with Arduino IDE (www.arduino.cc)

#define COPYRIGHT     "(C)2018, A.J. van Schouwen"
#define SW_VERSION_c  "5.00 (2019-03-29)"

//******************************************************************************
//**** IMPORTANT RADIO SETTINGS - YOU MUST CHANGE/CONFIGURE TO MATCH YOUR
//**** HARDWARE TRANSCEIVER CONFIGURATION!
//******************************************************************************
#define NODEID             254  // WASP RFM69 node ID
#define NETWORKID           77  // WASP NETWORKID
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
#define FREQUENCY   RF69_433MHZ

// NOTE: Encryption isn't currently being used.
#define ENCRYPTKEY "JVS_WASP_Key3456"    // exactly 16 chars: same on all nodes
//#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!

#define DEBUG_MODE false         //set 'true' to see verbose output from programming sequence

#define SERIAL_BAUD 115200
#define ACK_TIME    50  // # of ms to wait for an ack
#define TIMEOUT     3000

#define ASCII_NL  0x0A  // ASCII newline character

// Macro for defining strings that are stored in flash (program) memory rather
// than in RAM. Arduino defines the non-descript F("string") syntax.
#define FLASH(x) F(x)


RFM69 radio;
char c = 0;
char input[64]; //serial input buffer
byte targetID=0;



void Blink(byte pin, int delay_ms)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(delay_ms);
  digitalWrite(pin,LOW);
}


void setup()
{
  Serial.begin(SERIAL_BAUD);
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  //radio.encrypt(ENCRYPTKEY);

  Serial.println();
  Serial.print(FLASH("WASP network"));
  Serial.print(FLASH("...   Node #"));
  Serial.print(NODEID);
  Serial.println(FLASH("  (Wireless Programming Gateway)"));
  Serial.print(FLASH("WASP programming gateway S/W: "));
  Serial.print(SW_VERSION_c);
  Serial.print(FLASH("\t    "));
  Serial.println(COPYRIGHT);
  Serial.print(FLASH("Radio frequency: "));
  Serial.print(FREQUENCY==RF69_433MHZ ? 433 :
                 FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(FLASH("Mhz"));
  Serial.println();
  Serial.println(FLASH("To wirelessly program a WASP node:"));
  Serial.println(FLASH("- Using the Arduino IDE, open the new sketch to be "
                         "compiled for upload."));
  Serial.println(FLASH("- Select menu item Sketch > Export compiled Binary."));
  Serial.println(FLASH("- The resulting .hex files (one of them includes "
                          "'with_bootloader' in"));
  Serial.println(FLASH("  its name--ignore that one) will be located in the "
                         "sketch's folder"));
  Serial.println(FLASH("  (along with its .ino Arduino source code file)."));
  Serial.println(FLASH("- Before upgrading the WASP slave:"));
  Serial.println(FLASH("    - Make sure the WASP controller isn't sending WASP "
                             "commands--e.g. power"));
  Serial.println(FLASH("      it off."));
  Serial.println(FLASH("    - Ensure that this console is closed so that the "
                             "programmer"));
  Serial.println(FLASH("      application can use it."));
  Serial.println(FLASH("- Startup the WirelessProgramming.exe Windows "
                         "programmer application. A"));
  Serial.println(FLASH("  copy is located under folder"));
  Serial.println(FLASH("  .../sketchbook/Xmas_Pixel_Lighting/"
                         "Wireless_Programming_Support_Files/"));
  Serial.println(FLASH("    - Click the 'Browse Hex file' button and choose "
                             "the exported .hex file."));
  Serial.println(FLASH("    - Select the gateway's COM port and WASP slave "
                             "node ID."));
  Serial.println(FLASH("    - Click the 'Start!' button."));
  Serial.println(FLASH("    - Due to this help text, the first 'Start!' may "
                             "fail: Just retry."));
}

void loop()
{
  // readSerialLine(char* input, char endOfLineChar=10, byte maxLength=64,
  //                uint16_t timeout=1000);
  byte inputLen = readSerialLine(input, ASCII_NL, RF69_MAX_DATA_LEN, 100); 
  
  if ( inputLen==4 && input[0]=='F' && input[1]=='L' &&
       input[2]=='X' && input[3]=='?'
     )
  {
    if (0 == targetID)
      Serial.println("TO?");
    else
      CheckForSerialHEX( (byte*)input, inputLen, radio, targetID,
                          TIMEOUT, ACK_TIME, DEBUG_MODE );
  }
  else if ( inputLen>3 && inputLen<=6 && input[0]=='T' &&
            input[1]=='O' && input[2]==':'
          )
  {
    byte newTarget = 0;
    
    for (byte i = 3; i < inputLen; i++) //up to 3 characters for target ID
    {
      if (input[i] >= '0' && input[i] <= '9')
        newTarget = newTarget * 10 + (input[i] - '0');
      else
      {
        newTarget = 0;
        break;
      }
    }
    
    if (newTarget > 0)
    {
      targetID = newTarget;
      Serial.print("TO:");
      Serial.print(newTarget);
      Serial.println(":OK");
    }
    else
    {
      Serial.print(input);
      Serial.print(":INV");
    }
  }
  else if (inputLen>0) { //just echo back
    Serial.print("SERIAL IN > ");Serial.println(input);
  }

  if (radio.receiveDone())
  {
    for (byte i = 0; i < radio.DATALEN; i++)
    {
      Serial.print((char)radio.DATA[i]);
    }
    
    if (radio.ACK_REQUESTED)
    {
      radio.sendACK();
      Serial.print(" - ACK sent");
    }
    
    Serial.println();
  }
  Blink(LED_BUILTIN, 50); //heartbeat
}

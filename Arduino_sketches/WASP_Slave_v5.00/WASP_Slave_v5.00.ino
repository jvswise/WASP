/*******************************************************************************
 * Wireless Addressable Strings of Pixels (WASP) slave node
 *
 * Remote node to addressable LED lighting strip.
 *
 * This sketch implements a remote slave node that interfaces with a special
 * RGB addressable LED (pixel) strip and with a master multi-strip controller
 * node. It is intended to be executed on a (Arduino compatible) Moteino MPU.
 * 
 * The slave implementation is compatible with both regular Moteino boards
 * and Moteino Mega boards.
 * 
 * NOTES:
 *  - Regular Moteino nodes support only up to about 100 LED pixels; this is
 *    constrained by the limited onboard 2 KiB of RAM. A Moteino Mega (with
 *    its 16 KiB of RAM) can support more nodes.
 *  - When using multiple pixel strings, power each string separately. Each
 *    string causes a significant voltage drop that can't be tolerated by
 *    the next string. The usual three wires (ground, +ve, data) can be fed
 *    thru, but each string should get a direct ground and +ve positive feed
 *    from a power supply. (They could use the same power support, but would
 *    at least need separate, low voltage drop feeds.)
 *  - If you wish to be able to wirelessly reprogram the slave, the board
 *    must include the additional add-on Flash memory (4 MiB, or greater)
 *    chip.
 *  - Wireless programming capability uses up an additional ~5.1 KiB of
 *    program storage.
 *  - External add-on Flash memory chips use up the following Moteino pins:
 *       Regular Moteino:  D8  (chip select), D11 - D13
 *       Moteino Mega:     D23 (chip select), D5  - D7
 * History
 * =======
 * Version  Date     Author         Description
 * -------  -------- -------------- -------------------------------------
 * 0.1      18-03-31 J.van Schouwen Initial creation.
 * 0.2      18-04-05 J.van Schouwen Added: multi-slave support for SHIFT;
 *                                    STATE; more console commands.
 * 0.3      18-04-10 J.van Schouwen Some tuning.
 * 0.4      18-09-25 J.van Schouwen Integrate new, reliable 2018 version of
 *                                    the RFM69 library.
 * 0.5      18-09-26 J.van Schouwen Added WASPCMD_RESET, WASPCMD_SPEED, and
 *                                    WASPCMD_RAINBOW handling.
 * 0.6      18-09-27 J.van Schouwen Added WASP slave node groups and
 *                                    WASPCMD_RAINCYCLE handling.
 * 0.7      18-11-01 J.van Schouwen Fixed bug in SUSPEND/RESUME handling.
 * 0.8      18-11-02 J.van Schouwen Minor corrections to comments.
 * 1.00     18-11-02 J.van Schouwen Prototype v0.8 release as v1.00
 * 1.01     18-11-13 J.van Schouwen Minor change to console's copyright banner.
 * 2.00     18-11-16 J.van Schouwen Prototype v1.01 released as v2.00.
 * 2.01     18-11-23 J.van Schouwen Added support for wireless programming
 *                                    (requires add-on 4 MiB Flash memory chip).
 * 3.00     18-11-23 J.van Schouwen Prototype v2.01 release as v3.00. This
 *                                    release can run on all Moteino boards.
 * 3.01     19-02-22 J.van Schouwen Added power and WASP Rx/Tx activity
 *                                    indicator LEDS. Changed m_savedLeds[] to
 *                                    a dynamically allocated array sized to
 *                                    m_ledStripLen rather than MAX_PIXELS.
 * 4.00     19-03-10 J.van Schouwen Prototype v3.01 release as v4.00.
 * 4.01     19-03-22 J.van Schouwen Shortened the slave Tx time window length
 *                                    for SWAP command handling.
 * 4.02     19-03-29 J.van Schouwen Added PING and slave configuration
 *                                    commands, and function RamSize() which
 *                                    returns total SRAM size.
 * 5.00     19-03-29 J.van Schouwen Prototype v4.02 released as v5.00
 *
 ******************************************************************************/

//Ruler
//345678901234567890123456789012345678901234567890123456789012345678901234567890

#include <RFM69.h>
#include <RFM69_OTA.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <SPIFlash.h>
#include <SPI.h>
#include "WASP_defs.h"
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>

#define COPYRIGHT      "(C)2019, A.J. van Schouwen"
#define SW_DATE_c      "(2019-03-29)"  // Version 5.00
#define SW_VERS_MAJ_c  5   // This is major S/W version
#define SW_VERS_MIN_c  0   // This is the minor S/W version (in hundreths)
#define FW_VERSION_c   3   // Increment (with wraparound) for new F/W;
                           //   clears EEPROM.

#define CONSOLE_ENABLED     // Uncomment to enable the console
#ifdef CONSOLE_ENABLED
  //#define DEBUG_ON            // Uncomment to turn off debug output to serial port.
  #define LOGGING_ON          // Uncomment to turn off logging to serial port.
  #define SERIAL_CMDS_ENABLED // Enables command line at serial port
#endif

// I/O pin definitions
#define NEO_PIN          3 // Default digital pin for LED strip control
                           // (overidden by m_ledStripCtrlPin)
#define POWER_LED       17 // Blinking power indicator LED
#define WASP_TX_LED     18 // Strobing WASP Tx activity LED
#define WASP_RX_LED     19 // Strobing WASP Rx activity LED
#define MAX_DIO_PIN     19 // Moteino has digitial I/O pins 0 - 19
#define MAX_MEGADIO_PIN 31 // Moteino Mega has digital I/O pins 0 - 31

#define LEDS_PER_PIX  3   // 3 LEDs (colours) per pixel.

// LED strip operating frequency indicies
#define FREQ_IDX_400kHz   4
#define FREQ_IDX_800kHz   8

// LED strip colour wiring byte data order.
#define WIRING_RGB    0
#define WIRING_RBG    1
#define WIRING_GRB    2
#define WIRING_GBR    3
#define WIRING_BRG    4
#define WIRING_BGR    5
#define WIRING_MAX_VAL (WIRING_BGR)

// Define the Moteino pin used for Flash memory chip select.
#ifdef __AVR_ATmega1284P__
  #define FLASH_SS      23   // FLASH SS on D23
  #define IS_MEGA    (true)  // This is a Moteino Mega
  #define MAX_PIXELS   255   // Max supported pixels (yet to be validated).
                             //  - More might be supportable, but the string
                             //    length is currently a uint8 value.
#else
  #define FLASH_SS       8   // FLASH SS on D8
  #define IS_MEGA   (false)  // This is NOT a Moteino Mega
  #define MAX_PIXELS   100   // Maximum supported pixels
#endif

#define MAX_SERIAL_BUF_LEN           20
#define EEPROM_FW_ADDR                0
#define EEPROM_NODEID_ADDR            1
#define EEPROM_VALIDITY_ADDR          2
#define EEPROM_STRIP_CTRL_PIN         3
#define EEPROM_STRIP_FREQ_ADDR        4
#define EEPROM_STRIP_WIRING_ADDR      5
#define EEPROM_STRIP_LEN_ADDR         6  // 16-bit value
#define EEPROM_FIRST_OPEN_ADDR        (EEPROM_STRIP_LEN_ADDR + 2)
#define EEPROM_RESET_COUNT_ADDR    1023

#define SERIAL_BAUD                9600

#define DEL_CHAR                   0x7F  // ASCII Del character


// Command processing stages
#define CMD_PROMPT        0  // Display command prompt
#define CMD_INPUT         1  // Scan for complete line of input
#define CMD_PARSE         2  // Parse input line for command name
#define CMD_EXEC          3  // Execute the command


// Pixel shifting stages
#define PX_SHIFT_START    0  // Await start of shift command; shift out pixels
#define PX_SHIFT_WAIT1    1  // Await neighbor response before my response
#define PX_SHIFT_RESP     2  // Report my out-shifted pixels to neighbor
#define PX_SHIFT_WAIT2    3  // Await neighbor response following my response
#define PX_SHIFT_IN       4  // Shift in neighbor's pixels.


// Execution timer identifiers
#define TMR_BASELINE      0  // OMET just to update and OMET value.
#define TMR_CMD_SCAN      1  // Console input scanning
#define TMR_CMD_PARSE     2  // Console command parsing
#define TMR_CMD_EXEC      3  // Console command execution
#define TMR_CONSOLE       4  // Overall console input processing (scan,
                             //   parse, exec)
#define TMR_RADIO_IN      5  // Radio input processing
#define TMR_SHFT_STRT     6  // SHIFT cmd: START state processing
#define TMR_SHFT_WT1      7  // SHIFT cmd: WAIT1 state processing
#define TMR_SHFT_TX0      8  // SHIFT cmd: RESP Tx start elapsed
#define TMR_SHFT_TX1      9  // SHIFT cmd: RESP Tx end elapsed
#define TMR_SHFT_RSP     10  // SHIFT cmd: RESP state processing
#define TMR_SHFT_WT2     11  // SHIFT cmd: WAIT2 state processing
#define TMR_WASP_EXEC    12  // WASP command processing time
#define TMR_FX_EXEC      13  // Special FX execution time
#define TMR_PXL_UPD      14  // Pixel update (show())
#define TMR_LOOP         (MAX_TIMINGS - 1) // Last execution timer

#define MAX_TIMINGS      16

#define MAX_TX_EXEC_MS    5  // Max Tx time (in milliseconds)


// Macro for defining strings that are stored in flash (program) memory rather
// than in RAM. Arduino defines the non-descript F("string") syntax.
#define FLASH(x) F(x)


#ifdef DEBUG_ON
  #define dbgPrint(x)    Serial.print(x)
  #define dbgPrintln(x)  Serial.println(x)
#else
  #define dbgPrint(x)
  #define dbgPrintln(x)
#endif

#ifdef LOGGING_ON
  #define logPrint(x)        Serial.print(x)
  #define logPrintln(x)      Serial.println(x)
  #define logPrintEx(x,y)    Serial.print(x,y)
  #define logPrintlnEx(x,y) Serial.println(x,y)
#else
  #define logPrint(x)
  #define logPrintln(x)
  #define logPrintEx(x,y)
  #define logPrintlnEx(x,y)
#endif


/*-------------------  Device startup and configuration state  --------------*/
boolean  m_resetEeprom = false;
boolean  m_resetRequired = false;
uint8    m_resetCount;


/*-----------------------------  RFM69 State  --------------------------------*/
// Node ID.
// Must be unique for each node. Gateway is always 1.
// Range: 2 - 20  (0 = 'undefined')
uint8    m_myNodeId = NODEID_UNDEF;  // Must be unique for each node 

//*****************************************************************************************************************************
// flash(SPI_CS, MANUFACTURER_ID)
// SPI_CS          - Chip Select pin attached to SPI flash chip.
// MANUFACTURER_ID - OPTIONAL, 0x1F44 for adesto(ex atmel) 4mbit flash
//                             0xEF30 for windbond 4mbit flash
//                             0xEF40 for windbond 16/64mbit flash
//*****************************************************************************************************************************
SPIFlash m_flash(FLASH_SS, 0xEF30);  // EF30 for windbond 4mbit flash

RFM69    m_radio;
boolean  m_promiscuousMode = true;  // sniff all packets on network iff true
uint8    m_dstNodeId;
uint8    m_srcNodeId;
uint16   m_myRespDelay = 0;

uint8    m_radioInBuf[RF69_MAX_DATA_LEN];
uint8    m_radioInBufPos = 0;
uint8    m_radioOutBuf[RF69_MAX_DATA_LEN];
uint8    m_radioOutBufPos = 0;

boolean  m_ackRequested = false;
uint32   m_txLate = 0;


/*------------------------------  WASP State ---------------------------------*/
uint8    m_waspCmd = WASPCMD_NONE;
boolean  m_waspRunning = true;
uint8    m_myGroupId = NODEID_UNDEF;
uint8    m_leftNeighbour  = NODEID_UNDEF;
uint8    m_rightNeighbour = NODEID_UNDEF;
boolean  m_selfShift;
uint32   m_waspExecStart;


/*--------------------------  Console State  ---------------------------------*/
char     m_consoleBuffer[MAX_SERIAL_BUF_LEN];
uint8    m_consolePos = 0;


/*----------------------  Execution Timing State  ----------------------------*/
int32    m_timings[MAX_TIMINGS];
uint32   m_loopStartTime = 0;
uint32   m_loopRefTime = 0;
uint32   m_tmrUpdOverhead = 0;   // Execution time to call tmrUpdateOmet()
int32    m_baseTmrOverhead = 0;


/*---------------------------  Delay Timing State  ---------------------------*/
uint32   m_myRespStartTime = 0;
uint32   m_myRespDeadline = 0;


/*--------------------------  LED Indicator States  ---------------------------*/
uint32   m_loopStartMs = 0;
uint32   m_powerLedOnMs = 0;
uint32   m_powerLedOffMs = 0;


/*---------------------------  LED Pixels State  -----------------------------*/
Adafruit_NeoPixel *m_pStrip;
// IMPORTANT [Adafruit note]: To reduce NeoPixel burnout risk, add 1000 uF
// capacitor across pixel power leads, add 300 - 500 Ohm resistor on first
// pixel's data input and minimize distance between Arduino and first pixel.
// Avoid connecting on a live circuit...if you must, connect GND first.

uint8   *m_pPixels;
uint8   *m_savedLeds;  // Dynamically allocated array for saving pixel states.
uint8    m_shiftInLeds[MAX_SHIFT_SIZE * LEDS_PER_PIX];
uint8    m_shiftOutLeds[MAX_SHIFT_SIZE * LEDS_PER_PIX];
uint16   m_numPixelBytes;
uint8    m_offsRed   = 0; // Pixel byte data ordering.
uint8    m_offsGreen = 1;
uint8    m_offsBlue  = 2;
boolean  m_pixelShowSuspend = false;
boolean  m_updatePixels = false;


// Animated Local Effect parameters
WaspCmd_t  m_runningFx = WASPCMD_NONE;
uint16     m_fxDelay = 0;
boolean    m_runAnimation = false;
boolean    m_fxStep = true;  // Execute a single special F/X iteration.
boolean    m_fxRestart = true;
uint32     m_fxResumeTime = 0;
uint32     m_fxParam1 = 0;
uint32     m_fxParam2 = 0;
uint32     m_fxParam3 = 0;
uint32     m_fxParam4 = 0;


// LED strip parameters
uint8    m_ledStripFlags;

// LED strip configuration settings
int8     m_ledStripCtrlPin; // Configured output pin for LED strip control
                            //   (Default is NEO_PIN.)
                         
uint8    m_ledStripFreq;    // Configured LED strip frequency
  //   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
  //   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)

uint8    m_ledStripWiring;  // Configured LED strip colour wiring order (RGB,RBG,...)

uint16   m_ledStripLen;     // # of LED triplets (12V strip has 3 tricolour
                            //   LEDs per WS2811 driver). So the LEDs, in this
                            //   configuration, aren't really individually
                            //   addressable.

/* The following table defines which digital I/O pins can be allocated
 * for LED strip control (see m_ledStripCtrlPin). Entries that are false
 * are I/O pins that cannot be used for LED strip control because
 * they conflict with pre-determined pin allocations.
 * NOTES:
 *   - Moteino and Moteino Mega have different allocations for the radio, etc.
 *   - Moteino's highest digital I/O pin number is 19.
 *   - Digital pin numbers annotated with '*' are those that are allocated
 *     by WASP slave software already.
 */
const uint8 m_ledStripCtrlOk[MAX_MEGADIO_PIN + 1] =
    // D0     D1     D2     D3     D4     D5     D6     D7     D8     D9
  {   false, false, false,  true, false, false, false, false, false,  true,
  
    // D10    D11    D12    D13    D14    D15    D16    D17*   D18*   D19*
      false, false, false, false,  true,  true,  true, false, false, false,
      
    // D20    D21    D22    D23    D24    D25    D26    D27    D28    D29
       true,  true,  true, false,  true,  true,  true,  true,  true,  true,
      
    // D30    D31
       true,  true
  };
    


/*----------------  Console commands and command handlers  -------------------*/
// cmdHdlrFunc_t
//   - The command handler must parse the command line for parameters.
//   - The function must return true only when it has completed its
//     handling.
//   - Command handling must be done quickly, in stages if necessary.
typedef boolean cmdHdlrFunc_t(void);

typedef struct
{
  char          *cmdName;
  cmdHdlrFunc_t *cmdHdlr;
  
} CmdCtrl_t;

boolean cmdHdlrHelp(void);
boolean cmdHdlrLed(void);
boolean cmdHdlrLedCtrl(void);
boolean cmdHdlrNodeId(void);
boolean cmdHdlrPerf(void);
boolean cmdHdlrReset(void);
boolean cmdHdlrRun(void);
boolean cmdHdlrStop(void);
boolean cmdHdlSave(void);
boolean cmdHdlrStop(void);

// Command control data. Sort the table alphabetically by command name.
CmdCtrl_t m_cmdCtrl[] =
  {/*   Cmd Name      Handler
        -----------   ----------------- */
      { "h",          cmdHdlrHelp      },
      { "led",        cmdHdlrLed       },
      { "ledCtrl",    cmdHdlrLedCtrl   },
      { "nodeid",     cmdHdlrNodeId    },
      { "perf",       cmdHdlrPerf      },
      { "reset",      cmdHdlrReset     },
      { "run",        cmdHdlrRun       },
      { "s",          cmdHdlrStop      },
      { "save",       cmdHdlSave       },
      { "stop",       cmdHdlrStop      }
  };

const uint8 m_numCmds = sizeof(m_cmdCtrl)/sizeof(CmdCtrl_t);


/*--------------------------  WASP command handlers  -------------------------*/
// PxCmdHdlr_t
//   - The WASP command handler must parse the command line for parameters
//     contained in m_radioInBuf[].
//   - The function must return a WaspCmd_t value. Normally, when there is
//     no additional processing, the command should return WASPCMD_NONE. If
//     the command requires processing across multiple invocations, it should
//     return its own WASP command value.
//   - Command handling must be done quickly, in stages if necessary.
typedef WaspCmd_t PxCmdHdlr_t(void);

WaspCmd_t waspCmdHdlrNone(void);
WaspCmd_t waspCmdHdlrGroup(void);
WaspCmd_t waspCmdHdlrState(void);
WaspCmd_t waspCmdHdlrBackground(void);
WaspCmd_t waspCmdHdlrLine(void);
WaspCmd_t waspCmdHdlrShift(void);
WaspCmd_t waspCmdHdlrSwap(void);
WaspCmd_t waspCmdHdlrReset(void);
WaspCmd_t waspCmdHdlrSpeed(void);
WaspCmd_t waspCmdHdlrRainbow(void);
WaspCmd_t waspCmdHdlrRainCycle(void);
WaspCmd_t waspCmdHdlrTwinkle(void);
WaspCmd_t waspCmdHdlrPing(void);
WaspCmd_t waspCmdHdlrCfgNode(void);
WaspCmd_t waspCmdHdlrCfgCtrl(void);
WaspCmd_t waspCmdHdlrCfgLed(void);
WaspCmd_t waspCmdHdlrCfgSave(void);


// When updating the following, be sure to update m_pxFxHdlrs[] as well.
PxCmdHdlr_t *m_pxCmdHdlrs[MAX_WASPCMD_VAL + 1] =
  {
    waspCmdHdlrNone,          // WASPCMD_NONE
    waspCmdHdlrGroup,         // WASPCMD_GROUP
    waspCmdHdlrState,         // WASPCMD_STATE
    waspCmdHdlrBackground,    // WASPCMD_BKGRD
    waspCmdHdlrLine,          // WASPCMD_LINE
    waspCmdHdlrShift,         // WASPCMD_SHIFT
    waspCmdHdlrSwap,          // WASPCMD_SWAP
    waspCmdHdlrReset,         // WASPCMD_RESET
    waspCmdHdlrSpeed,         // WASPCMD_SPEED
    waspCmdHdlrRainbow,       // WASPCMD_RAINDBOW
    waspCmdHdlrRainCycle,     // WASPCMD_RAINCYCLE
    waspCmdHdlrTwinkle,       // WASPCMD_TWINKLE
    waspCmdHdlrPing,          // WASPCMD_PING
    waspCmdHdlrCfgNode,       // WASPCMD_CFG_NODE
    waspCmdHdlrCfgCtrl,       // WASPCMD_CFG_CTRL
    waspCmdHdlrCfgLed,        // WASPCMD_CFG_LED
    waspCmdHdlrCfgSave        // WASPCMD_CFG_SAVE
};


/*------------------------- Special FX Handlers -----------------------------*/
// This isn't the most economical way to implement these. But, this is
// good enough--at least, for now.
WaspCmd_t fxHdlrNull(void);
WaspCmd_t fxHdlrRainbow(void);
WaspCmd_t fxHdlrRainbowCycle(void);
WaspCmd_t fxHdlrTwinkle(void);

// When updating the following, be sure to update m_pxCmdHdlrs[] as well.
PxCmdHdlr_t *m_pxFxHdlrs[MAX_WASPCMD_VAL + 1] =
  {
    fxHdlrNull,               // WASPCMD_NONE
    fxHdlrNull,               // WASPCMD_NEIGHBOUR
    fxHdlrNull,               // WASPCMD_STATE
    fxHdlrNull,               // WASPCMD_BKGRD
    fxHdlrNull,               // WASPCMD_LINE
    fxHdlrNull,               // WASPCMD_SHIFT
    fxHdlrNull,               // WASPCMD_SWAP
    fxHdlrNull,               // WASPCMD_RESET
    fxHdlrNull,               // WASPCMD_SPEED
    fxHdlrRainbow,            // WASPCMD_RAINDBOW
    fxHdlrRainbowCycle,       // WASPCMD_RAINCYCLE
    fxHdlrTwinkle,            // WASPCMD_TWINKLE
    fxHdlrNull,               // WASPCMD_PING
    fxHdlrNull,               // WASPCMD_CFG_NODE
    fxHdlrNull,               // WASPCMD_CFG_CTRL
    fxHdlrNull,               // WASPCMD_CFG_LED
    fxHdlrNull                // WASPCMD_CFG_SAVE
  };



/*---------------------------  Forward declarations -------------------------*/
void CheckRam(void);
int  serialParseInt(void);




//-----------------------------------------------------------------------------
// Function: tmrUpdateOmet
//   Update a specified timer's per-loop Observed Maximum Execution Time
//   relative to the most recent value of m_loopRefTime.
// Parameters:
//   timerId:I  - Identifier of the timer to update (e.g. TMR_BASELINE).
// Returns: (none)
// Inputs/Outputs:
//   m_loopRefTime:I
//   m_timings:IO
//-----------------------------------------------------------------------------
static inline void tmrUpdateOmet(uint8 timerId)
{
  uint32   time;
  uint32   timeDelta = 0;
  int32   *pTimer;

  time = micros();
  if (time < m_loopRefTime)
    timeDelta = MAXUINT32 - m_loopRefTime + time;
  else
    timeDelta = time - m_loopRefTime;
  pTimer = &m_timings[timerId];
  if (timeDelta > *pTimer)
    *pTimer = timeDelta;
}
      
      
//-----------------------------------------------------------------------------
// Function: EepromLoad
//   Load EEPROM based parameters.
// Parameters: (none)
// Returns: (none)
//-----------------------------------------------------------------------------
void EepromLoad()
{
  int     addr;
  boolean dataIsValid;

  dataIsValid = EEPROM.read(EEPROM_VALIDITY_ADDR);
  m_ledStripCtrlPin = NEO_PIN;

  if (!dataIsValid)
    return;

  m_ledStripCtrlPin = EEPROM.read(EEPROM_STRIP_CTRL_PIN);
  m_ledStripFreq    = EEPROM.read(EEPROM_STRIP_FREQ_ADDR);
  m_ledStripWiring  = EEPROM.read(EEPROM_STRIP_WIRING_ADDR);
  m_ledStripLen     = ((uint16)EEPROM.read(EEPROM_STRIP_LEN_ADDR) << 8) +
                      EEPROM.read(EEPROM_STRIP_LEN_ADDR + 1);


  addr = EEPROM_FIRST_OPEN_ADDR;
  // Load additional parameters from this point onward.

  pinMode(m_ledStripCtrlPin, OUTPUT);
}


//-----------------------------------------------------------------------------
// Function: EepromSave
//   Save EEPROM based parameters.
// Parameters: (none)
// Returns: (none)
//-----------------------------------------------------------------------------
void EepromSave()
{
  const uint8 dataIsValid = 1;
  int addr;

  EEPROM.write(EEPROM_STRIP_CTRL_PIN,     m_ledStripCtrlPin);
  EEPROM.write(EEPROM_STRIP_FREQ_ADDR,    m_ledStripFreq);
  EEPROM.write(EEPROM_STRIP_WIRING_ADDR,  m_ledStripWiring);
  EEPROM.write(EEPROM_STRIP_LEN_ADDR,     ((m_ledStripLen & 0xFF00) >> 8));
  EEPROM.write(EEPROM_STRIP_LEN_ADDR + 1, (m_ledStripLen & 0x00FF));


  addr = EEPROM_FIRST_OPEN_ADDR;
  // Save additional parameters from this point onward.


  /* Once all data is written, validate it. */
  EEPROM.write(EEPROM_VALIDITY_ADDR,      dataIsValid);
}


//-----------------------------------------------------------------------------
// Function: CheckRam
//   Test how much RAM is left on the MPU, printing the results out to
//   the console serial port.
// Parameters: (none)
// Returns:    (none)
// Inputs/Outputs:
//   __bss_end:I
//   __brkval:I
//   m_consoleBuffer:IO
// Note:
//   IMPORTANT: Avoid calling this function during normal operation. This
//              should be treated as a temporary diagnostic test only. 
//-----------------------------------------------------------------------------
void CheckRam(void)
{
  extern int __bss_end;
  extern void *__brkval;
  /* Comment out either of the following to limit the CheckRam output */
  #define OUT_TO_SERIAL
  int freeValue;
  
  if ((int)__brkval == 0)
    freeValue = ((int)&freeValue) - ((int)&__bss_end);
  else
    freeValue = ((int)&freeValue) - ((int)__brkval);
    
  #ifdef OUT_TO_SERIAL
    Serial.print(F("Free RAM: "));
    Serial.println(freeValue);
  #endif
}


//-----------------------------------------------------------------------------
// Function: RamSize
//   Query the size of SRAM (in bytes).
// Parameters: (none)
// Returns:    Size of SRAM (in bytes)
// Inputs/Outputs:
//   __data_start:I
//   RAMEND:I
//-----------------------------------------------------------------------------
uint32 RamSize(void)
{
  extern unsigned int __data_start;
  
  return ((uint32)RAMEND - (uint32)&__data_start);
}
    
    
//-----------------------------------------------------------------------------
// Function: serialParseInt
//   Scan for an integer value in the conole input buffer.
// Parameters: (none)
// Returns: The integer value scanned, if found; otherwise, returns 0.
// Inputs/Outputs:
//   m_consolePos:IO
//   m_consoleBuffer:IO
//-----------------------------------------------------------------------------
int serialParseInt(void)
{
  uint8 tmp;
  char *token = NULL;
  
  tmp = strspn(&m_consoleBuffer[m_consolePos], " ,\n");
  token = strtok(&m_consoleBuffer[m_consolePos], " ,\n");
  m_consolePos += tmp;
  if (token != NULL)
  {
    m_consolePos += strlen(token) + 1;
    return atoi(token);
  }
  dbgPrintln(FLASH("Couldn't parse integer"));
  return 0;
}


//-----------------------------------------------------------------------------
// Function: cmdHdlrLed
//   Configure the addressable LED strip parameters.
// Parameters: (none)
// Returns: true iff command processing has completed.
// Inputs/Outputs:
//   m_consolePos:IO
//   m_consoleBuffer:IO
//   m_ledStripLen:O
//   m_ledStripFreq:O
//   m_ledStripWiring:O
//-----------------------------------------------------------------------------
boolean cmdHdlrLed(void)
{
  uint16  stripLen = 0;
  uint8   stripFreq = 0;
  uint8   stripCol = 0;

  stripLen  = serialParseInt();
  stripFreq = serialParseInt();
  stripCol  = serialParseInt();
  if ( (stripFreq != 8) && (stripFreq != 4) )
  {
    logPrint(FLASH("ERROR: Invalid LED strip frequency: "));
    logPrintln(stripFreq);
    return true;
  }
  if (stripCol > WIRING_BGR)
  {
    logPrint(FLASH("ERROR: Invalid LED strip colour wiring: "));
    logPrintln(stripCol);
    return true;
  }
  m_ledStripLen    = stripLen;
  m_ledStripFreq   = stripFreq;
  m_ledStripWiring = stripCol;
  logPrintln(FLASH("SAVE CONFIGURATION AND POWER CYCLE THE NODE"));
 
  return true;
}


//-----------------------------------------------------------------------------
// Function: cmdHdlrLedCtrl
//   Update the digital output pin used for controlling the addressable LED
//   strip.
// Parameters: (none)
// Returns: true iff command processing has completed.
// Inputs/Outputs:
//   m_consolePos:IO
//   m_consoleBuffer:IO
//   m_ledStripCtrlPin:O
//-----------------------------------------------------------------------------
boolean cmdHdlrLedCtrl(void)
{
  uint8  val = 0;
  
  val = serialParseInt();
  if ( ((val >= 3)  && (val <= 9)) ||
       ((val >= 14) && (val <= 21)) )
  {
    m_ledStripCtrlPin = (int8)val;
    logPrint(FLASH("  Output pin "));
    logPrint(m_ledStripCtrlPin);
    logPrintln(FLASH(" controls the addressable LED strip."));
    logPrintln(FLASH("SAVE CONFIGURATION AND POWER CYCLE THE NODE"));
  }
  else
  {
    logPrint(FLASH("ERROR: Invalid output pin, "));
    logPrintln(val);
  }
  return true;
}


//-----------------------------------------------------------------------------
// Function: cmdHdlrNodeId
//   Set the node's Node ID value.
// Parameters: (none)
// Returns: true iff command processing has completed.
// Inputs/Outputs:
//   m_consolePos:IO
//   m_consoleBuffer:IO
//   m_myNodeId:O
//   m_resetRequired:O
//-----------------------------------------------------------------------------
boolean cmdHdlrNodeId(void)
{
  m_myNodeId = serialParseInt();
  if ( (m_myNodeId >= FIRST_SLAVE) && (m_myNodeId <= NODEID_MAX) )
  {
    EEPROM.write(EEPROM_NODEID_ADDR, m_myNodeId);
    EEPROM.write(EEPROM_VALIDITY_ADDR, 0); // ID change invalidates data
    m_resetRequired = true;
    logPrint(FLASH("Node ID #"));
    logPrint(m_myNodeId);
    logPrintln(FLASH(" saved to EEPROM.\nPOWER CYCLE THE NODE."));
  }
  else
  {
    logPrintln(FLASH("*** Node ID value out of range."));
  }
}


//-----------------------------------------------------------------------------
// Function: cmdHdlSave
//   Save the current set of parameters to EEPROM.
// Parameters: (none)
// Returns: true iff command processing has completed.
// Inputs/Outputs:
//   m_consolePos:IO
//   m_consoleBuffer:IO
//-----------------------------------------------------------------------------
boolean cmdHdlSave(void)
{
  EepromSave();
  logPrintln(FLASH("Parameters have been saved to EEPROM"));
  
  return true;
}


//-----------------------------------------------------------------------------
// Function: cmdHdlrPerf
//   Display all timing data on the console.
// Parameters: (none)
// Returns: true iff command processing has completed.
// Inputs/Outputs:
//   m_timings:I
//   m_tmrUpdOverhead:I
//   m_txLate:I
//-----------------------------------------------------------------------------
boolean cmdHdlrPerf(void)
{
  int32  *pOmet;  // Ptr to current Observed Max Execution Time entry.
  
  pOmet = &m_timings[1];
  logPrintln(FLASH("\nTiming Data: (usec)"));
  for (uint8 i = 1; i < MAX_TIMINGS; i++)
  {
    logPrint(FLASH("\t"));
    logPrint(i);
    logPrint(FLASH("\t"));
    if (*pOmet > 0)
    {
      logPrint(*pOmet - m_tmrUpdOverhead);
      logPrint(FLASH("\t\t   actual ("));
      logPrint(*pOmet);
      logPrintln(FLASH(")"));
    }
    else
    {
      logPrintln(FLASH("-"));
    }
    pOmet++;
  }
  logPrint(FLASH("\tPer-timer update overhead: "));
  logPrint(m_tmrUpdOverhead);
  logPrintln(FLASH("us"));
  logPrint(FLASH("# times late (> "));
  logPrint(MAX_TX_EXEC_MS);
  logPrint(FLASH("ms): "));
  logPrintln(m_txLate);
  return true;
}


//-----------------------------------------------------------------------------
// Function: cmdHdlrReset
//   Perform a software reset.
// Parameters: (none)
// Returns: true iff command processing has completed.
//-----------------------------------------------------------------------------
boolean cmdHdlrReset(void)
{
  void (*resetFunc)(void) = 0; // Declare reset func @ address 0
  
  resetFunc(); // call reset
}



//-----------------------------------------------------------------------------
// Function: cmdHdlrRun
//   Start/resume running WASP commands.
// Parameters: (none)
// Returns: true iff command processing has completed.
//-----------------------------------------------------------------------------
boolean cmdHdlrRun(void)
{
  m_waspRunning = true;
  m_waspExecStart = millis();
  logPrintln(FLASH("WASP running"));
  return true;
}


//-----------------------------------------------------------------------------
// Function: cmdHdlrStop
//   Stop WASP command execution.
// Parameters: (none)
// Returns: true iff command processing has completed.
//-----------------------------------------------------------------------------
boolean cmdHdlrStop(void)
{
  m_waspRunning = false;
  logPrintln(FLASH("WASP stopped"));
  return true;
}


//-----------------------------------------------------------------------------
// Function: cmdHdlrHelp
//   Display the serial console port command syntax and synopses.
// Parameters: (none)
// Returns: true iff command processing has completed.
// Inputs/Outputs:
//   m_ledStripCtrlPin:I
//   m_ledStripFreq:I
//   m_ledStripLen:I
//   m_ledStripWiring:I
//-----------------------------------------------------------------------------
boolean cmdHdlrHelp(void)
{
  uint32 runTime;
  uint8  runDays;
  uint8  runHours;
  uint8  runMins;
  uint8  runSecs;

  logPrintln(FLASH("\n\nConsole commands are: (spaces can replace commas)"));
  logPrint(FLASH("  led l,f,c"));
      logPrintln(FLASH("\t- Configure addressable LED strip:"));
      logPrintln(FLASH("\t\t\tl = # tricolor LED pixels (default = 3)"));
      logPrintln(FLASH("\t\t\t      - Max 100 recommended for Moteino"));
      logPrintln(FLASH("\t\t\tf = 8, for 800kHz, WS2812 LEDs (default)"));
      logPrintln(FLASH("\t\t\t    4, for 400kHz, WS2811 drivers"));
      logPrintln(FLASH("\t\t\tc = 0, for RGB color wiring order (default)"));
      logPrintln(FLASH("\t\t\t    1, for RBG"));
      logPrintln(FLASH("\t\t\t    2, for GRB"));
      logPrintln(FLASH("\t\t\t    3, for GBR"));
      logPrintln(FLASH("\t\t\t    4, for BRG"));
      logPrintln(FLASH("\t\t\t    5, for BGR"));
  logPrint(FLASH("  ledCtrl n"));
      logPrintln(FLASH("\t- Change the default digital output pin that"));
      logPrintln(FLASH("\t\t  controls the LED strip. n in {3-9, 14-21}."));
  logPrint(FLASH("  nodeid n"));
      logPrintln(FLASH("\t- Set the Node Id to n. n in {2, 3, ..., 20}"));
  logPrint(FLASH("  perf"));
      logPrintln(FLASH("\t\t- Show performance timing data"));
  logPrint(FLASH("  reset"));
      logPrintln(FLASH("\t\t- Software reset"));
  logPrint(FLASH("  save"));
      logPrintln(FLASH("\t\t- Save parameters to EEPROM"));
  logPrint(FLASH("  h"));
      logPrintln(FLASH("\t\t- Print this help text"));
  logPrintln(FLASH("------------------------------------------------------"));
  logPrint(FLASH("  run"));
      logPrintln(FLASH("\t\t- Start/resume WASP"));
  logPrint(FLASH("  stop (or s)"));
      logPrintln(FLASH("\t- Stop WASP"));

      
  logPrintln();
  logPrintln(FLASH("\tCurrent LED strip parameters:"));
  logPrint(FLASH("\t\t  Length  l = "));
  logPrintln(m_ledStripLen);
  logPrint(FLASH("\t\t  Freq    f = "));
  switch (m_ledStripFreq)
  {
    case FREQ_IDX_400kHz: logPrintln(FLASH("4 (400 kHz)")); break;
    case FREQ_IDX_800kHz: logPrintln(FLASH("8 (800 kHz)")); break;
    default:
      logPrint(FLASH("<invalid> ("));
      logPrint(m_ledStripFreq);
      logPrintln(FLASH(")"));
  }
  logPrint(FLASH("\t\t  Colours c = "));
  switch (m_ledStripWiring)
  {
    case WIRING_RGB: logPrintln(FLASH("0 (RGB)")); break;
    case WIRING_RBG: logPrintln(FLASH("1 (RBG)")); break;
    case WIRING_GRB: logPrintln(FLASH("2 (GRB)")); break;
    case WIRING_GBR: logPrintln(FLASH("3 (GBR)")); break;
    case WIRING_BRG: logPrintln(FLASH("4 (BRG)")); break;
    case WIRING_BGR: logPrintln(FLASH("5 (BGR)")); break;
    default:
      logPrint(FLASH("<invalid> ("));
      logPrint(m_ledStripWiring);
      logPrintln(FLASH(")"));
  }
  logPrint(FLASH("\t\t  Ctrl Pin  = "));
  logPrintln(m_ledStripCtrlPin);
  
  CheckRam();
  
  logPrint(FLASH("WASP is ["));
  if (m_waspRunning)
  {
    runTime = millis() - m_waspExecStart;
    runTime  = (runTime + 500) / 1000; // seconds left
    runSecs  = runTime % 60;
    runTime  = runTime / 60; // minutes left
    runMins  = runTime % 60;
    runTime  = runTime / 60; // hours left
    runHours = runTime % 24;
    runDays  = runTime / 24; // days left
    logPrint(FLASH("running for "));
    logPrint(runDays);
    logPrint(FLASH(" days, "));
    logPrint(runHours);
    logPrint(FLASH(" hours, "));
    logPrint(runMins);
    logPrint(FLASH(" minutes, "));
    logPrint(runSecs);
    logPrintln(FLASH(" seconds]"));
  }
  else
  {
    logPrintln(FLASH("stopped]"));
  }
  
  return true;
}


//-----------------------------------------------------------------------------
// Function: processCommandLine
//   Check serial console input for a command and process it.
// Parameters: (none)
// Returns: (none)
// Inputs/Outputs:
//   m_cmdCtrl:I
//   m_consolePos:IO
//   m_consoleBuffer:IO
// Note:
//   Each command handler is responsible for parsing the command line for
//   its parameters.
//-----------------------------------------------------------------------------
void processCommandLine()
{
  static cmdHdlrFunc_t *cmdHdlr = NULL;
  static uint8          cmdStage = CMD_PROMPT;
  static uint8          cmdIdx = 0;
  static uint8          loIdx = 0;
  static uint8          hiIdx = 0;
  static char          *lineChar = NULL;
  static boolean        cmdFound;
  char       inChar;
  char      *cmdName;
  uint8      nameIdx;
  uint8      tmpPos;
  int8       result;

  if (CMD_PROMPT == cmdStage)
  {
    logPrint(FLASH("> "));
    cmdStage = CMD_INPUT;
    return;
  }
  
  /* Scan for a complete line of console input */
  else if (CMD_INPUT == cmdStage)
  {
    if (0 == Serial.available())
      return; // Nothing to process
 
    /* Process an input character */
    inChar = Serial.read();
    if (('\b' == inChar) || (DEL_CHAR == inChar))
    {
      /* Backup the buffer position and truncate the command line string. */
      if (m_consolePos > 0)
        m_consolePos--;
      m_consoleBuffer[m_consolePos] = '\0';
      return;
    }
    else if ('\r' == inChar)
    {
      /* We have a completed line of input. */
      m_consoleBuffer[m_consolePos] = '\0'; // Terminate command line string
    
      /* Flush the console input */
      while (Serial.available() > 0)
        Serial.read();
    }
    else if ( (MAX_SERIAL_BUF_LEN - 1) == m_consolePos )
    {
      /* We're going to overrun the buffer: discard the contents */
      m_consolePos = 0;
      return;
    }
    else
    {
      /* Append the new char to the buffer */
      m_consoleBuffer[m_consolePos++] = inChar;
      return;
    }
    
    /*----- A completed line of input is now available. -----*/

    /* First confirm that power cycling isn't required */
    if (m_resetRequired)
    {
      logPrintln(FLASH("\n\nPlease power cycle the node first!\n"));
      return;
    }

    /* Set up to parse the command upon next invocation. */
    cmdStage = CMD_PARSE;
    
    /* Skip leading spaces */
    m_consolePos = 0;
    for (lineChar = &m_consoleBuffer[0]; *lineChar == ' '; lineChar++)
      m_consolePos++;
      
    loIdx  = 0;
    hiIdx   = m_numCmds - 1;
    cmdIdx  = m_numCmds / 2;
    tmrUpdateOmet(TMR_CMD_SCAN);
  }
  
  
  /* Parse the command name using a binary search thru the command list. */
  else if (CMD_PARSE == cmdStage)
  {
    /* Compare the candidate command name to the command line */
    tmpPos = m_consolePos;
    lineChar = &m_consoleBuffer[tmpPos];
    cmdName = m_cmdCtrl[cmdIdx].cmdName;
    cmdFound = true;
    result = 0;
    while ( (*cmdName != '\0') && (*lineChar != '\0') )
    {
        if (*cmdName != *lineChar)
        {
          cmdFound = false;
          if (*lineChar < *cmdName)
            result = -1;
          else
            result = 1;
          break;  // This command doesn't match.
        }
        cmdName++;
        lineChar++;
        tmpPos++;
    }
    
    if (cmdFound)
    {
      if (*cmdName != '\0')
      {
        cmdFound = false;
        result = -1;
      }
    }

    if (cmdFound)
    {
      /* We've matched the candidate command name so far. However,
       * it's only a true match if the next character on the command
       * line is either a space or the end of the string.
       */
      if ( (*lineChar != ' ') && (*lineChar != '\0') )
      {
        cmdFound = false;
        result = 1;
      }
      else
      {
        /* Success. We've found the command. */
        logPrint(FLASH("["));
        logPrint(m_consoleBuffer);
        logPrintln(FLASH("]"));
        cmdHdlr = m_cmdCtrl[cmdIdx].cmdHdlr;
        if (cmdHdlr != NULL)
        {
          m_consolePos = tmpPos;
          cmdStage = CMD_EXEC;
        }
        else
        {
          logPrintln(FLASH("***Internal Error: Command handler is missing")); 
          m_consolePos = 0;
          memset(m_consoleBuffer, 0, sizeof(m_consoleBuffer));
          cmdStage = CMD_PROMPT;
          return;
        }
      }
    }
    
    if (!cmdFound)
    {
      /* The candidate command line doesn't match. Is there another
       * candidate still?
       */
      cmdFound = true;
      if ( (loIdx + 1) == hiIdx )
      {
        /* Our search has ended: command line is invalid */
        cmdFound = false;
      }
      if (result < 0)
      {
        /* We need an earlier command (alphanumerically) */
        hiIdx = cmdIdx;
        cmdIdx = loIdx + (hiIdx - loIdx)/2;
      }
      else
      {
        loIdx = cmdIdx;
        cmdIdx = loIdx + (hiIdx - loIdx + 1)/2;
      }
    }

    tmrUpdateOmet(TMR_CMD_PARSE);
    
    if (!cmdFound)
    {
      /* The command search has ended in failure. Discard the
       * command line and restart scanning.
       */
      logPrint(FLASH("["));
      logPrint(m_consoleBuffer);
      logPrintln(FLASH("]"));
      logPrintln(FLASH("***Command not found"));
      cmdStage = CMD_PROMPT;
      memset(m_consoleBuffer, 0, sizeof(m_consoleBuffer));
      m_consolePos = 0;
      return;
    }

    /* If we reach this point, we'll compare the next candidate
     * command upon next invocation.
     */
  }
  
  /* Execute the command */
  else /* CMD_EXEC == cmdStage */
  {
    if ( (*cmdHdlr)() )
    {
      /* We're done processing. Purge the command line buffer
       * and restart scanning.
       */
      cmdStage = CMD_PROMPT;
      memset(m_consoleBuffer, 0, sizeof(m_consoleBuffer));
      m_consolePos = 0;
    }
    tmrUpdateOmet(TMR_CMD_EXEC);
    /* Each command has to parse the command line parameters */
  }
}


//-----------------------------------------------------------------------------
// Function: receiveRadioWaspCmd
//   Check for and pixel commands from the RFM radio network.
// Parameters: (none)
// Returns:    (none)
// Inputs/Outputs:
//   m_myGroupId:I
//   m_myNodeId:I
//   m_radio:I
//   m_dstNodeId:O
//   m_waspCmd:O
//   m_radioInBuf:O
//   m_radioInBufPos:O
//   m_srcNodeId:O
//-----------------------------------------------------------------------------
void receiveRadioWaspCmd(void)
{
  const  uint32 k_rxDelay = MIN_UPD_PERIOD / 2;
  static uint32  lastRxTime = 0;
  static boolean waspRxLedOn = false;
  uint8 waspCmd = WASPCMD_NONE;
  
  m_radioInBuf[0] = WASPCMD_NONE;
  m_radioInBufPos = 0;
  
  if ( (millis() - lastRxTime) < k_rxDelay)
    return;

  if (m_radio.receiveDone())
  {
    m_dstNodeId = m_radio.TARGETID;
    m_srcNodeId = m_radio.SENDERID;

    // Check for wireless software update
    if (m_srcNodeId == PROG_GW_ID)
    {
      Serial.println(FLASH("Rx from WASP slave programming gateway node"));
      dbgPrint("Got [");
      dbgPrint(m_srcNodeId);
      dbgPrint(':');
      dbgPrint(m_radio.DATALEN);
      dbgPrint("] > ");
      #ifdef DEBUG_ON
      for (byte i = 0; i < m_radio.DATALEN; i++)
        logPrintEx((char)m_radio.DATA[i], HEX);
      #endif
      dbgPrintln();

      if (m_dstNodeId == m_myNodeId)
      {
        // Invoke function that checks for, and receive, a wireless external
        // Flash memory program update. (Set the last parameter to true to
        // see the protocol exchanges.)
        CheckForWirelessHEX(m_radio, m_flash, false);
      }
      else
      {
        dbgPrintln("Request is not for me");
      }
      
      dbgPrintln();
    }

    //noInterrupts();
    // Determine if we're an intended receiver
    if (   ( (m_dstNodeId == BROADCASTID) && (m_srcNodeId == CONTROLLERID) )
        || ( (m_dstNodeId == m_myNodeId) && (m_myNodeId != NODEID_UNDEF) )
        || ( (m_dstNodeId == m_myGroupId) && (m_myGroupId != NODEID_UNDEF) )
       )
    {
      m_ackRequested = m_radio.ACK_REQUESTED;
      memcpy(m_radioInBuf, (const void *)&m_radio.DATA[0], m_radio.DATALEN);
      m_radio.DATALEN = 0;
      waspRxLedOn = !waspRxLedOn;
      if (waspRxLedOn)
        digitalWrite(WASP_RX_LED, HIGH);
      else
        digitalWrite(WASP_RX_LED, LOW);
      //interrupts();
      m_radioInBufPos = 0;
      waspCmd = m_radioInBuf[m_radioInBufPos++];
      if (waspCmd > MAX_WASPCMD_VAL)
      {
        dbgPrint(FLASH(" ... invalid Rx WASP cmd="));
        dbgPrintln(waspCmd);
        m_radioInBuf[0] = WASPCMD_NONE;
        waspCmd = WASPCMD_NONE;
      }
      else
      {
        dbgPrint(FLASH("Rx Cmd["));
        dbgPrint(waspCmd);
        dbgPrint(FLASH("] Src["));
        dbgPrint(m_srcNodeId);
        dbgPrintln(FLASH("]"));
      }
    }
    else
    {
      interrupts();
//JVS
#if 0
      dbgPrint(FLASH(" ... ignored (pkt not for me). Src:"));
      dbgPrint(m_srcNodeId);
      dbgPrint(FLASH(" Dst:"));
      dbgPrint(m_dstNodeId);
      dbgPrintln(FLASH(")"));
#endif
    }
  }
  m_waspCmd = waspCmd;
}


//-----------------------------------------------------------------------------
// Function: radioSendBuf
//   Send a WASP response message.
// Parameters:
//   dst:I         - Destination node ID; Don't use BROADCASTID .
//   pPayload:I    - Pointer to the payload buffer.
//   payloadLen:I  - Number of bytes to send from the payload buffer.
// Returns: (none)
// Inputs/Outputs:
//   m_radio:I
//-----------------------------------------------------------------------------
uint8 radioSendBuf(uint8 dst, uint8 *pPayload, uint8 payloadLen)
{
  static uint32  lastTxTime = 0;
  static boolean waspTxLedOn = false;
  uint32  txStartTime;
    
  // Don't allow consecutive transmissions to be too close together.
//  while ( (millis() - lastTxTime) < MIN_UPD_PERIOD )
//    ;

  waspTxLedOn = !waspTxLedOn;
  if (waspTxLedOn)
    digitalWrite(WASP_TX_LED, HIGH);
  else
    digitalWrite(WASP_TX_LED, LOW);
  txStartTime = millis();
  m_radio.send(dst, pPayload, payloadLen, false);
  
  lastTxTime = millis();
  if ( (lastTxTime - txStartTime) > MAX_TX_EXEC_MS )
    m_txLate++;
}


//------------------------------------------------------------------------------
// Function: mapToColour
//   Map LED pixel RGB colour components to a uint32 colour value.
// Parameters:
//   r:I  - Amount of red.
//   g:I  - Amount of green.
//   b:I  - Amount of blue.
// Returns: uint32 version of the specified colour corrected for the pixel
//          strip's LED wiring.
//------------------------------------------------------------------------------
uint32 mapToColour(uint8 r, uint8 g, uint8 b)
{
  uint8  pixel[3];

  pixel[m_offsRed]   = r;
  pixel[m_offsGreen] = g;
  pixel[m_offsBlue]  = b;
  
  return   ((uint32)pixel[0] << 16)
         | ((uint32)pixel[1] <<  8)
         | pixel[2];
}


//-----------------------------------------------------------------------------
// Function: savePixels
//   Save the current pixel colours.
// Parameters: (none)
// Returns:    (none)
// Inputs/Outputs:
//   m_savedLeds:O
//-----------------------------------------------------------------------------
static inline void savePixels(void)
{
  memcpy(m_savedLeds, m_pPixels, m_numPixelBytes);
}


//-----------------------------------------------------------------------------
// Function: restorePixels
//   Restore the previously saved pixel colours.
// Parameters: (none)
// Returns:    (none)
// Inputs/Outputs:
//   m_savedLeds:O
//-----------------------------------------------------------------------------
static inline void restorePixels(void)
{
  memcpy(m_pPixels, m_savedLeds, m_numPixelBytes);
}


//-----------------------------------------------------------------------------
// Function: waspCmdHdlrNone
//   No-operation WASP command handler.
// Parameters:     (none)
// Returns:  WASPCMD_NONE
// Inputs/Outputs: (none)
//-----------------------------------------------------------------------------
WaspCmd_t waspCmdHdlrNone(void)
{
  return WASPCMD_NONE;
}


//-----------------------------------------------------------------------------
// Function: waspCmdHdlrGroup
//   Record my neighbouring nodes.
// Parameters:     (none)
// Returns:  WASPCMD_NONE
// Inputs/Outputs:
//   m_radioInBuf:I
//   m_myGroupId:O
//   m_leftNeighbour:O
//   m_rightNeighbour:O
//   m_selfShift:O
//   m_radioInBufPos:IO
//-----------------------------------------------------------------------------
WaspCmd_t waspCmdHdlrGroup(void)
{
  uint8 *buf;
  uint8  tmpGroup, tmpLeft, tmpRight;

  // Retrieve parameters
  buf = &m_radioInBuf[m_radioInBufPos];
  tmpGroup = *buf++;
  tmpLeft = *buf++;
  tmpRight = *buf++;
  m_radioInBufPos += 3;

  // Confirm that this is a unicast command.
  if (m_dstNodeId != m_myNodeId)
  {
    dbgPrintln(FLASH("***GROUP: Ignore broadcast command"));
    return WASPCMD_NONE;
  }

  if ( (tmpGroup < FIRST_GROUP) || (tmpGroup >= (FIRST_GROUP + MAX_GROUPS)) )
  {
    logPrint(FLASH("***GROUP: Invalid WASP slave group Id - "));
    logPrintln(tmpGroup);
    return WASPCMD_NONE;
  }

  m_myGroupId = tmpGroup;
  if (tmpLeft == m_myNodeId)
  {
    if (tmpRight == m_myNodeId)
    {
      // left = right = me. Ok, We're an isolated world unto ourself.
      m_leftNeighbour = tmpLeft;
      m_rightNeighbour = tmpRight;
      m_selfShift = true;
    }
    else
    {
      // left = me != right.  Not cool.
      logPrintln(FLASH("***GROUP: L = me != R"));
      return WASPCMD_NONE;
    }
  }
  else
  {
    if (tmpRight == m_myNodeId)
    {
        // right = me != left.  Not cool.
        logPrintln(FLASH("***GROUP: R = me != L"));
        return WASPCMD_NONE;
    }
    else
    {
      // Ok, we have distinct neighbour(s).
      m_leftNeighbour = tmpLeft;
      m_rightNeighbour = tmpRight;
      m_selfShift = false;
    }
  }

//??JVS
//  logPrint(FLASH("Grp="));
//  logPrintln(m_myGroupId);
//  logPrint(FLASH("L="));
//  logPrintln(m_leftNeighbour);
//  logPrint(FLASH("R="));
//  logPrintln(m_rightNeighbour);
  
  return WASPCMD_NONE;
}


//-----------------------------------------------------------------------------
// Function: waspCmdHdlrState
//   Respond to state control commands.
// Parameters:     (none)
// Returns:  WASPCMD_NONE
// Inputs/Outputs:
//   m_radioInBuf:I
//   m_radioInBufPos:IO
//   m_pixelShowSuspend:O
//   m_updatePixels:O
//-----------------------------------------------------------------------------
WaspCmd_t waspCmdHdlrState(void)
{
  uint8 *buf;
  uint8  opts;
  uint8  saveRestore;
  uint8  suspendResume;

  // Retrieve parameters
  buf = &m_radioInBuf[m_radioInBufPos];
  opts = *buf++; // default is red
  m_radioInBufPos += 1;
  
  saveRestore = opts & (F_SAVE | F_RESTORE);
  suspendResume = opts & (F_SUSPEND | F_RESUME);
  
  // Validate parameters
  if ( saveRestore == (F_SAVE | F_RESTORE) )
  {
    // Both bits can't be set. Ignore the command.
    return WASPCMD_NONE;
  }
  if ( suspendResume == (F_SUSPEND | F_RESUME) )
  {
    // Both bits can't be set. Ignore the command.
    return WASPCMD_NONE;
  }

  // Update pixel strip state accordingly
  if (saveRestore)
  {
    if (saveRestore & F_SAVE)
      savePixels();
    else
      restorePixels();
      m_updatePixels = true;
  }
  
  if (suspendResume)
  {
    m_pixelShowSuspend = ( (suspendResume & F_SUSPEND) != 0 );
  }
    
  return WASPCMD_NONE;
}


//-----------------------------------------------------------------------------
// Function: waspCmdHdlrBackground
//   Set and save the background colour for all pixels.
// Parameters:     (none)
// Returns:  WASPCMD_NONE
// Inputs/Outputs:
//   m_ledStripLen:I
//   m_offsBlue:I
//   m_offsGreen:I
//   m_offsRed:I
//   m_radioInBuf:I
//   m_pPixels:IO
//   m_radioInBufPos:IO
//   m_updatePixels:O
//-----------------------------------------------------------------------------
WaspCmd_t waspCmdHdlrBackground(void)
{
  uint8   red, green, blue;
  uint8  *buf;
  uint8  *pixels;
  uint8   numPixels;
  uint16  i;
  
  // Retrieve parameters
  buf = &m_radioInBuf[m_radioInBufPos];
  red   = *buf++; // default is red
  green = *buf++; // default is green
  blue  = *buf++; // blue
  m_radioInBufPos += 3;
  
  
  //  Set the pixel colours directly
  pixels = m_pPixels;
  for (i = m_ledStripLen; i != 0; i--)
  {
    *(pixels + m_offsRed)   = red;
    *(pixels + m_offsGreen) = green;
    *(pixels + m_offsBlue)  = blue;
    pixels += LEDS_PER_PIX;
  }
  
  savePixels();
  m_updatePixels = true;
  
  return WASPCMD_NONE;
}

 
//-----------------------------------------------------------------------------
// Function: waspCmdHdlrLine
//   Draw a line.
// Parameters:     (none)
// Returns:  WASPCMD_NONE
// Inputs/Outputs:
//   m_ledStripLen:I
//   m_offsBlue:I
//   m_offsGreen:I
//   m_offsRed:I
//   m_radioInBuf:I
//   m_radioInBufPos:IO
//   m_updatePixels:O
//-----------------------------------------------------------------------------
WaspCmd_t waspCmdHdlrLine(void)
{
  uint8  *buf;
  uint8  *pixels;
  uint16  start;
  uint16  len;
  uint8   red, green, blue;
  uint8   i;

  // Retrieve parameters
  buf = &m_radioInBuf[m_radioInBufPos];
  start = *buf++;
  len   = *buf++;
  red   = *buf++;
  green = *buf++;
  blue  = *buf++;
  m_radioInBufPos += 5;
  
  // Validate parameters
  if ( (start + len) > m_ledStripLen)
    return WASPCMD_NONE;
    
  // Set the pixel colours directly
  pixels = &m_pPixels[start * LEDS_PER_PIX];
  for (i = len; i != 0; i--)
  {
    *(pixels + m_offsRed)   = red;
    *(pixels + m_offsGreen) = green;
    *(pixels + m_offsBlue)  = blue;
    pixels += LEDS_PER_PIX;
  }
  m_updatePixels = true;
  
  return WASPCMD_NONE;
}
    
    
//-----------------------------------------------------------------------------
// Function: waspCmdHdlrShift
//   Shift pixels.
// Parameters:     (none)
// Returns:  WASPCMD_SHIFT, if SHIFT command handling is still in progress;
//           WASPCMD_NONE,  if SHIFT command has completed or timed out.
// Inputs/Outputs:
//   m_ledStripLen:I
//   m_ledStripWiring:I
//   m_leftNeighbour:I
//   m_myNodeId:I
//   m_numPixelBytes:I
//   m_radioInBuf:I
//   m_rightNeighbour:I
//   m_selfShift:I
//   m_srcNodeId:I
//   m_radioInBufPos:IO
//   m_shiftInLeds:IO
//   m_shiftOutLeds:IO
//   m_radioOutBuf:o
//   m_radioOutBufPos:O
//   m_updatePixels:O
// Note:
//   This handler gets called repeatedly when we have other nodes as
//   neighbours and a SHIFT command is in effect. The various stages need to
//   confirm whether or not an actual WASP message was received.
//-----------------------------------------------------------------------------
WaspCmd_t waspCmdHdlrShift(void)
{
//  const int8 k_TxStartOffs =  6; // Millisecond delay offset before Tx.
//  const int8 k_TxDeadOffs  =  5; // Millisecond deadline offset for Tx.
  const int8 k_TxStartOffs =  0; // Millisecond delay offset before Tx.
  const int8 k_TxDeadOffs  =  7; // Millisecond deadline offset for Tx.
  static uint32  startTime = 0;
  static uint32  timeout = 0;
  static uint32  releaseTime = 0;
  static uint32  deadline = 0;
  static uint16  numShifts = 0;
  static uint8   stage = PX_SHIFT_START; // Processing stage.
  static int16   num = 0;                // # pixels to shift.
  static uint8   numShiftBytes = 0;      // # of colour bytes involved in shift
  static uint8   neighbour = 0;
  static boolean shiftRight = true;      // TRUE = shift to the right; else left
  uint32   currentTime;
  uint8    rxCmd;
  uint8   *buf;
  uint8   *srcPixels;
  uint8   *dstPixels;
  uint8    i;


  rxCmd = m_radioInBuf[0];
  m_updatePixels = false; // Default is don't yet update the pixels.
  
  /*------------  PX_SHIFT_START stage (command start)  -----------------*/
  if (PX_SHIFT_START == stage)
  {
    // Did we actually receive a new command?
    if (rxCmd != WASPCMD_SHIFT)
    {
      // Hmm. So why were we called. Anyway, there's nothing more to do.
      return WASPCMD_NONE;
    }
    
    startTime = millis();
    numShifts++;
        
    // Confirm that the controller sent the command.
    if (m_srcNodeId != CONTROLLERID)
    {
      // Only the controller can request the command. Ignore the
      // message.
      return WASPCMD_NONE;
    }
//JVS
//logPrint(FLASH(">"));
//logPrintln(startTime);
    
    // Retrieve first parameter
    buf = &m_radioInBuf[m_radioInBufPos];
    num = (int8)*buf++;
    m_radioInBufPos += 1;

    if (num > 0)
    {
      shiftRight = true;
    }
    else
    {
      shiftRight = false;
      num = -num;
    }
    
    // Validate parameters
    if (num > MAX_SHIFT_SIZE)
      return WASPCMD_NONE;
    if (num > m_ledStripLen)
      return WASPCMD_NONE;


    /* Command is valid. Now shift the pixels outward, recording the pixels
     * that have shifted out of the strip.
     */
    numShiftBytes = num * LEDS_PER_PIX;
    if (shiftRight)
    {
      // Save pixels that are shifting out
      srcPixels = &m_pPixels[m_numPixelBytes - numShiftBytes];
      dstPixels = &m_shiftOutLeds[0];
      for (i = numShiftBytes; i != 0; i--)
        *dstPixels++ = *srcPixels++;
      
      // Shift the pixels over to the right.
      srcPixels = &m_pPixels[m_numPixelBytes - 1 - numShiftBytes];
      dstPixels = &m_pPixels[m_numPixelBytes - 1];
      for (i = m_numPixelBytes - numShiftBytes; i != 0; i--)
        *dstPixels-- = *srcPixels--;
    }
    else
    {
      // Shifting left
      
      // Save pixels that are shifting out
      srcPixels = &m_pPixels[0];
      dstPixels = &m_shiftOutLeds[0];
      for (i = numShiftBytes; i !=0; i--)
        *dstPixels++ = *srcPixels++;
        
      // Shift the pixels over to the left.
      srcPixels = &m_pPixels[numShiftBytes];
      dstPixels = &m_pPixels[0];
      for (i = m_numPixelBytes - numShiftBytes; i != 0; i--)
        *dstPixels++ = *srcPixels++;
    }
    
    if (m_selfShift)
    {
      // We don't have neighbouring nodes. So shift in the saved pixels now.
      // The command will then be finished.
      m_updatePixels = true;
      if (shiftRight)
      {      
        srcPixels = &m_shiftOutLeds[0];
        dstPixels = &m_pPixels[0];
        for (i = numShiftBytes; i !=0; i--)
          *dstPixels++ = *srcPixels++;
      }
      else
      {
        srcPixels = &m_shiftOutLeds[0];
        dstPixels = &m_pPixels[m_numPixelBytes - numShiftBytes];
        for (i = numShiftBytes; i !=0; i--)
          *dstPixels++ = *srcPixels++;
      }
      return WASPCMD_NONE;
    }
    else
    {
      // Interaction with neighbours is required. Determine to which
      // neighbour to report and from which neighbour to await a reply.
      // The relative WASP node ID's determine what the next stage
      // should be. Reponses are expected in sequence with the lowest
      // numbered node transmitting first.
      
      timeout = startTime + CMD_TIMEOUT;
      
      if (shiftRight)
      {
        // Who is first to transmit--us or our left neighbour?
        if (m_leftNeighbour < m_myNodeId)
        {
          neighbour = m_leftNeighbour;
          stage = PX_SHIFT_WAIT1;  // Left neighbour Tx first.
        }
        else
        {
          stage = PX_SHIFT_RESP;
        }
      }
      else
      {
        // Who is first to transmit--us or our right neighbour?
        if (m_rightNeighbour < m_myNodeId)
        {
          neighbour = m_rightNeighbour;
          stage = PX_SHIFT_WAIT1;  // Right neighbour Tx first.
        }
        else
        {
          stage = PX_SHIFT_RESP;
        }
      }
      
      tmrUpdateOmet(TMR_SHFT_STRT);
      
      if (PX_SHIFT_RESP == stage)
      {
        releaseTime = startTime + (m_myNodeId - FIRST_SLAVE) * SLAVE_TX_WIND;
        deadline = releaseTime + SLAVE_TX_WIND - k_TxDeadOffs;
        releaseTime += k_TxStartOffs;
      }
      else
      {
        releaseTime = startTime + (neighbour - FIRST_SLAVE) * SLAVE_TX_WIND;
        deadline = releaseTime + SLAVE_TX_WIND;
      }
      return WASPCMD_SHIFT;
    } /* else - !m_selfShift */
  }  /* if (PX_SHIFT_START == stage) */
  else
  {
    // Before proceeding, we need to check if:
    //   - The command has timed out, or
    //   - The Controller, for whatever reason, sent a new command.
    // In either case, we need to abort the current command, updating
    // the pixels to whatever state with which we're now stuck.
    currentTime = millis();
    if (    (currentTime > timeout)
         || ( (WASPCMD_SHIFT == rxCmd) && (m_srcNodeId == CONTROLLERID) )
       )
    {
//JVS
if (currentTime > timeout)
{
  //logPrint(FLASH("!TO:"));
  //logPrintln(currentTime);
  logPrintln("!");
}
else
//  logPrintln(FLASH("??"));
      m_updatePixels = true;
//m_updatePixels = false;
      stage = PX_SHIFT_START;
      return WASPCMD_NONE;
    }
  }
  
  
  /*------ PX_SHIFT_WAIT1 / PX_SHIFT_WAIT2 stage (await neighbour Tx) ------*/
  if ( (PX_SHIFT_WAIT1 == stage) || (PX_SHIFT_WAIT2 == stage) )
  {
    currentTime = millis();

    if (currentTime >= deadline)
    {
      // Our neighbour missed its chance. There's not much we can do:
      //   - We can't shift in any pixels. So just leave them as is.
      //   - Move on to the next stage.
//JVS
//logPrint(stage);
//logPrint(FLASH("  #"));
//logPrintln(numShifts);
logPrintln("@");
      if (PX_SHIFT_WAIT1 == stage)
      {
        // Move on to the next stage.
        stage = PX_SHIFT_RESP;
        releaseTime = startTime + (m_myNodeId - FIRST_SLAVE) * SLAVE_TX_WIND;
        deadline = releaseTime + SLAVE_TX_WIND - k_TxDeadOffs;
        releaseTime += k_TxStartOffs;
        //dbgPrintln(FLASH("***SHIFT: Missed Rx - WAIT1"));
        return WASPCMD_SHIFT;
      }
      else
      {
        // There's nothing more we can do. Just make sure that the
        // pixels get updated (for better or worse).
        stage = PX_SHIFT_START;
        m_updatePixels = true;
        //dbgPrintln(FLASH("***SHIFT: Missed Rx - WAIT2"));
        return WASPCMD_NONE;
      }
    }
    
    // Did we actually receive a new msg?
    if (rxCmd != WASPCMD_SHIFT)
    {
      // There's nothing yet. So just keep waiting.
      return WASPCMD_SHIFT;
    }
      
    // Is the message from the expected neighbour?
    if (m_srcNodeId != neighbour)
    {
      // We're still waiting to hear from our neighbour.
      {
        return WASPCMD_SHIFT;
      }
    }
      
    // Retrieve the parameters
    buf = &m_radioInBuf[m_radioInBufPos];
    if (*buf++ != num)
    {
      // The number of expected colours is incorrect. Just ignore the
      // message and hope there's a correct one coming.
//JVS
logPrint(FLASH("***SHIFT: Bad report - "));
logPrintln(stage);
      dbgPrint(FLASH("***SHIFT: Bad report - "));
      dbgPrintln(stage);
      return WASPCMD_SHIFT;
    }
    m_radioInBufPos += 1;
    dstPixels = &m_shiftInLeds[0];
    // Read in the pixel colours
    for (i = numShiftBytes; i != 0; i--)
      *dstPixels++ = *buf++;
    m_radioInBufPos += numShiftBytes;
      
    // Shift in the pixels
    if (shiftRight)
    {      
      srcPixels = &m_shiftInLeds[0];
      dstPixels = &m_pPixels[0];
      for (i = numShiftBytes; i !=0; i--)
        *dstPixels++ = *srcPixels++;
    }
    else
    {
      srcPixels = &m_shiftInLeds[0];
      dstPixels = &m_pPixels[m_numPixelBytes - numShiftBytes];
      for (i = numShiftBytes; i !=0; i--)
        *dstPixels++ = *srcPixels++;
    }
    if (PX_SHIFT_WAIT1 == stage)
    {
      tmrUpdateOmet(TMR_SHFT_WT1);
    }
    else
    {
      tmrUpdateOmet(TMR_SHFT_WT2);
    }
      
    // Move on to the next stage.
    if (PX_SHIFT_WAIT1 == stage)
    {
      stage = PX_SHIFT_RESP;
      releaseTime = startTime + (m_myNodeId - FIRST_SLAVE) * SLAVE_TX_WIND;
      deadline = releaseTime + SLAVE_TX_WIND - k_TxDeadOffs;
      releaseTime += k_TxStartOffs;
      return WASPCMD_SHIFT;
    }
    else
    {
      stage = PX_SHIFT_START;
      m_updatePixels = true;
      return WASPCMD_NONE;
    }
  }  /* if (PX_SHIFT_WAIT1 == stage) */


  /*------- PX_SHIFT_RESP stage (Tx to neighbour) -----------------*/
  if (PX_SHIFT_RESP == stage)
  {
    currentTime = millis();
    if (currentTime < releaseTime)
    {
      // It's not our turn to Tx yet.
      return WASPCMD_SHIFT;
    }

    if (currentTime <= deadline)
    {   
      /* We're clear to Tx now. */

      //dbgPrintln(FLASH("SHIFT: Tx start..."));
            
      // Datafill the output buffer.
      buf = &m_radioOutBuf[0];
      *buf++ = WASPCMD_SHIFT;
      *buf++ = num;
      m_radioOutBufPos = 2;
      srcPixels = &m_shiftOutLeds[0];
      for (i = numShiftBytes; i != 0; i--)
        *buf++ = *srcPixels++;
      m_radioOutBufPos += numShiftBytes;
      
      // Send the shifted-pixels report to the appropriate neighbour.
      if (shiftRight)
        neighbour = m_rightNeighbour;
      else
        neighbour = m_leftNeighbour;

      tmrUpdateOmet(TMR_SHFT_TX0);
      radioSendBuf(neighbour, m_radioOutBuf, m_radioOutBufPos);
      tmrUpdateOmet(TMR_SHFT_TX1);
      currentTime = millis();
      if (currentTime < releaseTime)
      {
        logPrint(FLASH("!! #"));
        logPrintln(numShifts);
      }
    }
    else
    {
      dbgPrintln(FLASH("***SHIFT: Missed Tx deadline"));
//JVS
logPrint(FLASH("!Tx #"));
logPrintln(numShifts);
    }
    
    // At this point, we either missed our chance to Tx or we
    // finished the Tx. Either way, we have to move on.
    if (shiftRight)
    {
      // Who was first to transmit--us or our left neighbour?
      if (m_leftNeighbour < m_myNodeId)
      {
        // OK, we're done.
        stage = PX_SHIFT_START;
      }
      else
      {
        if (m_rightNeighbour < m_myNodeId)
        {
          // OK, we've already heard from our neighbour
          stage = PX_SHIFT_START;
        }
        else
        {
          neighbour = m_leftNeighbour;
          stage = PX_SHIFT_WAIT2;
        }
      }
    }
    else
    {
      // Who was first to transmit--us or our right neighbour?
      if (m_leftNeighbour < m_myNodeId)
      {
        // OK, we're done.
        stage = PX_SHIFT_START;
      }
      else
      {
        if (m_rightNeighbour < m_myNodeId)
        {
          // Ok, we've already heard from our one neighbour
          stage = PX_SHIFT_START;
        }
        else
        {
          neighbour = m_rightNeighbour;
          stage = PX_SHIFT_WAIT2;
        }
      }
    }
    
    tmrUpdateOmet(TMR_SHFT_RSP);

    if (PX_SHIFT_START == stage)
    {
//JVS
//logPrint(FLASH("<"));
//logPrintln(startTime);
      m_updatePixels = true;
      return WASPCMD_NONE;
    }
    else
    {
      releaseTime = startTime + (neighbour - 1) * SLAVE_TX_WIND;
      deadline = releaseTime + SLAVE_TX_WIND;
      return WASPCMD_SHIFT;
    }    
  }  /* if (PX_SHIFT_RESP == stage) */
  
  // We shouldn't get this far.
  stage = PX_SHIFT_START;
  m_updatePixels = true;
  logPrintln(FLASH("***SHIFT: internal error"));
  return WASPCMD_NONE;
}
    
    
//-----------------------------------------------------------------------------
// Function: waspCmdHdlrSwap
//   Swap pixel colours.
// Parameters:     (none)
// Returns:  WASPCMD_NONE
// Inputs/Outputs:
//   m_ledStripLen:I
//   m_numPixelBytes:I
//   m_radioInBuf:I
//   m_radioInBufPos:IO
//   m_updatePixels:O
//-----------------------------------------------------------------------------
WaspCmd_t waspCmdHdlrSwap(void)
{
  uint8   *pixels;
  uint8   *buf;
  uint8    old_r, old_g, old_b, r, g, b;
  uint32   oldColour;
  uint32   tmpColour;
  uint16   i;
  
  // Retrieve parameters
  buf = &m_radioInBuf[m_radioInBufPos];
  old_r = *buf++;
  old_g = *buf++;
  old_b = *buf++;
  r = *buf++;
  g = *buf++;
  b = *buf++;
  m_radioInBufPos += 6;
  
  oldColour = mapToColour(old_r, old_g, old_b);
  pixels = &m_pPixels[0];
  for (i = m_ledStripLen; i != 0; i--)
  {
    tmpColour =   ((uint32)*pixels << 16) | ((uint32)*(pixels+1) << 8)  | *(pixels+2);
    if (tmpColour == oldColour)
    {
      pixels[m_offsRed]   = r;
      pixels[m_offsGreen] = g;
      pixels[m_offsBlue]  = b;
    }
    pixels += LEDS_PER_PIX;
  }
  m_updatePixels = true;
  
  return WASPCMD_NONE;
}


//-----------------------------------------------------------------------------
// Function: waspCmdHdlrReset
//   Reset the node.
// Parameters:     (none)
// Returns:  WASPCMD_NONE
// Inputs/Outputs:
//   m_radioInBuf:I
//   m_radioInBufPos:IO
//-----------------------------------------------------------------------------
WaspCmd_t waspCmdHdlrReset(void)
{
  uint8   *buf;
  char    tmpStr[5];
  
  // Retrieve parameters
  buf = &m_radioInBuf[m_radioInBufPos];
  tmpStr[0] = *buf++;
  tmpStr[1] = *buf++;
  tmpStr[2] = *buf++;
  tmpStr[3] = *buf++;
  m_radioInBufPos += 4;

  tmpStr[4] = '\0';

  if (strcmp(tmpStr, "DEAD") == 0)
  {
    cmdHdlrReset();
  }

  return WASPCMD_NONE;
}


//-----------------------------------------------------------------------------
// Function: waspCmdHdlrSpeed
//   Adjust the speed of the current animated effect (e.g. RAINBOW()).
// Parameters:     (none)
// Returns:  WASPCMD_NONE
// Inputs/Outputs:
//   m_radioInBuf:I
//   m_radioInBufPos:IO
//   m_fxDelay:O
//   m_fxResumeTime:O
//   m_fxStep:O
//   m_runAnimation:O
//-----------------------------------------------------------------------------
WaspCmd_t waspCmdHdlrSpeed(void)
{
  uint8 *buf;
  uint8  tmpDelay;

  // Retrieve parameters
  buf = &m_radioInBuf[m_radioInBufPos];
  tmpDelay = *buf++;
  m_radioInBufPos += 1;

  if (0 == tmpDelay)
  {
    m_runAnimation = false;
  }
  else if (1 == tmpDelay)
  {
    m_runAnimation = false;
    m_fxStep = true;
  }
  else
  {
    m_runAnimation = true;
    m_fxDelay = tmpDelay - 2;
    m_fxResumeTime = millis();
  }

  return WASPCMD_NONE;
}


//-----------------------------------------------------------------------------
// Function: waspCmdHdlrRainbow
//   Enable the animated rainbow effect.
// Parameters:     (none)
// Returns:   WASPCMD_NONE
// Inputs/Outputs:
//   m_radioInBuf:I
//   m_radioInBufPos:IO
//   m_fxParam1:O
//   m_fxRestart:O
//   m_runningFx:O
//-----------------------------------------------------------------------------
WaspCmd_t waspCmdHdlrRainbow(void)
{
  uint8 *buf;

  // Retrieve parameters
  buf = &m_radioInBuf[m_radioInBufPos];
  m_fxParam1 = *buf++;
  m_radioInBufPos += 1;
  
  m_runningFx = WASPCMD_RAINBOW;
  m_fxRestart = true;
  return WASPCMD_NONE;
}


//-----------------------------------------------------------------------------
// Function: waspCmdHdlrRainCycle
//   Enable the animated rainbow cycle effect where the pixels always span
//   the maximum range of rainbow colours.
// Parameters:     (none)
// Returns:   WASPCMD_NONE
// Inputs/Outputs:
//   m_fxRestart:O
//   m_runningFx:O
//-----------------------------------------------------------------------------
WaspCmd_t waspCmdHdlrRainCycle(void)
{
  m_runningFx = WASPCMD_RAINCYCLE;
  m_fxRestart = true;
  return WASPCMD_NONE;
}


//-----------------------------------------------------------------------------
// Function: waspCmdHdlrTwinkle
//   Enable the animated twinkle effect.
// Parameters:     (none)
// Returns:   WASPCMD_NONE
// Inputs/Outputs:
//   m_radioInBuf:I
//   m_radioInBufPos:IO
//   m_fxParam1:O  - minDly (in 100's of milliseconds)
//   m_fxParam2:O  - maxDly (in 100's of milliseconds)
//   m_fxParam3:O  - burst
//   m_fxParam4:O  - hold (in milliseconds)
//   m_fxRestart:O
//   m_runningFx:O
//-----------------------------------------------------------------------------
WaspCmd_t waspCmdHdlrTwinkle(void)
{
  uint8 *buf;

  // Retrieve parameters
  buf = &m_radioInBuf[m_radioInBufPos];
  m_fxParam1 = *buf++;
  m_fxParam2 = *buf++;
  m_fxParam3 = *buf++;
  m_fxParam4 = *buf++;
  m_radioInBufPos += 4;

  // Adjust the parameter values
  m_fxParam1 *= 100;
  m_fxParam2 *= 100;

  m_runningFx = WASPCMD_TWINKLE;
  m_fxRestart = true;
  return WASPCMD_NONE;
}


//-----------------------------------------------------------------------------
// Function: waspCmdHdlrPing
//   Respond to a PING with status information.
// Parameters:     (none)
// Returns:   WASPCMD_NONE
// Inputs/Outputs:
//   m_ledStripCtrlPin:I
//   m_ledStripLen:I
//   m_ledStripFreq:I
//   m_ledStripWiring:I
//   m_myNodeId:I
//   m_srcNodeId:I
//   m_radioOutBuf:O
//   m_radioOutBufPos:O
//-----------------------------------------------------------------------------
WaspCmd_t waspCmdHdlrPing(void)
{
  static boolean waitingToTx = false;
  static uint32  txTime = 0;
  uint8 *buf;

  if (!waitingToTx)
  {
    /* Initial processing phase */
    if (CONTROLLERID == m_srcNodeId)
    {
      waitingToTx = true;
      txTime = millis() + (m_myNodeId - FIRST_SLAVE) * SLAVE_PING_TX;
      return WASPCMD_PING;
    }
  }
  
  if (waitingToTx)
  {
    /* Is it time for me to respond yet? */
    if (millis() < txTime)
    {
      /* It isn't time yet */
      return WASPCMD_PING;
    }
    else
    {
      /* Ok, now we can send our response back to the WASP controller */
      waitingToTx = false;
      txTime = 0;
      buf = &m_radioOutBuf[0];
      *buf++ = WASPCMD_PING;
      *buf++ = FW_VERSION_c;
      *buf++ = SW_VERS_MAJ_c;
      *buf++ = SW_VERS_MIN_c;
      *buf++ = m_ledStripCtrlPin;
      *buf++ = m_ledStripLen;
      *buf++ = m_ledStripFreq;
      *buf++ = m_ledStripWiring;
      m_radioOutBufPos = 8;
      radioSendBuf(CONTROLLERID, m_radioOutBuf, m_radioOutBufPos);
    }
  }
 
  return WASPCMD_NONE;
}


//-----------------------------------------------------------------------------
// Function: waspCmdHdlrCfgNode
//   Modify my node ID, saving the new value to EEPROM.
// Parameters:     (none)
// Returns:   WASPCMD_NONE
// Inputs/Outputs:
//   m_dstNodeId:I
//   m_radioInBuf:I
//   m_radioInBufPos:I
//   m_myNodeId:IO
//   m_resetRequired:O
//-----------------------------------------------------------------------------
WaspCmd_t waspCmdHdlrCfgNode(void)
{
  uint8 *buf;
  uint8  newNodeId;

  if (m_dstNodeId != m_myNodeId)
  {
    return WASPCMD_NONE;
  }
  
  // Check for magic number
  buf = &m_radioInBuf[m_radioInBufPos];
  if ('W' != *buf++)
    return WASPCMD_NONE;
  if ('A' != *buf++)
    return WASPCMD_NONE;
  if ('S' != *buf++)
    return WASPCMD_NONE;
  if ('P' != *buf++)
    return WASPCMD_NONE;

  // Retrieve parameters
  newNodeId = *buf++;

  if ( (newNodeId >= FIRST_SLAVE) && (newNodeId <= NODEID_MAX) )
  {
    m_myNodeId = newNodeId;
    EEPROM.write(EEPROM_NODEID_ADDR, newNodeId);
    EEPROM.write(EEPROM_VALIDITY_ADDR, 0); // ID change invalidates data
    m_resetRequired = true;
    logPrint(FLASH("Node ID #"));
    logPrint(m_myNodeId);
    logPrintln(FLASH(" saved to EEPROM.\nPOWER CYCLE THE NODE."));
  }
  else
  {
    logPrintln(FLASH("*** Node ID value out of range."));
  }

  return WASPCMD_NONE;
}


//-----------------------------------------------------------------------------
// Function: waspCmdHdlrCfgCtrl
//   Modify the digital output pin used for controlling the pixel string.
// Parameters:     (none)
// Returns:   WASPCMD_NONE
// Inputs/Outputs:
//   m_dstNodeId:I
//   m_ledStripCtrlOk:I
//   m_myNodeId:I
//   m_radioInBuf:I
//   m_radioInBufPos:I
//   m_ledStripCtrlPin:O
//-----------------------------------------------------------------------------
WaspCmd_t waspCmdHdlrCfgCtrl(void)
{
  uint8 *buf;
  uint8  newPinNum;

  if (m_dstNodeId != m_myNodeId)
  {
    return WASPCMD_NONE;
  }
  
  // Check for magic number
  buf = &m_radioInBuf[m_radioInBufPos];
  if ('W' != *buf++)
    return WASPCMD_NONE;
  if ('A' != *buf++)
    return WASPCMD_NONE;
  if ('S' != *buf++)
    return WASPCMD_NONE;
  if ('P' != *buf++)
    return WASPCMD_NONE;

  newPinNum = *buf++;

  /* Confirm that the new pin number is valid */
  if (m_ledStripCtrlOk[newPinNum])
  {
    if ( IS_MEGA || (newPinNum <= MAX_DIO_PIN) )
    {
      /* We can update the control pin */
      m_ledStripCtrlPin = newPinNum;
    }
  }
  return WASPCMD_NONE;
}


//-----------------------------------------------------------------------------
// Function: waspCmdHdlrCfgLed
//   Modify the pixel string's parameters.
// Parameters:     (none)
// Returns:   WASPCMD_NONE
// Inputs/Outputs:
//   m_dstNodeId:I
//   m_myNodeId:I
//   m_radioInBuf:I
//   m_radioInBufPos:I
//   m_ledStripFreq:O
//   m_ledStripWiring:O
//   m_ledStripLen:O
//-----------------------------------------------------------------------------
WaspCmd_t waspCmdHdlrCfgLed(void)
{
  uint8 *buf;
  uint8 newLen;
  uint8 newFreq;
  uint8 newWiring;

  if (m_dstNodeId != m_myNodeId)
  {
    return WASPCMD_NONE;
  }

  // Check for magic number
  buf = &m_radioInBuf[m_radioInBufPos];
  if ('W' != *buf++)
    return WASPCMD_NONE;
  if ('A' != *buf++)
    return WASPCMD_NONE;
  if ('S' != *buf++)
    return WASPCMD_NONE;
  if ('P' != *buf++)
    return WASPCMD_NONE;

  newLen    = *buf++;
  newFreq   = *buf++;
  newWiring = *buf++;

  /* Validate parameters */
  if ( (newLen > MAX_PIXELS) || (0 == newLen) )
    return WASPCMD_NONE;

  if ( (newFreq != 4) && (newFreq != 8) )
    return WASPCMD_NONE;

  if (newWiring > WIRING_MAX_VAL)
    return WASPCMD_NONE;

  m_ledStripFreq   = newFreq;
  m_ledStripWiring = newWiring;
  m_ledStripLen    = newLen;

  return WASPCMD_NONE;
}


//-----------------------------------------------------------------------------
// Function: waspCmdHdlrCfgSave
//   Save the LED control pin and pixel string parameters to EEPROM.
// Parameters:     (none)
// Returns:   WASPCMD_NONE
// Inputs/Outputs: (none)
//-----------------------------------------------------------------------------
WaspCmd_t waspCmdHdlrCfgSave(void)
{
  EepromSave();
  return WASPCMD_NONE;
}


//-----------------------------------------------------------------------------
// Function: processWaspCommand
//   Execute the most recent pixel strip command.
// Parameters: (none)
// Returns:    (none)
// Inputs/Outputs:
//   m_waspCmd:I
//   m_pxCmdHdlrs:I
//   m_runningFx:O
// Note:
//   The function keeps track of whether or not command handling has completed.
//   Special effects typically complete their handling right away, but continue
//   to run until another command is received.
//-----------------------------------------------------------------------------
void processWaspCommand(void)
{
  static WaspCmd_t cmdInProgress = WASPCMD_NONE;
  WaspCmd_t currentCmd;

  currentCmd = (WASPCMD_NONE == cmdInProgress ? m_waspCmd : cmdInProgress);
  if ( (WASPCMD_NONE != currentCmd) &&
       (WASPCMD_SPEED != currentCmd) &&
       (WASPCMD_STATE != currentCmd)
     )
  {
    m_runningFx = WASPCMD_NONE;
  }

  cmdInProgress = (*m_pxCmdHdlrs[currentCmd])();
}



//-----------------------------------------------------------------------------
// Function: colourWheel
//   Select a colour from a wheel of 256 colours that range from
//   red to green to blue and back to red again.

//   pure red (0), thru red/greens to pure green (85), thru green/blues to
//   pure blue (170), thru blue/reds to pure red (255 -> 0) again.
// Parameters:
//   wheelPos:I  - Colour wheel slot:
//                      0 = red
//                     42 = yellowish (red/green)
//                     85 = green
//                    127 = cyan (green/blue)
//                    170 = blue
//                    212 = purple (blue/red)
// Returns:   WASPCMD_NONE
// Inputs/Outputs:
//   m_pStrip:I
//-----------------------------------------------------------------------------
uint32 colourWheel(uint8 wheelPos)
{
  if(wheelPos < 85)
  {
   return mapToColour(255 - wheelPos * 3, wheelPos * 3, 0);
  }
  else if(wheelPos < 170)
  {
   wheelPos -= 85;
   return mapToColour(0, 255 - wheelPos * 3, wheelPos * 3);
  }
  else
  {
   wheelPos -= 170;
   return mapToColour(wheelPos * 3, 0, 255 - wheelPos * 3);
  }
}


//-----------------------------------------------------------------------------
// Function: fxHdlrNull
//   The empty special FX handler.
// Parameters:     (none)
// Returns:   WASPCMD_NONE
// Inputs/Outputs:
//-----------------------------------------------------------------------------
WaspCmd_t fxHdlrNull(void)
{
  return WASPCMD_NONE;
}


//-----------------------------------------------------------------------------
// Function: fxHdlrRainbow
//   Run an iteration of the animated rainbow effect.
// Parameters:     (none)
// Returns:   WASPCMD_NONE
// Inputs/Outputs:
//   m_fxParam1:I
//   m_ledStripLen:I
//   m_fxRestart:IO
//   m_pPixels:O
//   m_pStrip:O
//   m_updatePixels:O
//-----------------------------------------------------------------------------
WaspCmd_t fxHdlrRainbow(void)
{
  static uint16 j = 255;
  uint32   colour;
  uint8   *pixels;
  uint16   i = 0;

  if (m_fxRestart)
  {
    m_fxRestart = false;
    j = m_fxParam1;
  }
  else
  {
    if (++j >= 256)
      j = 0;
  }

  pixels = &m_pPixels[0];
  for (i = 0; i < m_ledStripLen; i++)
  {
    colour = colourWheel((i+j) & 0xFF);
    pixels[m_offsRed]   = (uint8)((colour & 0x00FF0000) >> 16);
    pixels[m_offsGreen] = (uint8)((colour & 0x0000FF00) >>  8);
    pixels[m_offsBlue]  = (uint8)((colour & 0x000000FF) >>  0);
    pixels += LEDS_PER_PIX;
  }

  m_updatePixels = true;
  return WASPCMD_NONE;
}


//-----------------------------------------------------------------------------
// Function: fxHdlrRainbowCycle
//   Run an iteration of the animated rainbow cycle effect.
// Parameters:     (none)
// Returns:   WASPCMD_NONE
// Inputs/Outputs:
//   m_ledStripLen:I
//   m_fxRestart:IO
//   m_pPixels:O
//   m_pStrip:O
//   m_updatePixels:O
//-----------------------------------------------------------------------------
WaspCmd_t fxHdlrRainbowCycle(void)
{
  static uint16 j = 255;
  uint32   colour;
  uint8   *pixels;
  uint16   i = 0;

  if (m_fxRestart)
  {
    m_fxRestart = false;
    j = 0;
  }
  else
  {
    if (++j >= 256)
      j = 0;
  }

  pixels = &m_pPixels[0];
  for (i = 0; i < m_ledStripLen; i++)
  {
    colour = colourWheel(((i * 256 / m_ledStripLen) + j) & 0xFF);
    pixels[m_offsRed]   = (uint8)((colour & 0x00FF0000) >> 16);
    pixels[m_offsGreen] = (uint8)((colour & 0x0000FF00) >>  8);
    pixels[m_offsBlue]  = (uint8)((colour & 0x000000FF) >>  0);
    pixels += LEDS_PER_PIX;
  }

  m_updatePixels = true;
  return WASPCMD_NONE;
}


//-----------------------------------------------------------------------------
// Function: fxHdlrTwinkle
//   Run an iteration of the animated twinkling effect.
// Parameters:     (none)
// Returns:   WASPCMD_NONE
// Inputs/Outputs:
//   m_fxParam1:I  - minDly time
//   m_fxParam2:I  - maxDly time
//   m_fxParam3:I  - burst size
//   m_fxParam4:I  - hold time
//   m_ledStripLen:I
//   m_fxRestart:IO
//   m_pPixels:O
//   m_pStrip:O
//   m_updatePixels:O
//-----------------------------------------------------------------------------
WaspCmd_t fxHdlrTwinkle(void)
{
  const  uint32  k_white = mapToColour(255,255,255);
  static uint16  i = 0;
  static uint16  j = 0;
  static boolean newIteration = false;
  static uint8   phase = 0;
  static uint32  delayEnd = 0;
  static uint16  numSimultaneous;
  uint32 currTime = 0;
  uint16 k;
  
  if (m_fxRestart)
  {
    m_fxRestart = false;
    restorePixels();
    newIteration = true;
    m_updatePixels = true;
  }
  
  if (newIteration)
  {
    newIteration = false;
    i = 0;
    phase = 1;
    numSimultaneous = random(1, m_fxParam3);
  }
  else if (i < 10)
  {
    switch (phase)
    {
      case 1:
        // Set white pixels randomly
        for (j = 0; j < m_fxParam3; j++)
        {
          k = (uint16_t)random(0, m_ledStripLen);
          m_pStrip->setPixelColor(k, k_white);
        }
        m_updatePixels = true;
        phase = 2;
        delayEnd = millis() + m_fxParam4;
        break;
      
      case 2:
        // Hold the twinkle; account for timer wraparound
        currTime = millis();
        if (delayEnd >= m_fxParam4)
        {
          if (currTime < delayEnd)
            break;
        }
        else
        {
          if (currTime > delayEnd)
            break;
        }
        phase = 3;
        break;
      
      case 3:
        // Remove white pixels (revert to all background colour)
        restorePixels();
        m_updatePixels = true;
        delayEnd = millis() + random(m_fxParam1, m_fxParam2);
        phase = 4;
        break;
      
      case 4:
        // Delay the next twinkle; account for timer wraparound
        currTime = millis();
        if (delayEnd >= m_fxParam4)
        {
          if (currTime < delayEnd)
            break;
        }
        else
        {
          if (currTime > delayEnd)
            break;
        }
        i++;
        if (i >= 10)
          newIteration = true;
        phase = 1;
        break;
      
      default:
        logPrint(FLASH("ERROR: Twinkle entered default case"));
        break;
    }
  }

  return WASPCMD_NONE;
}



//-----------------------------------------------------------------------------
// Function: setup
//   Initialize Arduino.
// Parameters: (none)
// Returns: (none)
//-----------------------------------------------------------------------------
void setup()
{
#ifdef CONSOLE_ENABLED
  Serial.begin(SERIAL_BAUD);
#endif
  m_myNodeId = EEPROM.read(EEPROM_NODEID_ADDR);
  m_resetRequired = false;

  // Initialize RFM69 radio
  m_radio.initialize(FREQUENCY, m_myNodeId, NETWORKID);
#ifdef IS_RFM69HW
  m_radio.setHighPower();
#endif
  //m_radio.encrypt(ENCRYPTKEY);
  m_radio.promiscuous(m_promiscuousMode);
  
  m_resetCount = EEPROM.read(EEPROM_RESET_COUNT_ADDR) + 1;
  
  // Check if we need to clear out EEPROM and start from scratch
  m_resetEeprom = (EEPROM.read(EEPROM_FW_ADDR) != FW_VERSION_c);
  if (m_resetEeprom)
  {
    // Record new F/W version and start with blank parameters
    EEPROM.write(EEPROM_FW_ADDR, FW_VERSION_c);
    EEPROM.write(EEPROM_VALIDITY_ADDR, 0);  // dataIsValid = false;
    m_resetCount = 0;
  }
  else
  {
    // Read in parameters stored in EEPROM
    EepromLoad();
  }
  EEPROM.write(EEPROM_RESET_COUNT_ADDR, m_resetCount);

#ifdef CONSOLE_ENABLED
  Serial.print(FLASH("\nWireless Addressable Strings of Pixels (WASP) network"));
  Serial.print(FLASH("...   Node #"));
  Serial.print(m_myNodeId);
  Serial.println(FLASH("  (Slave)"));
  Serial.print(FLASH("WASP slave S/W: "));
  Serial.print((float)SW_VERS_MAJ_c + (float)SW_VERS_MIN_c / 100.0, 2);
  Serial.print(FLASH(" "));
  Serial.print(SW_DATE_c);
  Serial.print(FLASH("  F/W: "));
  Serial.print(FW_VERSION_c);
  Serial.print(FLASH("\t"));
  Serial.println(COPYRIGHT);
  Serial.print(FLASH("Radio frequency: "));
  Serial.print(FREQUENCY==RF69_433MHZ ? 433 :
                 FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.print(FLASH("Mhz"));
  Serial.print(FLASH("\t  #Resets: "));
  Serial.print(m_resetCount);
  Serial.print(FLASH("\tRAM: "));
  Serial.print(RamSize());
  Serial.print(FLASH(" bytes   "));
  CheckRam();
  Serial.println();

  #ifndef LOGGING_ON
    Serial.println();
    Serial.println(FLASH("CONSOLE OUTPUT IS DISABLED. To enable output, "
                         "recompile with LOGGING_ON"));
    Serial.println(FLASH("and SERIAL_CMDS_ENABLED defined, and reflash the "
                         "node."));
  #endif
#endif

  // Initialize external add-on Flash memory, if available
  Serial.print("SPI Flash for wireless software upgrades: ");
  if (m_flash.initialize())
    Serial.println("located onboard");
  else
    Serial.println("absent");

  #ifdef SERIAL_CMDS_ENABLED
    logPrintln();
    logPrintln(FLASH("Enter 'h' for help"));
  #endif
  
 
  // Parameter 1 = number of pixels in strip
  // Parameter 2 = Arduino pin number (most are valid)
  // Parameter 3 = pixel type flags, add together as needed:
  //   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
  //   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
  //   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
  //   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
  // JVS: On the 12V strip that I have, the colour order is RBG rather than RGB.
  m_ledStripFlags = WIRING_RGB;
  switch (m_ledStripWiring)
  {
    case WIRING_RGB:
      m_offsRed   = 0;
      m_offsGreen = 1;
      m_offsBlue  = 2;
      break;
    case WIRING_RBG:
      m_offsRed   = 0;
      m_offsGreen = 2;
      m_offsBlue  = 1;
      break;
    case WIRING_GRB:
      m_offsRed   = 1;
      m_offsGreen = 0;
      m_offsBlue  = 2;
      break;
    case WIRING_GBR:
      m_offsRed   = 2;
      m_offsGreen = 0;
      m_offsBlue  = 1;
      break;
    case WIRING_BRG:
      m_offsRed   = 1;
      m_offsGreen = 2;
      m_offsBlue  = 0;
      break;
    case WIRING_BGR:
      m_offsRed   = 2;
      m_offsGreen = 1;
      m_offsBlue  = 0;
      break;

  }
  m_ledStripFlags = NEO_RGB;
  if (m_ledStripFreq == FREQ_IDX_800kHz)
    m_ledStripFlags += NEO_KHZ800;
  else
    m_ledStripFlags += NEO_KHZ400;
  if (m_ledStripLen >= 1)
  {
    // Apparently valid configuration parameters
    m_pStrip = new Adafruit_NeoPixel( m_ledStripLen,
                                      (uint8_t)m_ledStripCtrlPin,
                                      (m_ledStripWiring + m_ledStripFreq) );
  }
  else
  {
    // Configuration parameters appear questionable. Create a default strip.
    m_pStrip = new Adafruit_NeoPixel( 1, NEO_PIN, NEO_RGB + NEO_KHZ800 );
  }

  m_savedLeds = (uint8 *)malloc(m_ledStripLen * LEDS_PER_PIX);
  
  // Get the direct pointer to the pixel data.
  m_pPixels = m_pStrip->getPixels();
  m_numPixelBytes = m_ledStripLen * LEDS_PER_PIX;
  
  for (int8 i = MAX_TIMINGS; i >= 0; i--)
    m_timings[i] = 0;
    
  m_myRespDelay = (m_myNodeId - CONTROLLERID) * SLAVE_TX_WIND;
  
  m_waspExecStart =  millis();

  m_loopRefTime = micros();
  tmrUpdateOmet(TMR_BASELINE);
  m_tmrUpdOverhead = m_timings[TMR_BASELINE];
    
  pinMode(POWER_LED, OUTPUT);
  pinMode(WASP_RX_LED, OUTPUT);
  pinMode(WASP_TX_LED, OUTPUT);
  m_pStrip->show();
}


//-----------------------------------------------------------------------------
// Function: loop
//   Main execution control loop.
// Parameters: (none)
// Returns: (none)
// Note: This function is repeatedly invoked following setup(). It effectively
//       executes as an infinite loop.
//-----------------------------------------------------------------------------
void loop()
{
  m_loopStartTime = micros();
  m_loopStartMs = millis();

  if (m_loopStartMs > m_powerLedOnMs)
  {
    digitalWrite(POWER_LED, HIGH);
    digitalWrite(WASP_RX_LED, LOW);
    digitalWrite(WASP_TX_LED, LOW);
    m_powerLedOffMs = m_loopStartMs + 250;
    m_powerLedOnMs = m_powerLedOffMs + 5000;
  }
  else
  {
    if (m_loopStartMs > m_powerLedOffMs)
    {
      digitalWrite(POWER_LED, LOW);
      m_powerLedOffMs = m_powerLedOnMs;
    }
  }
  
  // Process console commands if console is enabled.
  m_loopRefTime = m_loopStartTime;
  #ifdef SERIAL_CMDS_ENABLED
    processCommandLine();
  #endif
  tmrUpdateOmet(TMR_CONSOLE);
  
  // Listen to RFM radio
  m_loopRefTime = micros();
  receiveRadioWaspCmd();
  tmrUpdateOmet(TMR_RADIO_IN);

  // Process WASP commands
  if (m_waspRunning)
  {
    m_loopRefTime = micros();
    processWaspCommand();
    tmrUpdateOmet(TMR_WASP_EXEC);
  }

  // Iterate on a special effect, if enabled.
  if ( !m_pixelShowSuspend &&
       (   ( (true == m_runAnimation) && (millis() >= m_fxResumeTime) )
        || (true == m_fxStep)
       )
     )
  {
    m_loopRefTime = micros();

    // Run an FX iteration
    (void)(*m_pxFxHdlrs[m_runningFx])();
    
    if (!m_fxStep)
    {
      m_fxResumeTime = millis() + m_fxDelay;
    }
    m_fxStep = false;
    tmrUpdateOmet(TMR_FX_EXEC);
  }
  
  // Generate RFM radio response when necessary (not used this way currently)
  //processRadioOutput();

  // Update LED strip
  m_loopRefTime = micros();
  if (m_updatePixels && !m_pixelShowSuspend)
  {
    m_pStrip->show();
    m_updatePixels = false;
  }
  tmrUpdateOmet(TMR_PXL_UPD);
  
  // Update overall loop timing
  m_loopRefTime = micros();
  if (m_loopRefTime < m_loopStartTime)
    m_loopRefTime = MAXUINT32 - m_loopStartTime + m_loopRefTime;
  else
    m_loopRefTime = m_loopRefTime - m_loopStartTime;
  if (m_loopRefTime > m_timings[TMR_LOOP])
    m_timings[TMR_LOOP] = m_loopRefTime;
}

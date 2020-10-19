/*******************************************************************************
 * Wireless Addressable Strings of Pixels (WASP) controller node
 *
 * Controller for a network of WASP slave nodes that each control an
 * addressable RGB LED pixel strip.
 *
 * This sketch implements a controller for WASP slave nodes and is intended
 * for execution on a (Arduino compatible) Moteino MPU.  It can be used as a
 * driver for an operational WASP network or for  testing WASP slaves.
 * 
 * NOTE: As of version 2.01, the WASP controller is now only compatible with
 *       Moteino Mega boards which have more Flash (128 KiB), RAM (16 KiB),
 *       and EEPROM (4 KiB) memories, versus the regular Moteino's 32 KiB,
 *       2 KiB, and 1 KiB, respectively.
 * 
 * The sketch includes a programming environment, the WASP Interactive
 * Programming Environment (WIPE), that is BASIC-like but closer to being
 * an Assembly language. Programs can be written, stored, and run directly
 * from the Moteino. There are tight memory constraints (600 byte of RAM for
 * editing and execution, and 1021 bytes of EEPROM for multiple program
 * storage). WIPE program execution speed is on the order of 10,000
 * instructions per second.
 * 
 * WIPE Tutorial:
 * =============
 * - WIPE commands, program instructions, and identifiers are case
 *   sensitive.
 *   
 * WIPE Commands:
 *   help                 - Display brief help text.
 *   exit                 - Exit WIPE, back to the console command line I/F.
 *   
 *   load <program-name>  - Load a program from EEPROM into the RAM editor.
 *   save <program-name>  - Save program that's in RAM to non-volatile EEPROM.
 *   erase <program-name> - Erase a program from EEPROM storage.
 *   start <program-name> - Define a program as the startup program when
 *                          Auto-Run is enabled in the command console. To be
 *                          remembered over resets/power-ups, a save command
 *                          must be entered in the command console.
 *   dir                  - List the program's stored in EEPROM.
 *
 *   <statement>          - Immediately execute a WIPE program statement.
 *   <line #> <statement> - Add/edit a WIPE program statement at the given line
 *                          number.
 *   del <line-range>     - Delete program line(s).
 *   list [<line-range> ] - List the specified program lines. If <line-range>
 *                          is omitted, list all program lines.
 *   renum                - Renumber the program lines evenly from 5 thru
 *                          250. This tends to create edit space in the program.
 *
 *   run                  - Run the program currently in the RAM editor. To
 *                          stop the program prematurely, enter 'x' (unquoted)
 *                          in the console; there may be a delay while the
 *                          current program statement finishes executing.
 *   prof                 - Display all currently defined variables and labels
 *                          and their current values.
 *   
 *     where:
 *       <program-name> is an unquoted string of up to 12 characters.
 *       <line #> is a number in the range 1 - 255.
 *       <line-range> is one of:
 *         <n>           Specifies specific line #<n>.
 *         <n> - <m>     Specifies the lines within the range <n> thru <m>.
 *         <n> -         Specifies the lines within the range <n> thru 255.
 *             - <m>     Specifies the lines within the range 1 thru <m>.
 *
 * WIPE Statements:
 *   var <id> : <type>    - Define variable <id> of the given type, adding it
 *                          to the runtime symbol table. 
 *                          NOTES:
 *                            - The symbol table includes labels as well.
 *                            - A maximum of 20 symbols can be defined.
 *   let <id> = <expr>    - Assign the result of an expression to variable <id>.
 *   label <id>           - Marks a branch point in the program.
 *   goto <id>            - Jump to the line labelled <id>
 *   if <expr>            - If <expr> evaluates to TRUE, execute the next
 *                          line; othewise, skip over the next line. (Usually,
 *                          the next line is a goto statement.
 *   print "<string>"     - Print a quoted string to the console (without the
 *                          double quotes). There is no newline except where
 *                          the sequence \n is included in the string.
 *   print <id>           - Print the value of a variable. There is no newline.
 *   <func>(<param-list>) - Execute a function (with its parameters). Refer
 *                          to Functions below.
 *
 *     where:
 *       <id> is an unquoted, alphanumeric character string of up to 4
 *            characters that must start with a letter.
 *       <type> is in {int, float, bool}
 *       <expr> is a simple expression in which binary operators have
 *              equal precedence and are evaluated left-to-right. Unary
 *              operators have precedence over binary operators. Valid
 *              operators are:
 *                 Operator     Applicable Types
 *                 --------     -----------------------------------
 *                     =        (Assignment) all types
 *                     -        (Unary negation) int, float
 *                     !        (Unary negation) bool
 *                     +, -, *  int, float
 *                     /        int (quotient), float
 *                     %        int (remainder of division)
 *                     ||, &&   bool (OR, AND)
 *                     =        (Equality) all types
 *                     <, <=    all types
 *                     >, >=    all types
 *       <param-list> is a comma-seprated list of <expr>. The value of each
 *                    <expr> must be within the range 0 ... 255, or (in some
 *                    cases) -128 ... 127.
 *
 * WIPE Functions:
 *   NOTES:
 *     The broadcast destination address is BROADCASTID (255).
 *     Valid specific node destination addresses (nodeId's) are 2 - 6.
 *     Valid group destination addresses (groupId's) are 128 - 132.
 *   General:
 *     Pause(minutes, seconds, hundredths)
 *        Pause program execution.
 *
 *   WASP Generic (Basic graphics and control):
 *     NOTE: Unless otherwise specified, a dst value can specify either a
 *           single node, a group of nodes (a group Id), or all nodes (255).
 *     Bkgrd(dst, red, green, blue)
 *        Define background colour at the specified destination node(s).
 *     Group(dst, groupId, left, right)
 *        Define a specific node (dst) as a member of group (groupId) with
 *        left neighbouring node (left) and right neighbouring node (right).
 *     Line(dst, red, green, blue, startPixel, length)
 *        Draw a line of the specified colour at the specified destination
 *        node(s), starting a the specfied pixel offset and with the given
 *        number of pixels in length.
 *     Reset(dst)
 *        Reset the specified destination node(s).
 *     Shift(dst, shiftNum)
 *        Shift the pixels at the specified destination node(s) to the
 *        right (if shiftNum is positive) or left (if shiftNum is negative)
 *        by the number of pixels specified by the absolute value of
 *        shiftNum. The pixels that shift out of one node, shift into its
 *        corresponding left or right neighbour.
 *     State(dst, opts)
 *        Send a state control command to the specified destination node(s).
 *        The value of opts is a bitwise OR of the values:
 *          SAVE (0x01), RESTORE (0x02), RESUME (0x04), SUSPEND (0x08).
 *        When RESTORE flag is set, the restore happens before the
 *        suspend/resume action. SAVE and RESTORE, or SUSPEND and RESUME pairs
 *        are not permitted simultaneously.
 *     Swap(dst, r_old, g_old, b_old, r, g, b)
 *        Swap all pixels that have old colour (r_old, g_old, b_old) with
 *        new colour (r, g, b).
 *
 *   WASP Special Effects:
 *     Rain(dst, startOffs)
 *        Run a rainbow effect on the destination node(s) using the specified
 *        pixel offset.
 *     Cycle(dst)
 *        Run a rainbow cycling effect on the destination node(s).
 *     Speed(dst, delay)
 *        Adjust the speed of the special effect currently running on the
 *        specified desintation node(s). The smaller the delay value, the
 *        faster the effect, with the following execptions:
 *           0 - Pause the effect
 *           1 - Single-step the effect.
 *     Twinkle(dst, minDelay, maxDelay, burstSize, holdTime)
 *        Run a twinkling effect at the destination node(s), using the
 *        currently defined background colour; twinkles are in bright
 *        white. (Don't use a background colour of (255,255,255) as this
 *        will conflict with the twinkles. The minDelay and maxDelay values
 *        define the relative time range between random twinkles. The burstSize
 *        value defines the upper bound on the number of pixels that can
 *        twinkle simultaneously on each individual node. The holdTime value
 *        determines how long each twinkle lasts.
 *
 *
 * (The software could be converted to a gateway master node that interfaces
 * with another computer acting as the WASP controller.)
 *
 * History
 * =======
 * Version  Date     Author         Description
 * -------  -------- -------------- -------------------------------------
 * 0.1      18-03-31 J.van Schouwen Initial creation.
 * 0.2      18-04-05 J.van Schouwen Added multi-slave support and more
 *                                    console commands; fixed timing bugs.
 * 0.3      18-04-10 J.van Schouwen Some tuning of timing.
 * 0.4      18-09-25 J.van Schouwen Integrate new, reliable 2018 version of
 *                                    the RFM69 library
 * 0.5      18-09-26 J.van Schouwen Added WASPCMD_RESET, WASPCMD_SPEED, and
 *                                    WASPCMD_RAINBOW handling.
 * 0.6      18-09-27 J.van Schouwen Added WASP slave node groups and
 *                                    WASPCMD_RAINCYCLE handling. 
 * 0.7      18-10-26 J.van Schouwen Added WASP Interactive Programming
 *                                    Environment: WIPE print statement
 *                                    needs to be completed for expressions.
 * 0.8      18-10-27 J.van Schouwen Fixed bug in function execution:
 *                                    m_wipeRunByte wasn't updated correctly.
 * 0.9      18-10-28 J.van Schouwen Added WIPE tutorial comments above.
 *                                    print <expr> still needs to be completed.
 * 0.10     18-11-01 J.van Schouwen Fixed bug in function parameters.
 * 0.11     18-11-02 J.van Schouwen Added print <id> in lieu of print <expr> as
 *                                    expression parsing doesn't work if the
 *                                    expression type isn't known apriori. Also
 *                                    fixed bug in parsing functions. Fixed
 *                                    WIPE console input so that multiple lines
 *                                    of input can be pasted in at-a-time.
 * 0.12     18-11-02 J.van Schouwen Removed handling for run, stop, restart,
 *                                    and perf console commands. Added auto-run
 *                                    capability.
 * 1.00     18-11-02 J.van Schouwen Prototype v0.12 released as v1.00
 * 1.01     18-11-16 J.van Schouwen Added program header comments for Swap().
 *                                    Fixed WASP inter-command Tx timing issue.
 *                                    WIPE Ready msg has bytes used and bytes
 *                                    free. WIPE Pause(): added check for
 *                                    abort 'x' char.
 * 2.00     18-11-16 J.van Schouwen  Prototype v1.01 released as v2.00. This
 *                                    release can work for regular Moteino.
 * 2.01     18-11-16 J.van Schouwen  Moteino Mega changes: Increases EEPROM
 *                                     file space to the max ~4 KiB; increased
 *                                     WIPE program buffer size to match EEPROM
 *                                     max available space. Fixed break char
 *                                     ('x') handling while WIPE program is
 *                                     running.
 * 3.00     18-11-23 J.van Schouwen  Prototype v2.01 released as v3.00. This
 *                                     release can run only on Moteino Mega.
 * 3.01     19-03-10 J.van Schouwen  Added power, programming-mode, and
 *                                     program-running LED indications.
 *                                     Shortened the slave Tx time window length
 *                                     for SWAP command handling.
 * 3.02     19-03-29 J.van Schouwen  Added WASP ping and slave configuration
 *                                     commands.
 * 5.00     19-03-29 J.van Schouwen  Prototype v3.02 released as v5.00.
 * 
 ******************************************************************************/

//Ruler
//345678901234567890123456789012345678901234567890123456789012345678901234567890

#include <RFM69.h>
#include <SPI.h>
#include "WASP_defs.h"
#include <avr/pgmspace.h>
#include <EEPROM.h>

#define COPYRIGHT     "(C)2019, A.J. van Schouwen"
#define SW_VERSION_c  "5.00 (2019-03-29)"
#define FW_VERSION_c  7   // Increment (with wraparound) for new F/W;
                          //   clears EEPROM.

#define CONSOLE_ENABLED     // Uncomment to enable the console
#ifdef CONSOLE_ENABLED
  //#define DEBUG_ON            // Uncomment to turn off debug output to serial port.
  #define LOGGING_ON          // Uncomment to turn off logging to serial port.
  #define SERIAL_CMDS_ENABLED // Enables command line at serial port
#endif

// I/O pin definitions
#define POWER_LED     17  // Blinking power indicator LED (red)
#define WIPE_PRGM_LED 18  // Blinking programming-mode indicator LED (green)
#define WIPE_RUN_LED  19  // Program-running indicator (blue)

#define MAX_SERIAL_BUF_LEN           80
#define EEPROM_FW_ADDR                0
#define EEPROM_VALIDITY_ADDR          1
#define EEPROM_AUTORUN_ADDR           2
#define EEPROM_AUTORUM_DELAY_ADDR     3
#define EEPROM_RUNPROG_ADDR           4  // 12 characters (null-terminated if
                                         //   shorter than that).
#define EEPROM_FIRST_OPEN_ADDR        (EEPROM_RUNPROG_ADDR + 12)
#define EEPROM_RESET_COUNT_ADDR    4095  // Last byte of EEPROM

// Storage space for WIPE program files and directory
#define EEPROM_PROG_DIR_START  (EEPROM_FIRST_OPEN_ADDR + 1)
#define EEPROM_PROG_LEN        (EEPROM_RESET_COUNT_ADDR - EEPROM_PROG_DIR_START)

#define SERIAL_BAUD                9600

#define DEL_CHAR                   0x7F  // ASCII Del character


// Command modes
#define CMDMODE_CONSOLE   0  // Console command mode
#define CMDMODE_WIPE      1  // WASP Interactive Programming Environment submode


// Console command processing stages
#define CMD_PROMPT        0  // Display command prompt
#define CMD_INPUT         1  // Scan for complete line of input
#define CMD_PARSE         2  // Parse input line for command name
#define CMD_EXEC          3  // Execute the command


// WIPE command processing stages
#define PRG_PROMPT        0  // Display command prompt
#define PRG_INPUT         1  // Scan for complete line of input
#define PRG_PARSE         2  // Parse the command line

#define DFLT_AUTORUN_DELAY  10 // Default autorun delay (in seconds)


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
boolean m_resetEeprom = false;
boolean m_resetRequired = false;
boolean m_autoRun = false;        // Preserve value, except via console command.
boolean m_triggerAutoRun = false;
uint8   m_autoRunDelay = DFLT_AUTORUN_DELAY;
uint8   m_resetCount;
uint32  m_autoRunStart = 0;  // Auto-run startup time (in milliseconds).


/*-----------------------------  RFM69 State  --------------------------------*/
// Node ID.
// Must be unique for each node. Gateway is always 1.
uint8   m_myNodeId = CONTROLLERID;  // Must be unique for each node 

RFM69   m_radio;
boolean m_promiscuousMode = true;  // sniff all packets on network iff true
uint8   m_dstNodeId;
uint8   m_srcNodeId;

uint8   m_radioOutBuf[RF69_MAX_DATA_LEN];
uint8   m_radioOutBufPos = 0;
uint8   m_radioInBuf[RF69_MAX_DATA_LEN];
uint8   m_radioInBufPos = 0;

boolean m_ackRequested = false;


/*------------------------------  WASP State ---------------------------------*/
uint8   m_rxWaspRsp = WASPCMD_NONE;
uint32  m_cmdExecDelay = 0;


/*--------------------------  Console State  ---------------------------------*/
char    m_consoleBuffer[MAX_SERIAL_BUF_LEN];
uint8   m_consolePos = 0;  // Console current character position
uint8   m_consoleLen = 0;  // Console line length
boolean m_verboseWasp = false;
uint8   m_consoleMode = CMDMODE_CONSOLE;


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

boolean cmdHdlrAutoRun(void);
boolean cmdHdlrHelp(void);
boolean cmdHdlrNetLed(void);
boolean cmdHdlrNetLedCtrl(void);
boolean cmdHdlrNetNodeId(void);
boolean cmdHdlrNetPing(void);
boolean cmdHdlrNetReset(void);
boolean cmdHdlrNetSave(void);
boolean cmdHdlrWipe(void);
boolean cmdHdlrQuiet(void);
boolean cmdHdlrReset(void);
boolean cmdHdlrSave(void);
boolean cmdHdlrVerbose(void);


// Command control data. Sort the table alphabetically by command name.
CmdCtrl_t m_cmdCtrl[] =
  {/*   Cmd Name      Handler
        -----------   ----------------- */
      { "autoRun",    cmdHdlrAutoRun    },
      { "h",          cmdHdlrHelp       },
      { "netLed",     cmdHdlrNetLed     },
      { "netLedCtrl", cmdHdlrNetLedCtrl },
      { "netNodeId",  cmdHdlrNetNodeId  },
      { "netPing",    cmdHdlrNetPing    },
      { "netReset",   cmdHdlrNetReset   },
      { "netSave",    cmdHdlrNetSave    },
      { "program",    cmdHdlrWipe       },
      { "quiet",      cmdHdlrQuiet      },
      { "reset",      cmdHdlrReset      },
      { "save",       cmdHdlrSave       },
      { "verbose",    cmdHdlrVerbose    }
  };

const uint8 m_numCmds = sizeof(m_cmdCtrl)/sizeof(CmdCtrl_t);



/*------------  WASP Interpretive Programming Environment (WIPE) ------------*/
/* WIPE Environment Commands */
#define WIPE_UNDEF    0  // Undefined token

/* WIPE language keyword token Ids
 *  - WIPE WASP commands reuse their corresponding WASP protocol command
 *    codes and command syntax:
 *      e.g. GROUP <dst> <grpId> <left> <right>
 *            - The token Id for GROUP is WASPCMD_GROUP (1).
 */
#define WIPE_BASE   (LAST_NONCFG_CMD + 1)
#define WIPE_CMD_EXIT    (WIPE_BASE +  0) // Exit WIPE and go back to
                                          //   console command mode.
#define WIPE_CMD_HELP    (WIPE_BASE +  1) // Display help information.
#define WIPE_CMD_LINENUM (WIPE_BASE +  2) // A numbered program line vs
                                          //   interpreted cmd.
#define WIPE_CMD_SAVE    (WIPE_BASE +  3) // Save program:  SAVE <name>
#define WIPE_CMD_ERASE   (WIPE_BASE +  4) // Erase program: ERASE <name>
#define WIPE_CMD_DIR     (WIPE_BASE +  5) // List all saved programs:  DIR
                                          //   Result is a program list:
                                          //     #1  Program-1
                                          //     #2  Program-2
                                          //     ...
#define WIPE_CMD_LOAD    (WIPE_BASE +  6) // Load program:  LOAD <#>
#define WIPE_CMD_LIST    (WIPE_BASE +  7) // List program lines:  LIST
#define WIPE_CMD_RENUM   (WIPE_BASE +  8) // Renumber the program lines:
                                          //   RENUM [n]
                                          //   - Lines are renumbered
                                          //     n, 2n, 3n, 4n, ...
                                          //   - If n omitted, n = 5.
#define WIPE_CMD_DEL     (WIPE_BASE +  9) // Delete lines <m> thru <n>: DEL <m> <n>
#define WIPE_CMD_RUN     (WIPE_BASE + 10) // Run current program: RUN
#define WIPE_CMD_START   (WIPE_BASE + 11) // Define startup program: START
#define WIPE_CMD_PROF    (WIPE_BASE + 12) // Show profile of current variables.
#define LAST_WIPE_CMD    WIPE_CMD_PROF


/* WIPE language keyword token Ids */
#define KEY_BASE      (LAST_WIPE_CMD + 1)
#define WIPE_UNDEF                 0  // Undefined token
#define WIPE_FUNC     (KEY_BASE +  1) // Function call placeholder.
#define WIPE_LABEL    (KEY_BASE +  2) // Define a label: LABEL <id>
#define WIPE_GOTO     (KEY_BASE +  3) // Control transfer: GOTO <label-id>
#define WIPE_IF       (KEY_BASE +  4) // IF statement: IF <condition>
#define WIPE_LET      (KEY_BASE +  5) // Assignement: LET <id> = <expression>
#define WIPE_PRINT    (KEY_BASE +  6) // Print a value/string to the console.
#define WIPE_VAR      (KEY_BASE +  7) // Declare a variable: VAR <id>:<type>

/* Misc WIPE functions */
#define WIPE_FN_PAUSE (KEY_BASE +  8) // Pause execution

/* WIPE command not-executed flag
 *  - When bit 8 of the line's statement type byte is set, the line hasn't
 *    yet been executed.
 */
#define WIPE_NO_EXEC      0x80  // Command execution flag

/*---  Identifier types  ---*/
#define WIPE_ID_UNDEF      0
#define WIPE_ID_LABEL      1
#define WIPE_ID_INT        2
#define WIPE_ID_FLOAT      3
#define WIPE_ID_BOOL       4

/*---  Print argument types  ---*/
#define WIPE_PRT_STR       0   // Print a string
#define WIPE_PRT_VAR       1   // Print the value of a variable

/*---  Operator kinds  ---*/
// Arithmetic operators
#define WIPE_OP_UNDEF      0
#define WIPE_OP_PLUS       1   //  +  addition
#define WIPE_OP_MINUS      2   //  -  subtraction
#define WIPE_OP_MULT       3   //  *  multiplication
#define WIPE_OP_DIV        4   //  /  division (float or integer)
#define WIPE_OP_MOD        5   //  %  modulo
#define WIPE_OP_NEG        6   //  -  unary negation
#define WIPE_OP_COMPL      7   //  !  unary boolean complement
#define WIPE_OP_OR         8   //  || OR
#define WIPE_OP_AND        9   //  && AND

// Relational operators
#define WIPE_OP_EQ        10   //  =  equality
#define WIPE_OP_NE        11   // !=  inequality
#define WIPE_OP_LT        12   //  <  less than
#define WIPE_OP_LE        13   // <=  less than or equal
#define WIPE_OP_GT        14   //  >  greater than
#define WIPE_OP_GE        15   // >=  greater than or equal

#define WIPE_OP_DELIM     16   // , or )  expression list delimiter

#define WIPE_MAX_PARMS     7   // Max # of WIPE function parameters needed


/*---  Operand kinds  ---*/
#define WIPE_OPRND_UNDEF   0
#define WIPE_OPRND_LIT     1   // Literal operand
#define WIPE_OPRND_ID      2   // Variable identifier operand

/*---  Tokenized line constants  ---*/
#define TKNZD_LINE_OFFS    0   // Line # offset from start of tokenized line
#define TKNZD_LEN_OFFS     1   // Line length offset ...
#define TKNZD_STMT_OFFS    2   // Statement Id offset ...

/*---  Program directory constants  ---*/
#define MAX_PROGNAME_LEN  12
#define DIR_LEN_OFFS       0   // Program file length directory entry offset
#define DIR_NAME_OFFS      2   // Program filename directory entry offset
#define DIR_PROG_OFFS      (DIR_NAME_OFFS + MAX_PROGNAME_LEN + 1)

/*---  Misc Parameters  ---*/
#define MAX_LINE_NUM     255  // Program line number can't exceed this value.

/*--  Define symbol table  ---*/
#define MAX_ID_LEN         4
#define MAX_NUM_SYMBOLS  200
#define LABEL_UNDEF        0  // Undefined line # value for a label identifier

typedef union
{
  int32     intValue;
  float     floatValue;
  boolean   boolValue;
  void     *ptrValue;       // Not currently used
} SymValue_t;

typedef struct
{
  char        symbol[MAX_ID_LEN+1];
  uint8       typeId;               // WIPE_ID_LABEL, etc.
  SymValue_t  symValue;             // Symbol value.
  
} SymTblEntry_t;

SymTblEntry_t m_symTbl[MAX_NUM_SYMBOLS];


/*---  Define program memory buffer  ---*/

// Max # bytes of reserved program space in RAM
#define MAX_PROG_SIZE  (EEPROM_RESET_COUNT_ADDR - EEPROM_FIRST_OPEN_ADDR   \
                        - DIR_PROG_OFFS)

uint8 m_program[MAX_PROG_SIZE];

char    m_tokenBuffer[sizeof(m_consoleBuffer)];
char    m_autorunProg[MAX_PROGNAME_LEN+1] = "\0";
uint16  m_wipeProgByte = 0;  // Current byte edit position in program buffer.
uint16  m_wipeLineStart = 0; // Program buffer line start byte position.
uint16  m_wipeRunByte = 0;   // Running offset into program buffer.
uint16  m_wipeDirSpace = 0;  // EEPROM WIPE directory space free (bytes)
uint16  m_wipeSaveAddr = 0;  // First free write byte in EEPROM program space
uint8   m_exprOffs = 0;      // Current offset into an expression.
boolean m_breakProg = false; // true = 'x' entered; stop running program.


/*------------------------  LED Indicator States  ----------------------------*/
uint32   m_loopStartMs = 0;
uint32   m_powerLedOnMs = 0;
uint32   m_powerLedOffMs = 0;
uint8    m_wipeLedState = LOW;


/*--------------------  Forward declarations --------------------*/
void CheckRam(void);
int  serialParseInt(void);
void wipeDirSpaceCheck(void);




/*****************************************************************************
 *  SECTION  **************  Miscellaneous Utilities  ************************
 *****************************************************************************/

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
  if (!dataIsValid)
    return;

  m_autoRun = EEPROM.read(EEPROM_AUTORUN_ADDR);
  m_autoRunDelay = EEPROM.read(EEPROM_AUTORUM_DELAY_ADDR);
  for (uint8 i = 0; i < MAX_PROGNAME_LEN; i++)
    m_autorunProg[i] = EEPROM.read(EEPROM_RUNPROG_ADDR + i);
  m_autorunProg[MAX_PROGNAME_LEN] = '\0'; /* Ensure it's null-terminated */


  addr = EEPROM_FIRST_OPEN_ADDR;
  // Load additional parameters from this point onward.
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

  EEPROM.write(EEPROM_AUTORUN_ADDR, m_autoRun);
  EEPROM.write(EEPROM_AUTORUM_DELAY_ADDR, m_autoRunDelay);

  /* Write the autorun program name. Don't null-terminate if it has
   * the maximum name length.
   */
  for (uint8 i = 0; i < MAX_PROGNAME_LEN; i++)
    EEPROM.write(EEPROM_RUNPROG_ADDR + i, m_autorunProg[i]);


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




/*****************************************************************************
 *  SECTION  **********  Console Command Handling  ***************************
 *****************************************************************************/    
    
//-----------------------------------------------------------------------------
// Function: serialParseInt
//   Scan for an integer value in the conole input buffer.
// Parameters: (none)
// Returns: The integer value scanned, if found; otherwise, returns 0.
// Inputs/Outputs:
//   m_consolePos:IO
//   m_consoleBuffer:I
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
// Function: cmdHdlrAutoRun
//   Enable/disable WIPE program autorun.
// Parameters: (none)
// Returns: true iff command processing has completed.
// Inputs/Outputs:
//   m_autoRun:O
//   m_autoRunDelay:O
//-----------------------------------------------------------------------------
boolean cmdHdlrAutoRun(void)
{
  m_autoRun = serialParseInt();
  m_autoRunDelay = serialParseInt();
  if (!m_autoRun)
    m_triggerAutoRun = false;
  if (0 == m_autoRunDelay)
    m_autoRunDelay = DFLT_AUTORUN_DELAY;
  
    
  logPrint(FLASH("Autorun: "));
  if (m_autoRun)
    logPrintln(FLASH("on"));
  else
    logPrintln(FLASH("off"));
}


//-----------------------------------------------------------------------------
// Function: cmdHdlrSave
//   Save the current set of parameters to EEPROM.
// Parameters: (none)
// Returns: true iff command processing has completed.
// Inputs/Outputs:
//   m_consolePos:IO
//   m_consoleBuffer:IO
//-----------------------------------------------------------------------------
boolean cmdHdlrSave(void)
{
  EepromSave();
  logPrintln(FLASH("Parameters saved to EEPROM"));
  
  return true;
}


//-----------------------------------------------------------------------------
// Function: cmdHdlrWipe
//   Enter WASP Interpretive Programming Environment (WIPE) mode.
// Parameters: (none)
// Returns: true iff command processing has completed.
// Inputs/Outputs:
//   m_consoleMode:O
//-----------------------------------------------------------------------------
boolean cmdHdlrWipe(void)
{
  // Stop WASP commands and reset all nodes.
  waspReset(BROADCASTID);
  m_consoleMode = CMDMODE_WIPE;
  m_wipeProgByte = 0;
  logPrintln(FLASH("\n*\t\t\t\t\t\t*"));
  logPrintln(FLASH("*   WASP Interactive Programming Environment    *"));
  logPrintln(FLASH("*\t\t\t\t\t\t*"));
  wipeDirSpaceCheck();
  logPrintln(FLASH("(Enter 'help')"));
  logPrintln();
}


//-----------------------------------------------------------------------------
// Function: cmdHdlrQuiet
//   Stop the console display of WASP commands being executed.
// Parameters: (none)
// Returns: true iff command processing has completed.
// Inputs/Outputs:
//   m_verboseWasp:O
//-----------------------------------------------------------------------------
boolean cmdHdlrQuiet(void)
{
  m_verboseWasp = false;
  logPrintln(FLASH("WASP quiet"));
  return true;
}


//-----------------------------------------------------------------------------
// Function: cmdHdlrReset
//   Perform a software reset.
// Parameters: (none)
// Returns: true iff command processing has completed.
// Inputs/Outputs: (none)
//-----------------------------------------------------------------------------
boolean cmdHdlrReset(void)
{
  void (*resetFunc)(void) = 0; // Declare reset func @ address 0
  
  resetFunc(); // call reset
}


//-----------------------------------------------------------------------------
// Function: cmdHdlrVerbose
//   Display WASP commands as they're executed to the console.
// Parameters: (none)
// Returns: true iff command processing has completed.
//   m_verboseWasp:O
//-----------------------------------------------------------------------------
boolean cmdHdlrVerbose(void)
{
  m_verboseWasp = true;
  logPrintln(FLASH("WASP verbose"));
  return true;
}


//-----------------------------------------------------------------------------
// Function: cmdHdlrNetReset
//   Reset the remote WASP slave nodes.
// Parameters: (none)
// Returns: true iff command processing has completed.
// Inputs/Outputs: (none)
//-----------------------------------------------------------------------------
boolean cmdHdlrNetReset(void)
{
  uint8 dst;

  dst = serialParseInt();
  if (   (BROADCASTID == dst)
      || ( (dst >= FIRST_SLAVE) && (dst < (FIRST_SLAVE + MAX_SLAVES)) )
      || ( (dst >= FIRST_GROUP) && (dst < (FIRST_GROUP + MAX_GROUPS)) )
     )
  {
    waspReset(dst);
  }
  else
  {
    logPrint(FLASH("***Invalid dst: "));
    logPrintln(dst);
  }
  return true;
}


//-----------------------------------------------------------------------------
// Function: cmdHdlrNetNodeId
//   Change the configured node ID for a WASP slave node.
// Parameters: (none)
// Returns: true iff command processing has completed.
// Inputs/Outputs:
//   m_radioOutBuf:O
//   m_radioOutBufPos:O
// Note: The slave should be reset following this command.
//-----------------------------------------------------------------------------
boolean cmdHdlrNetNodeId(void)
{
  uint8 dst;
  uint8 newNodeId;

  dst = serialParseInt();
  newNodeId = serialParseInt();

  if ( (dst < FIRST_SLAVE) || (dst >= (FIRST_SLAVE + MAX_SLAVES)) )
  {
    logPrintln(FLASH("***Invalid destination specified"));
    return true;
  }

  if ( (newNodeId < FIRST_SLAVE) || (newNodeId >= (FIRST_SLAVE + MAX_SLAVES)) )
  {
    logPrintln(FLASH("***Invalid new slave ID specified"));
    return true;
  }

  m_radioOutBufPos = 0;
  appendRadioOut8(WASPCMD_CFG_NODE);
  appendRadioOut8('W');
  appendRadioOut8('A');
  appendRadioOut8('S');
  appendRadioOut8('P');
  appendRadioOut8(newNodeId);
  (void)radioSendBuf(dst, m_radioOutBuf, m_radioOutBufPos, false);
  if (m_verboseWasp)
  {
    logPrint(FLASH("CFG_NODE ->"));
    logPrintln(dst);
  }
  return true;
}


//-----------------------------------------------------------------------------
// Function: cmdHdlrNetLedCtrl
//   For a WASP slave node, change the configured digital output pin that it
//   uses to control its LED pixel strip.
// Parameters: (none)
// Returns: true iff command processing has completed.
// Inputs/Outputs:
//   m_radioOutBuf:O
//   m_radioOutBufPos:O
// Note: The slave should be reset following this command.
//-----------------------------------------------------------------------------
boolean cmdHdlrNetLedCtrl(void)
{
  uint8 dst;
  uint8 newCtrlPin;

  dst = serialParseInt();
  newCtrlPin = serialParseInt();

  if ( (dst < FIRST_SLAVE) || (dst >= (FIRST_SLAVE + MAX_SLAVES)) )
  {
    logPrintln(FLASH("***Invalid destination specified"));
    return true;
  }

  m_radioOutBufPos = 0;
  appendRadioOut8(WASPCMD_CFG_CTRL);
  appendRadioOut8('W');
  appendRadioOut8('A');
  appendRadioOut8('S');
  appendRadioOut8('P');
  appendRadioOut8(newCtrlPin);
  (void)radioSendBuf(dst, m_radioOutBuf, m_radioOutBufPos, false);
  if (m_verboseWasp)
  {
    logPrint(FLASH(" ->CFG_CTRL"));
    logPrintln(dst);
  }
  return true;
}


//-----------------------------------------------------------------------------
// Function: cmdHdlrNetLed
//   For a WASP slave node, change the configured parameters for its LED
//   pixel strip.
// Parameters: (none)
// Returns: true iff command processing has completed.
// Inputs/Outputs:
//   m_radioOutBuf:O
//   m_radioOutBufPos:O
// Note: The slave should be reset following this command.
//-----------------------------------------------------------------------------
boolean cmdHdlrNetLed(void)
{
  uint8 dst;
  uint8 newLen;
  uint8 newFreq;
  uint8 newWiring;

  dst       = serialParseInt();
  newLen    = serialParseInt();
  newFreq   = serialParseInt();
  newWiring = serialParseInt();

  if ( (dst < FIRST_SLAVE) || (dst >= (FIRST_SLAVE + MAX_SLAVES)) )
  {
    logPrintln(FLASH("***Invalid destination specified"));
    return true;
  }

  m_radioOutBufPos = 0;
  appendRadioOut8(WASPCMD_CFG_LED);
  appendRadioOut8('W');
  appendRadioOut8('A');
  appendRadioOut8('S');
  appendRadioOut8('P');
  appendRadioOut8(newLen);
  appendRadioOut8(newFreq);
  appendRadioOut8(newWiring);
  (void)radioSendBuf(dst, m_radioOutBuf, m_radioOutBufPos, false);
  if (m_verboseWasp)
  {
    logPrint(FLASH(" ->CFG_LED"));
    logPrintln(dst);
  }
  return true;
}


//-----------------------------------------------------------------------------
// Function: cmdHdlrNetPing
//   Query the status of a WASP slave node, group, or all nodes.
// Parameters: (none)
// Returns: true iff command processing has completed.
// Inputs/Outputs:
//   m_radioInBuf:I
//   m_srcNodeId:I
//   m_radioInBufPos:IO
//   m_rxWaspRsp:IO
//   m_radioOutBuf:O
//   m_radioOutBufPos:O
//-----------------------------------------------------------------------------
boolean cmdHdlrNetPing(void)
{
  static boolean pingInProgress = false;
  static uint32  timeout = 0;
  float  swVersion;
  uint8 *buf;
  uint8  dst;

  if (!pingInProgress)
  {
    /* Process a new ping command line */
    dst = serialParseInt();

    if (   (BROADCASTID != dst)
        && ( (dst < FIRST_SLAVE) || (dst >= (FIRST_SLAVE + MAX_SLAVES)) )
        && ( (dst < FIRST_GROUP) || (dst >= (FIRST_GROUP + MAX_GROUPS)) )
       )  {
      logPrintln(FLASH("***Invalid destination specified"));
      return true;
    }

    pingInProgress = true;
    m_radioOutBufPos = 0;
    appendRadioOut8(WASPCMD_PING);
    (void)radioSendBuf(dst, m_radioOutBuf, m_radioOutBufPos, false);
    timeout = millis() + PING_TIMEOUT;
    if (m_verboseWasp)
    {
      logPrint(FLASH(" ->PING"));
      logPrintln(dst);
    }
    return false;
  }
  else
  {
    /* Process responses to previously transmitted WASPCMD_PING */
    if (millis() >= timeout)
    {
      pingInProgress = false;
      logPrintln(FLASH("=> PING done"));
      return true;
    }
    else
    {
      if (WASPCMD_NONE != m_rxWaspRsp)
      {
        logPrint(FLASH("Node #"));
        logPrint(m_srcNodeId);
        logPrint(FLASH("==>  "));
        if (WASPCMD_PING == m_rxWaspRsp)
        {
          buf = &m_radioInBuf[m_radioInBufPos];
          logPrint(FLASH("F/W: "));
          logPrint(*buf++);
          swVersion = (float)*buf++;
          swVersion += (float)*buf++ / 100.0;
          logPrint(FLASH("  S/W: "));
          logPrintEx(swVersion, 2);
          logPrint(FLASH("  Pin: "));
          logPrint(*buf++);
          logPrint(FLASH("\t("));
          logPrint(*buf++);
          logPrint(FLASH(" pixels @ "));
          logPrint(*buf++);
          logPrint(FLASH("00kHz, LED order #"));
          logPrint(*buf++);
          logPrintln(FLASH(")"));
        }
        else
        {
          logPrintln(FLASH("*** bad response ["));
          logPrint(m_rxWaspRsp);
          logPrintln(FLASH("]"));
          m_rxWaspRsp = WASPCMD_NONE;
        }
      }
      return false;
    }
  }
  return true;
}


//-----------------------------------------------------------------------------
// Function: cmdHdlrNetSave
//   Request a WASP slave, group, or all slave to save their configuration
//   data to EEPROM.
// Parameters: (none)
// Returns: true iff command processing has completed.
// Inputs/Outputs:
//   m_radioOutBuf:O
//   m_radioOutBufPos:O
//-----------------------------------------------------------------------------
boolean cmdHdlrNetSave(void)
{
  uint8 dst;

  dst = serialParseInt();

  if (   (BROADCASTID != dst)
      && ( (dst < FIRST_SLAVE) || (dst >= (FIRST_SLAVE + MAX_SLAVES)) )
      && ( (dst < FIRST_GROUP) || (dst >= (FIRST_GROUP + MAX_GROUPS)) )
     )  {
    logPrintln(FLASH("***Invalid destination specified"));
    return true;
  }

  m_radioOutBufPos = 0;
  appendRadioOut8(WASPCMD_CFG_SAVE);
  (void)radioSendBuf(dst, m_radioOutBuf, m_radioOutBufPos, false);
  if (m_verboseWasp)
  {
    logPrint(FLASH(" ->CFG_SAVE"));
    logPrintln(dst);
  }
  return true;
}


//-----------------------------------------------------------------------------
// Function: cmdHdlrHelp
//   Display the serial console port command syntax and synopses.
// Parameters: (none)
// Returns: true iff command processing has completed.
// Inputs/Outputs:
//   m_autoRun:I
//   m_autoRunDelay:I
//-----------------------------------------------------------------------------
boolean cmdHdlrHelp(void)
{
  logPrintln(FLASH("\n\nConsole commands: (spaces can replace commas)"));
  logPrint(FLASH("  autoRun n [,m]"));
      logPrintln(FLASH("\t- Enable/disable WIPE program autorun. Enable if "
                           "n=1."));
      logPrintln(FLASH("\t\t\t  Optional run delay is 0 < m < 255 seconds; "));
      logPrintln(FLASH("\t\t\t  10 is default."));
  logPrint(FLASH("  program"));
      logPrint(FLASH("\t\t- Start the WASP Interactive Programming "));
      logPrintln(FLASH("Environment"));
  logPrint(FLASH("  quiet"));
      logPrintln(FLASH("\t\t\t- Don't display WASP commands being executed"));
  logPrint(FLASH("  reset"));
      logPrintln(FLASH("\t\t\t- Software reset"));
  logPrint(FLASH("  save"));
      logPrintln(FLASH("\t\t\t- Save parameters to EEPROM"));
  logPrint(FLASH("  verbose"));
      logPrintln(FLASH("\t\t- Display WASP commands when running"));
  logPrint(FLASH("  h"));
      logPrintln(FLASH("\t\t\t- Print this help text"));
  logPrintln(FLASH("------------------------------------"));
      logPrintln(FLASH("WASP slave network commands: "
                       "(nodeid 255 = all nodes)"));
  logPrint(FLASH("  netLed n, l, f, w"));
      logPrintln(FLASH("\t- For WASP slave n, configure its LED pixel strip"));
      logPrintln(FLASH("\t\t\t  parameters:"));
      logPrintln(FLASH("\t\t\t    l = # tricolor LED pixels (l > 0)"));
      logPrintln(FLASH("\t\t\t    f = 8, for 800kHz, WS2812 LED drivers"));
      logPrintln(FLASH("\t\t\t        4, for 400kHz, WS2811 drivers"));
      logPrintln(FLASH("\t\t\t    c = 0, for RGB color wiring order (default)"));
      logPrintln(FLASH("\t\t\t        1, for RBG"));
      logPrintln(FLASH("\t\t\t        2, for GRB"));
      logPrintln(FLASH("\t\t\t        3, for GBR"));
      logPrintln(FLASH("\t\t\t        4, for BRG"));
      logPrintln(FLASH("\t\t\t        5, for BGR"));
  logPrint(FLASH("  netLedCtrl n, m"));
      logPrintln(FLASH("\t- Change the control pin for the LED pixel"));
      logPrintln(FLASH("\t\t\t  strip of slave n to digital output pin #m"));
  logPrint(FLASH("  netNodeId n, m"));
      logPrintln(FLASH("\t- Change the nodeid for slave n to m"));
  logPrint(FLASH("  netPing n"));
      logPrintln(FLASH("\t\t- Query a WASP slave, group, or all"));
  logPrint(FLASH("  netReset n"));
      logPrintln(FLASH("\t\t- Reset WASP slave, group, or all."));
  logPrint(FLASH("  netSave n"));
      logPrintln(FLASH("\t\t- Request a WASP slave, group, or all node to"));
      logPrintln(FLASH("\t\t\t  save their configuration settings to EEPROM"));

  CheckRam();

  logPrint(FLASH("Autorun ["));
  if (m_autoRun)
  {
    logPrint(FLASH("ON] Delay: "));
    logPrint(m_autoRunDelay);
    logPrintln(FLASH(" seconds"));
  }
  else
  {
    logPrintln(FLASH("off]"));
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
//   m_numCmds:I
//   m_resetRequired:I
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
      logPrintln(FLASH("\n\nPower cycle node first!\n"));
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
        // We hit end-of-line before match the full candidate command name
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
          logPrintln(FLASH("***SYS ERR: No cmd handler")); 
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
    
    if (!cmdFound)
    {
      /* The command search has ended in failure. Discard the
       * command line and restart scanning.
       */
      logPrint(FLASH("["));
      logPrint(m_consoleBuffer);
      logPrintln(FLASH("]"));
      logPrintln(FLASH("***Cmd not found"));
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
    /* Each command has to parse the command line parameters */
  }
}




/*****************************************************************************
 *  SECTION  ***************  Radio Rx/Tx Functions  *************************
 *****************************************************************************/

//-----------------------------------------------------------------------------
// Function: receiveRadioWaspRsp
//   Check for and pixel commands from the RFM radio network.
// Parameters: (none)
// Returns:    (none)
// Inputs/Outputs:
//   m_myNodeId:I
//   m_radio:I
//   m_dstNodeId:O
//   m_rxWaspRsp:O
//   m_radioBuf:O
//   m_radioBufPos:O
//   m_srcNodeId:O
//-----------------------------------------------------------------------------
void receiveRadioWaspRsp(void)
{
  uint8 waspRsp = WASPCMD_NONE;
  
  if (m_radio.receiveDone())
  {
    // Determine if we're an intended receiver
    m_dstNodeId = m_radio.TARGETID;
    m_srcNodeId = m_radio.SENDERID;
    if (m_dstNodeId == m_myNodeId)
    {
      m_ackRequested = m_radio.ACK_REQUESTED;
      memcpy(m_radioInBuf, (const void *)&m_radio.DATA[0], m_radio.DATALEN);
      m_radioInBufPos = 0;
      waspRsp = m_radioInBuf[m_radioInBufPos++];
      if (waspRsp > MAX_WASPCMD_VAL)
      {
        dbgPrint(FLASH(" ... invalid Rx WASP response ="));
        dbgPrintln(waspRsp);
        waspRsp = WASPCMD_NONE;
      }
      else
      {
        dbgPrint(FLASH("Rx Rsp["));
        dbgPrint(waspRsp);
        dbgPrintln(FLASH("]"));
      }
    }
    else
    {
      dbgPrint(FLASH(" ... ignored (pkt not for me). Src"));
      dbgPrint(m_srcNodeId);
      dbgPrint(FLASH(" Dst:"));
      dbgPrint(m_dstNodeId);
      dbgPrintln(FLASH(")"));
    }
  }
  m_rxWaspRsp = waspRsp;
}


//-----------------------------------------------------------------------------
// Function: radioSendBuf
//   Send a WASP command.
// Parameters:
//   dst:I         - Destination node ID; Use BROADCASTID for a broadcast
//                   command.
//   pPayload:I    - Pointer to the payload buffer.
//   payloadLen:I  - Number of bytes to send from the payload buffer.
//   reqAck:I      - TRUE = an ACK is requested.
// Returns: ACK value (e.g. ACK_OK); see WASP_defs.h
// Inputs/Outputs:
//   m_radio:I
//-----------------------------------------------------------------------------
uint8 radioSendBuf(uint8 dst, uint8 *pPayload, uint8 payloadLen, boolean reqAck)
{
  static uint32 lastTxTime = 0;
  
  // Don't allow transmissions to be too close together.
  while ( (millis() - lastTxTime) < MIN_UPD_PERIOD )
    ;
  
  dbgPrint(FLASH("TX Dst["));
  dbgPrint(dst);
  dbgPrint(FLASH("] Size["));
  dbgPrint(payloadLen);
  dbgPrint(FLASH("] Cmd["));
  dbgPrint(pPayload[0]);
  dbgPrintln(FLASH("]"));
  
  for (uint8 i = TX_NUM_RETRIES + 1; i != 0; i--)
  {
    m_radio.send(dst, pPayload, payloadLen, reqAck);
    lastTxTime = millis();
    if (reqAck)
    {
      while ( (millis() - lastTxTime) < ACK_WAIT_TIME )
      {
        if (m_radio.ACKReceived(dst))
        {
          return m_radio.DATA[0];
        }
      }
    }
    else
    {
      return ACK_OK;
    }
  }
  
  return ACK_ETIME;
}


//-----------------------------------------------------------------------------
// Function: appendRadioOut8
//   Append a byte to the radio output buffer.
// Parameters:
//   value:I  - The byte value to append..
// Returns: (none)
// Inputs/Outputs:
//   m_radioOutBuf:O
//   m_radioOutBufPos:O
//-----------------------------------------------------------------------------
inline void appendRadioOut8(uint8 value)
{
  m_radioOutBuf[m_radioOutBufPos++] = value;
}




/*****************************************************************************
 *  SECTION  *************  WASP Command Handling  ***************************
 *****************************************************************************/

//-----------------------------------------------------------------------------
// Function: setGroup
//   Notify a node what are its left and right neighbours.
// Parameters:
//   dst:I      - The WASP slave destination. This must be a single WASP
//                slave node ID.
//   groupId:I  - Identifies the group of WASP slaves in which the destination
//                node belongs. The value must be in the range FIRST_GROUP ..
//                (FIRST_GROUP + MAX_GROUPS - 1). Singleton groups are
//                permitted. The number of members in a group is limited only
//                by the number of supported slaves (MAX_SLAVES).
//   left:I     - Node ID of left neighbour.
//   right:I    - Node ID of right neighbour.
// Returns: (none)
// Inputs/Outputs:
//   m_verboseWasp:I
//   m_radioOutBuf:O
//   m_radioOutBufPos:O
//-----------------------------------------------------------------------------
void setGroup(uint8 dst, uint8 groupId, uint8 left, uint8 right)
{
  m_radioOutBufPos = 0;
  if (BROADCASTID == dst)
  {
    logPrint(FLASH("NEIGH: Bad dst="));
    logPrintln(dst);
  }
  else
  {
    appendRadioOut8(WASPCMD_GROUP);
    appendRadioOut8(groupId);
    appendRadioOut8(left);
    appendRadioOut8(right);
    (void)radioSendBuf(dst, m_radioOutBuf, m_radioOutBufPos, false);
    if (m_verboseWasp)
    {
      logPrint(FLASH("NEIGH ->"));
      logPrint(dst);
      logPrint(" ");
      logPrint(left);
      logPrint(" ");
      logPrintln(right);
    }
  }
}


//-----------------------------------------------------------------------------
// Function: background
//   Set the colour for all pixels in the strip(s) specified by the WASP
//   destination address.
// Parameters:
//   dst:I  - The WASP slave destination. This is either a single WASP
//            slave node ID or BROADCASTID to set the pixel colours
//            on all LED strips.
//   r:I    - Amount of red.
//   g:I    - Amount of green.
//   b:I    - Amount of blue.
// Returns: (none)
// Inputs/Outputs:
//   m_verboseWasp:I
//   m_radioOutBuf:O
//   m_radioOutBufPos:O
//-----------------------------------------------------------------------------
void background(uint8 dst, uint8 r, uint8 g, uint8 b)
{
  m_radioOutBufPos = 0;
  appendRadioOut8(WASPCMD_BKGRD);
  appendRadioOut8(r);
  appendRadioOut8(g);
  appendRadioOut8(b);
  (void)radioSendBuf(dst, m_radioOutBuf, m_radioOutBufPos, false);
  if (m_verboseWasp)
  {
    logPrint(FLASH("BKGRD ->"));
    logPrint(dst);
    logPrint(" ");
    logPrint(r);
    logPrint(" ");
    logPrint(g);
    logPrint(" ");
    logPrintln(b);
  }
}


//-----------------------------------------------------------------------------
// Function: setState
//   Send a state operation command to the WASP destination address.
// Parameters:
//   dst:I  - The WASP slave destination. This is either a single WASP
//            slave node ID or BROADCASTID to set the pixel colours
//            on all LED strips.
//   opt:I  - The state flags value.
// Returns: (none)
// Inputs/Outputs:
//   m_verboseWasp:I
//   m_radioOutBuf:O
//   m_radioOutBufPos:O
//-----------------------------------------------------------------------------
void setState(uint8 dst, uint8 opt)
{
  m_radioOutBufPos = 0;
  appendRadioOut8(WASPCMD_STATE);
  appendRadioOut8(opt);
  (void)radioSendBuf(dst, m_radioOutBuf, m_radioOutBufPos, false);
  if (m_verboseWasp)
  {
    logPrint(FLASH("STATE ->"));
    logPrint(dst);
    logPrint(" ");
    logPrintln(opt);
  }
}


//-----------------------------------------------------------------------------
// Function: line
//   Draw a line of pixels in the strip(s) specified by the WASP
//   destination address.
// Parameters:
//   dst:I    - The WASP slave destination. This is either a single WASP
//              slave node ID or BROADCASTID to set the pixel colours
//              on all LED strips.
//   start:I  - The starting pixel number for the line. (Zero indicates the
//              first pixel.)
//   len:I    - The line length (in number of pixels).
//   r:I      - Amount of red.
//   g:I      - Amount of green.
//   b:I      - Amount of blue.
// Returns: (none)
// Inputs/Outputs:
//   m_verboseWasp:I
//   m_radioOutBuf:O
//   m_radioOutBufPos:O
//-----------------------------------------------------------------------------
void line(uint8 dst, uint8 start, uint8 len, uint8 r, uint8 g, uint8 b)
{
  m_radioOutBufPos = 0;
  appendRadioOut8(WASPCMD_LINE);
  appendRadioOut8(start);
  appendRadioOut8(len);
  appendRadioOut8(r);
  appendRadioOut8(g);
  appendRadioOut8(b);
  (void)radioSendBuf(dst, m_radioOutBuf, m_radioOutBufPos, false);
  if (m_verboseWasp)
  {
    logPrint(FLASH("LINE ->"));
    logPrintln(dst);
  }
}


//-----------------------------------------------------------------------------
// Function: shift
//   Shift the pixels in the strip(s) specified by the WASP destination
//   address.
// Parameters:
//   dst:I  - The WASP slave destination. This is either a single WASP
//            slave node ID or BROADCASTID to set the pixel colours
//            on all LED strips.
//   num:I  - Number of pixels to shift. Positive numbers shift right;
//            begative numbers shift left.
// Returns: (none)
// Inputs/Outputs:
//   m_verboseWasp:I
//   m_cmdExecDelay:0
//   m_radioOutBuf:O
//   m_radioOutBufPos:O
//-----------------------------------------------------------------------------
void shift(uint8 dst, uint8 num)
{
  const uint16 k_respDelay = CMD_TIMEOUT + 0;
  const uint16 k_minDelay = (k_respDelay > MIN_UPD_PERIOD ?
                             k_respDelay : MIN_UPD_PERIOD);
  m_radioOutBufPos = 0;
  appendRadioOut8(WASPCMD_SHIFT);
  appendRadioOut8(num);
  (void)radioSendBuf(dst, m_radioOutBuf, m_radioOutBufPos, false);
  m_cmdExecDelay = millis() + k_minDelay;
  if (m_verboseWasp)
  {
    logPrint(FLASH("SHIFT ->"));
    logPrint(dst);
    logPrint("  ");
    logPrintln(num);
  }
}


//-----------------------------------------------------------------------------
// Function: swap
//   Change the colour of every pixel having a specified colour to a new
//   colour in the strip(s) specified by the WASP destination
//   address.
// Parameters:
//   dst:I    - The WASP slave destination. This is either a single WASP
//              slave node ID or BROADCASTID to change the pixel colours
//              on all LED strips.
//   old_r:I  - Amount of red in colour to be changed.
//   old_g:I  - Amount of green in colour to be changed.
//   old_b:I  - Amount of blue in colour to be changed.
//   r:I      - Amount of red in new colour.
//   g:I      - Amount of green in new colour.
//   b:I      - Amount of blue in new colour.
// Returns: (none)
// Inputs/Outputs:
//   m_verboseWasp:I
//   m_radioOutBuf:O
//   m_radioOutBufPos:O
//-----------------------------------------------------------------------------
void swap( uint8 dst,
           uint8 old_r, uint8 old_g, uint8 old_b,
           uint8 r,     uint8 g,     uint8 b )
{
  m_radioOutBufPos = 0;
  appendRadioOut8(WASPCMD_SWAP);
  appendRadioOut8(old_r);
  appendRadioOut8(old_g);
  appendRadioOut8(old_b);
  appendRadioOut8(r);
  appendRadioOut8(g);
  appendRadioOut8(b);
  (void)radioSendBuf(dst, m_radioOutBuf, m_radioOutBufPos, false);
  if (m_verboseWasp)
  {
    logPrint(FLASH("SWAP ->"));
    logPrintln(dst);
  }
}


//-----------------------------------------------------------------------------
// Function: waspReset
//   Reset the destination node(s).
// Parameters:
//   dst:I    - The WASP slave destination(s). This is either a single WASP
//              slave node ID or BROADCASTID.
// Returns: (none)
// Inputs/Outputs:
//   m_verboseWasp:I
//   m_radioOutBuf:O
//   m_radioOutBufPos:O
//-----------------------------------------------------------------------------
void waspReset(uint8 dst)
{
  m_radioOutBufPos = 0;
  appendRadioOut8(WASPCMD_RESET);
  appendRadioOut8('D');
  appendRadioOut8('E');
  appendRadioOut8('A');
  appendRadioOut8('D');
  (void)radioSendBuf(dst, m_radioOutBuf, m_radioOutBufPos, false);
  if (m_verboseWasp)
  {
    logPrint(FLASH("RESET ->"));
    logPrintln(dst);
  }
}


//-----------------------------------------------------------------------------
// Function: fxSpeed
//   Adjust the speed of an animated effect that runs on the destination
//   node(s) without controller intervention (e.g. RAINBOW()).
// Parameters:
//   dst:I       - The WASP slave destination. This is either a single WASP
//                 slave node ID or BROADCASTID to change the pixel colours
//                 on all LED strips.
//   delayVal:I  - Relative delay. Smaller values increase the effect's
//                 speed. However, a value of 0 pauses the effect.
// Returns: (none)
// Inputs/Outputs:
//   m_verboseWasp:I
//   m_radioOutBuf:O
//   m_radioOutBufPos:O
//-----------------------------------------------------------------------------
void fxSpeed( uint8 dst, uint8 delayVal )
{
  m_radioOutBufPos = 0;
  appendRadioOut8(WASPCMD_SPEED);
  appendRadioOut8(delayVal);
  (void)radioSendBuf(dst, m_radioOutBuf, m_radioOutBufPos, false);
  if (m_verboseWasp)
  {
    logPrint(FLASH("SPEED ->"));
    logPrint(dst);
    logPrint("  ");
    logPrintln(delayVal);
  }
}


//-----------------------------------------------------------------------------
// Function: rainbow
//   Run a rainbow effect on the destination node(s).
// Parameters:
//   dst:I    - The WASP slave destination. This is either a single WASP
//              slave node ID or BROADCASTID to change the pixel colours
//              on all LED strips.
//   offs:I   - The number of pixel position colours to offset for the
//              first pixel.
// Returns: (none)
// Inputs/Outputs:
//   m_verboseWasp:I
//   m_radioOutBuf:O
//   m_radioOutBufPos:O
//-----------------------------------------------------------------------------
void rainbow(uint8 dst, uint8 offs)
{
  m_radioOutBufPos = 0;
  appendRadioOut8(WASPCMD_RAINBOW);
  appendRadioOut8(offs);
  (void)radioSendBuf(dst, m_radioOutBuf, m_radioOutBufPos, false);
  if (m_verboseWasp)
  {
    logPrint(FLASH("RAINBOW ->"));
    logPrintln(dst);
  }
}


//-----------------------------------------------------------------------------
// Function: rainbowCycle
//   Run a rainbow effect on the destination node(s) where the pixels on
//   each node span the range of rainbow colours all the time.
// Parameters:
//   dst:I    - The WASP slave destination. This is either a single WASP
//              slave node ID or BROADCASTID to change the pixel colours
//              on all LED strips.
// Returns: (none)
// Inputs/Outputs:
//   m_verboseWasp:I
//   m_radioOutBuf:O
//   m_radioOutBufPos:O
//-----------------------------------------------------------------------------
void rainbowCycle(uint8 dst)
{
  m_radioOutBufPos = 0;
  appendRadioOut8(WASPCMD_RAINCYCLE);
  (void)radioSendBuf(dst, m_radioOutBuf, m_radioOutBufPos, false);
  if (m_verboseWasp)
  {
    logPrint(FLASH("RAINCYCLE ->"));
    logPrintln(dst);
  }
}


//-----------------------------------------------------------------------------
// Function: twinkle
//   Run a twinkling effect on the destination node(s).
// Parameters:
//   dst:I     - The WASP slave destination. This is either a single WASP
//               slave node ID or BROADCASTID.
//   minDly:I  - The minimum delay between twinkles on any given node.
//   maxDly:I  - The maximum delay between twinkles on any given node.
//   burst:I   - The maximum number of pixels that can twinkle simultaneously
//               on any given node.
//   hold:I    - The duration for which a twinkle lasts.
// Returns: (none)
// Inputs/Outputs:
//   m_verboseWasp:I
//   m_radioOutBuf:O
//   m_radioOutBufPos:O
//-----------------------------------------------------------------------------
void twinkle(uint8 dst, uint8 minDly, uint8 maxDly, uint8 burst, uint8 hold)
{
  m_radioOutBufPos = 0;
  appendRadioOut8(WASPCMD_TWINKLE);
  appendRadioOut8(minDly);
  appendRadioOut8(maxDly);
  appendRadioOut8(burst);
  appendRadioOut8(hold);
  (void)radioSendBuf(dst, m_radioOutBuf, m_radioOutBufPos, false);
  if (m_verboseWasp)
  {
    logPrint(FLASH("TWINKLE ->"));
    logPrintln(dst);
  }
}



/*****************************************************************************
 *  SECTION  ******  WASP Interactive Programming Environment ****************
 *****************************************************************************/

// Use the following #define and #if definitions to temporarily
// disable wipeShowHelp() if Flash memory program space is getting short.
//#define wipeShowHelp()
//#if 0
//-----------------------------------------------------------------------------
// Function: wipeShowHelp
//   Show help text on the console.
// Parameters: (none)
// Returns: (none)
// Inputs/Outputs: (none)
//-----------------------------------------------------------------------------
void wipeShowHelp(void)
{
  logPrintln(FLASH("\n"));
  logPrintln(FLASH("** WIPE Help **\n"));
  logPrintln(FLASH("WIPE is BASIC-like and case sensitive"));
  logPrintln(FLASH("- To edit a line, use same line #"));


  logPrintln(FLASH("\nCOMMANDS"));
  logPrintln(FLASH("del <m> [- n]\t- Delete line m thru n"));
  logPrintln(FLASH("dir\t\t- List saved programs"));
  logPrintln(FLASH("erase <name>\t- Delete program"));
  logPrintln(FLASH("exit\t\t- Exit WIPE"));
  logPrintln(FLASH("help\t\t- Show help"));
  logPrintln(FLASH("load <name>\t- Load program"));
  logPrintln(FLASH("list m [- n]\t- List lines m thru n"));
  logPrintln(FLASH("prof\t\t- Show all symbols"));
  logPrintln(FLASH("renum\t\t- Renumber lines evenly from 5 - 250"));
  logPrintln(FLASH("run\t\t- Run program; 'x' to quit."));
  logPrintln(FLASH("save <name>\t- Save program"));
  logPrintln(FLASH("start <name>\t- Set startup program"));


  logPrintln(FLASH("\nSTATEMENTS"));
  logPrintln(FLASH("if <expr>\t\tSkip next line if <expr> false"));
  logPrintln(FLASH("label <id>"));
  logPrintln(FLASH("let <id> = <expr>"));
  logPrintln(FLASH("goto <label-id>"));
  logPrintln(FLASH("print <expr>"));
  logPrintln(FLASH("var <id> : <type>\tin {int, float, bool}"));
  logPrintln(FLASH("<func>(<params>)  (see FUNCTIONS)"));
  logPrintln(FLASH("\n  where: <expr> is with no precedence"));

  
  logPrintln(FLASH("\n... <Enter>"));
  while (Serial.available() == 0)
    ;
  Serial.read();


  logPrintln(FLASH("\nFUNCTIONS"));

  logPrintln(FLASH("GENERAL"));
  logPrintln(FLASH("\tPause(min,sec,100ths)"));
  
  logPrintln(FLASH("WASP GENERIC"));
  logPrintln(FLASH("\tBkgrd(dst,r,g,b)"));
  logPrintln(FLASH("\tGroup(dst,grp,l,r)"));
  logPrintln(FLASH("\tLine(dst,r,g,b,start,len)"));
  logPrintln(FLASH("\tReset(dst)"));
  logPrintln(FLASH("\tShift(dst,shift)"));
  logPrintln(FLASH("\tState(dst,opts),\topts is OR'd collection of "));
  logPrintln(FLASH("\t\t\t\t{SAVE(1), RESTORE(2), RESUME(4), SUSPEND(8)}"));
  logPrintln(FLASH("\tSwap(dst,r,g,b,r',g',b')"));

  logPrintln(FLASH("\nSPECIAL FX"));
  logPrintln(FLASH("\tRain(dst,start)"));
  logPrintln(FLASH("\tCycle(dst)"));
  logPrintln(FLASH("\tSpeed(dst, dly)\tFX speed"));
  logPrintln(FLASH("\tTwinkle(dst,minDly,maxDly,burst,hold)"));
  logPrintln();
}
//#endif


//-----------------------------------------------------------------------------
// Function: wipeDumpProgramHex
//   Dump program memory to the console.
// Parameters: (none)
// Returns:    (none)
// Inputs/Outputs:
//   m_program:I
//   m_wipeProgByte:I
// Notes:
//   This function is intended for WIPE interpreter debugging purposes.
//-----------------------------------------------------------------------------
void wipeDumpProgramHex(void)
{
  uint8  *pCurrByte;
  uint16  i;
  uint8   hexByte;

  pCurrByte = &m_program[0];
  logPrintln();
  logPrintln(FLASH("PROG HEX DUMP:"));
  for (i = 0; i < 10; i++)
  {
    logPrint(FLASH("  "));
    logPrint(i);
    logPrint(' ');
  }
  logPrintln();
  
  for (i = 10; i !=0; i--)
  {
    logPrint(FLASH("--- "));
  }
  logPrintln();

  for (i = 0; i < m_wipeProgByte; i++)
  {
    hexByte = *pCurrByte++;
    if (hexByte < 100)
      logPrint(FLASH(" "));
    if (hexByte < 10)
      logPrint(FLASH(" "));
    logPrint(hexByte);
    logPrint(FLASH(" "));
    if ( (i % 10) == 9 )
      logPrintln();
  }
  logPrintln();
}


//-----------------------------------------------------------------------------
// Function: wipeDirSpaceCheck
//   Determine the free space (in bytes) available in EEPROM for WIPE
//   programs.
// Parameters: (none)
// Returns:    (none)
// Inputs/Outputs:
//   m_wipeDirSpace:O
//   m_wipeSaveAddr:O
//-----------------------------------------------------------------------------
void wipeDirSpaceCheck(void)
{
  uint16 currFileSize;

  m_wipeSaveAddr = EEPROM_PROG_DIR_START;
  m_wipeDirSpace = EEPROM_PROG_LEN;
  currFileSize = (uint16)EEPROM.read(m_wipeSaveAddr) << 8 |
                 EEPROM.read(m_wipeSaveAddr + 1);
  while ( (currFileSize != 0) &&
          (m_wipeSaveAddr < EEPROM_PROG_DIR_START + EEPROM_PROG_LEN)
        )
  {
    /* Decrease available space by current file size and directory header */
    m_wipeDirSpace -= currFileSize + DIR_PROG_OFFS;

    /* Move to the next directory entry */
    m_wipeSaveAddr += currFileSize + DIR_PROG_OFFS;

    /* Get the next file's size */
    currFileSize = (uint16)EEPROM.read(m_wipeSaveAddr) << 8 |
                   EEPROM.read(m_wipeSaveAddr + 1);
  }

  logPrint(FLASH("Free EEPROM: "));
  logPrintln(m_wipeDirSpace);
}


//-----------------------------------------------------------------------------
// Function: wipeDirList
//   Display a directory listing of WIPE program files.
// Parameters: (none)
// Returns:    (none)
// Inputs/Outputs:
//   m_wipeDirSpace:I
//-----------------------------------------------------------------------------
void wipeDirList(void)
{
  uint16  currDirEntry;
  uint16  currFileSize;
  uint8   count = 0;
  boolean endName;
  boolean startupFound = false;
  char    tmpChar;
  char    filename[MAX_PROGNAME_LEN+1];

  currDirEntry = EEPROM_PROG_DIR_START;
  currFileSize = (uint16)EEPROM.read(currDirEntry) << 8 |
                 EEPROM.read(currDirEntry + 1);
  logPrintln();
  logPrintln(FLASH("Directory:"));
  while ( (currFileSize != 0) &&
          (currDirEntry < EEPROM_PROG_DIR_START + EEPROM_PROG_LEN)
        )
  {
    count++;
    endName = false;
    for (uint8 i = 0; i <= MAX_PROGNAME_LEN; i++)
    {
      /* Print filename */
      tmpChar = EEPROM.read(currDirEntry + DIR_NAME_OFFS + i);
      filename[i] = tmpChar;
      if (tmpChar == '\0')
      {
        endName = true;
        if (strcmp(filename, m_autorunProg) == 0)
          startupFound = true;
      }
      if (!endName)
      {
        logPrint(tmpChar);
      }
      else
      {
        logPrint(' ');
      }
    }
    
    /* Print file size */
    logPrint(FLASH("  "));
    logPrint(currFileSize);
    logPrintln(FLASH(" bytes"));

    /* Move to the next directory entry */
    currDirEntry += currFileSize + DIR_PROG_OFFS;

    /* Get the next file's size */
    currFileSize = (uint16)EEPROM.read(currDirEntry) << 8 |
                   EEPROM.read(currDirEntry + 1);
  }
  logPrint(FLASH("  "));
  logPrint(count);
  logPrint(FLASH(" files. "));
  logPrint(m_wipeDirSpace);
  logPrint(FLASH(" of "));
  logPrint(EEPROM_PROG_LEN);
  logPrintln(FLASH(" bytes free."));
  if (m_autorunProg[0] != '\0')
  {
    logPrint(FLASH("Startup ["));
    logPrint(m_autorunProg);
    if (startupFound)
      logPrintln(FLASH("]"));
    else
      logPrintln(FLASH("] is missing"));
  }
  logPrintln(FLASH("-----------------------"));
  logPrintln();
}


//-----------------------------------------------------------------------------
// Function: wipeDirFileSave
//   Save current program in RAM to EEPROM directory.
// Parameters:
//   progName:I  - Program file name string.
// Returns:    (none)
// Inputs/Outputs:
//   m_wipeProgByte:I
//   m_wipeDirSpace:IO
//   m_wipeSaveAddr:IO
//-----------------------------------------------------------------------------
void wipeDirFileSave(char *progName)
{
  uint16  currAddr;
  uint8  *pProgByte;

  currAddr = m_wipeSaveAddr;

  /* Save file size (in bytes) to the directory entry */
  EEPROM.write(currAddr++, (uint8)(m_wipeProgByte >> 8));
  EEPROM.write(currAddr++, (uint8)m_wipeProgByte);

  /* Save the file name */
  for (uint8 i = 0; i <= MAX_PROGNAME_LEN; i++)
  {
    EEPROM.write(currAddr++, *progName++);
  }

  /* Save the program */
  pProgByte = &m_program[0];
  for (uint16 i = m_wipeProgByte; i != 0; i--)
  {
    EEPROM.write(currAddr++, *pProgByte++);
  }

  /* Update write address and free space */
  m_wipeSaveAddr += m_wipeProgByte + DIR_PROG_OFFS;
  m_wipeDirSpace -= m_wipeProgByte + DIR_PROG_OFFS;

  /* Flag end of directory */
  EEPROM.write(m_wipeSaveAddr, 0);
  EEPROM.write(m_wipeSaveAddr + 1, 0);

  logPrintln();
  logPrintln(FLASH("Saved"));
}


//-----------------------------------------------------------------------------
// Function: wipeDirFileFind
//   Locate the specified program in the EEPROM directory.
// Parameters:
//   progName:I   - Program file name string.
//   pFileSize:O  - Address at which to return the located file's size (in
//                  bytes).
// Returns:
//   0, if file not found;
//   The directory entry base address in EEPROM, otherwise.
// Inputs/Outputs:
//   m_wipeDirSpace:IO
//   m_wipeSaveAddr:IO
//-----------------------------------------------------------------------------
uint16 wipeDirFileFind(char *progName, uint16 *pFileSize)
{
  uint16  currDirEntry;
  uint16  currFileSize;
  char   *pChar;
  char    currChar;

  *pFileSize = 0;
  
  currDirEntry = EEPROM_PROG_DIR_START;
  currFileSize = (uint16)EEPROM.read(currDirEntry) << 8 |
                 (uint16)EEPROM.read(currDirEntry+1);
  while ( (currFileSize != 0) &&
          (currDirEntry < EEPROM_PROG_DIR_START + EEPROM_PROG_LEN)
        )
  {
    pChar = progName;
    for (uint8 i = 0; i <= MAX_PROGNAME_LEN; i++)
    {
      currChar = EEPROM.read(currDirEntry + DIR_NAME_OFFS + i);
      if (*pChar != currChar)
        break;
      if (*pChar == '\0')
      {
        *pFileSize = currFileSize;
        return currDirEntry;
      }
      pChar++;
    }

    currDirEntry += currFileSize + DIR_PROG_OFFS;
    currFileSize = (uint16)EEPROM.read(currDirEntry) << 8 |
                   (uint16)EEPROM.read(currDirEntry+1);
  }

  return 0;
}


//-----------------------------------------------------------------------------
// Function: wipeDirFileErase
//   Erase the specified program from EEPROM directory.
// Parameters:
//   progName:I  - Program file name string.
// Returns:    (none)
// Inputs/Outputs:
//   m_wipeDirSpace:IO
//   m_wipeSaveAddr:IO
//-----------------------------------------------------------------------------
void wipeDirFileErase(char *progName)
{
  uint16  dirEntry;
  uint16  fileSize;
  uint16  numBytesToMove;
  uint16  src;

  dirEntry = wipeDirFileFind(progName, &fileSize);
  if (0 == dirEntry)
  {
    logPrintln(FLASH("Not found"));
    return;
  }

  /* Shift all bytes above the directory entry base address down
   * by the number of bytes occupied by the file and directory header.
   */
  numBytesToMove = (EEPROM_PROG_LEN - m_wipeDirSpace) -
                   (fileSize + DIR_PROG_OFFS);
  src = dirEntry + fileSize + DIR_PROG_OFFS;
  while (numBytesToMove-- != 0)
    EEPROM.write(dirEntry++, (EEPROM.read(src++)));

  /* Update write address and free space */
  m_wipeSaveAddr -= fileSize + DIR_PROG_OFFS;
  m_wipeDirSpace += fileSize + DIR_PROG_OFFS;

  /* Flag end of directory */
  EEPROM.write(m_wipeSaveAddr, 0);
  EEPROM.write(m_wipeSaveAddr + 1, 0);

  logPrintln();
  logPrintln(FLASH("Erased"));
}


//-----------------------------------------------------------------------------
// Function: wipeDirFileLoad
//   Load the specified program from EEPROM to RAM.
// Parameters:
//   progName:I  - Program file name string.
// Returns: true iff program was found.
// Inputs/Outputs:
//   m_wipeDirSpace:I
//   m_wipeSaveAddr:I
//   m_wipeLineStart:O
//   m_wipeProgByte:O
//-----------------------------------------------------------------------------
boolean wipeDirFileLoad(char *progName)
{
  uint16  currEepromPos;
  uint16  fileSize;
  uint16  i;

  currEepromPos = wipeDirFileFind(progName, &fileSize);
  if (0 == currEepromPos)
  {
    logPrintln(FLASH("Not found"));
    return false;
  }

  currEepromPos += DIR_PROG_OFFS;
  for (i = 0; i < fileSize; i++)
  {
    m_program[i] = EEPROM.read(currEepromPos + i);
  }

  m_wipeLineStart = i;
  m_wipeProgByte = i;

  logPrintln();
  logPrintln(FLASH("Loaded."));
  return true;
}


//-----------------------------------------------------------------------------
// Function: wipePrintStatement
//   Pretty print the tokenized WIPE statement starting at m_program[]
//   location m_wipeLineStart.
// Parameters:
//   lineStart:I  - Starting program memory location of the line.
// Returns: (none)
// Inputs/Outputs:
//   m_wipeLineStart:I
//   m_program:I
//-----------------------------------------------------------------------------
void wipePrintStatement(void)
{
  uint8  *linePos;
  char   *identifier;
  uint8   stmtId;
  uint8   lineNum;
  uint8   lineLen;
  uint8   byteVal;
  uint8   lineOffs;

  lineNum  = m_program[m_wipeLineStart + TKNZD_LINE_OFFS];
  lineLen  = m_program[m_wipeLineStart + TKNZD_LEN_OFFS];
  stmtId   = m_program[m_wipeLineStart + TKNZD_STMT_OFFS];
  lineOffs = TKNZD_STMT_OFFS + 1;
  
  if (lineNum != 0)
  {
    if (lineNum < 100)
      logPrint(FLASH("0"));
    if (lineNum < 10)
      logPrint(FLASH("0"));
    logPrint(lineNum);
    logPrint(FLASH(" "));
  }
  
  switch (stmtId)
  {
    case WIPE_FUNC:
      stmtId = m_program[m_wipeLineStart + lineOffs++];
      switch (stmtId)
      {
        case WASPCMD_GROUP:     logPrint(FLASH("Group"));   break;
        case WASPCMD_STATE:     logPrint(FLASH("State"));   break;
        case WASPCMD_BKGRD:     logPrint(FLASH("Bkgrd"));   break;
        case WASPCMD_LINE:      logPrint(FLASH("Line"));    break;
        case WASPCMD_SHIFT:     logPrint(FLASH("Shift"));   break;
        case WASPCMD_SWAP:      logPrint(FLASH("Swap"));    break;
        case WASPCMD_RESET:     logPrint(FLASH("Reset"));   break;
        case WASPCMD_SPEED:     logPrint(FLASH("Speed"));   break;
        case WASPCMD_RAINBOW:   logPrint(FLASH("Rain"));    break;
        case WASPCMD_RAINCYCLE: logPrint(FLASH("Cycle"));   break;
        case WASPCMD_TWINKLE:   logPrint(FLASH("Twinkle")); break;
        case WIPE_FN_PAUSE:     logPrint(FLASH("Pause"));   break;
        default:
          logPrint(FLASH("<unknown func: "));
          logPrint(stmtId);
          logPrintln(FLASH("?>"));
          return;
      }
      lineOffs++; /* Skip # of parameters */
      linePos = &m_program[m_wipeLineStart + lineOffs];
      for (uint8 i = 0; i < (lineLen - lineOffs); i++)
      {
        logPrint((char)*linePos++);
      }
      logPrintln();
      break;
    case WIPE_GOTO:
      logPrint(FLASH("goto "));
      identifier = (char *)&m_program[m_wipeLineStart + lineOffs];
      lineOffs += strlen(identifier) + 1;
      logPrintln(identifier);
      break;
    case WIPE_IF:
      logPrint(FLASH("if "));
      linePos = &m_program[m_wipeLineStart + lineOffs];
      for (uint8 i = 0; i < (lineLen - lineOffs); i++)
      {
        logPrint((char)*linePos++);
      }
      logPrintln();
      break;
    case WIPE_LABEL:
      logPrint(FLASH("label "));
      identifier = (char *)&m_program[m_wipeLineStart + lineOffs];
      lineOffs += strlen(identifier) + 1;
      logPrintln(identifier);
      break;
    case WIPE_LET:
      logPrint(FLASH("let "));
      identifier = (char *)&m_program[m_wipeLineStart + lineOffs];
      lineOffs += strlen(identifier) + 1;
      logPrint(identifier);
      logPrint(FLASH(" = "));
      linePos = &m_program[m_wipeLineStart + lineOffs];
      for (uint8 i = 0; i < (lineLen - lineOffs); i++)
      {
        logPrint((char)*linePos++);
      }
      logPrintln();
      break;
    case WIPE_PRINT:
    {
      char *pChar;

      logPrint(FLASH("print "));
      stmtId = m_program[m_wipeLineStart + lineOffs++];
      if (WIPE_PRT_STR == stmtId)
      {
        logPrint("\"");
        logPrint((char *)&m_program[m_wipeLineStart + lineOffs]);
        logPrintln("\"");
      }
      else
      {
        identifier = (char *)&m_program[m_wipeLineStart + lineOffs];
        lineOffs += strlen(identifier) + 1;
        logPrintln(identifier);
      }
      break;
    }
    case WIPE_VAR:
      logPrint(FLASH("var "));
      identifier = (char *)&m_program[m_wipeLineStart + lineOffs];
      logPrint(identifier);
      logPrint(FLASH(" : "));
      byteVal = m_program[m_wipeLineStart + lineOffs + strlen(identifier) + 1];
      switch (byteVal)
      {
        case WIPE_ID_INT:
          logPrintln(FLASH("int"));
          break;
        case WIPE_ID_FLOAT:
          logPrintln(FLASH("float"));
          break;
        case WIPE_ID_BOOL:
          logPrintln(FLASH("bool"));
          break;
        default:
          logPrint(FLASH("<type: "));
          logPrint(byteVal);
          logPrintln(FLASH("?>"));
      }
      break;
    default:
      logPrint(FLASH("<WIPE statement: "));
      logPrint(byteVal);
      logPrintln(FLASH("?>"));
  }
}


//-----------------------------------------------------------------------------
// Function: wipeShowConsoleLine
//   Display the current WIPE command line.
// Parameters: (none)
// Returns: (none)
// Inputs/Outputs:
//   m_consoleBuffer:I
//   m_consolePos:I
//-----------------------------------------------------------------------------
inline void wipeShowConsoleLine(void)
{
    logPrintln(m_consoleBuffer);
}


//-----------------------------------------------------------------------------
// Function: wipeShowError
//   Display WIPE syntax error on console.
// Parameters:
//   errStr:I  - Error string to display. The argument should be provided
//               using the FLASH() macro:
//                 e.g. wipeShowError(FLASH("Error text"));
// Returns: (none)
// Inputs/Outputs:
//   m_consoleBuffer:I
//   m_consolePos:I
//   m_wipeRunByte:I
//-----------------------------------------------------------------------------
void wipeShowError(const __FlashStringHelper *errStr)
{
  uint8 i;
  uint8 indent;

  if (m_wipeRunByte > 0)
  {
    indent = 0;
    wipePrintStatement();
  }
  else
  {
    indent = m_consolePos;
    if (m_consolePos >= 2)
      indent -= 1;
    
    wipeShowConsoleLine();
  }
  
  for (i = 1; i < indent; i++)
  {
    logPrint(FLASH(" "));
  }
  logPrintln(FLASH("^"));
  logPrint(FLASH("***ERROR: "));
  logPrintln(errStr);
}


//-----------------------------------------------------------------------------
// Function: wipeSymbolsClear
//   Initialize the WIPE symbol table.
// Parameters: (none)
// Returns: (none)
// Inputs/Outputs:
//   m_symTbl:O
//-----------------------------------------------------------------------------
void wipeSymbolsClear(void)
{
  uint8 i;

  for (i = 0; i < MAX_NUM_SYMBOLS; i++)
    m_symTbl[i].typeId = WIPE_ID_UNDEF;
}


//-----------------------------------------------------------------------------
// Function: wipeSymbolInsert
//   Insert a new symbol into the WIPE symbol table.
// Parameters:
//   identifier:I  - Identifier string.
//   type:I        - Identifier's type.
//   idx:O         - Symbol table index at which symbol was inserted. The
//                   value is valid only if the function returns true.
// Returns: true iff insertion was successful.
// Inputs/Outputs:
//   m_symTbl:IO
//-----------------------------------------------------------------------------
boolean wipeSymbolInsert(char *identifier, uint8 type, uint8 *idx)
{
  uint8 i;
  uint8 freeIdx = MAX_NUM_SYMBOLS;

  *idx = MAX_NUM_SYMBOLS;
  for (i = 0; i < MAX_NUM_SYMBOLS; i++)
  {
    if (m_symTbl[i].typeId == WIPE_ID_UNDEF)
    {
      /* Found an unused table entry */
      if (MAX_NUM_SYMBOLS == freeIdx)
        freeIdx = i;
    }
    else
    {
      if (strcmp(m_symTbl[i].symbol, identifier) == 0)
      {
        *idx = i;
        return false;
      }
    }
  }
  if (freeIdx != MAX_NUM_SYMBOLS)
  {
    strncpy(m_symTbl[freeIdx].symbol, identifier, MAX_ID_LEN + 1);
    m_symTbl[freeIdx].symbol[MAX_ID_LEN] = '\0';
    m_symTbl[freeIdx].typeId = type;
    m_symTbl[freeIdx].symValue.intValue = 0;
    *idx = freeIdx;
    return true;
  }
  return false;
}


//-----------------------------------------------------------------------------
// Function: wipeSymbolLookup
//   Find a symbol in the WIPE symbol table.
// Parameters:
//   identifier:I  - Identifier string.
//   idx:O         - Symbol table index at which symbol was found. Value on
//                   invocation is unchanged if the function returns false.
// Returns: true iff the symbol was found.
// Inputs/Outputs:
//   m_symTbl:I
//-----------------------------------------------------------------------------
boolean wipeSymbolLookup(char *identifier, uint8 *idx)
{
  uint8 i;

  for (i = 0; i < MAX_NUM_SYMBOLS; i++)
  {
    if (m_symTbl[i].typeId != WIPE_ID_UNDEF)
    {
      if (strncmp(m_symTbl[i].symbol, identifier, MAX_ID_LEN) == 0)
      {
        *idx = i;
        return true;
      }
    }
  }
  return false;
}


//-----------------------------------------------------------------------------
// Function: wipeSymbolsShow
//   Show the known symbols in the symbol table.
// Parameters: (none)
// Returns: (none)
// Inputs/Outputs:
//   m_symTbl:I
//-----------------------------------------------------------------------------
void wipeSymbolsShow(void)
{
  uint8 i, j;
  boolean validEntry;

  logPrintln(FLASH("\nSYMBOLS\nName\tType\t\tValue"));
  logPrintln(FLASH("----\t----\t\t-----"));
  for (i = 0; i < MAX_NUM_SYMBOLS; i++)
  {
    validEntry = true;
    if (m_symTbl[i].typeId != WIPE_ID_UNDEF)
    {
      logPrint(m_symTbl[i].symbol);
      for (j = strlen(m_symTbl[i].symbol); j <= MAX_ID_LEN; j++)
        logPrint(FLASH(" "));
      logPrint(FLASH("\t"));
      switch (m_symTbl[i].typeId)
      {
        case WIPE_ID_LABEL:
          logPrint(FLASH("label"));
          break;
        case WIPE_ID_INT:
          logPrint(FLASH("int"));
          break;
        case WIPE_ID_FLOAT:
          logPrint(FLASH("float"));
          break;
        case WIPE_ID_BOOL:
          logPrint(FLASH("bool"));
          break;
        default:
          logPrintln(FLASH("<unknown>"));
          validEntry = false;
      }
      if (validEntry)
      {
        logPrint(FLASH("\t\t"));
        switch (m_symTbl[i].typeId)
        {
          case WIPE_ID_LABEL:
            logPrintln(m_symTbl[i].symValue.intValue);
            break;
          case WIPE_ID_INT:
            logPrintln(m_symTbl[i].symValue.intValue);
            break;
          case WIPE_ID_FLOAT:
            logPrintlnEx(m_symTbl[i].symValue.floatValue,5);
            break;
          case WIPE_ID_BOOL:
            logPrintln(m_symTbl[i].symValue.boolValue);
            break;
          default:
            logPrintln(FLASH("<unknown>"));
        }
      }
    }
  }
}


//-----------------------------------------------------------------------------
// Function: wipeListProgram
//   Display program statements for a specified range of line numbers.
// Parameters:
//   startLine:I  - First line number to display.
//   endLine:I    - Last line number to display.
// Returns: (none)
// Inputs/Outputs:
//   m_symTbl:I
//-----------------------------------------------------------------------------
void wipeListProgram(uint8 startLine, uint8 endLine)
{
  uint16 savedWipeLineStart = m_wipeLineStart;
  uint8  currLineNum;
  uint8  currLineLen;

  logPrintln();
  m_wipeLineStart = 0;
  currLineNum = m_program[m_wipeLineStart + TKNZD_LINE_OFFS];
  currLineLen = m_program[m_wipeLineStart + TKNZD_LEN_OFFS];
  while ( (currLineNum < startLine) && (m_wipeLineStart < m_wipeProgByte) )
  {
    m_wipeLineStart += currLineLen;
    currLineNum = m_program[m_wipeLineStart + TKNZD_LINE_OFFS];
    currLineLen = m_program[m_wipeLineStart + TKNZD_LEN_OFFS];
  }

  while ( (currLineNum <= endLine) && (m_wipeLineStart < m_wipeProgByte) )
  {
    wipePrintStatement();
    m_wipeLineStart += currLineLen;
    currLineNum = m_program[m_wipeLineStart + TKNZD_LINE_OFFS];
    currLineLen = m_program[m_wipeLineStart + TKNZD_LEN_OFFS];
  }

  m_wipeLineStart = savedWipeLineStart;
}


//-----------------------------------------------------------------------------
// Function: wipeParseInt
//   Scan for an integer value in the conole input buffer.
// Parameters:
//   intVal:O   - The integer value scanned, if the function returns true;
//                zero, otherwise.
//   advance:I  - If true, then advance the m_consolePos value past the
//                scanned integer.
// Returns: true iff a valid integer value was scanned.
// Inputs/Outputs:
//   m_consoleLen:I
//   m_consolePos:IO
//   m_tokenBuffer:IO
//-----------------------------------------------------------------------------
boolean wipeParseInt(int32 *intVal, boolean advance)
{
  char *token = NULL;
  uint8 tmp;

  *intVal = 0;
  if (m_consolePos >= m_consoleLen)
    return false;
  tmp = strspn(&m_tokenBuffer[m_consolePos], " ,\r\0");
  token = strtok(&m_tokenBuffer[m_consolePos], " ,\r\0");
  if (token != NULL)
  {
    if (strspn(token, "+-0123456789") == strlen(token))
    {
      if ( (strlen(token) > 1) ||
           ( (token[0] != '-') && (token[0] != '+') )
         )
      {
        if (advance)
          m_consolePos += strlen(token) + tmp + 1;
        *intVal = atoi(token);
        return true;
      }
    }
  }
  dbgPrintln(FLASH("Couldn't parse integer"));
  return false;
}


//-----------------------------------------------------------------------------
// Function: wipeEvalIntRelation
//   Evaluate an integer relation.
// Parameters:
//   value1:IO     - The first operand value.
//   value2:I      - The second operand value.
//   operation:I   - The operator ID.
// Returns: The truth of the relation.
// Inputs/Outputs: (none)
//-----------------------------------------------------------------------------
boolean wipeEvalIntRelation(int32 value1, int32 value2, uint8 operation)
{
  switch (operation)
  {
    case WIPE_OP_EQ:
      return (value1 == value2);
    case WIPE_OP_NE:
      return (value1 != value2);
    case WIPE_OP_LT:
      return (value1 < value2);
    case WIPE_OP_LE:
      return (value1 <= value2);
    case WIPE_OP_GT:
      return (value1 > value2);
    case WIPE_OP_GE:
      return (value1 >= value2);
    default:
      return false;
  }
}


//-----------------------------------------------------------------------------
// Function: wipeEvalFloatRelation
//   Evaluate an floating point relation.
// Parameters:
//   value1:IO     - The first operand value.
//   value2:I      - The second operand value.
//   operation:I   - The operator ID.
// Returns: The truth of the relation.
// Inputs/Outputs: (none)
//-----------------------------------------------------------------------------
boolean wipeEvalFloatRelation(float value1, float value2, uint8 operation)
{
  switch (operation)
  {
    case WIPE_OP_EQ:
      return (value1 == value2);
    case WIPE_OP_NE:
      return (value1 != value2);
    case WIPE_OP_LT:
      return (value1 < value2);
    case WIPE_OP_LE:
      return (value1 <= value2);
    case WIPE_OP_GT:
      return (value1 > value2);
    case WIPE_OP_GE:
      return (value1 >= value2);
    default:
      return false;
  }
}


//-----------------------------------------------------------------------------
// Function: wipeEvalBoolRelation
//   Evaluate an boolean relation.
// Parameters:
//   value1:IO     - The first operand value.
//   value2:I      - The second operand value.
//   operation:I   - The operator ID.
// Returns: The truth of the relation.
// Inputs/Outputs: (none)
//-----------------------------------------------------------------------------
boolean wipeEvalBoolRelation(boolean value1, boolean value2, uint8 operation)
{
  switch (operation)
  {
    case WIPE_OP_EQ:
      return (value1 == value2);
    case WIPE_OP_NE:
      return (value1 != value2);
    default:
      return false;
  }
}


//-----------------------------------------------------------------------------
// Function: wipeEvalIntOperation
//   Evaluate a simple (unary or binary) integer expression.
// Parameters:
//   value1:IO     - Address of the first operand value and into which the
//                   the result is stored upon return.
//   value2:I      - The second operand value. The value is ignored if
//                   operation is unary.
//   operation:I   - The operator ID.
// Returns: true, iff the operation is valid for this type.
// Inputs/Outputs: (none)
//-----------------------------------------------------------------------------
boolean wipeEvalIntOperation(int32 *value1, int32 value2, uint8 operation)
{
  switch (operation)
  {
    case WIPE_OP_NEG:
      *value1 = -*value1;
      break;
    case WIPE_OP_PLUS:
      *value1 = *value1 + value2;
      break;
    case WIPE_OP_MINUS:
      *value1 = *value1 - value2;
      break;
    case WIPE_OP_MULT:
      *value1 = *value1 * value2;
      break;
    case WIPE_OP_DIV:
      *value1 = *value1 / value2;
      break;
    case WIPE_OP_MOD:
      *value1 = *value1 % value2;
      break;
    default:
      return false;
  }
  return true;
}


//-----------------------------------------------------------------------------
// Function: wipeEvalFloatOperation
//   Evaluate a simple (unary or binary) floating point expression.
// Parameters:
//   value1:IO     - Address of the first operand value and into which the
//                   the result is stored upon return.
//   value2:I      - The second operand value. The value is ignored if
//                   operation is unary.
//   operation:I   - The operator ID.
// Returns: true, iff the operation is valid for this type.
// Inputs/Outputs: (none)
//-----------------------------------------------------------------------------
boolean wipeEvalFloatOperation(float *value1, float value2, uint8 operation)
{
  switch (operation)
  {
    case WIPE_OP_NEG:
      *value1 = -*value1;
    case WIPE_OP_PLUS:
      *value1 = *value1 + value2;
      break;
    case WIPE_OP_MINUS:
      *value1 = *value1 - value2;
      break;
    case WIPE_OP_MULT:
      *value1 = *value1 * value2;
      break;
    case WIPE_OP_DIV:
      *value1 = *value1 / value2;
      break;
    default:
      return false;
  }
  return true;
}


//-----------------------------------------------------------------------------
// Function: wipeEvalBoolOperation
//   Evaluate a simple (unary or binary) boolean point expression.
// Parameters:
//   value1:IO     - Address of the first operand value and into which the
//                   the result is stored upon return.
//   value2:I      - The second operand value. The value is ignored if
//                   operation is unary.
//   operation:I   - The operator ID.
// Returns: true, iff the operation is valid for this type.
// Inputs/Outputs: (none)
//-----------------------------------------------------------------------------
boolean wipeEvalBoolOperation(boolean *value1, boolean value2, uint8 operation)
{
  switch (operation)
  {
    case WIPE_OP_COMPL:
      *value1 = !*value1;
    case WIPE_OP_OR:
      *value1 = *value1 + value2;
      break;
    case WIPE_OP_AND:
      *value1 = *value1 - value2;
      break;
    default:
      return false;
  }
  return true;
}


//-----------------------------------------------------------------------------
// Function: wipeIsRelational
//   Determine if an operator is a relational operator.
// Parameters:
//   operation:I  - ID of the operator to check.
// Returns: true, iff operation is a relational operator.
// Inputs/Outputs: (none)
//-----------------------------------------------------------------------------
boolean wipeIsRelational(uint8 operation)
{
  switch (operation)
  {
    case WIPE_OP_EQ:
    case WIPE_OP_NE:
    case WIPE_OP_LT:
    case WIPE_OP_LE:
    case WIPE_OP_GT:
    case WIPE_OP_GE:
      break;
    default:
      return false;
  }
  return true;
}


//-----------------------------------------------------------------------------
// Function: wipeAssign
//   Assign a value to a variable with type casting where possible.
// Parameters:
//   var:O      - Address of variable to assign.
//   varType:I  - Assigned variable's type.
//   value:I    - Value to be assigned.
//   valType:I  - Value's type.
// Returns: true, iff the assignment was possible.
// Inputs/Outputs: (none)
//-----------------------------------------------------------------------------
boolean wipeAssign( SymValue_t *var, uint8 varType, SymValue_t value,
                    uint8 valType )
{  
  if (varType == valType)
  {
    *var = value;
    return true;
  }
  else if (WIPE_ID_BOOL == varType)
  {
    if ( (WIPE_ID_INT == valType) &&
         ((0 == value.intValue) || (1 == value.intValue))
       )
    {
      *var = value;
      return true;
    }
  }
  else if (WIPE_ID_INT == varType)
  {
    if (WIPE_ID_BOOL == valType)
    {
      *var = value;
      return true;
    }
    else
    {
      if (WIPE_ID_FLOAT == valType)
      {
        if ( (value.floatValue >= (float)MININT32) &&
             (value.floatValue <= (float)MAXINT32) )
        {
          (*var).intValue = (int)value.floatValue;
          return true;
        }
      }
    }
  }
  else
  {
    if (WIPE_ID_FLOAT == varType)
    {
      if (WIPE_ID_INT == valType)
      {
        (*var).floatValue = (float)value.intValue;
        return true;
      }
      else
      {
        if (WIPE_ID_BOOL == valType)
        {
          (*var).floatValue = (float)value.boolValue;
          return true;
        }
      }
    }
  }
  return false;
}


//-----------------------------------------------------------------------------
// Function: wipeAreTypesValid
//   Determine if a given binary operator applied to two operands yields the
//   specified result type.
// Parameters:
//   operation:I  - Operator.
//   type1:I      - First operand's type.
//   value1:I     - First operand's value.
//   type2:I      - Second operand's type.
//   value2:I     - Second operand's value.
//   resType:I    - Result type expected.
// Returns: true, iff the operands are type-compatible within the context
//                of the operator being applied.
// Inputs/Outputs: (none)
//-----------------------------------------------------------------------------
boolean wipeAreTypesValid( uint8      operation,
                           uint8      type1,
                           SymValue_t value1,
                           uint8      type2,
                           SymValue_t value2,
                           uint8      resType ) 
{
  SymValue_t tmpVal;

  if (type1 == type2)
  {
    if (resType == type1)
    {
      return true;
    }
    else
    {
      if ( (WIPE_ID_BOOL == resType) && wipeIsRelational(operation) )
        return true;
    }
  }
  else
  {
    /* type1 != type2 */
    if ( (WIPE_ID_BOOL == type1) || (WIPE_ID_BOOL == type2) )
    {
      if (WIPE_ID_BOOL == type1)
        tmpVal = value2;
      else
        tmpVal = value1;
      if ( (0 == tmpVal.intValue) || (1 == tmpVal.intValue) )
      {
        if (WIPE_ID_BOOL == resType)
          return true;
      }
    }
  }
  return false;
}


//-----------------------------------------------------------------------------
// Function: wipeScanToken
//   Scan for the next token in the WIPE command line.
// Parameters: (none)
// Returns: WIPE_UNDEF or WIPE token (e.g. WIPE_GOTO, WIPE_LABEL, etc.).
// Inputs/Outputs:
//   m_consoleLen:I
//   m_consolePos:IO
//   m_tokenBuffer:IO
//-----------------------------------------------------------------------------
uint8 wipeScanToken(void)
{
  char   *token = NULL;
  uint8   tmp;

  if (m_consolePos >= m_consoleLen)
    return WIPE_UNDEF;
  tmp = strspn(&m_tokenBuffer[m_consolePos], " \n");
  token = strtok(&m_tokenBuffer[m_consolePos], " (\n");
  m_consolePos += tmp;
    
  if (token != NULL)
  {
    m_consolePos += strlen(token) + 1;
    if (strcmp(token, "if") == 0)
      return WIPE_IF;
    else if (strcmp(token, "goto") == 0)
      return WIPE_GOTO;
    else if (strcmp(token, "label") == 0)
      return WIPE_LABEL;
    else if (strcmp(token, "let") == 0)
      return WIPE_LET;
    else if (strcmp(token, "var") == 0)
      return WIPE_VAR;
    else if (strcmp(token, "print") == 0)
      return WIPE_PRINT;
    else
    {
      if (m_consoleBuffer[m_consolePos - 1] != '(')
      {
        return WIPE_UNDEF;
      }
      if (token != NULL)
      {
        if (strcmp(token, "Bkgrd") == 0)
          return WASPCMD_BKGRD;
        else if (strcmp(token, "Group") == 0)
          return WASPCMD_GROUP;
        else if (strcmp(token, "Line") == 0)
          return WASPCMD_LINE;
        else if (strcmp(token, "Reset") == 0)
          return WASPCMD_RESET;
        else if (strcmp(token, "State") == 0)
          return WASPCMD_STATE;
        else if (strcmp(token, "Shift") == 0)
          return WASPCMD_SHIFT;
        else if (strcmp(token, "Swap") == 0)
          return WASPCMD_SWAP;
        else if (strcmp(token, "Rain") == 0)
          return WASPCMD_RAINBOW;
        else if (strcmp(token, "Cycle") == 0)
          return WASPCMD_RAINCYCLE;
        else if (strcmp(token, "Speed") == 0)
          return WASPCMD_SPEED;
        else if (strcmp(token, "Twinkle") == 0)
          return WASPCMD_TWINKLE;
        else if (strcmp(token, "Pause") == 0)
          return WIPE_FN_PAUSE;
        else
          return WIPE_UNDEF;
      }
    }
  }
  return WIPE_UNDEF;
}


//-----------------------------------------------------------------------------
// Function: wipeScanIdentifier
//   Scan for an identifier.
// Parameters:
//   identifier:O  - Pointer to identifier string, if the function returns
//                   true; NULL, otherwise.
//   maxLen:I      - Max # of characters allowed in identifier.
// Returns: Length of identifier, if a valid identifier was scanned; zero,
//          otherwise.
// Inputs/Outputs:
//   m_consoleLen:I
//   m_consolePos:IO
//   m_tokenBuffer:IO
//-----------------------------------------------------------------------------
uint8 wipeScanIdentifier(char **identifier, uint8 maxLen)
{
  char    *token = NULL;
  char     tmpStr[1] = "";
  uint8    strLen;
  uint8    tmp;
  uint8    i;

  *identifier = NULL;
  if (m_consolePos >= m_consoleLen)
    return false;
  tmp = strspn(&m_tokenBuffer[m_consolePos], " \n");
  token = strtok(&m_tokenBuffer[m_consolePos], " \n");
  m_consolePos += tmp;
  if (token != NULL)
  {
    strLen = strlen(token);
    m_consolePos += strLen + 1;
    for (i = 0; i < strLen; i++)
    {
      if (!isalnum(token[i]) && (token[i] != '_') )
      {
        wipeShowError(FLASH("Bad ID"));
        return 0;
      }
    }
    if ( isdigit(token[0]) || (0 == strLen) )
    {
      wipeShowError(FLASH("Bad ID"));
      return 0;
    }
    if (strLen > maxLen)
    {
      wipeShowError(FLASH("ID too long"));
      return 0;
    }

    *identifier = token;
    return strLen;
  }
  return 0;
}


//-----------------------------------------------------------------------------
// Function: wipeShowExprErr
//   Show detected error in an expression string.
// Parameters:
//   expStr:I   - Pointer to the expression string.
//   errPos:I   - Position at which error is detected within the expression.
//   eval:I     - An expression is being evaluate, if true. An expression is
//                just being parsed, if false.
//   errStr:I   - Error string to display. The argument should be provided
//                using the FLASH() macro:
//                  e.g. wipeShowExprErr(FLASH("Error text"));
// Returns: (none)
// Inputs/Outputs:
//   m_consolePos:I
//   m_exprOffs:I
//   m_wipeRunByte:I
//-----------------------------------------------------------------------------
void wipeShowExprErr(char    *exprStr,
                     uint8    errPos,
                     boolean  eval,
                     const __FlashStringHelper *errStr)
{
  if (eval)
  {
    /* Pretty print the tokenized WIPE statement. */
    wipePrintStatement();
    for (uint8 i = 0; i < (m_wipeRunByte + errPos); i++)
      logPrint(FLASH(" "));
  }
  else
  {
    /* Output the raw ASCII console line being that's being parsed */
    wipeShowConsoleLine();
    for (uint8 i = 0; i < (m_consolePos + m_exprOffs + errPos); i++)
      logPrint(FLASH(" "));
  }

  logPrintln(FLASH("^"));
  logPrintln(errStr);
}


//-----------------------------------------------------------------------------
// Function: wipeScanOperator
//   Scan for an operator in an expression.
// Parameters:
//   ppPos:IO   - Address of a pointer to the current character in the
//                expression string. Upon invocation, this should point to a
//                string position preceding a presumed operand. Upon
//                return this points to the first white space character
//                following the operand, if the function returns false;
//                otherwise it points to the position at which the error
//                was detected.
//   pKind:O    - Address at which to return the kind of operator (e.g.
//                WIPE_OP_PLUS).
//   pOpLen:O   - Number of characters in the operator.
//   pScanLen:O - Address at which to return the total number of characters
//                scanned.
// Returns: true, iff a valid operator was successfully scanned or the end
//                of line was reached.
// Inputs/Outputs: (none)
//-----------------------------------------------------------------------------
boolean wipeScanOperator( char **ppPos, uint8 *pKind, uint8 *pOpLen,
                          uint8 *pScanLen )
{
  char     c;
  char    *pChar;
  boolean  ret;
  
  *pKind = WIPE_OP_UNDEF;
  pChar = *ppPos;
  *pScanLen = 0;

  /* Skip white space */
  c = *pChar++;
  while (c == ' ')
    c = *pChar++;

  if ( ('\r' == c) || ('\0' == c) )
  {
    return true;
  }

  *pScanLen = pChar - *ppPos;
  *ppPos = pChar;
  *pOpLen = 1;
  switch (c)
  {
    case '+':
      *pKind = WIPE_OP_PLUS;
      return true;
    case '-':
      *pKind = WIPE_OP_MINUS;
      return true;
    case '*':
      *pKind = WIPE_OP_MULT;
      return true;
    case '/':
      *pKind = WIPE_OP_DIV;
      return true;
    case '%':
      *pKind = WIPE_OP_MOD;
      return true;
    case '=':
      *pKind = WIPE_OP_EQ;
      return true;
    case ',':
    case ')':
      *pKind = WIPE_OP_DELIM;
      return true;
    case '|':
      if ('|' == *pChar)
      {
        if (' ' == *(pChar+1))
        {
          (*ppPos)++;
          (*pOpLen)++;
          (*pScanLen)++;
          *pKind = WIPE_OP_OR;
          return true;
        }
        else
          return false;
      }
      else
        return false;
    case '&':
      if ('&' == *pChar)
      {
        if (' ' == *(pChar+1))
        {
          (*ppPos)++;
          (*pOpLen)++;
          (*pScanLen)++;
          *pKind = WIPE_OP_AND;
          return true;
        }
        else
          return false;
      }
      else
        return false;
    case '!':
      if ('=' == *pChar)
      {
        (*ppPos)++;
        (*pOpLen)++;
        (*pScanLen)++;
        *pKind = WIPE_OP_NE;
        return true;
      }
      else
        return false;
    case '<':
      if (' ' == *pChar)
      {
        *pKind = WIPE_OP_LT;
        return true;
      }
      else if ('=' == *pChar)
      {
        if (' ' == *(pChar+1))
        {
          (*ppPos)++;
          (*pOpLen)++;
          (*pScanLen)++;
          *pKind = WIPE_OP_LE;
          return true;
        }
        else
          return false;
      }
      else
        return false;
    case '>':
      if (' ' == *pChar)
      {
        *pKind = WIPE_OP_GT;
        return true;
      }
      else if ('=' == *pChar)
      {
        if (' ' == *(pChar+1))
        {
          (*ppPos)++;
          (*pOpLen)++;
          (*pScanLen)++;
          *pKind = WIPE_OP_GE;
          return true;
        }
        else
          return false;
      }
      else
        return false;
    default:
      return false;
  }
}


//-----------------------------------------------------------------------------
// Function: wipeScanOperand
//   Scan for the next operand in an expression. It also scan unary operators.
// Parameters:
//   ppPos:IO   - Address of a pointer to the current character in the
//                expression string. Upon invocation, this should point to a
//                string position preceding a presumed operand. Upon
//                return this points to the first white space character
//                following the operand, if the function returns false;
//                otherwise it points to the position at which the error
//                was detected.
//   eval:I     - Expression is being evaluated.
//   pKind:O    - Address at which to return the kind of operand (e.g.
//                WIPE_OPRND_LIT).
//   ppStart:O  - Address at which to return the pointer to the first
//                character of the operand.
//   pLen:O     - Address at which to return the number of characters in
//                the operand.
//   pScanLen:O - Address at which to return the total number of characters
//                scanned.
//   pUnary:O   - Address at which to return the unary operator (e.g.
//                WIPE_OP_NEG). The output value is WIPE_OP_UNDEF if
//                there wasn't one.
// Returns: true, iff an operand was successfully scanned.
// Inputs/Outputs: (none)
//-----------------------------------------------------------------------------
boolean wipeScanOperand( char    **ppPos,
                         boolean   eval,
                         uint8    *pKind,
                         char    **ppStart,
                         uint8    *pLen,
                         uint8    *pScanLen,
                         uint8    *pUnary)
{
  char     c;
  char    *pChar;
  uint8    strLen;
  uint8    tmp;
  boolean  foundDecPoint = false;

  *pKind = WIPE_OPRND_UNDEF;
  *pUnary = WIPE_OP_UNDEF;
  pChar = *ppPos;
  *ppStart = pChar;
  *pLen = 0;
  *pScanLen = 0;
  
  /* Skip white space */
  c = *pChar++;
  while (c == ' ')
    c = *pChar++;
    
  if ('\0' == c)
  {
    return false;
  }

  /* Scan for a negation/complement */
  if ( (c == '-') || (c == '!') )
  {
    if (c == '-')
      *pUnary = WIPE_OP_NEG;
    else
      *pUnary = WIPE_OP_COMPL;
    c = *pChar++;
    if (' ' == c)
    {
      wipeShowExprErr( *ppPos, (pChar - *ppPos) + m_exprOffs, eval,
                       FLASH("Invalid unary operation"));
    }
  }

  *ppStart = (pChar - 1);


  /*---  scan the operand  ---*/

  /* Determine if operand is a literal */
  if (isdigit(c))
  {
    *pKind = WIPE_OPRND_LIT;
    c = *pChar++;
  }
  else
  {
    *pKind = WIPE_OPRND_ID;
  }

  while ( (c != ' ') && (c != '\0') && (c != ',') && (c != ')') )
  {
    if (WIPE_OPRND_LIT == *pKind)
    {
      if (!isdigit(c) && (c != '.'))
      {
        wipeShowExprErr( *ppPos, (pChar - *ppPos) + m_exprOffs, eval,
                         FLASH("Invalid literal value") );
        return false;
      }
      if ('.' == c)
      {
        if (!foundDecPoint)
        {
          foundDecPoint = true;
        }
        else
        {
          wipeShowExprErr( *ppPos, (pChar - *ppPos) + m_exprOffs, eval,
                           FLASH("Invalid floating point number") );
          return false;
        }
      }
    }
    else
    {
      /* (WIPE_OPRND_ID == *pKind) */
      if (!isalnum(c) && (c != '_'))
      {
        wipeShowExprErr( *ppPos, (pChar - *ppStart) + m_exprOffs, eval,
                         FLASH("Invalid identifier") );
        return false;
      }
    }
    c = *pChar++;
  }
  pChar--;

  if ( (WIPE_OPRND_LIT != *pKind) && ((pChar - *ppStart) > MAX_ID_LEN) )
  {
    wipeShowExprErr( *ppPos, (pChar - *ppPos), eval,
                     FLASH("Identifier name is too long"));
    return false;
  }

  *pLen = (pChar - *ppStart);
  *pScanLen = (pChar - *ppPos);
  *ppPos = pChar;
  return true;
}


//-----------------------------------------------------------------------------
// Function: wipeParseExpression
//   Parse, and optionally evaluate, an expression.
// Parameters:
//   pExpr:I       - Pointer to the start of the expression string.
//   type:I        - The type to which the expression should evaluate. If set
//                   to WIPE_ID_UNDEF, the expression is only parsed. Valid
//                   types are: WIPE_ID_INT, WIPE_ID_FLOAT, WIPE_ID_BOOL.
//   listOk:I      - Allow an expression to be terminated by a comma or
//                   closed parenthesis, iff true.
//   pResult:O     - Address at which to place the result, if evaluate
//                   is set to true.
// Returns: true, iff parsing and/or evaluation was successful.
// Inputs/Outputs:
//   m_symTbl:I
//   m_exprOffs:O
//-----------------------------------------------------------------------------
boolean wipeParseExpression( char *pExpr, uint8 type, boolean listOk,
                             SymValue_t *pResult )
{
  char       *pPos = pExpr;
  char       *pStart;
  SymValue_t  oprndValue1;
  SymValue_t  currOprndValue;
  uint32      dummyValue;
  float       tmpValue;
  boolean     scanOk;
  boolean     firstOprnd = true;
  boolean     legalOp;
  uint8       oprndKind;
  uint8       oprndType1;
  uint8       currOprndType;
  uint8       oprndLen;
  uint8       scanLen;
  uint8       operatorLen;
  uint8       operation;
  uint8       unary;
  uint8       oprndIdx;     // Symbol table index for variable
  uint8       i;
  char        tmpChar;

  m_exprOffs = 0;
  while (true)
  {
    scanOk = wipeScanOperand( &pPos, (type != WIPE_ID_UNDEF), &oprndKind,
                              &pStart, &oprndLen, &scanLen, &unary );
    if (!scanOk)
    {
      return false;
    }
    
    if (type != WIPE_ID_UNDEF)
    {
      /* We have to evaluate the expression. */

      /* First determine the current operand's type and value */
      if (WIPE_OPRND_ID == oprndKind)
      {
        /* Find the variable's symbol table index. */
        tmpChar = *(pStart + oprndLen);
        *(pStart + oprndLen) = '\0';
        if (!wipeSymbolLookup(pStart, &oprndIdx))
        {
          *(pStart + oprndLen) = tmpChar;
          wipeShowExprErr( pExpr, (pStart - pExpr + oprndLen), true,
                           FLASH("Variable not defined") );
          return false;
        }
        *(pStart + oprndLen) = tmpChar;

        currOprndType = m_symTbl[oprndIdx].typeId;
        currOprndValue.intValue = (uint32)m_symTbl[oprndIdx].symValue.intValue;
      }
      else
      {
        /* (oprndKind == WIPE_OPRND_LIT): Determine the literal value. */
        currOprndValue.intValue = 0;
        currOprndType = WIPE_ID_INT;
        for ( ; oprndLen > 0; oprndLen--)
        {
          if ('.' == *pStart)
          {
            currOprndType = WIPE_ID_FLOAT;
            pStart++;
            break;
          }
          currOprndValue.intValue *= 10;
          currOprndValue.intValue += *pStart++ - (uint32)'0';
        }
        if (oprndLen > 0)
        {
          tmpValue = (float)currOprndValue.intValue;
          i = oprndLen - 1;
          for ( ; i > 0; i--)
          {
            tmpValue += ((float)(*pStart++ - (uint32)'0') /
                        pow(10, oprndLen - i));
          }
          currOprndValue.floatValue = tmpValue;
        }
      }

      /* Apply unary operator is there is one. */
      if (unary != WIPE_OP_UNDEF)
      {
        switch (currOprndType)
        {
          case WIPE_ID_INT:
            legalOp = wipeEvalIntOperation(
                                    (int32 *)&currOprndValue.intValue,
                                    (int32)dummyValue,
                                    unary );
            break;
          case WIPE_ID_FLOAT:
            legalOp = wipeEvalFloatOperation(
                                    (float *)&currOprndValue.floatValue,
                                    (float)dummyValue,
                                    unary );
            break;
          case WIPE_ID_BOOL:
            legalOp = wipeEvalBoolOperation(
                                    (boolean *)&currOprndValue.boolValue,
                                    (boolean)dummyValue,
                                    unary );
            break;
          default:
            logPrint(FLASH("***SYS ERROR: unknown expr type-"));
            logPrintln(type);
            return false;
        }
        if (!legalOp)
        {
          wipeShowExprErr( pExpr, (pStart - pExpr), true,
                           FLASH("Bad unary operator") );
        } 
      }

      if (firstOprnd)
      {
        /* Just record the very first operand. */
        oprndValue1.intValue = currOprndValue.intValue;
        oprndType1 = currOprndType;
      }
      else
      {
        /* Compute intermediate result */
        
        /* Check type compatibility */
        if (!wipeAreTypesValid( operation, oprndType1, oprndValue1,
                                currOprndType, currOprndValue, type )
           )
        {
          wipeShowExprErr( pExpr, (pStart - pExpr), true,
                           FLASH("Incompatible operand types") );
          return false;
        }
        
        if (WIPE_ID_BOOL == type)
        {
          /* Perform binary computation */
          if (wipeIsRelational(operation))
          {
            switch (oprndType1)
            {
              case WIPE_ID_INT:
                oprndValue1.intValue = (uint32)wipeEvalIntRelation(
                                               (int32)oprndValue1.intValue,
                                               (int32)currOprndValue.intValue,
                                               operation );
                break;
              case WIPE_ID_FLOAT:
                oprndValue1.floatValue = (uint32)wipeEvalFloatRelation(
                                               (float)oprndValue1.floatValue,
                                               (float)currOprndValue.floatValue,
                                               operation );
                break;
              case WIPE_ID_BOOL:
                oprndValue1.boolValue = (uint32)wipeEvalBoolRelation(
                                               (boolean)oprndValue1.boolValue,
                                               (boolean)currOprndValue.boolValue,
                                               operation );
                break;
              default:
                logPrint(FLASH("***SYS ERROR: unknown expr type-"));
                logPrintln(type);
                return false;
            }
          }
        }
        else
        {
          /* Evaluate arithmetic operator */
          switch (oprndType1)
          {
            case WIPE_ID_INT:
              legalOp = wipeEvalIntOperation( (int32 *)&oprndValue1.intValue,
                                              (int32)currOprndValue.intValue,
                                              operation );
              break;
            case WIPE_ID_FLOAT:
              legalOp = wipeEvalFloatOperation( (float *)&oprndValue1.floatValue,
                                                (float)currOprndValue.floatValue,
                                                operation );
              break;
            case WIPE_ID_BOOL:
              legalOp = wipeEvalBoolOperation( (boolean *)&oprndValue1.boolValue,
                                               (boolean)currOprndValue.boolValue,
                                               operation );
              break;
            default:
              logPrint(FLASH("***SYS ERROR: unknown expr type-"));
              logPrintln(type);
              return false;
          }
          if (!legalOp)
          {
            wipeShowExprErr( pExpr, (pStart - pExpr), true,
                             FLASH("Bad operator for type") );
          } 
        }
      }
    }
    m_exprOffs += scanLen;

    scanOk = wipeScanOperator(&pPos, &operation, &operatorLen, &scanLen);
    m_exprOffs += scanLen;
    if (scanOk)
    {
      /* Potential operator or End-of-line found */
      if (!listOk && (operation == WIPE_OP_DELIM))
      {
        wipeShowExprErr( pExpr, (pStart - pExpr + scanLen),
                         (type != WIPE_ID_UNDEF),
                         FLASH("Unexpected list") );
        return false;
      }
      if ( (operation == WIPE_OP_UNDEF) || (operation == WIPE_OP_DELIM))
      {
        /* We've hit the end of the expression. */
        if (type != WIPE_ID_UNDEF)
        {
          /* We have to provide a result. */
          if (!wipeAssign(pResult, type, oprndValue1, oprndType1))
          {
            wipeShowExprErr( pExpr, (pStart - pExpr), true,
                             FLASH("Expression is of wrong type") );
            return false;
          }
        }
        return true;
      }
    }
    else
    {
      wipeShowExprErr( pExpr, (pStart - pExpr), (type != WIPE_ID_UNDEF),
                       FLASH("Bad operator") );
      return false;
    }
    firstOprnd = false;
  }
}


//-----------------------------------------------------------------------------
// Function: wipeLabelLocate
//   Locate a specific program label. If located and not already in the
//   symbol table, the label definition is added to the symbol table.
// Parameters:
//   label:I     - Program label identifier.
//   progPos:O   - Address at which to return the m_program[] index at the
//                 located line starts.
// Returns: true iff the label was located and was in, or was able to be added
//               to, the symbol table.
// Inputs/Outputs:
//   m_program:I
//   m_wipeProgByte:I
//   m_symTbl:IO
//-----------------------------------------------------------------------------
boolean wipeLabelLocate(char *label, uint16 *progPos)
{
  uint16   currLine;
  uint8    symIdx = MAX_NUM_SYMBOLS;
  boolean  located = false;

  /* Check if label is already in the symbol table. */
  if (wipeSymbolLookup(label, &symIdx))
  {
    *progPos = m_symTbl[symIdx].symValue.intValue;
    return true;
  }

  /* The label isn't in the symbol table. */
  for ( currLine = m_wipeLineStart;
        currLine < m_wipeProgByte;
        currLine += m_program[currLine + TKNZD_LEN_OFFS]
      )
  {
    if (m_program[currLine + TKNZD_STMT_OFFS] == WIPE_LABEL)
    {
      /* Current program line is a 'label' statement. */
      if (strcmp((char *)&m_program[currLine + TKNZD_STMT_OFFS + 1], label)
          == 0)
      {
        /* The current program line's label matches the target label.
         * Add the label definition to the symbol table and return
         * the target line's starting position in m_program[].
         */
        if (wipeSymbolInsert(label, WIPE_ID_LABEL, &symIdx))
        {
          m_symTbl[symIdx].symValue.intValue = currLine;
          *progPos = currLine;
          return true;
        }
        else
        {
          logPrint(FLASH("No room for '"));
          logPrint(label);
          logPrintln(FLASH("' in sym table"));
          return false;
        }
      }
    }
  }
  return false;
}



//-----------------------------------------------------------------------------
// Function: wipeFuncGetParmNum
//   Retrieve the number of parameters expected for each of WIPE's WASP
//   functions.
// Parameters:
//   funcId:I  - Function identifier.
// Returns: The number of parameters expected.
// Inputs/Outputs:
//-----------------------------------------------------------------------------
uint8 wipeFuncGetParmNum(uint8 funcId)
{
  switch (funcId)
  {
    case WASPCMD_GROUP:       return 4;
    case WASPCMD_STATE:       return 2;
    case WASPCMD_BKGRD:       return 4;
    case WASPCMD_LINE:        return 6;
    case WASPCMD_SHIFT:       return 2;
    case WASPCMD_SWAP:        return 7;
    case WASPCMD_RESET:       return 1;
    case WASPCMD_SPEED:       return 2;
    case WASPCMD_RAINBOW:     return 2;
    case WASPCMD_RAINCYCLE:   return 1;
    case WASPCMD_TWINKLE:     return 5;
    case WIPE_FN_PAUSE:       return 3;
    default:
      logPrintln(FLASH("***SYS ERROR: Unknown func"));
      return 0;
  }
}


//-----------------------------------------------------------------------------
// Function: wipeParseStatement
//   Parse and tokenize a WIPE programming language statement.
// Parameters:
//   lineNum:I     - The line number assigned to the candidate statement. A
//                   value of 0 means that the line is going to be executed
//                   immediately.
//   initToken:I   - The prescanned token, if applicable. Set the value
//                   to WIPE_UNDEF if a statement token needs to be scanned
//                   first by the function.
//   stmtErr:O     - The parameter is set to true upon return iff a WIPE
//                   statement has been detected but contains an error that
//                   has already been reported.
// Returns: true iff the statement is valid.
// Inputs/Outputs:
//   m_consoleBuffer:I
//   m_consoleLen:I
//   m_exprOffs:I
//   m_wipeLineStart:I
//   m_consolePos:IO
//   m_program:IO
//   m_tokenBuffer:IO
//   m_wipeProgByte:IO
// Notes:
//   WIPE staement format:
//      byte   byte
//    +-------+-----+------+-
//    | LINE# | LEN | STMT |              b
//    +-------+-----+------+-             y
//                           strlen() <=  t
//                     ...   MAX_ID_LEN   e    byte
//                  +------+-------------+--+--------+  (LEN =
//                  | var  | NAME-STRING |\0| TYPEID |   strlen() + 5)
//                  +------+-------------+--+--------+--+
//                  | let  | NAME-STRING |\0| EXPR   |\0|
//                  +------+-------------+--+--------+--+
//                  |label | NAME-STRING |\0|
//                  +------+-------------+--+
//                  | goto | NAME-STRING |\0|
//                  +------+-------------+--+
//                  +------+------+--+
//                  |  if  | EXPR |\0|
//                  +------+------+--+
//                  +------+-------+-------+-------+
//                  |pause | uint8 | uint8 | uint8 |
//                  +------+-------+-------+-------+
//                  +------+--------------+---------------+--+
//                  |print | WIPE_PRT_STR | QUOTED-STRING |\0|
//                  +------+--------------+---------------+--+
//                         | WIPE_PRT_VAR | NAME-STRING   |\0|
//                         +--------------+---------------+--+
//                  +------+---+---+------+-----+------+
//                  |[func]|<f>|<n>| EXPR | ... | EXPR | (<n> EXPR instances)
//                  +------+---+---+------+-----+------+
//
//    EXPR = a simple expression with no operator precedence
//    LEN  = total # of bytes in the line.
//    STMT = statement type
//    <f> in {WASP command codes (e.g. WASPCMD_GROUP) & misc functions
//            (e.g. WIPE_FN_PAUSE)}
//    <n> = the number of uint8 parameters defined for <f> that follow.
//-----------------------------------------------------------------------------
boolean wipeParseStatement(uint8 lineNum, uint8 initToken, boolean *stmtErr)
{
  char       *identifier;
  char       *typeStr;
  char       *token;
  SymValue_t  dummyValue;
  uint8       tokenId;
  uint8       typeId;
  uint8       symIdx;
  uint8       tmp;
  uint8       strLen;
  uint8       parmsLen = 0;


  if (initToken != WIPE_UNDEF)
    tokenId = initToken;
  else
    tokenId = wipeScanToken();
  if (WIPE_UNDEF == tokenId)
    return false;

  m_program[m_wipeProgByte++] = lineNum;
  m_program[m_wipeProgByte++] = 3;
  m_program[m_wipeProgByte++] = tokenId;

  *stmtErr = true;
  switch (tokenId)
  {
    case WIPE_IF:
      /* Parse the expression */
      if (!wipeParseExpression( &m_tokenBuffer[m_consolePos],
                                 WIPE_OPRND_UNDEF, false, &dummyValue ))
      {
        wipeShowError(FLASH("Expr Syntax"));
        return false;
      }

      /* Statement is valid */
      memcpy( &m_program[m_wipeProgByte], &m_consoleBuffer[m_consolePos],
              m_exprOffs + 1 );
      m_wipeProgByte += m_exprOffs;
      m_program[m_wipeLineStart + TKNZD_LEN_OFFS] =
                             (m_wipeProgByte - m_wipeLineStart + 1);
      *stmtErr = false;
      return true;
      break;
    case WIPE_GOTO:
      strLen = wipeScanIdentifier(&identifier, MAX_ID_LEN);
      if (0 == strLen)
        return false;

      /* Statement is valid */
      strcpy((char *)&m_program[m_wipeProgByte], identifier);
      m_wipeProgByte += strlen(identifier) + 1;
      m_program[m_wipeLineStart + TKNZD_LEN_OFFS] =
                             (m_wipeProgByte - m_wipeLineStart);
      *stmtErr = false;
      return true;
      break;
    case WIPE_LABEL:
      strLen = wipeScanIdentifier(&identifier, MAX_ID_LEN);
      if (0 == strLen)
        return false;

      /* Statement is valid */
      strcpy((char *)&m_program[m_wipeProgByte], identifier);
      m_wipeProgByte += strlen(identifier) + 1;
      m_program[m_wipeLineStart + TKNZD_LEN_OFFS] =
                             (m_wipeProgByte - m_wipeLineStart);
      *stmtErr = false;
      return true;
      break;
    case WIPE_LET:
      strLen = wipeScanIdentifier(&identifier, MAX_ID_LEN);
      if (0 == strLen)
        return false;
      strcpy((char *)&m_program[m_wipeProgByte], identifier);
      m_wipeProgByte += strlen(identifier) + 1;
      
      /* Scan for, and skip over, the "=" */
      tmp = strspn(&m_tokenBuffer[m_consolePos], " \n");
      token = strtok(&m_tokenBuffer[m_consolePos], " \n");
      m_consolePos += tmp + 1;
      if (strcmp(token, "=") != 0)
      {
        wipeShowError(FLASH("Missing '='"));
        return false;
      }
      m_consolePos += strlen(token);

      /* Parse the expression */
      if (!wipeParseExpression( &m_tokenBuffer[m_consolePos],
                                 WIPE_OPRND_UNDEF, false, &dummyValue ))
      {
        wipeShowError(FLASH("Expr Syntax"));
        return false;
      }

      /* Expression is valid */
      memcpy( &m_program[m_wipeProgByte], &m_consoleBuffer[m_consolePos],
              m_exprOffs + 1 );
      m_wipeProgByte += m_exprOffs;
      m_program[m_wipeLineStart + TKNZD_LEN_OFFS] =
                             (m_wipeProgByte - m_wipeLineStart + 1);
      *stmtErr = false;
      return true;
      break;
    case WIPE_PRINT:
    {
      char  *pChar;
      uint8  maxLen;
      
      if (m_consoleBuffer[m_consolePos] == '\"')
      {
        /* Print a string */
        m_consolePos++;
        maxLen = MAX_SERIAL_BUF_LEN - m_consolePos - 1;
        m_program[m_wipeProgByte++] = WIPE_PRT_STR;
        pChar = &m_tokenBuffer[m_consolePos];
        strLen = 0;
        for (parmsLen = 0; parmsLen < maxLen; parmsLen++)
        {
          if ('"' == *pChar)
            break;

          strLen++;
          if ( (*pChar == '\\') && (*(pChar+1) == 'n') )
          {
            pChar++;
            parmsLen++;
            m_program[m_wipeProgByte++] = '\n';
          }
          else
          {
            m_program[m_wipeProgByte++] = *pChar;
          }
          pChar++;
        }
        if (parmsLen == maxLen)
        {
          wipeShowError(FLASH("Bad string length"));
          return false;
        }
        m_program[m_wipeProgByte++] = '\0';
        m_program[m_wipeLineStart + TKNZD_LEN_OFFS] = m_wipeProgByte
                                                    - m_wipeLineStart;
        *stmtErr = false;
        return true;
      }
      else
      {
        /* Print result of an expression */
        m_program[m_wipeProgByte++] = WIPE_PRT_VAR;
        strLen = wipeScanIdentifier(&identifier, MAX_ID_LEN);
        if (0 == strLen)
          return false;
        strcpy((char *)&m_program[m_wipeProgByte], identifier);
        m_wipeProgByte += strlen(identifier) + 1;
        m_program[m_wipeLineStart + TKNZD_LEN_OFFS] = m_wipeProgByte
                                                    - m_wipeLineStart;
        *stmtErr = false;
        return true;
      }
      break;
    }
    case WIPE_VAR:
      strLen = wipeScanIdentifier(&identifier, MAX_ID_LEN);
      if (0 == strLen)
        return false;

      /* Scan for, and skip over, the ':' */
      tmp = strspn(&m_tokenBuffer[m_consolePos], " \n");
      token = strtok(&m_tokenBuffer[m_consolePos], " \n");
      m_consolePos += tmp + 1;
      if (strcmp(token, ":") != 0)
      {
        wipeShowError(FLASH("Missing ':'"));
        return false;
      }
      m_consolePos += strlen(token);

      /* Scan for the type identifier */
      if (0 == wipeScanIdentifier(&typeStr, 10))
        return false;

      /* Determine the variable's type */
      if (strcmp(typeStr, "label") == 0)
        typeId = WIPE_ID_LABEL;
      else if (strcmp(typeStr, "int") == 0)
        typeId = WIPE_ID_INT;
      else if (strcmp(typeStr, "float") == 0)
        typeId = WIPE_ID_FLOAT;
      else if (strcmp(typeStr, "bool") == 0)
        typeId = WIPE_ID_BOOL;
      else
      {
        wipeShowError(FLASH("Bad type"));
        return false;
      }

      /* Statement is valid */
      strcpy((char *)&m_program[m_wipeProgByte], identifier);
      m_wipeProgByte += strlen(identifier) + 1;
      m_program[m_wipeProgByte++] = typeId;
      m_program[m_wipeLineStart + TKNZD_LEN_OFFS] =
                             (m_wipeProgByte - m_wipeLineStart);
      *stmtErr = false;
      return true;
      break;
    case WASPCMD_BKGRD:
    case WASPCMD_GROUP:
    case WASPCMD_LINE:
    case WASPCMD_RESET:
    case WASPCMD_STATE:
    case WASPCMD_SHIFT:
    case WASPCMD_SWAP:
    case WASPCMD_RAINBOW:
    case WASPCMD_RAINCYCLE:
    case WASPCMD_SPEED:
    case WASPCMD_TWINKLE:
    case WIPE_FN_PAUSE:
    {
      uint8   numParms;
      uint8   consolePosStart = m_consolePos - 1;
      
      numParms = wipeFuncGetParmNum(tokenId);
      --m_wipeProgByte; /* Back up to statement id */
      m_program[m_wipeProgByte++] = WIPE_FUNC;
      m_program[m_wipeProgByte++] = tokenId;
      m_program[m_wipeProgByte++] = numParms;
      
      for (uint8 i = numParms; i != 0; i--)
      {
        /* Parse the expression */
        if (!wipeParseExpression( &m_tokenBuffer[m_consolePos],
                                   WIPE_OPRND_UNDEF, true, &dummyValue ))
        {
          wipeShowError(FLASH("Expr Syntax"));
          return false;
        }
        tmp = m_tokenBuffer[m_consolePos + m_exprOffs - 1];
        if ( (i > 1) && (tmp != ',') && (tmp != ')') )
        {
           wipeShowError(FLASH("Bad params"));
           return false;
        }
        parmsLen += m_exprOffs;
        m_consolePos += m_exprOffs;
      }
      
      if (m_tokenBuffer[m_consolePos - 1] != ')')
      {
        wipeShowError(FLASH("# of parms"));
        return false;
      }

      /* Statement is valid */
      memcpy( &m_program[m_wipeProgByte], &m_consoleBuffer[consolePosStart],
              parmsLen + 1);
      m_wipeProgByte += parmsLen + 1;
      m_program[m_wipeProgByte] = '\0';
      m_program[m_wipeLineStart + TKNZD_LEN_OFFS] =
                             (m_wipeProgByte - m_wipeLineStart + 1);

      *stmtErr = false;
      return true;
      break;
    }
    default:
      *stmtErr = true;
      return false;
  }
  wipeShowError(FLASH("Parse fail"));
  return false;
}


//-----------------------------------------------------------------------------
// Function: wipeScanCommand
//   Scan for the next command name in the WIPE command line.
// Parameters: (none)
// Returns: WIPE_UNDEF or WIPE command id (e.g. WIPE_CMD_EXIT, etc).
// Inputs/Outputs:
//   m_consoleLen:I
//   m_consolePos:IO
//   m_tokenBuffer:IO
//-----------------------------------------------------------------------------
uint8 wipeScanCommand(void)
{
  int32  intVal;
  char   *token = NULL;
  uint8   tmp;

  if (m_consolePos >= m_consoleLen)
    return WIPE_UNDEF;
  tmp = strspn(&m_tokenBuffer[m_consolePos], " \n");
  token = strtok(&m_tokenBuffer[m_consolePos], "( \n");
  m_consolePos += tmp;
  if (token != NULL)
  {
    m_consolePos += strlen(token) + 1;
    if (strcmp(token, "del") == 0)
      return WIPE_CMD_DEL;
    else if (strcmp(token, "dir") == 0)
      return WIPE_CMD_DIR;
    else if (strcmp(token, "erase") == 0)
      return WIPE_CMD_ERASE;
    else if (strcmp(token, "exit") == 0)
      return WIPE_CMD_EXIT;
    else if (strcmp(token, "help") == 0)
      return WIPE_CMD_HELP;
    else if (strcmp(token, "list") == 0)
      return WIPE_CMD_LIST;
    else if (strcmp(token, "load") == 0)
      return WIPE_CMD_LOAD;
    else if (strcmp(token, "prof") == 0)
      return WIPE_CMD_PROF;
    else if (strcmp(token, "renum") == 0)
      return WIPE_CMD_RENUM;
    else if (strcmp(token, "run") == 0)
      return WIPE_CMD_RUN;
    else if (strcmp(token, "save") == 0)
      return WIPE_CMD_SAVE;
    else if (strcmp(token, "start") == 0)
      return WIPE_CMD_START;
    else if (strcmp(token, "hexdump") == 0)
      wipeDumpProgramHex();
    else
    {
      m_consolePos -= strlen(token) + 1;
      if (wipeParseInt(&intVal, false))
      {
          return WIPE_CMD_LINENUM;
      }
      else
        m_consolePos = 0;
    }
  }
  return WIPE_UNDEF;
}


//-----------------------------------------------------------------------------
// Function: pause
//   Pause WIPE program execution.
// Parameters:
//   minutes:I  - # of minutes, seconds, and hundredths of seconds to delay.
//   seconds:I
//   hundredths:I
// Returns: (none)
// Inputs/Outputs:
//   m_breakProg:O
//-----------------------------------------------------------------------------
void pause(uint8 minutes, uint8 seconds, uint8 hundredths)
{
  uint32 delayTime;

  delayTime = minutes;                       /* Minutes      */
  delayTime = delayTime *  60 + seconds;     /* Seconds      */
  delayTime = delayTime * 100 + hundredths;  /* Centiseconds */
  delayTime = delayTime *  10;               /* Milliseconds */
  delayTime += millis();
  while (delayTime > millis())
  {
    /* Check for 'x' from the console */
    if ( (Serial.available() > 0) && (Serial.read() == 'x') )
    {
        m_breakProg = true;
        return;
    }
  }
}


//-----------------------------------------------------------------------------
// Function: wipeExecuteStatement
//   Execute the tokenized WIPE statement.
// Parameters:
//   lineStart:I  - Starting program memory location of the line.
// Returns: Program memory location of end of tokenized WIPE statement, if
//          statement execution was successful; MAX_PROG_SIZE, otherwise.
// Inputs/Outputs:
//   m_breakProg:I
//   m_program:I
//   m_symTbl:IO
//   m_wipeLineStart:IO
//   m_wipeRunByte:IO
//-----------------------------------------------------------------------------
uint16 wipeExecuteStatement(uint16 lineStart)
{
  SymValue_t  exprVal;
  char       *identifier;
  char       *exprStart;
  uint16      rc = MAX_PROG_SIZE;
  uint8       symIdx;
  uint8       typeId;
  char        inChar;
  boolean     execError;

  /* Check for 'x' from the console */
  if (m_breakProg)
    return MAX_PROG_SIZE;
  if (Serial.available() > 0)
  {
    inChar = Serial.read();
    if (inChar == 'x')
    {
      m_wipeRunByte = MAX_PROG_SIZE;
      return MAX_PROG_SIZE;
    }
  }

  execError = false;
  m_wipeRunByte = lineStart + TKNZD_STMT_OFFS + 1;
  m_wipeLineStart = lineStart;
  switch (m_program[lineStart + TKNZD_STMT_OFFS])
  {
    case WIPE_FUNC:
    {
      uint8 numParms;
      uint8 funcId;
      uint8 parmValue[WIPE_MAX_PARMS];

      funcId   = m_program[m_wipeRunByte++];
      numParms = m_program[m_wipeRunByte++];
      m_wipeRunByte++; /* Skip open parenthesis */
      for (uint8 i = 0; i < numParms; i++)
      {
        if ( !wipeParseExpression( (char *)&m_program[m_wipeRunByte],
                                   WIPE_ID_INT,
                                   true,
                                   &exprVal )
           )
        {
          wipePrintStatement();
          logPrintln(FLASH("***SYS ERROR"));
          return rc;
        }
        if ( (exprVal.intValue < -128) || (exprVal.intValue > 255) )
        {
          wipePrintStatement();
          logPrintln(FLASH("***ERROR: arg value out of range"));
        }
        parmValue[i] = (uint8)exprVal.intValue;
        m_wipeRunByte += m_exprOffs;
      }
      switch (funcId)
      {
        case WASPCMD_BKGRD:
          background(parmValue[0], parmValue[1], parmValue[2], parmValue[3]);
          break;
        case WASPCMD_GROUP:
          setGroup(parmValue[0], parmValue[1], parmValue[2], parmValue[3]);
          break;
        case WASPCMD_LINE:
          line(parmValue[0], parmValue[4], parmValue[5], parmValue[1],
               parmValue[2], parmValue[3]);
          break;
        case WASPCMD_RESET:
          waspReset(parmValue[0]);
          break;
        case WASPCMD_STATE:
          setState(parmValue[0], parmValue[1]);
          break;
        case WASPCMD_SHIFT:
          shift(parmValue[0], parmValue[1]);
          break;
        case WASPCMD_SWAP:
          swap(parmValue[0], parmValue[1], parmValue[2], parmValue[3],
               parmValue[4], parmValue[5], parmValue[6]);
          break;
        case WASPCMD_RAINBOW:
          rainbow(parmValue[0], parmValue[1]);
          break;
        case WASPCMD_RAINCYCLE:
          rainbowCycle(parmValue[0]);
          break;
        case WASPCMD_SPEED:
          fxSpeed(parmValue[0], parmValue[1]);
          break;
        case WASPCMD_TWINKLE:
          twinkle(parmValue[0], parmValue[1], parmValue[2], parmValue[3],
                  parmValue[4]);
          break;
        case WIPE_FN_PAUSE:
          pause(parmValue[0], parmValue[1], parmValue[2]);
          break;
        default:
          wipeShowError(FLASH("<unknown func>"));
          return rc;
      }
      break;
    }
    case WIPE_GOTO:
      identifier = (char *)&m_program[m_wipeRunByte];
      m_wipeRunByte += strlen(identifier) + 1;
      if (wipeLabelLocate(identifier, &m_wipeRunByte))
      {
        return m_wipeRunByte;
      }
      wipeShowError(FLASH("Can't jump to label"));
      return rc;
      break;
    case WIPE_IF:
      exprStart = (char *)&m_program[m_wipeRunByte];
      if ( wipeParseExpression(exprStart, WIPE_ID_BOOL, false, &exprVal) )
      {
        if (0 == exprVal.boolValue)
        {
          /* Skip over the next command */
          m_wipeRunByte = lineStart + m_program[lineStart + TKNZD_LEN_OFFS];
          if (m_wipeRunByte < m_wipeProgByte)
          {
            m_wipeRunByte += m_program[m_wipeRunByte + TKNZD_LEN_OFFS];
            return m_wipeRunByte;
          }
          else
          {
            /* We've hit the end of the program */
            m_wipeRunByte = MAX_PROG_SIZE;
            return m_wipeRunByte;
          }
        }
      }
      else
      {
        wipePrintStatement();
        return rc;
      }
      break;
    case WIPE_LABEL:
      identifier = (char *)&m_program[m_wipeRunByte];
      m_wipeRunByte += strlen(identifier) + 1;
      if (!wipeSymbolInsert(identifier, WIPE_ID_LABEL, &symIdx))
      {
        if (symIdx == MAX_NUM_SYMBOLS)
        {
          wipePrintStatement();
          logPrint(FLASH("***ERROR: No space for label "));
          logPrintln(identifier);
          return rc;
        }
      }
      else
      {
        m_symTbl[symIdx].symValue.intValue = lineStart;
      }
      break;
    case WIPE_LET:
      identifier = (char *)&m_program[m_wipeRunByte];
      m_wipeRunByte += strlen(identifier) + 1;
      exprStart = (char *)&m_program[m_wipeRunByte];
      m_wipeRunByte += 2;
      if (!wipeSymbolLookup(identifier, &symIdx))
      {
        wipePrintStatement();
        logPrint(FLASH("***ERROR: Variable '"));
        logPrint(identifier);
        logPrintln(FLASH("' undefined."));
        return rc;
      }
      else
      {
        if ( !wipeParseExpression( exprStart,
                                   m_symTbl[symIdx].typeId,
                                   false,
                                   &m_symTbl[symIdx].symValue )
           )
        {
          wipePrintStatement();
          logPrintln(FLASH("***SYS ERROR"));
          return rc;
        }
      }
      break;
    case WIPE_PRINT:
      if (m_program[lineStart + TKNZD_STMT_OFFS + 1] == WIPE_PRT_STR)
      {
        /* Print quoted string */
        logPrint((char *)&m_program[lineStart + TKNZD_STMT_OFFS + 2]);
      }
      else
      {
        /* Print result of expression evaluation */
        m_wipeRunByte++;
        identifier = (char *)&m_program[m_wipeRunByte];
        m_wipeRunByte += strlen(identifier) + 1;
        if (!wipeSymbolLookup(identifier, &symIdx))
        {
          wipePrintStatement();
          logPrint(FLASH("***ERROR: Variable '"));
          logPrint(identifier);
          logPrintln(FLASH("' undefined."));
          return rc;
        }
        else
        {
          exprVal = m_symTbl[symIdx].symValue;
          switch (m_symTbl[symIdx].typeId)
          {
            case WIPE_ID_INT:
              logPrint(exprVal.intValue);
              break;
            case WIPE_ID_FLOAT:
              logPrint(exprVal.floatValue);
              break;
            case WIPE_ID_BOOL:
              logPrint(exprVal.boolValue);
            default:
              wipePrintStatement();
              logPrint(FLASH("***Can't print value for "));
              logPrintln(identifier);
              return rc;
          }
        }
      }
      break;
    case WIPE_VAR:
      identifier = (char *)&m_program[m_wipeRunByte];
      m_wipeRunByte += strlen(identifier) + 1;
      typeId = m_program[m_wipeRunByte];
      if (!wipeSymbolInsert(identifier, typeId, &symIdx))
      {
        wipePrintStatement();
        logPrint(FLASH("***ERROR: "));
        if (symIdx != MAX_NUM_SYMBOLS)
        {
          logPrint(FLASH("Variable '"));
          logPrint(identifier);
          logPrintln(FLASH("' already defined."));
          return rc;
        }
        else
        {
          logPrint(FLASH("No space for variable "));
          logPrintln(identifier);
          return rc;
        }
      }
      break;
    default:
      logPrint(FLASH("***ERROR: Unknown statement: "));
      logPrintln(m_program[lineStart + TKNZD_STMT_OFFS]);
  }
  
  if (execError)
  {
    wipePrintStatement();
    logPrintln(FLASH("***ERROR: Statement execution"));
    return rc;
  }

  m_wipeRunByte = lineStart + m_program[lineStart + TKNZD_LEN_OFFS];
  return m_wipeRunByte;
}


//-----------------------------------------------------------------------------
// Function: wipeSeekLineStart
//   Find the starting address, in program memory, of the program line
//   with the smallest line number that is not less than specified target
//   line number.
// Parameters:
//   targetNum:I   - The line number of the line to be found.
//   lineExists:O  - Address at which to return indication that the
//                   specified line number exists. The output
//                   value is true iff targetNum already exists.
// Returns: MAX_PROG_SIZE, if line cannot be found;
//          The m_program[] index at which the found line starts, otherwise.
// Inputs/Outputs:
//   m_program:I
//   m_wipeProgByte:I
//-----------------------------------------------------------------------------
uint16 wipeSeekLineStart(uint8 targetNum, boolean *lineExists)
{
  uint16 currPos;
  uint8  currLineNum;

  *lineExists = false;
  currPos = 0;
  while ( (currPos < MAX_PROG_SIZE) && (currPos < m_wipeProgByte) )
  {
    currLineNum = m_program[currPos + TKNZD_LINE_OFFS];
    if (currLineNum == targetNum)
    {
      *lineExists = true;
      return currPos;
    }
    else
    {
      if (m_program[currPos + TKNZD_LINE_OFFS] < targetNum)
      {
        currPos += m_program[currPos + TKNZD_LEN_OFFS];
      }
      else
      {
        return currPos;
      }
    }
  }

  return MAX_PROG_SIZE;
}


//-----------------------------------------------------------------------------
// Function: wipeDelLines
//   Delete a program line, with a specified starting address and length,
//   from program memory. Byte following the line-to-be-deleted as shifted
//   downward to fill the gap.
// Parameters:
//   lineStart:I  - Starting address of the line to be deleted.
//   lineLen:I    - Length of the line to be deleted.
// Returns: (none)
// Inputs/Outputs:
//   m_program:I
//   m_wipeLineStart:IO
//   m_wipeProgByte:IO
//-----------------------------------------------------------------------------
void wipeDeleteLine(uint16 lineStart, uint8 lineLen)
{
  uint8  *dst;
  uint8  *src;
  uint16 numBytesToMove;
  
  numBytesToMove = m_wipeProgByte - lineLen + 1;
  dst = &m_program[lineStart];
  src = dst + m_program[lineStart + TKNZD_LEN_OFFS];
  for (uint16 i = numBytesToMove; i > 0; i--)
    *dst++ = *src++;
   m_wipeLineStart -= lineLen;
   m_wipeProgByte  -= lineLen;
}


//-----------------------------------------------------------------------------
// Function: wipeDelLines
//   Delete program lines.
// Parameters:
//   lower:I  - Lower line number limit.
//   upper:I  - Upper line number limit.
// Returns: (none)
// Inputs/Outputs:
//   m_program:I
//   m_wipeProgByte:I
//-----------------------------------------------------------------------------
void wipeDelLines(uint8 lower, uint8 upper)
{
  uint16   delLineStart;
  boolean  lineExists;
  uint8    lineNum;
  uint8    lineLen;

  /* Find first line whose line number is >= lower */
  delLineStart = wipeSeekLineStart(lower, &lineExists);

  lineNum = m_program[delLineStart + TKNZD_LINE_OFFS];
  while ( (delLineStart < m_wipeProgByte) && (lineNum <= upper) )
  {
    lineLen = m_program[delLineStart + TKNZD_LEN_OFFS];
    wipeDeleteLine(delLineStart, lineLen);
    lineNum = m_program[delLineStart + TKNZD_LINE_OFFS];
  }
}


//-----------------------------------------------------------------------------
// Function: wipeRenumLines
//   Reunumber program lines evenly from 5 - 250.
// Parameters:
//   lower:I  - Lower line number limit.
//   upper:I  - Upper line number limit.
// Returns: (none)
// Inputs/Outputs:
//   m_wipeProgByte:I
//   m_program:IO
//-----------------------------------------------------------------------------
void wipeRenumLines(void)
{
  #define  LOWEST_LINE_NUM   5  /* The lowest line # to allow for edit room */
  uint8   *currLinePos;
  uint8    count;
  uint8    newLineNum;

  /* Determine how many program lines there are */
  count = 0;
  for ( currLinePos = &m_program[0];
        currLinePos < &m_program[m_wipeProgByte];
        currLinePos += *(currLinePos + TKNZD_LEN_OFFS)
      )
    count++;

  /* Determine line number increment for line range #LOWEST_LINE_NUM thru 
   * approx #(MAX_LINE_NUM - LOWEST_LINE_NUM). So the increment is calculated
   * as:   (MAX_LINE_NUM - 2*LOWEST_LINE_NUM) / (count - 1)
   */
  count = (MAX_LINE_NUM - (LOWEST_LINE_NUM << 1)) / (count - 1);
  
  currLinePos = 0;
  newLineNum = LOWEST_LINE_NUM;
  for ( currLinePos = &m_program[0];
        currLinePos < &m_program[m_wipeProgByte];
        currLinePos += *(currLinePos + TKNZD_LEN_OFFS),
          newLineNum += count
      )
  {
    *(currLinePos + TKNZD_LINE_OFFS) = newLineNum;
  }
}


//-----------------------------------------------------------------------------
// Function: wipeSaveLine
//   Save the numbered, parsed line that is being edited into program store
//   in it correct location.
// Parameters: (none)
// Returns: true, iff there was enough memory to store the line.
// Inputs/Outputs:
//   m_program:I
//   m_wipeLineStart:IO
//   m_wipeProgByte:IO
//-----------------------------------------------------------------------------
boolean wipeSaveLine(void)
{
  uint8  *dst;
  uint8  *src;
  uint16  insertPos;
  uint16  moveStartPos;
  uint16  numBytesToMove;
  uint8   insertLineNum;
  uint8   insertLineLen;
  uint8   deleteLineLen;
  uint8   transientByte;
  boolean isReplacement; /* true = The edit line replaces an existing line */

  insertLineNum = m_program[m_wipeLineStart + TKNZD_LINE_OFFS];
  insertLineLen = m_program[m_wipeLineStart + TKNZD_LEN_OFFS];
  insertPos = wipeSeekLineStart(insertLineNum, &isReplacement);

  if (insertPos == m_wipeLineStart)
  {
    /* Current edit line is already in the right position */
    m_wipeLineStart += insertLineLen;
    return true;
  }
  
  else if (insertPos < MAX_PROG_SIZE)
  {
    /*---  Insertion is ok.  ---*/

    if (isReplacement)
    {
      /* The edit line replaces an existing line. So delete the latter. */
      wipeDeleteLine(insertPos, m_program[insertPos + TKNZD_LEN_OFFS]);
    }
    
    numBytesToMove = m_wipeLineStart - insertPos;
    for (uint16 i = 0; i < insertLineLen; i++)
    {
      /*---  Insert the next byte of the edit line.  ---*/
      
      dst = &m_program[m_wipeLineStart + i];
      src = dst - 1;
      transientByte = *dst; /* Note byte being moved */

      /* Shift program memory bytes upward by one byte */
      for (uint16 j = numBytesToMove; j > 0; j--)
      {
        *dst-- = *src--;
      }

      m_program[insertPos + i] = transientByte; /* Store byte being moved */
    }

    m_wipeLineStart += insertLineLen;
    return true;
  }

  else
  {
    return false;
  }
}


//-----------------------------------------------------------------------------
// Function: wipeParseLineRange
//   Parse for and return a line range from the WIPE command line.
// Parameters:
//   lower:O  - Address at which to return the lower line number limit.
//   upper:O  - Address at which to return the upper line number limit.
// Returns: true, iff at least one line number was specified.
// Inputs/Outputs:
//   m_consolePos:IO
//   m_tokenBuffer:IO
//-----------------------------------------------------------------------------
boolean wipeParseLineRange(uint8 *lower, uint8 *upper)
{
  int32 tmpLower;
  int32 tmpUpper;
  boolean hasLineNum;

  hasLineNum = false;

  /* Check for lower limit or only line number */
  if (wipeParseInt(&tmpLower, true))
  {
    hasLineNum = true;
  }
  else
  {
    tmpLower = 0;
  }
  
  tmpUpper = strspn(&m_tokenBuffer[m_consolePos], " \r\0");
  m_consolePos += tmpUpper;
  tmpUpper = 0;
  if (m_tokenBuffer[m_consolePos] == '-')
  {
    m_consolePos += 2;
    if (wipeParseInt(&tmpUpper, true))
    {
      hasLineNum = true;
    }
    else
    {
      tmpUpper = MAX_LINE_NUM;
    }
  }
  if (0 == tmpLower)
  {
    tmpLower = 1;
    if (0 == tmpUpper)
    {
      tmpUpper = MAX_LINE_NUM;
    }
  }
  else
  {
    if (0 == tmpUpper)
    {
      tmpUpper = tmpLower;
    }
  }

  if ( (tmpLower < 1) || (tmpUpper < 1) ||
       (tmpUpper > MAX_LINE_NUM) || (tmpUpper > MAX_LINE_NUM) ||
       (tmpUpper < tmpLower)
     )
  {
    logPrintln(FLASH("Bad line range"));
    *lower = 0;
    *upper = 0;
    return false;
  }
  *lower = (uint8)tmpLower;
  *upper = (uint8)tmpUpper;
  return hasLineNum;
}


//-----------------------------------------------------------------------------
// Function: wipeRunProgram
//   Run the program that's currently loaded in RAM.
// Parameters: (none)
// Returns: (none)
// Inputs/Outputs:
//   m_wipeProgByte:I
//   m_wipeRunByte:IO
//-----------------------------------------------------------------------------
void wipeRunProgram(void)
{
  uint32 currentTime;
  
  m_wipeRunByte = 0;
  wipeSymbolsClear();
  logPrintln();
  digitalWrite(WIPE_PRGM_LED, LOW);
  while (m_wipeRunByte < m_wipeProgByte)
  {
    digitalWrite(WIPE_RUN_LED, m_wipeLedState);
    if (LOW == m_wipeLedState)
      m_wipeLedState = HIGH;
    else
      m_wipeLedState = LOW;
    currentTime = millis();

    /* Check command-specific timeout (for long-running command) */
    if (m_cmdExecDelay)
    {
      while (m_cmdExecDelay > currentTime)
      {
        delay(m_cmdExecDelay - currentTime);
        currentTime = millis();      
      }
      m_cmdExecDelay = 0;
    }
    
    //wipePrintStatement();
    m_wipeRunByte = wipeExecuteStatement(m_wipeRunByte);
  }
  digitalWrite(WIPE_PRGM_LED, HIGH);
  logPrintln();
}


//-----------------------------------------------------------------------------
// Function: processWipeCommand
//   Process WASP Interpretive Programming Environment (WIPE) console commands.
// Parameters: (none)
// Returns: true iff command processing has completed.
// Inputs/Outputs:
//   m_consoleBuffer:IO
//   m_consolePos:IO
//   m_wipeProgByte:IO
//   m_wipeRunByte:O
//-----------------------------------------------------------------------------
void processWipeCommand(void)
{
#define WIPE_PARSE_START  0
  static int32    lineNum = 0;
  static char    *lineChar = NULL;
  static uint8    wipeStage = PRG_PROMPT;
  static uint8    wipeSubstage = WIPE_PARSE_START;
  static uint8    token;
  static boolean  saveToRam = false;  // Save program line to RAM buffer
  char    *progName;
  uint16   fileSize;
  uint8    lower, upper;
  uint8    tmpUint8;
  char     inChar;
  boolean  errorReported;

  m_breakProg = false;
  if (PRG_PROMPT == wipeStage)
  {
    if (m_triggerAutoRun)
    {
      /* Trigger auto-run: only triggered once after reset. */
      m_triggerAutoRun = false;
      if (!wipeDirFileLoad(m_autorunProg))
      {
        logPrintln(FLASH("***Autorun file missing"));
        return;
      }
      wipeRunProgram();
      waspReset(BROADCASTID);
      logPrintln(FLASH("Autorun done"));
    }
    logPrint(FLASH(" Ready ["));
    logPrint(m_wipeProgByte);
    logPrint(FLASH(" bytes; "));
    logPrint(MAX_PROG_SIZE - m_wipeProgByte);
    logPrintln(FLASH(" free]"));
    logPrint(FLASH("* "));
    wipeStage = PRG_INPUT;
    m_consolePos = 0;
  }

  while (PRG_INPUT == wipeStage)
  {
    if (0 == Serial.available())
      continue; // Nothing to process
 
    /* Process an input character */
    inChar = Serial.read();
    if (('\b' == inChar) || (DEL_CHAR == inChar))
    {
      /* Backup the buffer position and truncate the command line string. */
      if (m_consolePos > 0)
        m_consolePos--;
      m_consoleBuffer[m_consolePos] = '\0';
      continue;
    }
    else if ('\r' == inChar)
    {
      /* We have a completed line of input. */
      m_consoleBuffer[m_consolePos] = '\0'; // Terminate command line string
    }
    else if ( (MAX_SERIAL_BUF_LEN - 1) == m_consolePos )
    {
      /* We're going to overrun the buffer: discard the contents */
      m_consolePos = 0;
      wipeStage = PRG_PROMPT;

      /* Flush the console input */
      delay(5);
      while (Serial.available() > 0)
      {
         Serial.read();
         delay(5);
      }
         
      logPrint(FLASH("*** Line length ("));
      logPrint(MAX_SERIAL_BUF_LEN);
      logPrintln(FLASH(") exceeded. Ignored\n"));
      return;
    }
    else if ( (m_wipeProgByte + m_consolePos) > MAX_PROG_SIZE )
    {
      /* We've hit the end of program memory */
      m_consolePos = 0;
      wipeStage = PRG_PROMPT;

      /* Flush the conole input */
      delay(5);
      while (Serial.available() > 0)
      {
         Serial.read();
         delay(5);
      }

      logPrintln(FLASH("***Out of RAM"));
      return;
    }
    else
    {
      /* Append the new char to the buffer */
      m_consoleBuffer[m_consolePos++] = inChar;
      continue;
    }
    
    /*----- A completed line of input is now available. -----*/
    /* Set up to parse the command upon next invocation. */
    m_consoleLen = m_consolePos;
    wipeStage = PRG_PARSE;
    wipeSubstage = WIPE_PARSE_START;
    token = WIPE_UNDEF;
    lineNum = 0;
    m_wipeLineStart = m_wipeProgByte;

    strncpy(m_tokenBuffer, m_consoleBuffer, sizeof(m_consoleBuffer));
    
    /* Skip leading spaces */
    m_consolePos = 0;
    for (lineChar = &m_consoleBuffer[0]; *lineChar == ' '; lineChar++)
      m_consolePos++;
    if ('\0' == *lineChar)
    {
      wipeStage = PRG_PROMPT;
      return;
    }
  }

  while (PRG_PARSE == wipeStage)
  {
    saveToRam = false;
    token = wipeScanCommand();
    wipeStage = PRG_PROMPT;
    switch (token)
    {
      case WIPE_CMD_DEL:
        if (wipeParseLineRange(&lower, &upper))
        {
          wipeDelLines(lower,upper);
        }
        else
        {
          wipeShowError(FLASH("Bad line range"));
        }
        break;
      case WIPE_CMD_DIR:
        wipeDirList();
        break;
      case WIPE_CMD_ERASE:
        tmpUint8 = wipeScanIdentifier(&progName, MAX_PROGNAME_LEN);
        if (tmpUint8 == 0)
        {
          wipeShowError(FLASH("No filename"));
          break;
        }
        wipeDirFileErase(progName);
        break;
      case WIPE_CMD_EXIT:
        m_consoleMode = CMDMODE_CONSOLE;
        m_consolePos = 0;
        logPrintln(FLASH("... exiting WIPE"));
        logPrintln(FLASH("Enter 'h' for help"));
        break;
      case WIPE_CMD_LIST:
        (void)wipeParseLineRange(&lower, &upper);
        wipeListProgram(lower, upper);
        break;
      case WIPE_CMD_LOAD:
        tmpUint8 = wipeScanIdentifier(&progName, MAX_PROGNAME_LEN);
        if (tmpUint8 == 0)
        {
          wipeShowError(FLASH("No filename"));
          break;
        }
        (void)wipeDirFileLoad(progName);
        break;
      case WIPE_CMD_HELP:
        wipeShowHelp();
        break;
      case WIPE_CMD_PROF:
        wipeSymbolsShow();
        break;
      case WIPE_CMD_RENUM:
        wipeRenumLines();
        break;
      case WIPE_CMD_RUN:
        wipeRunProgram();
        waspReset(BROADCASTID);
        break;
      case WIPE_CMD_SAVE:
        if ( (m_wipeProgByte + DIR_PROG_OFFS) > m_wipeDirSpace )
        {
          wipeShowError(FLASH("Out of file space"));
          break;
        }
        tmpUint8 = wipeScanIdentifier(&progName, MAX_PROGNAME_LEN);
        if (tmpUint8 == 0)
        {
          wipeShowError(FLASH("No filename"));
          break;
        }
        if (wipeDirFileFind(progName, &fileSize) != 0)
        {
          wipeShowError(FLASH("File exists"));
          break;
        }
        wipeDirFileSave(progName);
        break;
      case WIPE_CMD_START:
        tmpUint8 = wipeScanIdentifier(&progName, MAX_PROGNAME_LEN);
        if (tmpUint8 == 0)
        {
          wipeShowError(FLASH("No filename"));
          break;
        }
        memcpy(m_autorunProg, progName, MAX_PROGNAME_LEN);
        m_autorunProg[MAX_PROGNAME_LEN] = '\0';
        logPrintln(FLASH("Startup set"));
        break;
        
      case WIPE_CMD_LINENUM:
        (void)wipeParseInt(&lineNum, true);
        if ( (lineNum >= 1) && (lineNum <= MAX_LINE_NUM) )
        {
          saveToRam = true;
          token = WIPE_UNDEF;
        }
        else
        {
          wipeShowError(FLASH("Bad line#"));
          break;
        }
        /* Fall thru to next case */
      default:  /* WIPE_UNDEF */
        errorReported = false;
        if (wipeParseStatement(lineNum, token, &errorReported))
        {
          /* If the saveToRam is flagged, just continue on.
           * Otherwise, execute the line directly and reset the
           * program program memory pointer.
           */
          if (saveToRam)
          {
            if (!wipeSaveLine())
            {
              wipeShowError(FLASH("Out of RAM"));
            }
          }
          else
          {
            /* Execute the tokenized line directly. */
            //wipePrintStatement();

            m_wipeProgByte = m_wipeLineStart;
            m_wipeRunByte = m_wipeLineStart;
            (void)wipeExecuteStatement(m_wipeLineStart);
            logPrintln();
          }
        }
        else
        {
          if (!errorReported)
            wipeShowError(FLASH("Bad WIPE cmd"));
        }
        m_wipeProgByte = m_wipeLineStart;
    } /* End: switch (token) */
  } /* End: while (PRG_PARSE == wipeStage) */
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
  m_resetRequired = false;
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
    EEPROM.write(EEPROM_VALIDITY_ADDR, 0); // dataIsValid = false;

    // Initialize WIPE program directory to empty.
    EEPROM.write(EEPROM_PROG_DIR_START, 0);
    EEPROM.write(EEPROM_PROG_DIR_START+1, 0);
    m_resetCount = 0;
  }
  else
  {
    // Read in parameters stored in EEPROM
    EepromLoad();
  }
  EEPROM.write(EEPROM_RESET_COUNT_ADDR, m_resetCount);

#ifdef CONSOLE_ENABLED
  Serial.println();
  Serial.print(FLASH("Wireless Addressable Strings of Pixels (WASP) network"));
  Serial.print(FLASH("...   Node #"));
  Serial.print(m_myNodeId);
  Serial.println(FLASH("  (Controller)"));
  Serial.print(FLASH("WASP controller S/W: "));
  Serial.print(SW_VERSION_c);
  Serial.print(FLASH("\tF/W: "));
  Serial.print(FW_VERSION_c);
  Serial.print(FLASH("\t    "));
  Serial.println(COPYRIGHT);
  Serial.print(FLASH("Radio frequency: "));
  Serial.print(FREQUENCY==RF69_433MHZ ? 433 :
                 FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.print(FLASH("Mhz"));
  Serial.print(FLASH("\t\t\t#Resets: "));
  Serial.print(m_resetCount);
  Serial.print(FLASH("\t"));
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

  #ifdef SERIAL_CMDS_ENABLED
    logPrintln();
    logPrintln(FLASH("Enter 'h' for help"));
  #endif

  m_autoRunStart = millis() + 1000 * m_autoRunDelay;
  if (m_autoRun)
  {
    m_triggerAutoRun = true;
    logPrintln(FLASH("Autorun pending..."));
  }

  pinMode(POWER_LED, OUTPUT);
  pinMode(WIPE_PRGM_LED, OUTPUT);
  pinMode(WIPE_RUN_LED, OUTPUT);
  digitalWrite(POWER_LED, LOW);
  digitalWrite(WIPE_PRGM_LED, LOW);
  digitalWrite(WIPE_RUN_LED, LOW);
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
  m_loopStartMs = millis();

  if (CMDMODE_CONSOLE == m_consoleMode)
  {
    if (m_loopStartMs > m_powerLedOnMs)
    {
      if (!m_triggerAutoRun)
      {
        digitalWrite(POWER_LED, HIGH);
        digitalWrite(WIPE_PRGM_LED, LOW);
        digitalWrite(WIPE_RUN_LED, LOW);
        m_powerLedOffMs = m_loopStartMs + 250;
        m_powerLedOnMs = m_powerLedOffMs + 5000;
      }
      else
      {
        digitalWrite(POWER_LED, LOW);
        digitalWrite(WIPE_PRGM_LED, LOW);
        digitalWrite(WIPE_RUN_LED, HIGH);
        m_powerLedOffMs = m_loopStartMs + 200;
        m_powerLedOnMs = m_powerLedOffMs + 200;
      }
    }
    else
    {
      if (m_loopStartMs > m_powerLedOffMs)
      {
        if (!m_triggerAutoRun)
        {
          digitalWrite(POWER_LED, LOW);
        }
        else
        {
          digitalWrite(WIPE_RUN_LED, LOW);
          digitalWrite(WIPE_PRGM_LED, HIGH);
        }
        m_powerLedOffMs = m_powerLedOnMs;
      }
    }
  }
  else
  {
    digitalWrite(POWER_LED, LOW);
    digitalWrite(WIPE_PRGM_LED, HIGH);
  }

  // Process console commands if console is enabled.
  #ifdef SERIAL_CMDS_ENABLED
    if (CMDMODE_CONSOLE == m_consoleMode)
    {
      processCommandLine();
      if (m_triggerAutoRun)
      {
        if (millis() >= m_autoRunStart)
        {
          m_consoleMode = CMDMODE_WIPE;
        }
      }
    }
    else
    {
      processWipeCommand();
    }
  #endif
  
  // Listen to RFM radio
  receiveRadioWaspRsp();

  // Generate RFM radio response when necessary
  //processRadioOutput();
}

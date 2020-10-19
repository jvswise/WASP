/* WASP_defs.h */
#ifndef Wasp_defs_h
#define Wasp_defs_h

// Ruler
//345678901234567890123456789012345678901234567890123456789012345678901234567890


/*************************************************************************
 * Definitions for Wireless Addressable Strings of Pixels network (WASP)
 *************************************************************************/

#define NODEID_UNDEF     0 // Undefined RFM node ID.
#define CONTROLLERID     1 // Designated RFM node ID of controller
#define BROADCASTID    255 // Designated RFM node ID for broadcasts
#define PROG_GW_ID     254 // Wireless programmer gateway node ID
#define NETWORKID       77 // The same for all nodes on the network

#define FIRST_SLAVE      2 // Node ID of first slave node
#define MAX_SLAVES       5 // Max # of WASP slave nodes.
#define FIRST_GROUP    128 // Node ID of first group of slave nodes.
#define MAX_GROUPS       5 // Max # of WASP slave groups.
#define NODEID_MAX     (MAX_SLAVES + CONTROLLERID)

// Match frequency to the Moteino's radio hardware
#define FREQUENCY      RF69_433MHZ

//#define IS_RFM69HW     // uncomment only for RFM69HW transceivers.
// NOTE: Encryption isn't currently being used.
#define ENCRYPTKEY     "JVS_WASP_Key3456" // exactly 16 chars: same on all nodes
#define ACK_WAIT_TIME    10  // max # of ms to wait for an ack
#define TX_NUM_RETRIES    2  // # of TX transmission attempts when ACK needed

// Time windows for slave Tx (milliseconds)
#define SLAVE_TX_WIND    15  // Time windows for slave-to-slave Tx (milliseconds)
#define CMD_TIMEOUT     (MAX_SLAVES * SLAVE_TX_WIND)
#define SLAVE_PING_TX   100  // Slave Tx window for WASPCMD_PING (milliseconds)
#define PING_TIMEOUT    (MAX_SLAVES * SLAVE_PING_TX)
#define MIN_UPD_PERIOD   20  // Min # milliseconds between WASP commands.
#define MAX_SHIFT_SIZE   20  // Max # pixels to shift for WASPCMD_SHIFT command.


// WASP Command Codes
//=======================
// Note: Unless stated otherwise, the dst parameter for each command
//       can be the BROADCAST address, a Group address, or a specific
//       node address. The dst parameter is implcitly transmitted in
//       the radio packet header and is not encoded in the message
//       payload. The radio packet payload is in the following format:
//           +-------+----------+-----+----------+
//           | <cmd> | <arg #1> | ... | <arg #n> |
//           +-------+----------+-----+----------+
//       where <cmd> is one of the WASPCMD_... values below.

/*     =========================  Base Commands  ========================     */
#define WASPCMD_NONE      0  // No WASP command received.

#define WASPCMD_GROUP     1  // GROUP(dst:8, groupId:8, l:8, r:8)
                             //       ^^^               ^    ^
                             //   Registers specific WASP slave #dst to
                             //   group #groupId with specific neigbhours
                             //   node #l to its left and node #r to its right.
                             //   Node dst is its own neighbour if l = r = dst;
                             //   otherwise l != dst and r != dst. Neighbours
                             //   must form a connected loop of size >= 1. The
                             //   groupId value must be in the range
                             //   FIRST_GROUP .. (FIRST_GROUP + MAX_GROUPS - 1).
                             //   The groupId value can be used in subsequent
                             //   commands as a dst value.

#define WASPCMD_STATE     2  // STATE(dst:8, opts:8) - State operations. The
                             // opts arg is a bit field with the following
                             // definitions:
                             //     Bxxxxxx01, Save pixel colours
                             //     Bxxxxxx10, Restore pixel colours
                             //     Bxxxx10xx, Suspend drawing after subsequent
                             //                  commands.
                             //     Bxxxx01xx, Resume drawing on subsequent
                             //                  commands.
                             //     Note: When restore flag is set, the restore
                             //           happens before the suspend/resume
                             //           action.
                             //   Invalid combinations:
                             //     Bxxxx11xx
                             //     Bxxxxxx11
                             
#define WASPCMD_BKGRD     3  // BKGRD(dst:8, r:8, g:8, b:8) - Set (and save)
                             //   the background colour of all pixels.
                             
#define WASPCMD_LINE      4  // LINE(dst:8, r:8, g:8, b:8, s:8, l:8)
                             //  Draw a line starting at pixel #s of length
                             //  l pixels.
                             
#define WASPCMD_SHIFT     5  // SHIFT(dst:8, n:8) from controller, where n is a
                             //   signed int.
                             //   Shift the pixels by |n| positions.
                             //   If n > 0, shift right; else shift left. Each
                             //   node, in turn, reports the colours of
                             //   the pixels that it shifted out (in the order
                             //   they were shifted out) to its neighbouring
                             //   node so that it can shift them in.
                             //
                             // SHIFT(dst:8, n:8, [r:8, g:8, b:8]n)
                             //       ^^^
                             //   where n is a positive integer specifying the
                             //   number of (r,g,b) triples that follow, and
                             //   dst must be a specific node Id. This is
                             //   each node's response (when needed) to the
                             //   controller's SHIFT command. The colours are
                             //   from leftmost (first) to rightmost (last) of
                             //   the pixels that were shifted out.
                             //   Notes:
                             //     1) n must be <= ((MAX_DATA_LEN-1) / 3) = 20
                             //     2) MAX_DATA_LEN (= 61) is defined in the
                             //        RFM69 library.
                             //     3) Each node, n, broadcasts its response
                             //        the following # milliseconds after
                             //        the command is sent:
                             //           (n - 1) * INTERNODE_DLY
                             //     4) Each WASP node must defer updating its
                             //        pixels until it has both shifted in its
                             //        neighbor's pixels and reported the pixels
                             //        that it has shifted out.
                             
#define WASPCMD_SWAP      6  // SWAP( dst:8, r_old:8, g_old:8, b_old:8,
                             //       r:8, g:8, b:8 )
                             //   Swap all pixels having the old colour with
                             //   the new colour (r,g,b).

#define WASPCMD_RESET     7  // RESET(dst:8, "DEAD") - Request the dst node(s)
                             //   to reset themselves.

#define WASPCMD_SPEED     8  // SPEED(dst:8, delay:8) - Adjust the speed
                             //   of an animated effect that is currently
                             //   running without controller intervention
                             //   (e.g. RAINBOW()). Generally, smaller delay
                             //   values increase the effect's speed. The
                             //   following dleay values are distinguished:
                             //     0 = Pause the current special F/X.
                             //     1 = Single-step the current special F/X;
                             //           F/X progress is, thus, under control
                             //           of the WASP controller.

/*     ========================  Special Effects  =======================     */
#define WASPCMD_RAINBOW   9  // RAINBOW(dst:8, offs:8) - Run a rainbow effect
                             //   with a starting colour offset (offs) for
                             //   LED #0 on the (each) destination. The offset
                             //   is equivalent to a number of LEDs. Each
                             //   subsequent pixel picks up the next colour
                             //   in a 256 colour wheel (the wheel cycling
                             //   thru red to green to blue, back to red.

#define WASPCMD_RAINCYCLE 10 // RAINCYCLE(dst:8) - Run a rainbow effect with
                             //   the rainbow colours always spanning the
                             //   number of pixels at the (each) dst node.
                             //   The effect is similar to RAINBOW() but tends
                             //   to have the colours more compressed.

#define WASPCMD_TWINKLE   11 // TWINKLE(dst:8, minDly:8, maxDly:8,
                             //          burst:8, hold:8)
                             //   Run a twinkling effect on the currently
                             //   defined background colour. The dst value
                             //   can be any single node, group, or the
                             //   BROADCASTID. The minDly and maxDly values
                             //   define the relative time range between
                             //   random twinkles. The burst value defines the
                             //   upper bound on the number of pixels that can
                             //   twinkle simultaneously on each individual
                             //   node. The hold value determine how long
                             //   each twinkle lasts.
                             //   NOTE: Send a WASPCMD_BKGRD first to set the
                             //         background colour.

/*     ====================  Configuration Commands  ====================     */
#define WASPCMD_PING      12 // PING(dst:8)
                             //   Query the presence of a specific node or all
                             //   nodes on the network. Each node will reply
                             //   in turn with its response delay dictated by
                             //   its node ID. Each node replies with the
                             //   sequence of unsigned 8-bit values:
                             //     - Firmware version number
                             //     - Major software version number
                             //     - Minor software version number
                             //     - Digital output pin # used for LED control
                             //     - Number of pixels in the pixel string
                             //     - Pixel string frequency
                             //     - RGB wiring order for pixels
                             //     (For the latter three items, refer to
                             //     WASPCMD_CFG_LED.)
                             //   Node #2 is the first to transmit. Each slave
                             //   node, n, transmits at ((n - 2) * SLAVE_TX_WIND)
                             //   milliseconds following receipt of the command.

#define WASPCMD_CFG_NODE  13 // CFG_NODE(dst:8, magic[4], newNodeId:8)
                             //   Modify a known node's (dst) node ID to new
                             //   value, newNodeId. The destination node will
                             //   have to be power cycled for the change to
                             //   take affect. The value of array magic[] must
                             //   be confirmed to be "WASP"; this is intended
                             //   to minimize accidental corruption of a node's
                             //   configuration.
                             //   RECOMMENDATION: Ping the node afterward to
                             //                   confirm the change.

#define WASPCMD_CFG_CTRL  14 // CFG_CTRL(dst:8, magic[4], pinNumber:8)
                             //   Modify the digital output pin used for
                             //   controlling the LED pixel string on a specific
                             //   node. The value of array magic[] must
                             //   be confirmed to be "WASP"; this is intended
                             //   to minimize accidental corruption of a node's
                             //   configuration.
                             //   NOTE: Following this command, the CFG_SAVE
                             //         command should be sent by the controller
                             //         and then the slave should be reset.

#define WASPCMD_CFG_LED   15 // CFG_LED(dst:8, magic[4], len:8, freq:8, order:8)
                             //   Modify pixel string parameters on a specific
                             //   node:
                             //     len:   n in [1 - 255], the number of pixels
                             //              in the string. For a regular Moteino
                             //              slave, n should < 100 due to RAM
                             //              size constraints. (Default value
                             //              is 3.)
                             //     freq:  8 = 800 kHz update frequency (default)
                             //            4 = 400 kHz
                             //     order: The RGB colour wirting order.
                             //            0 = RGB (default)
                             //            1 = RBG
                             //            2 = GRB
                             //            3 = GBR
                             //            4 = BRG
                             //            5 = BGR
                             //   The value of array magic[] must be confirmed
                             //   to be "WASP"; this is intended to minimize
                             //   accidental corruption of a node's
                             //   configuration.
                             //   NOTE: Following this command, the CFG_SAVE
                             //         command should be sent by the controller
                             //         and then the slave should be reset.

#define WASPCMD_CFG_SAVE  16 // CFG_SAVE(dst:8)
                             //   Save the configuration for a specific slave
                             //   node, or all slaves, to its (their) EEPROM(s)
                             //   so that the changes are permanent. (This
                             //   command saves the LED control pin and pixel
                             //   string parameters.)

                             
#define MAX_WASPCMD_VAL   (WASPCMD_CFG_SAVE)  // Max valid WASP command value.
#define LAST_NONCFG_CMD   (WASPCMD_TWINKLE)


// ACK codes
#define ACK_OK        0    // No error.
#define ACK_ECMD      1    // Command unsupported.
#define ACK_ETIME     2    // Timed out waiting for ACK.
#define ACK_ERR       254  // Unspecified error.
#define ACK_NULL      255  // 'undefined' ACK value


// WASP node state operation flags:
#define F_SAVE         B00000001
#define F_RESTORE      B00000010
#define F_SUSPEND      B00001000
#define F_RESUME       B00000100


#ifndef int8
  typedef signed char   int8;
#endif
#ifndef int16
  typedef int           int16;
#endif
#ifndef int32
  typedef long          int32;
#endif
#ifndef uint8
  typedef unsigned char uint8;
#endif
#ifndef uint16
  typedef unsigned int  uint16;
#endif
#ifndef uint32
  typedef unsigned long uint32;
#endif
#ifndef MAXUINT32
  #define MAXUINT32     0xFFFFFFFF
#endif
#ifndef MAXINT32
  #define MAXINT32      ((int32)0x7FFFFFFF)
#endif
#ifndef MININT32
  #define MININT32      ((int32)0x80000000)
#endif

typedef uint8 AckCode_t;  /* ACK codes (e.g. ACK_OK)           */
typedef uint8 WaspCmd_t;  /* WASP commands (e.g. WASPCMD_NONE) */

#endif  /* Wasp_defs_h */

Wireless Addressable Strings of Pixels (WASP)

A Moteino based controller and slave nodes for running animated seasonal lighting effects.
The controller has a built-in programming language, the WASP Interactive Programming Environment (WIPE), for designing effect sequences. Programs are stored in the Moteino-MEGA controller's 4 KiB of EEPROM. WIPE is a very rudimentary language that is somewhat similar to BASIC (though closer to an assembly language). WIPE also supports direct execution of WIPE program statements at the console. Once of the programs stored in EEPROM can be selected for automatic execution after the controller is powered up.

Slave nodes can be updated wirelessly from a simple Moteino-MEGA based controller connected to a PC USB port.

Folders:
  Arduino_sketches:
    - Arduino sketches for the WASP controller, WASP remote nodes, and remote node programming unit.
      Wirelessly uploadable remote node .HEX files are also available.

  Documentation:
    - User Manual for the controller and for WASP network setup and wireless reprogramming of the remote
      nodes.
    - Contruction help for the controller, remote nodes, and wireless programming units.

  Sample_WIPE_Programs:
    - Sample WIPE test and seasonal effects programs

  Wireless_Programming_Support_Files:
    - Wireless Programming support files by Felix Rusu, originally obtained from LowPowerLab.


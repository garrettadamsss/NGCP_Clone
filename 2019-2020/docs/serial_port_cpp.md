# serial_port.cpp
--------------------------
Establishes a serial connection to send and receive data


## Imports:
serial_port.h

## Variables to track:

*fd*: __int__. Used for file descriptor that handles valid serial port opening.  
*baudrate*: __int__.  
*uart_name*: __char*__. Used for device path.  
*lastStatus*: **mavlink_status_t**.  

# Functions:

## Serial_Port() (*No argument constructor*)

Calls method *initialize_defaults()*.  
  

## Serial_Port(const char *uart_name_, int baudrate_)

Calls method *initialize_defaults()*.  
Sets *uart_name* to given name.  
Sets *baudrate* to given rate.  
  

## void initialize_defaults()

Initializes debug, file descriptor, UART name and baud rate to predefined values.  
Initializes thread mutex.  
  

## void start()

Calls method *open_serial()*.  
This method is designed to ensure we have a valid serial port and configure
the connection.
  

## void open_serial()

Ensures open port by calling *_open_port(uart_name)* (fails otherwise).  
Ensures port setup by calling *_setup_port(baudrate, 8, 1, false, false)* (fails otherwise).  
Sets *lastStatus* packet drop count to zero.  
  

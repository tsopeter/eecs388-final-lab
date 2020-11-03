#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "eecs388_lib.h"


#define UART0 0
#define UART1 1 
int main()
{
    // initialize UART channels
    ser_setup(0); // uart0 (debug)
    ser_setup(1); // uart1 (raspberry pi)
    
    printf("Setup completed.\n");
    printf("Begin the main loop.\n");
    
    //data in byte
    uint8_t data = 0x00;

    //run
    while (1) {
        // YOUR CODE HERE

        //run the code when serial uart1 is ready
        if(ser_isread(UART1)){
            //get data from uart1
            data = ser_read(UART1);

            //output data back to uart0
            ser_printline(UART0, data);

        }
    }
    return 0;
}

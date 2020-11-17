#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "eecs388_lib.h"
#include "metal/i2c.h"

#define UART0 0
#define UART1 1

struct metal_i2c *i2c;
uint8_t bufWrite[5];
uint8_t bufRead[1];
volatile int g_angle;

//The entire setup sequence
void set_up_I2C()
{
    uint8_t oldMode;
    uint8_t newMode;
    _Bool success;

    bufWrite[0] = PCA9685_MODE1;
    bufWrite[1] = MODE1_RESTART;
    printf("%d\n",bufWrite[0]);
    
    i2c = metal_i2c_get_device(0);

    if(i2c == NULL){
        printf("Connection Unsuccessful\n");
    }
    else{
        printf("Connection Successful\n");
    }
    
    //Setup Sequence
    metal_i2c_init(i2c,I2C_BAUDRATE,METAL_I2C_MASTER);
    success = metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,2,bufWrite,METAL_I2C_STOP_DISABLE);//reset
    delay(100);
    printf("resetting PCA9685 control 1\n");

    //Initial Read of control 1
    bufWrite[0] = PCA9685_MODE1;//Address
    success = metal_i2c_transfer(i2c,PCA9685_I2C_ADDRESS,bufWrite,1,bufRead,1);//initial read
    printf("Read success: %d and control value is: %d\n", success, bufWrite[0]);
    
    //Configuring Control 1
    oldMode = bufRead[0];
    newMode = (oldMode & ~MODE1_RESTART) | MODE1_SLEEP;
    printf("sleep setting is %d\n", newMode);
    bufWrite[0] = PCA9685_MODE1;//address
    bufWrite[1] = newMode;//writing to register
    success = metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,2,bufWrite,METAL_I2C_STOP_DISABLE);//sleep
    bufWrite[0] = PCA9685_PRESCALE;//Setting PWM prescale
    bufWrite[1] = 0x79;
    success = metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,2,bufWrite,METAL_I2C_STOP_DISABLE);//sets prescale
    bufWrite[0] = PCA9685_MODE1;
    bufWrite[1] = 0x01 | MODE1_AI | MODE1_RESTART;
    printf("on setting is %d\n", bufWrite[1]);
    success = metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,2,bufWrite,METAL_I2C_STOP_DISABLE);//awake
    delay(100);
    printf("Setting the control register\n");
    bufWrite[0] = PCA9685_MODE1;
    success = metal_i2c_transfer(i2c,PCA9685_I2C_ADDRESS,bufWrite,1,bufRead,1);//initial read
    printf("Set register is %d\n",bufRead[0]);
} 

void breakup(int bigNum, uint8_t* low, uint8_t* high){
    *low = (uint8_t)bigNum;
    *high = (uint8_t)(bigNum >> 8);
}

void steering(int angle){
    uint8_t low = 0x00, high = 0x00;
    breakup(getServoCycle(angle), &low, &high);

    //set buffers
    bufWrite[0] = PCA9685_LED1_ON_L;
    bufWrite[1] = bufWrite[2] = 0;
    bufWrite[3] = low;
    bufWrite[4] = high;

    metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);
}

void stopMotor(){
    uint8_t low = 0x18, high = 0x01;

    //set buffers
    bufWrite[0] = PCA9685_LED0_ON_L;
    bufWrite[1] = bufWrite[2] = 0;
    bufWrite[3] = low;
    bufWrite[4] = high;

    metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);
}

void driveForward(uint8_t speedFlag){
    if(speedFlag >= 1 && speedFlag <= 3){
        uint16_t speed = (speedFlag - 1) * 2;
        uint16_t forwardDiff = 23;
        uint16_t num = 0x0118 + forwardDiff + speed;
        uint8_t low = 0x00, high = 0x00;
        breakup(num, &low, &high);

        //set buffers
        bufWrite[0] = PCA9685_LED0_ON_L;
        bufWrite[1] = bufWrite[2] = 0;
        bufWrite[3] = low;
        bufWrite[4] = high;

        metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);

    }
}

void driveReverse(uint8_t speedFlag){
    if(speedFlag >= 1 && speedFlag <= 3){
        uint16_t speed = (speedFlag - 1) * 2;
        uint16_t backwardDiff = 13;
        uint16_t num = 0x118 - (backwardDiff + speed);
        uint8_t low = 0x00, high = 0x00;
        breakup(num, &low, &high);

        //set buffers
        bufWrite[0] = PCA9685_LED0_ON_L;
        bufWrite[1] = bufWrite[2] = 0;
        bufWrite[3] = low;
        bufWrite[4] = high;

        metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);

    }
}

//interrupt handler
void raspberrypi_int_handler(int devid){

}

int convertStringtoInterger(char* str){
    int val = 0x00;
    sscanf(str, "%d", val);
    return val;
}

void reverse(){
    driveReverse(1);
    stopMotor();
    delay(2000);
    driveReverse(1);
}

uint8_t readFromTOF(){
    uint16_t tstop = 75;
    //read from tfmini
    if('Y' == ser_read(UART1) && 'Y' == ser_read(UART1)){
        uint16_t dist = ser_read(UART1);
        uint8_t dist_temp = ser_read(UART1);
        dist |= ((uint16_t)dist_temp << 8);

        if(dist < tstop){
            stopMotor();
            return 1;
        }
        else{
            return 0;
        }

    }
    else{
        return 0;
    }

}

int main(){
    // Initialize I2C
    set_up_I2C();
    delay(2000);

    // Calibrate Motor
    printf("Calibrate Motor.\n");
    stopMotor();
    delay(2000);

    // initialize UART channels
    ser_setup(UART1); // uart1 (receive from raspberry pi)
    ser_setup(UART0); // uart0 (receive from tfmini)

    
    printf("Setup completed.\n");
    printf("Begin the main loop.\n");
    
    // Initialize global angle
    g_angle = 0;
    char input[BUFFER_SIZE];
    char temp[BUFFER_SIZE];

    for(int i = 0; i < BUFFER_SIZE; i++){
        input[i] = temp[i] = ' ';
    }
    volatile int y_angle = 0;
    int nLength = 0;
    // Drive loop

    uint8_t flag = 0;
    while (1){
        /*
        if(ser_isready(UART0)){
            if(readFromTOF()){
                flag = 1;
            }
        }

        if(flag){
            break;
        }
        */
       if(ser_isready(UART0)){
           nLength = ser_readline(UART0, BUFFER_SIZE, input);

           //if same, discard
           if(0 != strncmp(input, temp, nLength)){
               //get number
               sscanf(input, "%d", &y_angle);

               //reassign input
               for(int i = 0; i < BUFFER_SIZE; i++){
                   temp[i] = input[i];
               }

               steering(y_angle);
           }
       }

    };
    return 0;


}

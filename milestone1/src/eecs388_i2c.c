#include <stdio.h>
#include <stdint.h>
#include "eecs388_lib.h"
#include "metal/i2c.h"


struct metal_i2c *i2c;
uint8_t bufWrite[9];
uint8_t bufRead[1];


//The entire setup sequence
void set_up_I2C(){
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

/* Task 1*/
void breakup(int bigNum, uint8_t* low, uint8_t* high){
   //get the low by implicit casting
   *low = (uint8_t)bigNum;

   //get the high by shifting 8 bits right
   //and implicit casting
   *high = (uint8_t)(bigNum >> 8);
}

/* Task 2*/
void steering(int angle){
    uint8_t success = 0x01; //false
    uint8_t diff = 0x04;  // 6 registers forward
    //set the cycle
    int servoCycle = getServoCycle(angle);

    //set the low and high inputs
    uint8_t low = 0x00, high = 0x00;
    //call breakup to set low and high
    breakup(servoCycle, &low, &high);

    //set buffers
    //PCA9685_LEC0_ON_L = 0x06
    bufWrite[0] = PCA9685_LED0_ON_L + diff;    //set the reg address low 0x0CA
    bufWrite[1] = 0;     //set the values as low
    bufWrite[2] = 0;  //set the reg address for high
    bufWrite[3] = low;
    bufWrite[4] = high;    //set the values as high
    success = metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);

    //check the success of the transfer function
    if(success != 0x00){
        //if failed, print fail statement
        printf("steering: transfer failed\n");
    }
    else{   
        //if passed, print the read data buffers
        printf("steering: transfer successful\n");
    }
}

void stopMotor(){
    /*
        Write Task 3 code here
    */
   //low + high = 0x0118 = 280d
   uint8_t low = 0x18, high = 0x01;
   uint8_t succ = 0x00;
   uint8_t diff = 0x02;
   bufWrite[0] = PCA9685_LED0_ON_L;  //set register for LED0_OFF_L
   bufWrite[1] = 0;                       //data
   bufWrite[2] = 0;   
   bufWrite[3] = low;
   bufWrite[4] = high;                    //data
   if(succ == metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1)){
       printf("stopMotor: transfer successful\n");  //print statement if transfer == 0x00
   }
   else{
       printf("stopMotor: transfer failed\n");      //print statement if transfer != 0x00
   }
}

void driveForward(uint8_t speedFlag){
    /*
        Write Task 4 code here
    */
   if(speedFlag >= 1 && speedFlag <= 3){
       uint8_t speed = (speedFlag - 1) * 2;     //sets the speed of the vehicle
       uint16_t forwardDiff = 23;               //forward Diff to move motor
       uint16_t num = 0x0118 + forwardDiff + (uint16_t)speed;   //number to transfer
       uint8_t low = 0x00, high = 0x00;         
       uint8_t succ = 0x00, diff = 0x02;        //difference between register
       breakup(num, &low, &high);               //breakup the number

        //set buffers
        bufWrite[0] = PCA9685_LED0_ON_L;
        bufWrite[1] = 0;
        bufWrite[2] = 0;
        bufWrite[3] = low;
        bufWrite[4] = high;

       if(succ == metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1)){
           printf("driveForward: transfer successful\n");
       }
       else{
           printf("driveForward: transfer not successful\n");
       }



   }
}

void driveReverse(uint8_t speedFlag){
    /*
        Write task 5 code here
    */
   if(speedFlag >= 1 && speedFlag <= 3){
       uint8_t speed = (speedFlag - 1) * 2;
       uint16_t backwardDiff = 13;
       uint16_t num = 0x0118 - (backwardDiff + (uint16_t)speed);
       uint8_t low = 0x00, high = 0x00;
       uint8_t succ = 0x00, diff = 0x02;
       breakup(num, &low, &high);

       //set buffers
       bufWrite[0] = PCA9685_LED0_ON_L;
       bufWrite[1] = 0;
       bufWrite[2] = 0;
       bufWrite[3] = low;
       bufWrite[4] = high;

       if(succ == metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1)){
           printf("driveReverse: transfer successful\n");
       }
       else{
           printf("driveReverse: transfer not successful\n");
       }
   }
}

void stop(){
    delay(2000);
}

int main()
{
    set_up_I2C();
    stopMotor();    //configure the motors
    steering(0);    //set steering to 0 deg
    stop();    //stop for 2 sec

    driveForward(1);    //drive forward with speedFlag == 1 (303)
    stop();     //delay for 2 sec

    steering(20);    //set steering to 20 deg
    stop();   //delay for 2 sec

    stopMotor();
    stop();

    driveForward(1);
    stop();

    steering(0);
    stop();

    stopMotor();


    
    //Defining the breakup function
    /*
        Task 1: breaking 12 bit into two 8-bit
        Define the function created that recieves a 12 bit number,
        0 to 4096 and breaks it up into two 8 bit numbers.

        Assign these values to a referenced value handed into
        the function. 

        ex: 
        uint8_t variable1;
        uint8_t variable2;
        breakup(2000,&variable1,&variable2);
        variable1 -> low 8 bits of 2000
        variable2 -> high 8 bits of 2000


    */    
    
    //Changing Steering Heading
    /*
        Task 2: using getServoCycle(), bufWrite, bufRead, 
        breakup(), and and metal_i2c_transfer(), implement 
        the function defined above to control the servo
        by sending it an angle ranging from -45 to 45.

        Use the getServoCycle function to get the value to 
        breakup.

        ex: 
        int valToBreak = getServoCycle(45);
        >>>sets valToBreak to 355
        
        note: the motor's speed controller is either 
        LED0 or LED1 depending on where its plugged into 
        the board. If its LED1, simply add 4 to the LED0
        address

        ex: steering(0); -> driving angle forward
    */
    
    
    //Motor config/stop. This will cause a second beep upon completion
    /*
        -Task 3: using bufWrite, bufRead, breakup(), and
        and metal_i2c_transfer(), implement the funcion made
        above. This function Configure the motor by 
        writing a value of 280 to the motor.

        -include a 2 second delay after calling this function
        in order to calibrate

        -Note: the motor's speed controller is either 
        LED0 or LED1 depending on where its plugged into 
        the board. If its LED1, simply add 4 to the LED0
        address

        ex: stopMotor();
    */


    /*
    ############################################################
        ATTENTION: The following section will cause the        
        wheels to move. Confirm that the robot is              
        Propped up to avoid it driving away, as well as         
        that nothing is touching the wheels and can get 
        caught in them

        If anything goes wrong, unplug the hifive board from
        the computer to stop the motors from moving 
        
        Avoid sticking your hand inside the 
        car while its wheels are spinning
    #############################################################
    */
    

    //Motor Forward
    /*
        -Task 4: using bufWrite, bufRead, breakup(), and
        and metal_i2c_transfer(), implement the function
        made above to Drive the motor forward. The given
        speedFlag will alter the motor speed as follows:
        
        speedFlag = 1 -> value to breakup = 303 
        speedFlag = 2 -> value to breakup = 305(Optional)
        speedFlag = 3 -> value to breakup = 307(Optional)

        -note 1: the motor's speed controller is either 
        LED0 or LED1 depending on where its plugged into 
        the board. If its LED1, simply add 4 to the LED0
        address

        ex: driveForward(1);
    */
    
    //Motor Reverse
    /*
        -Task 5: using bufWrite, bufRead, breakup(), and
        and metal_i2c_transfer(), implement the function
        made above to Drive the motor backward. The given
        speedFlag will alter the motor speed as follows:
        
        speedFlag = 1 -> value to breakup = 257 
        speedFlag = 2 -> value to breakup = 255(Optional)
        speedFlag = 3 -> value to breakup = 253(Optional)

        -note 1: the motor's speed controller is either 
        LED0 or LED1 depending on where its plugged into 
        the board. If its LED1, simply add 4 to the LED0
        address

        ex: driveReverse(1);
    */
    
    
    //Fully Controlling the PCA9685
    /*
        -Task 6: using previously defined functions, 
        perform the following sequence of actions
        
        -Configure the motors (wait for 2 seconds)
        -Set the steering heading to 0 degrees 
        -Drive forward (wait for 2 seconds)
        -Change the steering heading to 20 degrees (wait for 2 seconds)
        -Stop the motor (wait for 2 seconds)
        -Drive forward (wait for 2 seconds)
        -Set steering heading to 0 degrees (wait for 2 seconds)
        -Stop the motor
    */

}

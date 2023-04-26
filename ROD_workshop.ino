#include <krnl.h>
#include <PID_v1.h>
// pinout for motor
#define W1 2
#define W2 9
#define Pot A0
#define MotorPWM 10
#define DirectionPin 12

// doubles used calculating speed
double Setpoint, Input, Output;
//speed
double Speed;
// True = backwards , False = forward
bool MeasuredDirection = true;
// value read from the potentiometer
int potValue;
// used to count signal from quadratic encoder pr second
long int count;

//semaphores
k_t *s_sem, *pid_sem, *pot_sem;

// Fill in for PID function as we have not been taught how to make a P.I.D
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);

char Heap_ReadSerial[200];
char Heap_Readpot[100];
char Heap_MotorControl[100];
char Heap_Loggin[200];
char Heap_CalculateSpeed[100];

/*
void installIRS()
{
    DI();
    pinMode(2, INPUT);     // set som input
    digitalWrite(2, HIGH); // enable pullup resistor
    EIMSK |= (1 << INT0);  // enable external intr
    EICRA |= (1 << ISC01); // trigger INT0 on falling edge
    EI();
}
*/

// reads pot and sets potValue variable.
void ReadPot()
{
    while (1)
    {
        //k_wait(s_sem, 0);
        //Serial.println("Reading potentiometer");
        //k_signal(s_sem);

        k_wait(pot_sem,0);
        //511 is subtracted so that the potentiometer range becomes -511 to 511
        potValue = analogRead(Pot) - 511;
        k_signal(pot_sem);

        k_sleep(100);
    }
}

//reads speed, wanted speed and direction and uses them to adjust output and sets PWM
void MotorControl()
{
    while (1)
    {
        //k_wait(s_sem, 0);
        //Serial.println("Controling motor");
        //k_signal(s_sem);

        k_wait(pot_sem,0);
        // sets wanted speed
        Setpoint = map(potValue, -511, 511, -255, 255);
        k_signal(pot_sem);
        //sets the measured speed to negative if the measured direction is backwards
        MeasuredDirection ? Input = Speed : Input = -Speed;
        k_wait(pid_sem,0);
        //Output += (Setpoint-Input)*myPID.GetKp();
        //Replacement code for PID controller
        //if speed is slower then wanted add 0.01 if speed is higher subtract 0.01
        if((Setpoint-Input) > 0){
            Output+=0.01;
        } else if((Setpoint-Input) < 0){
            Output-=0.01;
        }

        //clamp output between 255 and -255 as that is our max PWM values.
        if (Output > 255.0) Output = 255.0;
        else if (Output < -255.0) Output = -255.0;
        //myPID.Compute();
        k_signal(pid_sem);

        //set PWM
        analogWrite(MotorPWM,abs(Output));
        //set H-bridge direction
        digitalWrite(DirectionPin,Output > 0);
        k_sleep(10);
    }
}

// uses the count from MeasureSpeed to calculate our speed every second
void CalculateSpeed(){
    while(1){
        //our max freq is 17khz and count becomes a frequency because we reset it every second
        Speed = map(count,0,17000,0,255);
        //reset count
        count = 0;
        k_sleep(1000);
    }
}

void MeasureSpeed()
{
    //counts how many times the encoder changes in a second
    count++;
    //sets the direction based on the second 180 deg shifted encoder
    MeasuredDirection = (PINB & (1<<(W2-8))) > 0;
    //Serial.println((PINB), BIN);
    // MeasuredDirection = digitalRead(W2);
}

// reads the serial input and if it is in the correct form of our protocol, it changes P I and D
void ReadSerial()
{
    while (1)
    {
        //k_wait(s_sem, 0);
        //Serial.println("Reading serial");
        //k_signal(s_sem);
        static char Buffer[100]; // need to be global.... otherwise we use all memory ;)
        static char BeginCharBuffer;
        double Ki, Kp, Kd;

        // runs while there are more than 0 characters in the serial buffer
        while (Serial.available() > 0)
        {
            // read and remove 1 char.
            BeginCharBuffer = Serial.read();
            //check if it is beginning char of our protocol
            if (BeginCharBuffer == '$')
            {
                // read until , and set temp P variable
                memset(Buffer, NULL, 100);
                Serial.readBytesUntil(',', Buffer, 100);
                Kp = strtod(Buffer,NULL);
                k_wait(s_sem,0);
                Serial.println(Kp);
                k_signal(s_sem);

                //same but with for I
                // zero buffer
                memset(Buffer, NULL, 100);
                Serial.readBytesUntil(',', Buffer, 100);
                Ki = strtod(Buffer,NULL);

                // reads until protocol stop variable, and sets temp D variable.
                // zero buffer
                memset(Buffer, NULL, 100);
                Serial.readBytesUntil('&', Buffer, 100);
                Kd = strtod(Buffer,NULL);
            }
        }
        //checks if temp P I and D variables are real values.
        if (Kd > 0.001 && Ki > 0.001 && Kp > 0.001)
        {
            //sets our P I and D if they are.
            k_wait(pid_sem, 0);
            myPID.SetTunings(Kp,Ki,Kd);
            k_signal(pid_sem);
        }
        k_sleep(500);
    }
}

// prints speed, wanted speed, output speed, direction and P I D values every second
void loggin()
{
    while (1)
    {
        k_wait(s_sem, 0);
        Serial.print("Speed: ");
        Serial.print(Input);
        Serial.print(" wanted speed: ");
        Serial.print(Setpoint);
        Serial.print(" out speed: ");
        Serial.print(int(Output));
        Serial.print(" P: ");
        Serial.print(myPID.GetKp());
        Serial.print(" I: ");
        Serial.print(myPID.GetKi());
        Serial.print(" D: ");
        Serial.print(myPID.GetKd());
        Serial.print(" direction: ");
        Serial.println(MeasuredDirection);
        k_signal(s_sem);
        k_sleep(1000);
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(W1, INPUT_PULLUP);
    pinMode(W2, INPUT_PULLUP);
    pinMode(MotorPWM, OUTPUT);
    pinMode(DirectionPin,OUTPUT);
    attachInterrupt(digitalPinToInterrupt(W1), MeasureSpeed, RISING);

    // init kernel
    k_init(5, 3, 0);
    //create semaphores
    s_sem = k_crt_sem(1, 1);
    pid_sem = k_crt_sem(1, 1);
    pot_sem = k_crt_sem(1,1);
    //create tasks
    k_crt_task(CalculateSpeed,1,Heap_CalculateSpeed,100);
    k_crt_task(MotorControl, 2, Heap_MotorControl, 100);
    k_crt_task(ReadPot, 10, Heap_Readpot, 100);
    k_crt_task(ReadSerial, 20, Heap_ReadSerial, 200);
    k_crt_task(loggin, 30, Heap_Loggin, 200);
    // start kernel
    k_start();
    Serial.println("stuff has gone wrong if you see this!");
}

void loop()
{
}
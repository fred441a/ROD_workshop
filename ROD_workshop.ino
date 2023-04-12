#include <krnl.h>
#include <PID_v1.h>

#define W1 2
#define W2 3
#define Pot A3
#define MotorPWM 10

double Setpoint, Input, Output;

double Speed;
bool direction;
int potValue;

k_t *s_sem, *pid_sem, *pot_sem;

PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);

char Heap_ReadSerial[200];
char Heap_Readpot[100];
char Heap_MotorControl[100];
char Heap_Loggin[200];

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

void ReadPot()
{
    while (1)
    {
        k_wait(s_sem, 0);
        Serial.println("Reading potentiometer");
        k_signal(s_sem);
        k_wait(pot_sem,0);
        potValue = analogRead(Pot) - 511;
        k_signal(pot_sem);
        k_sleep(100);
    }
}

void MotorControl()
{
    while (1)
    {
        k_wait(s_sem, 0);
        Serial.println("Controling motor");
        k_signal(s_sem);

        k_wait(pot_sem,0);
        Setpoint = map(abs(potValue), 0, 511, 0, 255);
        k_signal(pot_sem);

        Input = map(Speed * 1000, 0, 1000, 0, 255); // some value selected as the max speed of motor is unkown.

        k_wait(pid_sem,0);
        myPID.Compute();
        k_signal(0);
        
        analogWrite(MotorPWM, Output);
        k_sleep(10);
    }
}

void MeasureSpeed()
{
    static int lastTime;
    Speed = 1 / (millis() - lastTime);
    lastTime = millis();
    /*
    if(digitalRead(W2)== HIGH){
        direction = true;
    }else{
        direction = false;
    }
    */
    direction = digitalRead(W2);
}

void ReadSerial()
{
    while (1)
    {
        k_wait(s_sem, 0);
        Serial.println("Reading serial");
        k_signal(s_sem);
        static char Buffer[100]; // need to be global.... otherwise we use all memory ;)
        static char BeginCharBuffer;
        double Ki, Kp, Kd;

        while (Serial.available() > 0)
        {
            BeginCharBuffer = Serial.read();
            if (BeginCharBuffer == '$')
            {

                memset(Buffer, NULL, 100);
                Serial.readBytesUntil(',', Buffer, 100);
                Kp = String(Buffer).toDouble();

                // zero buffer
                memset(Buffer, NULL, 100);
                Serial.readBytesUntil(',', Buffer, 100);
                Ki = String(Buffer).toDouble();

                // zero buffer
                memset(Buffer, NULL, 100);
                Serial.readBytesUntil('&', Buffer, 100);
                Kd = String(Buffer).toDouble();
            }
        }
        if (Kd > 0.001 && Ki > 0.001 && Kp > 0.001)
        {

            k_wait(pid_sem, 0);
            myPID.SetTunings(Kp,Ki,Kd);
            k_signal(pid_sem);
        }
        k_sleep(500);
    }
}

void loggin()
{
    while (1)
    {
        k_wait(s_sem, 0);
        Serial.println("loggin");
        k_signal(s_sem);
        // Serial.println(printf("speed: %lf, wanted speed: %lf, output speed: %lf ,P:%lf,I:%lf,D:%lf direction: %d",Speed, Setpoint,Output,Kp,Ki,Kd,direction));

        k_sleep(1000);
    }
}

void k_breakout()
{
    bool flip = true;
    digitalWrite(11, flip);
    flip = !flip;
}

void setup()
{
    Serial.begin(115200);

    pinMode(11, OUTPUT);
    pinMode(W1, INPUT_PULLUP);
    pinMode(W2, INPUT_PULLUP);
    pinMode(MotorPWM, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(W1), MeasureSpeed, RISING);

    k_init(4, 3, 0);
    s_sem = k_crt_sem(1, 1);
    pid_sem = k_crt_sem(1, 1);
    pot_sem = k_crt_sem(1,1);
    k_crt_task(MotorControl, 1, Heap_MotorControl, 100);
    k_crt_task(ReadPot, 10, Heap_Readpot, 100);
    k_crt_task(ReadSerial, 20, Heap_ReadSerial, 200);
    k_crt_task(loggin, 30, Heap_Loggin, 200);

    k_start();
    Serial.println("stuff has gone wrong if you see this!");
}

void loop()
{
}
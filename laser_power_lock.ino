#include <PID_v1.h>
#include <math.h>

/* Photodiode analog input pin */
#define pd_pin 0

/* Analog pin for output voltage control in manual mode */
#define man_pin 1

/* Digital pin for manual mode switch */
#define man_sw 2

/* Analog pin control of proportional PID term */
#define kp_pin 2

/* Analog pin control of integral PID term */
#define ki_pin 3

#define aref_volt 5.0

/* Input/Output bit resolution */
#define numbits 10

/* Resistor ladder Arduino UNO analog output */
#define msb_pin 4
#define num_pins_portd 8

/* Manual Control Global Variable */
int man_control = 0;

/* PID parameters */
double Input=0, Output=0, Setpoint=0;
double kp=0.5125, ki=0.4875, kd=0.0;

int res = pow(2, numbits);

PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);

int printDelay = 50;
int printCounter = 0;

int error = 0;
const int dataLength = 50;
int errorArray[dataLength]; //Array of last (dataLength) error values
                            //Serial port should output and clear
                            //this array when it fills up.
int errorArrayIndex = 0;    //Number of new entries in errorArray

double voltin = 0;
double voltout = 0;

void setup()
{ 
  
  pinMode(man_sw, INPUT);
  
  /* Resistor ladder initialization of Digital pins */
  for(int j = msb_pin; j < msb_pin + numbits; j++){
  
    pinMode(j, OUTPUT);
    
  }

  Serial.begin(9600);
  
  /* tell the PID to range between 0 and the full window size */
  myPID.SetOutputLimits(-res/2.0 + 1, res/2.0 - 1);
  
  /* turn the PID on */
  myPID.SetMode(AUTOMATIC);
  
  myPID.SetTunings(kp,ki,kd);
  
  /* Set output to middle of the range */
  writePorts(res/2);
  delay(500);
  
  /* Read initial input and use it as setpoint */
  analogRead(pd_pin);
  delay(500);
  Setpoint = analogRead(pd_pin);
  delay(100);
  
}


void loop()
{
  
  man_control = digitalRead(man_sw);
  
  float pd_read = 0;

  for(int i = 0; i < 30; i++)
    pd_read += analogRead(pd_pin);

  pd_read /= 30;
  
  Input = pd_read;

  error = Input - Setpoint;
  
  //  listenForTunings();
  analogRead(kp_pin);
  kp = (double) analogRead(kp_pin) / res;
  
  analogRead(ki_pin);
  ki = (double) analogRead(ki_pin) / res;

  myPID.SetTunings(kp,ki,kd);

  myPID.Compute();

  int final_out;

  if (!man_control) {
    
    final_out = Output + res/2;
  
  } else {
   
    analogRead(man_pin);
    final_out = analogRead(man_pin);  
    analogRead(pd_pin);
    Setpoint = analogRead(pd_pin);
  }
  
  writePorts(final_out);

  voltin = (double) pd_read * aref_volt / res;
  
  voltout = (double) final_out / res;
  
//  updateErrorArray();
  
  printCounter += 1;
  if (printCounter > printDelay){
    debugPrint();
   printCounter = 0;
  }

}

void writePorts(long int digits){
  
    PORTD = ((digits << msb_pin) & 0B1111000000) | (PORTD & 0B0000111111);
    PORTB = ((digits >> (num_pins_portd - msb_pin)) & 0B0000111111) | (PORTB & 0B1111000000);

}

void updateErrorArray(){
  errorArray[errorArrayIndex] = error;
  errorArrayIndex += 1;

  if(errorArrayIndex == 49){ //when the 50th error is entered, output data and reset i
               //arrays in C are zero-indexed, so errorArray[49] is the
               //50th entry
    errorArrayIndex = 0;
    
    Serial.println();
    for(int i = 0; i <= 49; i++){
      Serial.print(errorArray[i]);
      Serial.print(';');
    }
  }
}


void listenForTunings(){
  /*  check if data has been sent from the computer: */
     while (Serial.available()) {
     /* read the most recent byte */
        byte byteRead = Serial.read();
    
        if(byteRead == 'P'){
          kp = Serial.parseFloat();
        }
        if(byteRead == 'I'){
          ki = Serial.parseFloat();
        }
      
      /*Listen for an equal sign (byte code 61) 
      to send ki and kp back to the computer*/
      if(byteRead==61){
        Serial.print(kp);Serial.print(" ");
        Serial.println(ki);
      }
   }
}

void debugPrint(){
    
    Serial.print("set ");
    Serial.print(Setpoint);
    Serial.print("  ");  
    Serial.print("in ");
    Serial.print(Input);
    Serial.print("  ");
    Serial.print("vin ");
    Serial.print(voltin,3);
    Serial.print("  ");
    Serial.print("err ");
    Serial.print(error);
    Serial.print("  ");
    Serial.print("out ");
    Serial.print(Output);
    Serial.print("  ");
    Serial.print("vout ");
    Serial.print(voltout,3);
    Serial.print("  ");
    Serial.print("man: ");
    Serial.print(man_control);
    Serial.print("  ");
    Serial.print("ki: ");
    Serial.print(ki,3);
    Serial.print("  ");
    Serial.print("kp: ");
    Serial.print(kp,3);
    Serial.print("  ");
    Serial.println();
    
}

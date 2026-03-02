#include "LIDARLite_v4LED.h"
LIDARLite_v4LED myLIDAR; 
double LidarDistance;
bool flag;

void setupTimer1() {
  noInterrupts();
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // 10 Hz (16000000/((6249+1)*256))
  OCR1A = 6249;
  // CTC
  TCCR1B |= (1 << WGM12);
  // Prescaler 256
  TCCR1B |= (1 << CS12);
  // Output Compare Match A Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

ISR(TIMER1_COMPA_vect) {
  flag = true;
}


void setup() {
  setupTimer1();
  Serial1.begin(9600);
  Wire.begin();
  if (myLIDAR.begin() == false) {
    Serial1.println("Lidar not connected, check your wiring and I2C address!");
    while(1);
  }
}

void loop() {

    if(!myLIDAR.getBusyFlag()) { // if the distance is ready
    LidarDistance = 10*myLIDAR.readDistance();  // Read and make it mm 
    myLIDAR.takeRange(); // Take another measurement for next time
    }
    
  if(flag){

    Serial1.println(LidarDistance); 

    flag = false;
  
  }
  
}

const byte outPin = 13; // Output pin: digital pin 13(D13)
const byte interruptPin1 = 2; // Interrupt pin: D2
const byte interruptPin2 = 2; // Interrupt pin: D2
const byte resetPin = 5;
volatile byte state = 0;
unsigned long cnt1 = 0;
unsigned long cnt2 = 0;

void setup()
{
   pinMode(outPin, OUTPUT); // Output mode
   pinMode(interruptPin1, INPUT_PULLUP); // Input mode, pull-up
   pinMode(interruptPin2, INPUT_PULLUP); // Input mode, pull-up
   pinMode(resetPin, INPUT);
   attachInterrupt(digitalPinToInterrupt(interruptPin1), intfunc1, RISING); // Enable interrupt
   attachInterrupt(digitalPinToInterrupt(interruptPin2), intfunc2, RISING); // Enable interrupt
   Serial.begin(115200);
}

void intfunc1() // Interrupt function
{
  cnt1++;
  if (state == 0) // If D13 output is low
  {
     digitalWrite(outPin, HIGH);
     state = 1;
  }
  else
  {
     digitalWrite(outPin, LOW);
     state = 0;
  }
}

void intfunc2() // Interrupt function
{
  cnt2++;
  if (state == 0) // If D13 output is low
  {
     digitalWrite(outPin, HIGH);
     state = 1;
  }
  else
  {
     digitalWrite(outPin, LOW);
     state = 0;
  }
}


void loop()
{
  Serial.println(cnt1);
  Serial.println(cnt2);
  
  if(digitalRead(resetPin) == HIGH)
  {
    cnt1 = 0;
    cnt2 = 0;
  }
  
  delay(1000);
}

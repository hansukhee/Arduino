#define AOpin  A0    // Analog out pin - yellow
#define SIpin  22    // Start Integration - Orange
#define CLKpin 23
#define NPIXELS 128  // NO. of pixels in array

byte Pixel[NPIXELS];  // Sield for measured values <0 - 255>

int LineSensor_Data[NPIXELS];
int LineSensor_Data_Adaption[NPIXELS];
int MAX_LineSensor_Data[NPIXELS];
int MIN_LineSensor_Data[NPIXELS];
int flag_line_adapation;

#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

void setup() 
{  // put your setup code here, to run once:
  int i;

  for (i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data[i] = 0;
    LineSensor_Data_Adaption[i] = 0;
    MAX_LineSensor_Data[i] = 1023;
    MIN_LineSensor_Data[i] = 0;
  }
  
  pinMode(SIpin, OUTPUT);
  pinMode(CLKpin, OUTPUT);
  pinMode(AOpin, OUTPUT);

  digitalWrite(SIpin, LOW);
  digitalWrite(CLKpin, LOW);


  #if FASTADC
  // set prescale to 16
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  #endif
  
  flag_line_adapation = 0;
  
  Serial.begin(115200);
}

void read_line_camera(void)
{
  delay(1);
  
  digitalWrite(CLKpin, LOW);
  digitalWrite(SIpin, HIGH);
  digitalWrite(CLKpin, HIGH);
  digitalWrite(SIpin,  LOW);
  delayMicroseconds (1);

  for (int i = 0; i < NPIXELS; i++)
  {
    Pixel[i] = analogRead(AOpin) / 4;
    digitalWrite(CLKpin, LOW);
    delayMicroseconds (1);
    digitalWrite(CLKpin, HIGH);
  }
  digitalWrite(CLKpin, LOW);
}

void loop()
{
  read_line_camera();

  for (int i = 0; i < NPIXELS; i++)
  {
    Serial.println((byte)Pixel[i] + 1);
  }
}

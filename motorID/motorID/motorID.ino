int encoderL = 10;
int encoderR = 11;

volatile int encoderLNow = 0;
volatile int encoderRNow = 0;

void setup(){
  attachInterrupt(0, EncoderL, RISING);
  attachInterrupt(1, EncoderR, RISING);
  pinMode(encoderL, INPUT);
  pinMode(encoderR, INPUT);
  Serial.begin(9600);
}

void loop(){
  Serial.print(encoderLNow);
  Serial.print(" ");
  Serial.print(encoderRNow);
  Serial.print(" ");
  Serial.println(millis());
}

void EncoderL(){
  if (digitalRead(encoderL) == LOW) // Forward
  {
    encoderLNow += 1; 
  }
  else // Backward
  {
    encoderLNow -= 1; 
  }
}

void EncoderR()
{
  if (digitalRead(encoderR) == HIGH) // Forward
  {
    encoderRNow += 1; 
  }
  else // Backward
  {
    encoderRNow -= 1; 
  }
}

#define IN_PIN 2

volatile bool flagComplete;


void IRInterrupt(void) {

  flagComplete = true;
    
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Ready...");

  flagComplete = false;

  attachInterrupt(digitalPinToInterrupt(IN_PIN), IRInterrupt, CHANGE);

}

void loop() {
  // put your main code here, to run repeatedly:
  if (flagComplete){
    Serial.println("Pin Changed");
    flagComplete = false;
  }
  else{
    Serial.println("Pin Not Changed");
  }

}

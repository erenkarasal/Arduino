int buzzerPin = 2;

//GND (+ kutup )

void setup() {
  // put your setup code here, to run once:
  pinMode(buzzerPin , OUTPUT) ;
}

void loop() {
  // put your main code here, to run repeatedly:

digitalWrite(buzzerPin , HIGH);
delay(1000);
digitalWrite(buzzerPin , LOW);
delay(1000);
}

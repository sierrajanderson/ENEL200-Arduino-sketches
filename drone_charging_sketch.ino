const int sensorPin = A0;  // IR Distance sensor input after conditioning
const int chargerPin = 7;  // LED output

enum ChargingStatus {DRONE_ABSENT, DRONE_PRESENT};
ChargingStatus droneChargingState = DRONE_ABSENT;
int droneCount = 0;

#define DRONE_ARRIVED_THRESHOLD_CM 24
#define DRONE_DEPART_THRESHOLD_CM 26

void setup() {
  pinMode(sensorPin, INPUT);       // Configure this pin as an input
  pinMode(chargerPin, OUTPUT);     // Configure the pin as an output
  digitalWrite(chargerPin, LOW);   // Initially set to LOW
  
  Serial.begin(115200);            // Start the serial monitor
}

// Repeatedly read the A0 ADC, and send the ADC value
// through the serial output
void loop() { 
  int adcValue = analogRead(sensorPin);
  float sensorVoltage = float(adcValue) * 5 / 1023;
  float objectDistance_cm = 3.5 * 11.425 * pow(sensorVoltage, -0.511);

  switch (droneChargingState) {
    case DRONE_ABSENT:
      if (objectDistance_cm < DRONE_ARRIVED_THRESHOLD_CM) {
        digitalWrite(chargerPin, HIGH);
        droneCount++;
        Serial.print("Charging drone number ");
        Serial.println(droneCount);
        droneChargingState = DRONE_PRESENT;
      }
      break;
    case DRONE_PRESENT:
      if (objectDistance_cm > DRONE_DEPART_THRESHOLD_CM) {
        digitalWrite(chargerPin, LOW);
        Serial.println("Drone has departed. Charger turned off. ");
        droneChargingState = DRONE_ABSENT;
      }
      break;
  }

  delay(100);
}

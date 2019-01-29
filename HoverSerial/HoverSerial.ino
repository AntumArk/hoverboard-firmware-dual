

struct Serialcommand
{
  int16_t steer;
  int16_t speed;
  //uint32_t crc;
} ;
Serialcommand cmd;


void setup() {
  // initialize serial:
  Serial.begin(115200);
  Serial1.begin(115200);
  // reserve 200 bytes for the inputString:

  // set the data rate for the SoftwareSerial port
  delay(5000);
  Serial.println("Hello, world?");
}

void loop() {
  // print the string when a newline arrives:
  if (Serial.available()) {
    Serial.println("Hello");
    cmd.steer = Serial.parseInt();
    cmd.speed = Serial.parseInt();
    Serial.print("Your params: steer ");
    Serial.print(cmd.steer);
    Serial.print("  speed ");
    Serial.println(cmd.speed);
    Serial.flush();

  }
  Serial1.write((uint8_t*)&cmd.steer, sizeof(cmd.steer));
  Serial1.write((uint8_t*)&cmd.speed, sizeof(cmd.speed));
  delay(20);
}



struct Serialcommand
{
  int16_t steer;
  int16_t speed;
  //uint32_t crc;
} ;
Serialcommand cmd;


void setup() {
  // initialize serial:
  Serial1.begin(115200);// A10 A9 USART1 Port of Blue pill
  Serial3.begin(115200);// USART3 port of blue pill for out comms.
  // reserve 200 bytes for the inputString:

  // set the data rate for the SoftwareSerial port
  delay(5000);
  Serial1.println("Hello, world?");
}

void loop() {
  // print the string when a newline arrives:
  if (Serial1.available()) { //Enter your speed and steer separated by space
    Serial1.println("Hello");
    cmd.steer = Serial1.parseInt();
    cmd.speed = Serial1.parseInt();
    Serial1.print("Your params: steer ");
    Serial1.print(cmd.steer);
    Serial1.print("  speed ");
    Serial1.println(cmd.speed);
    Serial1.flush();

  }
  Serial3.write((uint8_t*)&cmd.steer, sizeof(cmd.steer)); //Off it goes
  Serial3.write((uint8_t*)&cmd.speed, sizeof(cmd.speed));
  delay(20);
}

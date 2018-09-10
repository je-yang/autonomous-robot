//Define encoder inputs
#define ENC_A 8
#define ENC_B 9
#define ENC_PORT PINB

void setup() {
  // Set up encoder pins as inputs
  pinMode(ENC_A, INPUT);
  digitalWrite(ENC_A, HIGH);
  pinMode(ENC_B, INPUT);
  digitalWrite(ENC_B, INPUT);
  Serial.begin(115200);
  Serial.println("Start");
}

void loop() {
  static uint8_t counter = 0; //value that changes with the encoder
  int8_t tmpdata;

  tmpdata = read_encoder();
  if(tmpdata){
  	Serial.print("Counter value:");
  	Serial.println(counter, DEC);
  	counter += tmpdata;
  }
}

//function that returns the change in the encoder state (-1, 0, 1)
int8_t read_encoder(){
	static int8_t enc_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
	static uint8_t old_AB = 0;

	old_AB <<= 2; //remembers previous state
	old_AB |= (ENC_PORT & 0x03); //adds current state
	return (enc_states[(old_AB & 0x0f)]);
}
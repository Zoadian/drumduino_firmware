#define PORT_CNT 6
#define CHAN_CNT 8

#define MULTIPLEX_PIN_A 2
#define MULTIPLEX_PIN_B 3
#define MULTIPLEX_PIN_C 4

void multiplexSelectChan(uint8_t chan)
{
	digitalWrite(MULTIPLEX_PIN_A, chan & 0x1);
	digitalWrite(MULTIPLEX_PIN_B, chan & 0x2);
	digitalWrite(MULTIPLEX_PIN_C, chan & 0x4);
}


byte valueFrame[1 + CHAN_CNT* PORT_CNT] = {0xff, 0};


void setup()
{
	// Setup MultiplexSelection Pins
	pinMode(MULTIPLEX_PIN_A, OUTPUT);
	pinMode(MULTIPLEX_PIN_B, OUTPUT);
	pinMode(MULTIPLEX_PIN_C, OUTPUT);

	// Setup AD Pins
	analogReference(DEFAULT);

	// Setup Serial
	Serial.begin(115200);
	Serial.flush();
}

void loop()
{
	for(uint8_t chan = 0; chan < CHAN_CNT; ++chan) {
		multiplexSelectChan(chan);

		for(uint8_t port = 0; port < PORT_CNT; ++port) {
			int channelNumber = port * CHAN_CNT + chan;

			byte& value = *(valueFrame + 1 + channelNumber);

			value = byte(analogRead(port) / 8); //map [0..1023] -> [0..127]
		}
	}

	Serial.write(valueFrame, sizeof(valueFrame));
}

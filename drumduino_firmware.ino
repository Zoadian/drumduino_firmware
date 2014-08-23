#define PORT_CNT 6
#define CHAN_CNT 8

#define MULTIPLEX_PIN_A 2
#define MULTIPLEX_PIN_B 3
#define MULTIPLEX_PIN_C 4



inline void setPrescalers(byte i)
{
	i = i % 7;
	const byte prescalers[] = {
		B00000000, // PS_2
		B00000010, // PS_4
		B00000011, // PS_8
		B00000100, // PS_16
		B00000101, // PS_32
		B00000110, // PS_64
		B00000111, // PS_128
	};

	ADCSRA &= ~prescalers[6];
	ADCSRA |= prescalers[0];
}

inline void multiplexSelectChan(uint8_t chan)
{
	PORTD = B00011100 & (chan << 2);
}

struct SysexFrame {
	byte begin;
	byte manufacturer;
	byte values[CHAN_CNT* PORT_CNT];
	byte end;

	SysexFrame()
		: begin(0xf0)
		, manufacturer(42)
		, end(0xF7)
	{
		memset(values, 0, sizeof(values));
	}
};

SysexFrame _frame;
byte _throttle = 1;
byte _cnt = 0;

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

	//Configure Prescaler
	setPrescalers(2);
}

inline void handleMessage(byte* msg, byte length)
{
	switch(msg[0] && length == 3) {
		case 0xff: {
			setPrescalers(msg[1]);
			_throttle = msg[2] != 0 ? msg[2] : 1;
		}
	}
}

inline void input()
{
	//read until we receive a sysex
	while(Serial.peek() >= 0 && Serial.peek() != 0xF0) {
		Serial.read();
	}

	if(Serial.available() >= 6) {
		byte start = Serial.read();
		byte manufacturerId = Serial.read();
		byte deviceId = Serial.read();
		byte length = Serial.read();
		byte value[128];

		if(length > 0) {
			Serial.readBytes(value, length);
			handleMessage(value, length);
		}
	}
}


inline void output()
{
	for(uint8_t chan = 0; chan < CHAN_CNT; ++chan) {
		multiplexSelectChan(chan);

		for(uint8_t port = 0; port < PORT_CNT; ++port) {

			int channelNumber = port * CHAN_CNT + chan;

			byte& value = *(_frame.values + channelNumber);

			byte v = byte(analogRead(port) >> 3); //map [0..1023] -> [0..127]

			value = (value > v) ? value : v;
		}
	}

	if(_cnt % _throttle == 0) {
		Serial.write((byte*)&_frame, sizeof(_frame));
		memset(_frame.values, 0, sizeof(_frame.values));
	}

	++_cnt;
}

void loop()
{
	input();
	output();
}

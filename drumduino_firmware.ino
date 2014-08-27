#define PORT_CNT 6
#define CHAN_CNT 8

#define MULTIPLEX_PIN_A 2
#define MULTIPLEX_PIN_B 3
#define MULTIPLEX_PIN_C 4

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


//=================================================================================
// Prescaler
//=================================================================================
// Maximum sampling frequency    // Resolution
#define Prescaler_2   B00000000 // 16 MHz / 2 = 8 MHz            //
#define Prescaler_4   B00000010 // 16 MHz / 4 = 4 MHz            // ~5.9
#define Prescaler_8   B00000011 // 16 MHz / 8 = 2 MHz            // ~7.4
#define Prescaler_16  B00000100 // 16 MHz / 16 = 1 MHz           // ~8.6
#define Prescaler_32  B00000101 // 16 MHz / 32 = 500 kHz         // ~8.9
#define Prescaler_64  B00000110 // 16 MHz / 64 = 250 kHz         // ~9.0
#define Prescaler_128 B00000111 // 16 MHz / 128 = 125 kHz        // ~9.1

inline void setPrescaler(int prescaler)
{
	ADCSRA &= B11111000;
	ADCSRA |= prescaler;
}


//=================================================================================
//  Adc Pin
//=================================================================================
#define AdPin_0   B00000000
#define AdPin_1   B00000001
#define AdPin_2   B00000010
#define AdPin_3   B00000011
#define AdPin_4   B00000100
#define AdPin_5   B00000101
#define AdPin_6   B00000110 // Bei Atmega8 nur in der Gehäusebauform TQFP und MLF verfügbar, nicht in PDIP
#define AdPin_7   B00000111 // Bei Atmega8 nur in der Gehäusebauform TQFP und MLF verfügbar, nicht in PDIP
#define AdPin_Vbg B00001110 // 1.23V
#define AdPin_GND B00001111 // 0V

inline void setAdPin(int adPin)
{
	ADMUX &= B11110000;
	ADMUX |= adPin;
}

//=================================================================================
// ADC Alignment
//=================================================================================
// Das Ergebnis wird in den Registern ADCH/ADCL linksbündig ausgerichtet.
// Die 8 höchstwertigen Bits des Ergebnisses werden in ADCH abgelegt.
// Die verbleibenden 2 niederwertigen Bits werden im Register ADCL in den Bits 6 und 7 abgelegt.
#define ADAlignmentLeft  B00100000
#define ADAlignmentRight B00000000


inline void setADAlignment(int align)
{
	ADMUX &= ~B00100000;
	ADMUX |= align;
}

//=================================================================================
inline void startADCConversion()
{
	ADCSRA |= B01000000;
}

//=================================================================================

inline void disableAnalogComparator()
{
	ACSR = B10000000;
}

inline void multiplexSelectChan(uint8_t chan)
{
	PORTD = B00011100 & (chan << 2);
}

struct SysexFrame {
	byte begin;
	byte manufacturer;
	unsigned long time1;
	unsigned long time2;
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




void setup()
{
	// Setup MultiplexSelection Pins
	pinMode(MULTIPLEX_PIN_A, OUTPUT);
	pinMode(MULTIPLEX_PIN_B, OUTPUT);
	pinMode(MULTIPLEX_PIN_C, OUTPUT);

	// Setup ADCs
	analogReference(DEFAULT);
	disableAnalogComparator();
	setPrescaler(Prescaler_8);
	//setADAlignment(ADAlignmentLeft);

	// Disable digital input buffers on all analog input pins
	DIDR0 = DIDR0 | B00111111;

	// Setup Serial
	//Serial.begin(115200);
	Serial.begin(2000000);
	Serial.flush();
}


inline void output()
{
	_frame.time1 = micros();

	for(uint8_t chan = 0; chan < CHAN_CNT; ++chan) {
		multiplexSelectChan(chan);

		for(uint8_t port = 0; port < PORT_CNT; ++port) {

			int channelNumber = port * CHAN_CNT + chan;

			byte& value = *(_frame.values + channelNumber);
			
			value = byte(analogRead(port) >> 3);
		}
	}

	_frame.time2 = micros();
	Serial.write((byte*)&_frame, sizeof(_frame));
}

void loop()
{
	//input();
	output();
}

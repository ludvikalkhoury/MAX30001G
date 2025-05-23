#include <MAX30001G.h>

MAX30001G ecg(10); // CS pin

void setup() {
	Serial.begin(115200);
	ecg.begin();
}

void loop() {
	int32_t raw = ecg.readECGRaw();
	Serial.println(raw);
	delay(10);
}

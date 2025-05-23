#ifndef MAX30001G_H
#define MAX30001G_H

#include <Arduino.h>
#include <SPI.h>

class MAX30001G {
public:
	MAX30001G(uint8_t csPin);
	void begin();
	int32_t readECGRaw();
	float computeHeartRate();
	float computeRespRate();

private:
	uint8_t _csPin;
	void writeRegister(uint8_t addr, uint32_t data);
	uint32_t readRegister(uint8_t addr);
};

#endif

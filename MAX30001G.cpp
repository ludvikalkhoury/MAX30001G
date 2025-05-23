#include "MAX30001G.h"

#define BUFFER_SIZE 250
#define SAMPLE_RATE_HZ 200
#define RESP_BUFFER_SIZE 1000

// Buffers for ECG and respiratory processing
static int32_t ecg_buffer[BUFFER_SIZE];
static int buffer_index = 0;
static unsigned long last_peak_time = 0;
static float hr_bpm = 0;

static float resp_buffer[RESP_BUFFER_SIZE];
static int resp_index = 0;
static float breath_rate_bpm = 0;

/*
 * ===== Constructor =====
 * Saves the chip select pin for SPI communication.
 */
MAX30001G::MAX30001G(uint8_t csPin) {
	_csPin = csPin;
}

/*
 * ===== begin =====
 * Initializes SPI and configures the CS pin.
 */
void MAX30001G::begin() {
	SPI.begin();
	pinMode(_csPin, OUTPUT);
	digitalWrite(_csPin, HIGH);
}

/*
 * ===== readECGRaw =====
 * Reads a single raw ECG value from the MAX30001G's ECG FIFO register (e.g., address 0x21).
 * The ECG sample is a 24-bit signed integer. If the 23rd bit (sign bit) is set, we must sign-extend to 32 bits.
 */
int32_t MAX30001G::readECGRaw() {
	uint32_t data = readRegister(0x21);  // Read 24-bit ECG value from ECG FIFO register
	int32_t ecg = data & 0xFFFFFF;       // Mask to 24 bits
	if (ecg & 0x800000) ecg |= 0xFF000000; // Sign extension for negative values
	return ecg;  // Return signed ECG sample
}

/*
 * ===== computeHeartRate =====
 * Implements the Pan–Tompkins algorithm to detect QRS complexes in ECG signals.
 * Returns real-time heart rate (bpm) from R-peak intervals.
 * Core steps:
 *   1. Bandpass filter (IIR: LPF + HPF approximation)
 *   2. Derivative filter
 *   3. Squaring
 *   4. Moving Window Integration
 *   5. Adaptive Thresholding
 *   6. Slope-based T-wave suppression
 *   7. RR interval update and heart rate calculation
 */
float MAX30001G::computeHeartRate() {
	const int fs = 200;
	const int min_rr = 300;
	const int window_size = 30;

	static float ecg_buffer[250] = {0};
	static int buf_index = 0;

	static float mw_buffer[window_size] = {0};
	static int mw_index = 0;

	static float deriv_kernel[5] = {1, 2, 0, -2, -1};
	static float signal_level = 0, noise_level = 0;
	static float threshold = 0;
	static float rr_intervals[8] = {1000};
	static int rr_idx = 0;

	static unsigned long last_peak_time = 0;
	static bool found_peak = false;

	static float last_mw = 0;
	static float slope_buffer[8] = {0};
	static int slope_idx = 0;

	float sample = (float)readECGRaw();

	static float lp = 0, hp = 0;
	lp += 0.1 * (sample - lp);
	hp += 0.1 * (lp - hp);
	float filtered = hp;

	float deriv = 0;
	for (int i = 0; i < 5; i++) {
		int idx = (buf_index - i + 250) % 250;
		deriv += ecg_buffer[idx] * deriv_kernel[i];
	}
	deriv *= (1.0 / 8.0) * fs;

	float squared = deriv * deriv;

	mw_buffer[mw_index] = squared;
	mw_index = (mw_index + 1) % window_size;
	float mw_integrated = 0;
	for (int i = 0; i < window_size; i++) mw_integrated += mw_buffer[i];
	mw_integrated /= window_size;

	slope_buffer[slope_idx] = mw_integrated - last_mw;
	slope_idx = (slope_idx + 1) % 8;
	last_mw = mw_integrated;

	ecg_buffer[buf_index] = filtered;
	buf_index = (buf_index + 1) % 250;

	if (threshold == 0) {
		threshold = mw_integrated * 1.0 / 3.0;
		noise_level = mw_integrated * 1.0 / 2.0;
		signal_level = threshold;
	}

	unsigned long now = millis();
	bool is_peak = (mw_integrated >= threshold);

	if (is_peak && !found_peak) {
		float slope = 0;
		for (int i = 0; i < 8; i++) slope += slope_buffer[i];
		slope /= 8;

		if (slope > 0.5 * fabs(slope)) {
			if (now - last_peak_time > min_rr) {
				float rr = now - last_peak_time;
				last_peak_time = now;

				rr_intervals[rr_idx] = rr;
				rr_idx = (rr_idx + 1) % 8;

				float rr_mean = 0;
				for (int i = 0; i < 8; i++) rr_mean += rr_intervals[i];
				rr_mean /= 8;

				if (rr > 0) hr_bpm = 60000.0 / rr;

				signal_level = 0.125 * mw_integrated + 0.875 * signal_level;
			}
		} else {
			noise_level = 0.125 * mw_integrated + 0.875 * noise_level;
		}
		found_peak = true;
	} else if (!is_peak) {
		found_peak = false;
		noise_level = 0.125 * mw_integrated + 0.875 * noise_level;
	}

	threshold = noise_level + 0.25 * fabs(signal_level - noise_level);
	return hr_bpm;
}

/*
 * ===== computeRespRate =====
 * Estimates breathing rate (Respiratory Rate in breaths per minute) using ECG baseline wander.
 * Baseline shifts due to respiration are low-frequency components (~0.2–0.4 Hz).
 * This method applies a low-pass filter to extract this modulation and counts the peaks.
 */
float MAX30001G::computeRespRate() {
	int32_t ecg = readECGRaw();
	static float filtered = 0;
	filtered = 0.95 * filtered + 0.05 * ecg;

	resp_buffer[resp_index] = filtered;
	resp_index = (resp_index + 1) % RESP_BUFFER_SIZE;

	static unsigned long last_eval_time = 0;
	unsigned long now = millis();

	if (now - last_eval_time >= 5000) {
		last_eval_time = now;

		int peaks = 0;
		for (int i = 1; i < RESP_BUFFER_SIZE - 1; i++) {
			int prev = (i - 1 + RESP_BUFFER_SIZE) % RESP_BUFFER_SIZE;
			int next = (i + 1) % RESP_BUFFER_SIZE;

			if (resp_buffer[i] > resp_buffer[prev] &&
				resp_buffer[i] > resp_buffer[next] &&
				resp_buffer[i] > 1000) {
				peaks++;
			}
		}
		breath_rate_bpm = (peaks * 60.0) / 5.0;
	}

	return breath_rate_bpm;
}

/*
 * ===== computeHRV =====
 * Returns 3 HRV metrics based on last 32 RR intervals:
 * - RMSSD: short-term variability (parasympathetic)
 * - SDNN: overall HRV
 * - pNN50: percentage of RR pairs differing > 50ms
 */
HRVResult MAX30001G::computeHRV() {
	static float rr_intervals[32] = {1000};
	static int rr_index = 0;
	static unsigned long last_peak_time = 0;

	unsigned long now = millis();
	if (now - last_peak_time > 300 && now - last_peak_time < 2000) {
		float rr = now - last_peak_time;
		last_peak_time = now;
		rr_intervals[rr_index] = rr;
		rr_index = (rr_index + 1) % 32;
	}

	const int N = 32;
	float mean_rr = 0, sdnn = 0, rmssd = 0;
	int count_nn50 = 0;
	int valid_pairs = 0;

	for (int i = 0; i < N; i++) mean_rr += rr_intervals[i];
	mean_rr /= N;

	for (int i = 0; i < N; i++) {
		float diff = rr_intervals[i] - mean_rr;
		sdnn += diff * diff;
	}
	sdnn = sqrt(sdnn / N);

	for (int i = 1; i < N; i++) {
		float diff = rr_intervals[i] - rr_intervals[i - 1];
		rmssd += diff * diff;
		if (fabs(diff) > 50) count_nn50++;
		valid_pairs++;
	}
	rmssd = sqrt(rmssd / valid_pairs);
	float pnn50 = (float)count_nn50 / valid_pairs * 100.0;

	HRVResult result = { rmssd, sdnn, pnn50 };
	return result;
}

/*
 * ===== writeRegister =====
 * Sends a 24-bit value to a register address on the MAX30001G via SPI.
 */
void MAX30001G::writeRegister(uint8_t addr, uint32_t data) {
	digitalWrite(_csPin, LOW);
	SPI.transfer(addr & 0x7F);               // Write command
	SPI.transfer((data >> 16) & 0xFF);       // MSB
	SPI.transfer((data >> 8) & 0xFF);        // Mid
	SPI.transfer(data & 0xFF);               // LSB
	digitalWrite(_csPin, HIGH);
}

/*
 * ===== readRegister =====
 * Reads a 24-bit value from a register on the MAX30001G via SPI.
 */
uint32_t MAX30001G::readRegister(uint8_t addr) {
	digitalWrite(_csPin, LOW);
	SPI.transfer(0x80 | addr);               // Read command
	uint32_t value = SPI.transfer(0x00);
	value = (value << 8) | SPI.transfer(0x00);
	value = (value << 8) | SPI.transfer(0x00);
	digitalWrite(_csPin, HIGH);
	return value;
}

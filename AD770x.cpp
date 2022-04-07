/*
 * Benjamin Zhao
 * Modified for ESP8266 Module
 * Using SPI Library
 * version 1.0
 * Date: 2016-10
 * Description: (a) Using Arduino SPI Library
 *              (b) change constructor AD770X()
 *              (c) add funtion resetHard() by uisng RESET pin
 *              (d) replace spiTransfer() with SPI.transfer()
 *              (e) setup ESP8266 SPI to Mode2 & MSBFIRST & 1 MHz
 *
 * Orignal Source Code by:
 * AD7705/AD7706 Library
 * Kerry D. Wong
 * http://www.kerrywong.com
 * Initial version 1.0 3/2011
 * Updated 1.1 4/2012
 */

#include "AD770X.h"
#include "printutils.h"

//write communication register
//   7        6      5      4      3      2      1      0
//0/DRDY(0) RS2(0) RS1(0) RS0(0) R/W(0) STBY(0) CH1(0) CH0(0)
void AD770X::setNextOperation(byte reg, byte channel, byte readWrite) {
    byte r = 0;
    r = reg << 4 | readWrite << 3 | channel;

	AD770X_CS_LOW();
	SPI.transfer(r);
	AD770X_CS_HIGH();
}

//Clock Register - Read (Reading not vital for basic operation of lib)
//   7      6       5        4        3        2      1      0
//ZERO(0) ZERO(0) ZERO(0) CLKDIS(0) CLKDIV(0) CLK(1) FS1(0) FS0(1)
//
//CLKDIS: master clock disable bit
//CLKDIV: clock divider bit

byte AD770X::readClockRegister(byte channel) {
    setNextOperation(REG_CLOCK, channel, 1); // 1=read
    AD770X_CS_LOW();
    byte r = SPI.transfer(0x0);
    AD770X_CS_HIGH();
    Serial.print("Clock register [");
    Serial.print(channel);
    Serial.print("]: ");
    Serial.println(r);
   	return r;
}

//Clock Register - Write
//   7      6       5        4        3        2      1      0
//ZERO(0) ZERO(0) ZERO(0) CLKDIS(0) CLKDIV(0) CLK(1) FS1(0) FS0(1)
//
//CLKDIS: master clock disable bit
//CLKDIV: clock divider bit

void AD770X::writeClockRegister(byte CLKDIS, byte CLKDIV, byte outputUpdateRate) {
    byte r = CLKDIS << 4 | CLKDIV << 3 | outputUpdateRate;

    r &= ~(1 << 2); // clear CLK

    AD770X_CS_LOW();
    SPI.transfer(r);
    AD770X_CS_HIGH();
}

//Setup Register
//  7     6     5     4     3      2      1      0
//MD10) MD0(0) G2(0) G1(0) G0(0) B/U(0) BUF(0) FSYNC(1)

void AD770X::writeSetupRegister(byte operationMode, byte gain, byte unipolar, byte buffered, byte fsync) {
    byte r = operationMode << 6 | gain << 3 | unipolar << 2 | buffered << 1 | fsync;

    AD770X_CS_LOW();
    SPI.transfer(r);
    AD770X_CS_HIGH();
}

unsigned int AD770X::readADResult() {
    AD770X_CS_LOW();
    byte b1 = SPI.transfer(0x0);
    byte b2 = SPI.transfer(0x0);
    AD770X_CS_HIGH();

    unsigned int r = b1 << 8 | b2;

    return r;
}

unsigned int AD770X::readADResultRaw(byte channel) {
    while (!dataReady(channel)) { yield(); };
    setNextOperation(REG_DATA, channel, 1);

    return readADResult();
}

double AD770X::readADResult(byte channel, float refOffset) {
    return readADResultRaw(channel) * 1.0 / 65536.0 * VRef - refOffset;
}

/* The second method for determining when calibration is complete is to monitor
 * the MD1 and MD0 bits of the setup register. When these bits return to 0
 * following a calibration command, the calibration sequence is complete */
byte AD770X::readRegSetup(byte channel) {
	/* Table 14. Setup Register
	 * MD1 (0), MD0 (0), G2 (0), G1 (0), G0 (0), B/U (0), BUF (0), FSYNC (1), */
    setNextOperation(REG_SETUP, channel, 1);
    AD770X_CS_LOW();
    byte r = SPI.transfer(0x0);
    AD770X_CS_HIGH();
    return r;
}
bool AD770X::calDoneSetup(byte channel) {
	/* Calibration completion can be determined by the Data Ready
	 * bits, from the communications register, OR by this method,
	 * which checks the Setup Register's MD1 and MD0 bits, which
	 * will drop to 0 when calibration is done.
	 * The docs say this method informs us sooner that calibration
	 * is done, but that they don't indicate when data is actually
	 * ready */
	byte r = readRegSetup(channel);
    return ((r & (128 | 64)) == 0x0); // MD1=128, MD0=64
}

bool AD770X::dataReady(byte channel) {
    setNextOperation(REG_CMM, channel, 1);

    AD770X_CS_LOW();
    byte b1 = SPI.transfer(0x0);
    AD770X_CS_HIGH();

    return (b1 & 0x80) == 0x0;
}

void AD770X::reset() {
    AD770X_CS_LOW();
    for (int i = 0; i < 1000; i++){
        SPI.transfer(0xff);
    }
        
    AD770X_CS_HIGH();
}

/* Convenience object creation if both CS and RST are unneeded (unused) */
AD770X::AD770X(double vref, int _pinMOSI, int _pinMISO, int _pinSPIClock) {
	#if defined(AD770X_DISABLE_CS) && defined(AD770X_DISABLE_RST)
	#else
		#error "The 3-pin AD770X() method cannot be used unless AD770X_DISABLE_CS and AD770X_DISABLE_RST are both defined. Please use the full call even if only one of those pins are needed (we will just ignore the one you disable."
	#endif
	AD770X(vref, -1, _pinMOSI, _pinMISO, _pinSPIClock, -1);
}

AD770X::AD770X(double vref, int _pinCS, int _pinMOSI, int _pinMISO, int _pinSPIClock, int _pinRST) {
    VRef = vref;
    pinMOSI = _pinMOSI;
    pinMISO = _pinMISO;
    pinSPIClock = _pinSPIClock;
    pinCS = _pinCS;
    pinRST = _pinRST;
    
    pinMode(pinMOSI, OUTPUT);
    pinMode(pinMISO, INPUT);
    pinMode(pinSPIClock, OUTPUT);
	#ifndef AD770X_DISABLE_CS
    	pinMode(pinCS, OUTPUT);
	#endif
	#ifndef AD770X_DISABLE_RST
		pinMode(pinRST, OUTPUT);
	#endif

    SPI.begin();
	#ifdef AD770X_DISABLE_CS
		#warning "This may be wrong. Lib default was SPI_MODE2."
		SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
		/* SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3)); */
	#else
		#error "In testing this branch, we should not be here."
		SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
	#endif
    
    AD770X_CS_HIGH();
}

void AD770X::init(byte channel, byte clkDivider, byte polarity, byte gain, byte updRate) {
	spl("init(...)");
    setNextOperation(REG_CLOCK, channel, 0);
    writeClockRegister(0, clkDivider, updRate);

    setNextOperation(REG_SETUP, channel, 0);
    writeSetupRegister(MODE_SELF_CAL, gain, polarity, 0, 0);

	#ifdef INIT_FAST_CAL_RESULT // WIP: cal speed the same, but this method returns faster
	    int i;
	    i=0;
	    byte r;
	    do {
	    	r = readRegSetup(channel);
	    	Serial.print("!setup Reg Cal Result [");
	    	Serial.print(i);
	    	Serial.print("]: ");
	    	Serial.println(r);
	    	i++;
		} while ((r & (128 | 64)) != 0); // testing if calibration is done
	#else
		int i=0;
		while (!dataReady(channel)) {
			if (!(i%2000)) {
				Serial.print("!dataready [ch:");
				Serial.print(channel);
				Serial.print("]: ");
				Serial.println(i);
			}
			i++;
		};
	#endif
	spl("/init(...)");
}

void AD770X::init(byte channel) {
    init(channel, CLK_DIV_1, BIPOLAR, GAIN_1, UPDATE_RATE_25);
}

void AD770X::resetHard(){
	#ifndef AD770X_DISABLE_RST
		digitalWrite(pinRST, HIGH);
		delay(1);
		digitalWrite(pinRST, LOW);
		delay(2);
		digitalWrite(pinRST, HIGH);
		delay(1);
	#endif
}


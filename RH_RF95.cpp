// RH_RF22.cpp
//
// Copyright (C) 2011 Mike McCauley
// $Id: RH_RF95.cpp,v 1.8 2015/08/12 23:18:51 mikem Exp $

#include <RH_RF95.h>

// Interrupt vectors for the 3 Arduino interrupt pins
// Each interrupt can be handled by a different instance of RH_RF95, allowing you to have
// 2 or more LORAs per Arduino
RH_RF95* RH_RF95::_deviceForInterrupt[RH_RF95_NUM_INTERRUPTS] = {0, 0, 0};
uint8_t RH_RF95::_interruptCount = 0; // Index into _deviceForInterrupt for next device

// These are indexed by the values of ModemConfigChoice
// Stored in flash (program) memory to save SRAM
PROGMEM static const RH_RF95::ModemConfig MODEM_CONFIG_TABLE[] =
{
    //  1d,     1e,      26
    { 0x72,   0x74,    0x00}, // Bw125Cr45Sf128 (the chip default)
    { 0x92,   0x74,    0x00}, // Bw500Cr45Sf128
    { 0x48,   0x94,    0x00}, // Bw31_25Cr48Sf512
    { 0x78,   0xc4,    0x00}, // Bw125Cr48Sf4096
    
};

RH_RF95::RH_RF95(uint8_t slaveSelectPin, uint8_t interruptPin, RHGenericSPI& spi)
    :
    RHSPIDriver(slaveSelectPin, spi),
    _rxBufValid(0)
{
    _interruptPin = interruptPin;
    _myInterruptIndex = 0xff; // Not allocated yet
}

bool RH_RF95::init()
{
    if (!RHSPIDriver::init())
	return false;

    // Determine the interrupt number that corresponds to the interruptPin
    int interruptNumber = digitalPinToInterrupt(_interruptPin);
    if (interruptNumber == NOT_AN_INTERRUPT)
	return false;
#ifdef RH_ATTACHINTERRUPT_TAKES_PIN_NUMBER
    interruptNumber = _interruptPin;
#endif

    // No way to check the device type :-(
    
    // Set sleep mode, so we can also set LORA mode:
    spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE);
    delay(10); // Wait for sleep mode to take over from say, CAD
    // Check we are in sleep mode, with LORA set
    if (spiRead(RH_RF95_REG_01_OP_MODE) != (RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE))
    {
//	Serial.println(spiRead(RH_RF95_REG_01_OP_MODE), HEX);
	return false; // No device present?
    }

    // Add by Adrien van den Bossche <vandenbo@univ-tlse2.fr> for Teensy
    // ARM M4 requires the below. else pin interrupt doesn't work properly.
    // On all other platforms, its innocuous, belt and braces
    pinMode(_interruptPin, INPUT); 

    // Set up interrupt handler
    // Since there are a limited number of interrupt glue functions isr*() available,
    // we can only support a limited number of devices simultaneously
    // ON some devices, notably most Arduinos, the interrupt pin passed in is actuallt the 
    // interrupt number. You have to figure out the interruptnumber-to-interruptpin mapping
    // yourself based on knwledge of what Arduino board you are running on.
    if (_myInterruptIndex == 0xff)
    {
	// First run, no interrupt allocated yet
	if (_interruptCount <= RH_RF95_NUM_INTERRUPTS)
	    _myInterruptIndex = _interruptCount++;
	else
	    return false; // Too many devices, not enough interrupt vectors
    }
    _deviceForInterrupt[_myInterruptIndex] = this;
    if (_myInterruptIndex == 0)
	attachInterrupt(interruptNumber, isr0, RISING);
    else if (_myInterruptIndex == 1)
	attachInterrupt(interruptNumber, isr1, RISING);
    else if (_myInterruptIndex == 2)
	attachInterrupt(interruptNumber, isr2, RISING);
    else
	return false; // Too many devices, not enough interrupt vectors

    // Set up FIFO
    // We configure so that we can use the entire 256 byte FIFO for either receive
    // or transmit, but not both at the same time
    spiWrite(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0);
    spiWrite(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0);

    // Packet format is preamble + explicit-header + payload + crc
    // Explicit Header Mode
    // payload is TO + FROM + ID + FLAGS + message data
    // RX mode is implmented with RXCONTINUOUS
    // max message data length is 255 - 4 = 251 octets

    setModeIdle();

    // Set up default configuration
    // Sync Word is not supported by the chip in LORA mode.
//    setModemConfig(Bw125Cr45Sf128); // Radio default
    setModemConfig(Bw125Cr48Sf4096); // slow and reliable?
    setPreambleLength(8); // Default is 8
    // An innocuous ISM frequency, same as RF22's
    setFrequency(868.0);
    // Lowish power
	// 14 = 25mW
    setTxPower(14);
	// setTxPower(20);
	
	// RegOcp
	spiWrite(0x0B, 0x3B);
	
    return true;
}

// C++ level interrupt handler for this instance
// LORA is unusual in that it has several interrupt lines, and not a single, combined one.
// On MiniWirelessLoRa, only one of the several interrupt lines (DI0) from the RFM95 is usefuly 
// connnected to the processor.
// We use this to get RxDone and TxDone interrupts
void RH_RF95::handleInterrupt()
{
    // Read the interrupt register
	// Serial.println("debug|Interrupt from RFM!");
    uint8_t irq_flags = spiRead(RH_RF95_REG_12_IRQ_FLAGS);
	
    if (_mode == RHModeRx && irq_flags & (RH_RF95_RX_TIMEOUT | RH_RF95_PAYLOAD_CRC_ERROR)) {
		// Serial.print("debug|Bad packet received - irq_flags: ");
		// Serial.println(irq_flags, BIN);
	    _rxBad++;
		spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
		return;
		// after this point, we know that the data in the FIFO is corrupted
    }
	
	if (_mode == RHModeRx && irq_flags & RH_RF95_RX_DONE) {
	    // Have received a packet
        uint8_t len = spiRead(RH_RF95_REG_13_RX_NB_BYTES);
		
		// Serial.println("debug|RX interrupt - we got some data!");

	    // Reset the fifo read ptr to the beginning of the packet
	    spiWrite(RH_RF95_REG_0D_FIFO_ADDR_PTR, spiRead(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
	    spiBurstRead(RH_RF95_REG_00_FIFO, _buf, len);
	    _bufLen = len;

	    // Remember the RSSI of this packet
	    // this is according to the doc, but is it really correct?
	    // weakest receiveable signals are reported RSSI at about -94
	    _lastRssi = -137 + (uint8_t) spiRead(RH_RF95_REG_1A_PKT_RSSI_VALUE);

	    byte snr;
	    snr = spiRead(0x19);
		
	    if( snr & 0x80 ) {
			// The SNR sign bit is True
		    // 2's complement -> Invert and divide by 4
		    snr = ( ( ~snr + 1 ) & 0xFF ) >> 2;
            _lastSnr = -snr;
        } else {
		    // Divide by 4
		    // Serial.println(snr, BIN);
		    _lastSnr = ( snr & 0xFF ) >> 2;
	    }
	
	    // We have received a message.
	    validateRxBuf(); 
	    if (_rxBufValid) {
	        setModeIdle(); // Got one 
	    } else {
			// Serial.println("!!! What is this state?");
			// Serial.flush();
		}
    } else if (_mode == RHModeTx && irq_flags & RH_RF95_TX_DONE) {
		// Serial.println("debug|TX done interrupt caught - switching into IDLE mode");
	    _txGood++;
	    setModeIdle();
    } else {
		// Serial.println("Unknown interrupt handler state happened");
	}
    
    spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
}

int16_t RH_RF95::getRSSI() {
	int16_t rssi_mean = 0;
    uint8_t total = 10;
    for(uint8_t i = 0; i < total; i++) {
		// -137 is OFFSET_RSSI
        _RSSI = -137 + spiRead(0x1B);
        rssi_mean += _RSSI;         
    }
 
    rssi_mean = rssi_mean / total;  
    _RSSI = rssi_mean;

	//Serial.print(F("## Current RSSI value is "));
	//Serial.print(_RSSI, DEC);
	//Serial.println(F("dBm ##"));
	return _RSSI;
}

// These are low level functions that call the interrupt handler for the correct
// instance of RH_RF95.
// 3 interrupts allows us to have 3 different devices
void RH_RF95::isr0()
{
    if (_deviceForInterrupt[0])
	_deviceForInterrupt[0]->handleInterrupt();
}
void RH_RF95::isr1()
{
    if (_deviceForInterrupt[1])
	_deviceForInterrupt[1]->handleInterrupt();
}
void RH_RF95::isr2()
{
    if (_deviceForInterrupt[2])
	_deviceForInterrupt[2]->handleInterrupt();
}

// Check whether the latest received message is complete and uncorrupted
void RH_RF95::validateRxBuf()
{
	// skip this step in a polite way
    _rxGood++;
	_rxBufValid = true;
	return;
	
    if (_bufLen < 4)
	return; // Too short to be a real message

    // Extract the 4 headers
    //_rxHeaderTo    = _buf[0];
    //_rxHeaderFrom  = _buf[1];
    //_rxHeaderId    = _buf[2];
    //_rxHeaderFlags = _buf[3];
	
    if (_promiscuous || _rxHeaderTo == _thisAddress || _rxHeaderTo == RH_BROADCAST_ADDRESS) {
	    _rxGood++;
	    _rxBufValid = true;
    }
}

bool RH_RF95::available()
{
    if (_mode == RHModeTx) {
	    return false;
	}
    setModeRx();
    return _rxBufValid; // Will be set by the interrupt handler when a good message is received
}

void RH_RF95::clearRxBuf()
{
    ATOMIC_BLOCK_START;
    _rxBufValid = false;
    _bufLen = 0;
    ATOMIC_BLOCK_END;
}

bool RH_RF95::recv(uint8_t* buf, uint8_t* len)
{
    if (!available())
	return false;
    if (buf && len)
    {
	ATOMIC_BLOCK_START;
	// Skip the 4 headers that are at the beginning of the rxBuf
	if (*len > _bufLen-RH_RF95_HEADER_LEN)
	    *len = _bufLen-RH_RF95_HEADER_LEN;
	memcpy(buf, _buf+RH_RF95_HEADER_LEN, *len);
	ATOMIC_BLOCK_END;
    }
    clearRxBuf(); // This message accepted and cleared
    return true;
}

bool RH_RF95::send(const uint8_t* data, uint8_t len)
{
    if (len > RH_RF95_MAX_MESSAGE_LEN) {
	    return false;
	}

    waitPacketSent(); // Make sure we dont interrupt an outgoing message
    setModeIdle();

    // Position at the beginning of the FIFO
    spiWrite(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0);
    // The headers
    //spiWrite(RH_RF95_REG_00_FIFO, _txHeaderTo);
    //spiWrite(RH_RF95_REG_00_FIFO, _txHeaderFrom);
    //spiWrite(RH_RF95_REG_00_FIFO, _txHeaderId);
    //spiWrite(RH_RF95_REG_00_FIFO, _txHeaderFlags);
    // The message data
    spiBurstWrite(RH_RF95_REG_00_FIFO, data, len);
	//spiWrite(RH_RF95_REG_22_PAYLOAD_LENGTH, len + RH_RF95_HEADER_LEN);
    spiWrite(RH_RF95_REG_22_PAYLOAD_LENGTH, len);
	
    setModeTx(); // Start the transmitter
    // when Tx is done, interruptHandler will fire and radio mode will return to STANDBY
    return true;
}

bool RH_RF95::printRegisters()
{
    uint8_t i;
    for (i = 0; i < 67; i++)
    {
	Serial.print(i, HEX);
	Serial.print(": ");
	Serial.print(spiRead(i), HEX);
	Serial.print(": ");
	Serial.println(spiRead(i), BIN);
    }
    return true;
}

uint8_t RH_RF95::maxMessageLength()
{
    return RH_RF95_MAX_MESSAGE_LEN;
}

bool RH_RF95::setFrequency(float centre)
{
    // Frf = FRF / FSTEP
    uint32_t frf = (centre * 1000000.0) / RH_RF95_FSTEP;
    spiWrite(RH_RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff);
    spiWrite(RH_RF95_REG_07_FRF_MID, (frf >> 8) & 0xff);
    spiWrite(RH_RF95_REG_08_FRF_LSB, frf & 0xff);

    return true;
}

bool RH_RF95::setSF(int chips)
{
	uint8_t reg_new, reg_old;
	
	reg_old = spiRead(RH_RF95_REG_1E_MODEM_CONFIG2);
    reg_new = reg_old;
	
	// SF 7 -> 128 chips / symbol
	if ((chips == 128) || (chips == 7)) {
	    reg_new = reg_old << 4;
	    reg_new = reg_new >> 4;
		reg_new = reg_new | 0x70;
	}

	if ((chips == 1024) || (chips == 10)) {
	    reg_new = reg_old << 4;
	    reg_new = reg_new >> 4;
		reg_new = reg_new | 0xA0;
	}
	
	spiWrite(RH_RF95_REG_1E_MODEM_CONFIG2, reg_new);

    return true;
}

// return spreading number (not in chips / symbol, but value from datasheet)
uint8_t RH_RF95::getSF()
{
	uint8_t sf_rate;
	uint8_t reg_val;
	
	reg_val = spiRead(RH_RF95_REG_1E_MODEM_CONFIG2);
    sf_rate = reg_val >> 4;
	
    return sf_rate;
}

bool RH_RF95::setCR(int rate)
{
	uint8_t reg_new, reg_old;
	
	reg_old = spiRead(RH_RF95_REG_1D_MODEM_CONFIG1);
    reg_new = reg_old;
	
	// CR 7 -> 4/7 etc..
	
	if (rate == 5) {
		reg_new = reg_old & 0xF1;
		reg_new = reg_new | 0x02;
	}

	if (rate == 6) {
		reg_new = reg_old & 0xF1;
		reg_new = reg_new | 0x04;
	}
	
	if (rate == 7) {
		reg_new = reg_old & 0xF1;
		reg_new = reg_new | 0x06;
	}

	
    if (rate == 8) {
		reg_new = reg_old & 0xF1;
		reg_new = reg_new | 0x08;
	}

	
	spiWrite(RH_RF95_REG_1D_MODEM_CONFIG1, reg_new);

    return true;
}

uint8_t RH_RF95::getCR()
{
	uint8_t cr_rate;
	uint8_t reg_val;
	
	reg_val = spiRead(RH_RF95_REG_1D_MODEM_CONFIG1);
    cr_rate = ( reg_val >> 1 ) & 0b00000111;
	
    return cr_rate + 4;
}

bool RH_RF95::setBandWidth(float khz)
{
	uint8_t reg_new, reg_old;
	
	reg_old = spiRead(RH_RF95_REG_1D_MODEM_CONFIG1);
    reg_new = reg_old;
	
	if (khz == 7.8) {
	    // set BandWidth to 7.8KHz (clear bits 4-7) register 1D
	    reg_new = reg_old << 4;
	    reg_new = reg_new >> 4;
	}
	
	// 20.8KHz
	if (khz == 20.8) {
	    // set BandWidth to 20.8KHz
		reg_new = reg_old << 4;
		reg_new = reg_new >> 4;
	    reg_new = reg_new | 0x30;
	}

	// 41.7KHz
	if (khz == 41.7) {
		reg_new = reg_old << 4;
		reg_new = reg_new >> 4;
	    reg_new = reg_new | 0x50;
	}
	
	if (khz == 62.5) {
	    // set BandWidth to 62.5KHz
		reg_new = reg_old << 4;
		reg_new = reg_new >> 4;
	    reg_new = reg_new | 0x60;
	}
	
	if (khz == 125) {
	    // set BandWidth to 125KHz
	    reg_new = ((reg_old << 1) & 0xFF) >> 1;
	}
	
	spiWrite(RH_RF95_REG_1D_MODEM_CONFIG1, reg_new);

    return true;
}

void RH_RF95::setModeIdle()
{
    if (_mode != RHModeIdle)
    {
	spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY);
	_mode = RHModeIdle;
    }
}

bool RH_RF95::sleep()
{
    if (spiRead(RH_RF95_REG_01_OP_MODE) != (RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE)) {
	    // Serial.println("RFM goes to sleep.");
	    spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE);
		delay(5);
		if (spiRead(RH_RF95_REG_01_OP_MODE) != (RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE)) {
			// unable to set the sleep mode
			return false;
		} else {
			// sleep mode entered
	        _mode = RHModeSleep;
		    return true;
		}
    } else {
		// we are already in sleep mode
		_mode = RHModeSleep;
		return true;
	}
}

void RH_RF95::setModeRx()
{
    if (_mode != RHModeRx) {
	    spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_RXCONTINUOUS);
	    spiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x00); // Interrupt on RxDone
	    _mode = RHModeRx;
    }
}

void RH_RF95::setModeTx()
{
    if (_mode != RHModeTx) {
		spiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone
	    spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_TX);
	    _mode = RHModeTx;
    }
}

void RH_RF95::setTxPower(int8_t power)
{
    if (power > 23)
	power = 23;
    if (power < 5)
	power = 5;

    // For RH_RF95_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
    // RH_RF95_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will us it
    // for 21, 22 and 23dBm
    if (power > 20)
    {
	spiWrite(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_ENABLE);
	power -= 3;
    }
    else
    {
	spiWrite(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_DISABLE);
    }

    // RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
    // pin is connected, so must use PA_BOOST
    // Pout = 2 + OutputPower.
    // The documentation is pretty confusing on this topic: PaSelect says the max power is 20dBm,
    // but OutputPower claims it would be 17dBm.
    // My measurements show 20dBm is correct
    spiWrite(RH_RF95_REG_09_PA_CONFIG, RH_RF95_PA_SELECT | (power-5));
    //Serial.print("TX power set to ");
	//Serial.print(power);
	//Serial.println("dBm");
}

// Sets registers from a canned modem configuration structure
void RH_RF95::setModemRegisters(const ModemConfig* config)
{
    spiWrite(RH_RF95_REG_1D_MODEM_CONFIG1,       config->reg_1d);
    spiWrite(RH_RF95_REG_1E_MODEM_CONFIG2,       config->reg_1e);
    spiWrite(RH_RF95_REG_26_MODEM_CONFIG3,       config->reg_26);
}

// Set one of the canned FSK Modem configs
// Returns true if its a valid choice
bool RH_RF95::setModemConfig(ModemConfigChoice index)
{
    if (index > (signed int)(sizeof(MODEM_CONFIG_TABLE) / sizeof(ModemConfig)))
        return false;

    ModemConfig cfg;
    memcpy_P(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(RH_RF95::ModemConfig));
    setModemRegisters(&cfg);

    return true;
}

void RH_RF95::setPreambleLength(uint16_t bytes)
{
    spiWrite(RH_RF95_REG_20_PREAMBLE_MSB, bytes >> 8);
    spiWrite(RH_RF95_REG_21_PREAMBLE_LSB, bytes & 0xff);
}


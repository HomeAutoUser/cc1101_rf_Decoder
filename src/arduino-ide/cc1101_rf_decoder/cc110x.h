struct Data {
  const uint8_t* reg_val;
  byte length;
  const char* name;
  byte packet_length;
};


const uint8_t Config_Default_433[] PROGMEM = {
  /*
    Address Config = No address check
    Base Frequency = 433.919830
    CRC Autoflush = false
    CRC Enable = false
    Carrier Frequency = 433.919830
    Channel Number = 0
    Channel Spacing = 349.914551
    Data Format = Asynchronous serial mode
    Data Rate = 5.60379
    Deviation = 1.586914
    Device Address = 0
    Manchester Enable = false
    Modulation Format = ASK/OOK
    PA Ramping = false
    Packet Length = 61
    Packet Length Mode = Infinite packet length mode
    Preamble Count = 4
    RX Filter BW = 325.000000
    Sync Word Qualifier Mode = No preamble/sync
    TX Power = unknown
    Whitening = false
  */

  0x0D,  // IOCFG2              GDO2 Output Pin Configuration
  0x2E,  // IOCFG1              GDO1 Output Pin Configuration
  0x2D,  // IOCFG0              GDO0 Output Pin Configuration
  //  0x07,  // FIFOTHR             RX FIFO and TX FIFO Thresholds
  0x47,  // FIFOTHR             RX FIFO and TX FIFO Thresholds
  0xD3,  // SYNC1               Sync Word, High Byte
  0x91,  // SYNC0               Sync Word, Low Byte
  0x3D,  // PKTLEN              Packet Length
  0x04,  // PKTCTRL1            Packet Automation Control
  0x32,  // PKTCTRL0            Packet Automation Control
  0x00,  // ADDR                Device Address
  0x00,  // CHANNR              Channel Number
  0x06,  // FSCTRL1             Frequency Synthesizer Control
  0x00,  // FSCTRL0             Frequency Synthesizer Control
  0x10,  // FREQ2               Frequency Control Word, High Byte
  0xB0,  // FREQ1               Frequency Control Word, Middle Byte
  0x71,  // FREQ0               Frequency Control Word, Low Byte
  0x57,  // MDMCFG4             Modem Configuration
  0xC4,  // MDMCFG3             Modem Configuration
  0x30,  // MDMCFG2             Modem Configuration
  0x23,  // MDMCFG1             Modem Configuration
  0xB9,  // MDMCFG0             Modem Configuration
  0x00,  // DEVIATN             Modem Deviation Setting
  0x07,  // MCSM2               Main Radio Control State Machine Configuration
  0x00,  // MCSM1               Main Radio Control State Machine Configuration
  0x18,  // MCSM0               Main Radio Control State Machine Configuration
  0x14,  // FOCCFG              Frequency Offset Compensation Configuration
  0x6C,  // BSCFG               Bit Synchronization Configuration
  //  0x00,  // AGCCTRL2            AGC Control // 24 dB
  0x07,  // AGCCTRL2            AGC Control // 42 dB
  0x00,  // AGCCTRL1            AGC Control
  0x91,  // AGCCTRL0            AGC Control
  0x87,  // WOREVT1             High Byte Event0 Timeout
  0x6B,  // WOREVT0             Low Byte Event0 Timeout
  0xF8,  // WORCTRL             Wake On Radio Control
  0xB6,  // FREND1              Front End RX Configuration
  0x11,  // FREND0              Front End TX Configuration
  0xE9,  // FSCAL3              Frequency Synthesizer Calibration
  0x2A,  // FSCAL2              Frequency Synthesizer Calibration
  0x00,  // FSCAL1              Frequency Synthesizer Calibration
  0x1F,  // FSCAL0              Frequency Synthesizer Calibration
  0x41,  // RCCTRL1             RC Oscillator Configuration
  0x00,  // RCCTRL0             RC Oscillator Configuration
  0x59,  // FSTEST              Frequency Synthesizer Calibration Control
  0x7F,  // PTEST               Production Test
  0x3F,  // AGCTEST             AGC Test
  0x88,  // TEST2               Various Test Settings
  0x31,  // TEST1               Various Test Settings
  0x09,  // TEST0               Various Test Settings
};

const uint8_t Config_Default_868[] PROGMEM = {
  /*
    Address Config = No address check
    Base Frequency = 868.349854
    CRC Autoflush = false
    CRC Enable = false
    Carrier Frequency = 868.349854
    Channel Number = 0
    Channel Spacing = 349.914551
    Data Format = Asynchronous serial mode
    Data Rate = 5.60379
    Deviation = 1.586914
    Device Address = 0
    Manchester Enable = false
    Modulation Format = ASK/OOK
    PA Ramping = false
    Packet Length = 61
    Packet Length Mode = Infinite packet length mode
    Preamble Count = 4
    RX Filter BW = 325.000000
    Sync Word Qualifier Mode = No preamble/sync
    TX Power = 0
    Whitening = false
  */

  0x0D,  // IOCFG2              GDO2 Output Pin Configuration
  0x2E,  // IOCFG1              GDO1 Output Pin Configuration
  0x2D,  // IOCFG0              GDO0 Output Pin Configuration
  0x47,  // FIFOTHR             RX FIFO and TX FIFO Thresholds
  //  0x07,  // FIFOTHR             RX FIFO and TX FIFO Thresholds
  0xD3,  // SYNC1               Sync Word, High Byte
  0x91,  // SYNC0               Sync Word, Low Byte
  0x3D,  // PKTLEN              Packet Length
  0x04,  // PKTCTRL1            Packet Automation Control
  0x32,  // PKTCTRL0            Packet Automation Control
  0x00,  // ADDR                Device Address
  0x00,  // CHANNR              Channel Number
  0x06,  // FSCTRL1             Frequency Synthesizer Control
  0x00,  // FSCTRL0             Frequency Synthesizer Control
  0x21,  // FREQ2               Frequency Control Word, High Byte
  0x65,  // FREQ1               Frequency Control Word, Middle Byte
  0xE8,  // FREQ0               Frequency Control Word, Low Byte
  0x57,  // MDMCFG4             Modem Configuration
  0xC4,  // MDMCFG3             Modem Configuration
  0x30,  // MDMCFG2             Modem Configuration
  0x23,  // MDMCFG1             Modem Configuration
  0xB9,  // MDMCFG0             Modem Configuration
  0x00,  // DEVIATN             Modem Deviation Setting
  0x07,  // MCSM2               Main Radio Control State Machine Configuration
  0x00,  // MCSM1               Main Radio Control State Machine Configuration
  0x18,  // MCSM0               Main Radio Control State Machine Configuration
  0x14,  // FOCCFG              Frequency Offset Compensation Configuration
  0x6C,  // BSCFG               Bit Synchronization Configuration
  //  0x00,  // AGCCTRL2            AGC Control // 24 dB
  0x07,  // AGCCTRL2            AGC Control // 42 dB
  0x00,  // AGCCTRL1            AGC Control
  0x91,  // AGCCTRL0            AGC Control
  0x87,  // WOREVT1             High Byte Event0 Timeout
  0x6B,  // WOREVT0             Low Byte Event0 Timeout
  0xF8,  // WORCTRL             Wake On Radio Control
  0xB6,  // FREND1              Front End RX Configuration
  0x11,  // FREND0              Front End TX Configuration
  0xE9,  // FSCAL3              Frequency Synthesizer Calibration
  0x2A,  // FSCAL2              Frequency Synthesizer Calibration
  0x00,  // FSCAL1              Frequency Synthesizer Calibration
  0x1F,  // FSCAL0              Frequency Synthesizer Calibration
  0x41,  // RCCTRL1             RC Oscillator Configuration
  0x00,  // RCCTRL0             RC Oscillator Configuration
  0x59,  // FSTEST              Frequency Synthesizer Calibration Control
  0x7F,  // PTEST               Production Test
  0x3F,  // AGCTEST             AGC Test
  0x88,  // TEST2               Various Test Settings
  0x31,  // TEST1               Various Test Settings
  0x09,  // TEST0               Various Test Settings
};


struct Data Registers[] = {
  { Config_Default_433, sizeof(Config_Default_433) / sizeof(Config_Default_433[0]), "CC110x Factory 433 Default", 32  },
  { Config_Default_868, sizeof(Config_Default_868) / sizeof(Config_Default_868[0]), "CC110x Factory 868 Default", 32  },
};


uint8_t CC1101_cmdStrobe(byte cmd) {
  /* Send command strobe to the CC1101 IC via SPI
     'cmd'  Command strobe  */

  digitalWrite(SS, LOW);              // Select (SPI) CC1101
  while (digitalRead(MISO) > 0);      // Wait until SPI MISO line goes low
  uint8_t ret = SPI.transfer(cmd);    // Send strobe command
  digitalWrite(SS, HIGH);             // Deselect (SPI) CC1101
  return ret;                         // Chip Status Byte
}

uint8_t CC1101_readReg(uint8_t regAddr, uint8_t regType) {
  /*
    Read CC1101 register via SPI

    'regAddr'  Register address
    'regType'  Type of register: CC1101_CONFIG_REGISTER or CC1101_STATUS_REGISTER

    Return: Data byte returned by the CC1101 IC
  */
  byte addr, val;

  addr = regAddr | regType;
  digitalWrite(SS, LOW);          // Select (SPI) CC1101
  while (digitalRead(MISO) > 0);  // Wait until SPI MISO line goes low
  SPI.transfer(addr);             // Send register address
  val = SPI.transfer(0x00);       // Read result
  digitalWrite(SS, HIGH);         // Deselect (SPI) CC1101
  return val;
}


void CC1101_writeReg(uint8_t regAddr, uint8_t value) {
  /*
    Write single register into the CC1101 IC via SPI

    'regAddr' Register address
    'value'   Value to be writen
  */
  digitalWrite(SS, LOW);          // Select (SPI) CC1101
  while (digitalRead(MISO) > 0);  // Wait until SPI MISO line goes low
  SPI.transfer(regAddr);          // Send register address
  SPI.transfer(value);            // Send value
  digitalWrite(SS, HIGH);         // Deselect (SPI) CC1101
}


void CC1101_writeRegFor(const uint8_t *reg_name, uint8_t reg_length) {
  /*
    variable loop to write register

    'reg_name'    Register name from register.cpp
    'reg_length'  Register name length from register.cpp
    'reg_modus'   Text for Modus from simplification.h
  */

  CC1101_writeReg(0x0C, 0);       // 0x0C: FSCTRL0 – Frequency Synthesizer Control
  for (byte i = 0; i < reg_length; i++) {
    CC1101_writeReg(i, pgm_read_byte_near(reg_name + i)); /* write value to cc110x */
  }
}

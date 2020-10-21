class Roomba {
    Stream& _stream;
    const unsigned long _timeouttime;
    boolean _timeouterror;
    byte _motors;
  public:
    Roomba(Stream& stream, unsigned long timeouttime)
      : _stream(stream), _timeouttime(timeouttime), _timeouterror(false), _motors(0)
    {
    }
    boolean isTimeoutError() const {
      return _timeouterror;
    }
    void start() {
      _stream.write(128); // Start
    }
    void reset() {
      _stream.write(7); // Reset
    }
    void stop() {
      _stream.write(173); // Stop
    }
    void safe() {
      _stream.write(131); // Safe
    }
    void full() {
      _stream.write(132); // Full
    }
    void clean() {
      _stream.write(135); // Clean
    }
    void cleanMax() {
      _stream.write(136); // Max
    }
    void spot() {
      _stream.write(134); // Spot
    }
    void seekDock() {
      _stream.write(143); // Seek Dock
    }
    void power() {
      _stream.write(133); // Power
    }
    void sideBrushOn() {
      _stream.write(138); // Motor
      _motors |= 0x01;
      _stream.write(_motors);
    }
    void sideBrushOff() {
      _stream.write(138); // Motor
      _motors &= ~0x01;
      _stream.write(_motors);
    }
    void mainBrushOn() {
      _stream.write(138); // Motor
      _motors |= 0x04;
      _stream.write(_motors);
    }
    void mainBrushOff() {
      _stream.write(138); // Motor
      _motors &= ~0x04;
      _stream.write(_motors);
    }
    void vacuumOn() {
      _stream.write(138); // Motor
      _motors |= 0x02;
      _stream.write(_motors);
    }
    void vacuumOff() {
      _stream.write(138); // Motor
      _motors &= ~0x02;
      _stream.write(_motors);
    }
    void driveDirect(int16_t left, int16_t right) {
      _stream.write(145); // Drive Direct
      _stream.write((right >> 8) & 0xff); // Right High
      _stream.write(right & 0xff); // Right Low
      _stream.write((left >> 8) & 0xff); // Left High
      _stream.write(left & 0xff); // Left Low
    }
    struct LightBumperSensors {
      uint16_t left;
      uint16_t frontleft;
      uint16_t centerleft;
      uint16_t centerright;
      uint16_t frontright;
      uint16_t right;
    };
    LightBumperSensors getLightBumperSensors() {
      _timeouterror = false;
      _stream.write(142); // Sensor
      _stream.write(106); // Group 106(46-51)
      LightBumperSensors sensors;
      sensors.left = readUINT16();
      sensors.frontleft = readUINT16();
      sensors.centerleft = readUINT16();
      sensors.centerright = readUINT16();
      sensors.frontright = readUINT16();
      sensors.right = readUINT16();
      return sensors;
    }
    byte getBumper() {
      _timeouterror = false;
      _stream.write(142); // Sensor
      _stream.write(45); // Light Bumper
      return readUINT8();
    }
    enum OIMode {
      OI_OFF = 0,
      OI_PASSIVE = 1,
      OI_SAFE = 2,
      OI_FULL = 3,
    };
    byte getOIMode() {
      _timeouterror = false;
      _stream.write(142); // Sensor
      _stream.write(35); // OI Mode
      return readUINT8();
    }
    unsigned int getWallSignal() {
      _timeouterror = false;
      _stream.write(142); // Sensor
      _stream.write(27); // Wall Signal
      return readUINT16();
    }
    enum BumpsAndWheelsBitMask {
      BUMP_R_BIT = 0x01,
      BUMP_L_BIT = 0x02,
      BUMP_MASK = BUMP_R_BIT | BUMP_L_BIT,
      DROP_R_BIT = 0x04,
      DROP_L_BIT = 0x08,
      DROP_MASK = DROP_R_BIT | DROP_L_BIT,
    };
    byte getBumpsAndWheelDrops() {
      _timeouterror = false;
      _stream.write(142); // Sensor
      _stream.write(7); // Bumps and Wheel Drops
      return readUINT8();
    }
    enum ChargingState {
      BATTERY_NOT_CHARGING = 0,
      BATTERY_RECONDITIONING_CHARGING = 1,
      BATTERY_FULL_CHARGING = 2,
      BATTERY_TRICLE_CHARGING = 3,
      BATTERY_WAITING = 4,
      BATTERY_CHARGING_FAULT_CONDITION = 5,
    };
    ChargingState getChargingState() {
      _timeouterror = false;
      _stream.write(142); // Sensor
      _stream.write(21); // Charging State
      return (ChargingState)readUINT8();
    }
    uint16_t getVoltage() {
      _timeouterror = false;
      _stream.write(142); // Sensor
      _stream.write(22); // Voltage
      return readUINT16();
    }
    uint16_t getCurrent() {
      _timeouterror = false;
      _stream.write(142); // Sensor
      _stream.write(23); // Current
      return readUINT16();
    }
    boolean getChargerAvailable() {
      _timeouterror = false;
      _stream.write(142); // Sensor
      _stream.write(34); // Charging Sources Available
      return readUINT8() != 0;
    }
  private:
    uint16_t readUINT16() {
      uint16_t h = readUINT8();
      uint16_t l = readUINT8();
      return (h << 8) | l;
    }
    uint8_t readUINT8() {
      if (_timeouterror) {
        return 0;
      }
      unsigned long t0 = millis();
      while (_stream.available() <= 0) {
        if (millis() - t0 >= _timeouttime) {
          _timeouterror = true;
          return 0;
        }
      }
      return _stream.read();
    }
};

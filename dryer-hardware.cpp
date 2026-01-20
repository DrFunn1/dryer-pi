#include "dryer-hardware.h"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <termios.h>
#include <chrono>
#include <thread>
#include <pigpio.h>

// ADS1115 Register addresses
#define ADS1115_REG_CONVERSION  0x00
#define ADS1115_REG_CONFIG      0x01

// ADS1115 Config register bits
#define ADS1115_OS_SINGLE       0x8000  // Start single conversion
#define ADS1115_MUX_AIN0        0x4000  // Single-ended AIN0
#define ADS1115_MUX_AIN1        0x5000  // Single-ended AIN1
#define ADS1115_MUX_AIN2        0x6000  // Single-ended AIN2
#define ADS1115_MUX_AIN3        0x7000  // Single-ended AIN3
#define ADS1115_PGA_4_096V      0x0200  // +/-4.096V range
#define ADS1115_MODE_SINGLE     0x0100  // Single-shot mode
#define ADS1115_DR_128SPS       0x0080  // 128 samples per second

DryerHardware::DryerHardware() 
    : i2cHandle(-1)
    , uartHandle(-1)
    , initialized(false)
    , ads1115Available(false)
    , midiAvailable(false)
{
    trigger1State.active = false;
    trigger2State.active = false;
}

DryerHardware::~DryerHardware() {
    shutdown();
}

bool DryerHardware::initialize() {
    std::cout << "Initializing Dryer hardware..." << std::endl;
    
    // Initialize pigpio library
    if (gpioInitialise() < 0) {
        std::cerr << "ERROR: Failed to initialize pigpio" << std::endl;
        return false;
    }
    
    // Initialize components
    bool gpioOk = initGPIO();
    bool adsOk = initADS1115();
    bool midiOk = initMIDI();
    
    if (!gpioOk) {
        std::cerr << "WARNING: GPIO initialization failed" << std::endl;
    }
    
    if (!adsOk) {
        std::cerr << "WARNING: ADS1115 not found, using default parameters" << std::endl;
    }
    
    if (!midiOk) {
        std::cerr << "WARNING: MIDI UART not available" << std::endl;
    }
    
    initialized = true;
    std::cout << "Hardware initialization complete" << std::endl;
    std::cout << "  GPIO: " << (gpioOk ? "OK" : "FAILED") << std::endl;
    std::cout << "  ADS1115: " << (adsOk ? "OK" : "NOT FOUND") << std::endl;
    std::cout << "  MIDI: " << (midiOk ? "OK" : "NOT AVAILABLE") << std::endl;
    
    return gpioOk;  // GPIO is minimum requirement
}

void DryerHardware::shutdown() {
    if (!initialized) return;
    
    // Close I2C
    if (i2cHandle >= 0) {
        close(i2cHandle);
        i2cHandle = -1;
    }
    
    // Close UART
    if (uartHandle >= 0) {
        close(uartHandle);
        uartHandle = -1;
    }
    
    // Cleanup GPIO
    gpioTerminate();
    
    initialized = false;
    std::cout << "Hardware shutdown complete" << std::endl;
}

bool DryerHardware::initGPIO() {
    // Configure input pins (switches)
    gpioSetMode(GPIO_BALL_TYPE, PI_INPUT);
    gpioSetMode(GPIO_LINT_TRAP, PI_INPUT);
    gpioSetMode(GPIO_MOON_GRAVITY, PI_INPUT);
    
    // Enable pull-down resistors (switch connects to 3.3V)
    gpioSetPullUpDown(GPIO_BALL_TYPE, PI_PUD_DOWN);
    gpioSetPullUpDown(GPIO_LINT_TRAP, PI_PUD_DOWN);
    gpioSetPullUpDown(GPIO_MOON_GRAVITY, PI_PUD_DOWN);
    
    // Configure output pins (triggers)
    gpioSetMode(GPIO_TRIGGER_OUT_1, PI_OUTPUT);
    gpioSetMode(GPIO_TRIGGER_OUT_2, PI_OUTPUT);
    gpioWrite(GPIO_TRIGGER_OUT_1, 0);
    gpioWrite(GPIO_TRIGGER_OUT_2, 0);
    
    return true;
}

bool DryerHardware::initADS1115() {
    // Open I2C bus
    i2cHandle = open("/dev/i2c-1", O_RDWR);
    if (i2cHandle < 0) {
        return false;
    }
    
    // Set I2C slave address
    if (ioctl(i2cHandle, I2C_SLAVE, ADS1115_ADDRESS) < 0) {
        close(i2cHandle);
        i2cHandle = -1;
        return false;
    }
    
    ads1115Available = true;
    return true;
}

bool DryerHardware::initMIDI() {
    // Open UART device
    uartHandle = open(UART_DEVICE, O_WRONLY | O_NOCTTY);
    if (uartHandle < 0) {
        return false;
    }
    
    // Configure UART for MIDI (31.25 kbaud, 8N1)
    struct termios tty;
    if (tcgetattr(uartHandle, &tty) != 0) {
        close(uartHandle);
        uartHandle = -1;
        return false;
    }
    
    // Set baud rate to 31250 (MIDI standard)
    // Note: 38400 baud is often used as it's closest standard rate
    // For true 31.25k, you may need custom clock divisor
    cfsetospeed(&tty, B38400);
    
    // 8N1 mode
    tty.c_cflag &= ~PARENB;         // No parity
    tty.c_cflag &= ~CSTOPB;         // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;             // 8 bits
    tty.c_cflag &= ~CRTSCTS;        // No hardware flow control
    tty.c_cflag |= CLOCAL | CWRITE; // Ignore modem controls, enable writing
    
    // Raw mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    
    if (tcsetattr(uartHandle, TCSANOW, &tty) != 0) {
        close(uartHandle);
        uartHandle = -1;
        return false;
    }
    
    midiAvailable = true;
    return true;
}

uint16_t DryerHardware::readADC(uint8_t channel) {
    if (!ads1115Available) {
        return ADC_MAX_VALUE / 2;  // Return mid-scale
    }
    
    // Select channel
    uint16_t config = ADS1115_OS_SINGLE |      // Start conversion
                     ADS1115_PGA_4_096V |       // ±4.096V range
                     ADS1115_MODE_SINGLE |      // Single-shot
                     ADS1115_DR_128SPS;         // 128 SPS
    
    // Set MUX based on channel
    switch(channel) {
        case 0: config |= ADS1115_MUX_AIN0; break;
        case 1: config |= ADS1115_MUX_AIN1; break;
        case 2: config |= ADS1115_MUX_AIN2; break;
        case 3: config |= ADS1115_MUX_AIN3; break;
        default: return 0;
    }
    
    // Write config register
    uint8_t writeBuffer[3] = {
        ADS1115_REG_CONFIG,
        static_cast<uint8_t>(config >> 8),
        static_cast<uint8_t>(config & 0xFF)
    };
    
    if (write(i2cHandle, writeBuffer, 3) != 3) {
        return 0;
    }
    
    // Wait for conversion (max 8ms @ 128 SPS)
    usleep(10000);  // 10ms
    
    // Read conversion register
    uint8_t reg = ADS1115_REG_CONVERSION;
    if (write(i2cHandle, &reg, 1) != 1) {
        return 0;
    }
    
    uint8_t readBuffer[2];
    if (read(i2cHandle, readBuffer, 2) != 2) {
        return 0;
    }
    
    uint16_t value = (readBuffer[0] << 8) | readBuffer[1];
    
    // Convert to positive range (ADS1115 is signed)
    // For 0-3.3V input with 4.096V reference, we want 0-26400 range
    if (value > 32768) value = 0;  // Clip negative values
    
    return value;
}

bool DryerHardware::readGPIO(int pin) {
    return gpioRead(pin) == 1;
}

void DryerHardware::writeGPIO(int pin, bool value) {
    gpioWrite(pin, value ? 1 : 0);
}

HardwareParameters DryerHardware::readParameters() {
    HardwareParameters params;
    
    // Read ADC values and map to parameter ranges
    uint16_t rpmADC = readADC(ADC_CHAN_RPM);
    uint16_t drumADC = readADC(ADC_CHAN_DRUM_SIZE);
    uint16_t vanesADC = readADC(ADC_CHAN_VANES);
    uint16_t heightADC = readADC(ADC_CHAN_VANE_HEIGHT);
    
    params.rpm = mapADCToRange(rpmADC, ParamRanges::RPM_MIN, ParamRanges::RPM_MAX);
    params.drumSize = mapADCToRange(drumADC, ParamRanges::DRUM_SIZE_MIN, ParamRanges::DRUM_SIZE_MAX);
    
    // Vanes need to be integer
    float vanesFloat = mapADCToRange(vanesADC, ParamRanges::VANES_MIN, ParamRanges::VANES_MAX);
    params.vanes = static_cast<int>(vanesFloat + 0.5f);  // Round to nearest
    
    params.vaneHeight = mapADCToRange(heightADC, ParamRanges::VANE_HEIGHT_MIN, ParamRanges::VANE_HEIGHT_MAX);
    
    // Read switches
    params.ballTypeBalloon = readGPIO(GPIO_BALL_TYPE);
    params.lintTrapEnabled = readGPIO(GPIO_LINT_TRAP);
    params.moonGravityEnabled = readGPIO(GPIO_MOON_GRAVITY);
    
    return params;
}

void DryerHardware::sendMIDIByte(uint8_t byte) {
    if (midiAvailable && uartHandle >= 0) {
        write(uartHandle, &byte, 1);
    }
}

void DryerHardware::sendMIDINoteOn(uint8_t noteNumber, uint8_t velocity, uint8_t channel) {
    if (!midiAvailable) return;
    
    uint8_t statusByte = 0x90 | (channel & 0x0F);  // Note On
    sendMIDIByte(statusByte);
    sendMIDIByte(noteNumber & 0x7F);
    sendMIDIByte(velocity & 0x7F);
    
    // Debug output
    // std::cout << "MIDI: Note On " << (int)noteNumber << " vel=" << (int)velocity << std::endl;
}

void DryerHardware::sendMIDINoteOff(uint8_t noteNumber, uint8_t channel) {
    if (!midiAvailable) return;
    
    uint8_t statusByte = 0x80 | (channel & 0x0F);  // Note Off
    sendMIDIByte(statusByte);
    sendMIDIByte(noteNumber & 0x7F);
    sendMIDIByte(0);  // Velocity 0
}

void DryerHardware::triggerPulse(int triggerPin, int durationMs) {
    // Start trigger pulse
    writeGPIO(triggerPin, true);
    
    // Calculate end time
    auto now = std::chrono::steady_clock::now();
    auto endTime = now + std::chrono::milliseconds(durationMs);
    
    // Store state
    if (triggerPin == GPIO_TRIGGER_OUT_1) {
        trigger1State.active = true;
        trigger1State.endTime = std::chrono::duration_cast<std::chrono::microseconds>(
            endTime.time_since_epoch()).count();
    } else if (triggerPin == GPIO_TRIGGER_OUT_2) {
        trigger2State.active = true;
        trigger2State.endTime = std::chrono::duration_cast<std::chrono::microseconds>(
            endTime.time_since_epoch()).count();
    }
}

void DryerHardware::updateTriggers() {
    auto now = std::chrono::steady_clock::now();
    uint64_t nowMicros = std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()).count();
    
    // Check trigger 1
    if (trigger1State.active && nowMicros >= trigger1State.endTime) {
        writeGPIO(GPIO_TRIGGER_OUT_1, false);
        trigger1State.active = false;
    }
    
    // Check trigger 2
    if (trigger2State.active && nowMicros >= trigger2State.endTime) {
        writeGPIO(GPIO_TRIGGER_OUT_2, false);
        trigger2State.active = false;
    }
}

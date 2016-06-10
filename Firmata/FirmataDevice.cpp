//
//  FirmataDevice.cpp
//  Firmata
//
//  Created by Christopher Stawarz on 6/10/16.
//  Copyright © 2016 The MWorks Project. All rights reserved.
//

#include "FirmataDevice.hpp"


BEGIN_NAMESPACE_MW


enum {
    DIGITAL_MESSAGE       = 0x90, // send data for a digital port (collection of 8 pins)
    ANALOG_MESSAGE        = 0xE0, // send data for an analog pin (or PWM)
    REPORT_ANALOG         = 0xC0, // enable analog input by pin #
    REPORT_DIGITAL        = 0xD0, // enable digital input by port pair
    
    SET_PIN_MODE          = 0xF4, // set a pin to INPUT/OUTPUT/PWM/etc
    SET_DIGITAL_PIN_VALUE = 0xF5, // set value of an individual digital pin
    
    REPORT_VERSION        = 0xF9, // report protocol version
    SYSTEM_RESET          = 0xFF, // reset from MIDI
    
    START_SYSEX           = 0xF0, // start a MIDI Sysex message
    END_SYSEX             = 0xF7, // end a MIDI Sysex message
    
    REPORT_FIRMWARE       = 0x79  // report name and version of the firmware
};


const std::string FirmataDevice::SERIAL_PORT("serial_port");


void FirmataDevice::describeComponent(ComponentInfo &info) {
    IODevice::describeComponent(info);
    
    info.setSignature("iodevice/firmata");
    
    info.addParameter(SERIAL_PORT);
}


FirmataDevice::FirmataDevice(const ParameterValueMap &parameters) :
    IODevice(parameters),
    serialPort(parameters[SERIAL_PORT].str()),
    fd(-1),
    deviceProtocolVersionReceived(false),
    deviceProtocolVersionMajor(0),
    deviceProtocolVersionMinor(0),
    running(false)
{
}


FirmataDevice::~FirmataDevice() {
    if (receiveDataThread.joinable()) {
        continueReceivingData.clear();
        receiveDataThread.join();
    }
    
    if (-1 != fd) {
        disconnect();
    }
}


void FirmataDevice::addChild(std::map<std::string, std::string> parameters,
                             ComponentRegistryPtr reg,
                             boost::shared_ptr<Component> child)
{
    if (auto digitalChannel = boost::dynamic_pointer_cast<FirmataDigitalChannel>(child)) {
        auto &channel = getChannelForPin(digitalChannel->getPinNumber());
        if (channel) {
            merror(M_IODEVICE_MESSAGE_DOMAIN, "Digital pin %d is already in use", digitalChannel->getPinNumber());
        } else {
            channel = digitalChannel;
        }
        return;
    }
    
    throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN, "Invalid channel type for Firmata device");
}


bool FirmataDevice::initialize() {
    mprintf(M_IODEVICE_MESSAGE_DOMAIN, "Configuring Firmata device \"%s\"...", getTag().c_str());
    
    if (!connect()) {
        return false;
    }
    
    continueReceivingData.test_and_set();
    receiveDataThread = std::thread([this]() {
        receiveData();
    });
    
    // Wait for connection to be established
    Clock::instance()->sleepMS(3000);
    
    {
        unique_lock lock(mutex);
        
        if (!sendData({ REPORT_VERSION, SYSTEM_RESET }) ||
            !checkProtocolVersion(lock) ||
            !configureDigitalPorts())
        {
            return false;
        }
    }
    
    mprintf(M_IODEVICE_MESSAGE_DOMAIN, "Firmata device \"%s\" is ready", getTag().c_str());
    
    return true;
}


bool FirmataDevice::startDeviceIO() {
    unique_lock lock(mutex);
    
    if (!running) {
        if (!startDigitalIO()) {
            return false;
        }
        running = true;
    }
    
    return true;
}


bool FirmataDevice::stopDeviceIO() {
    unique_lock lock(mutex);
    
    if (running) {
        if (!stopDigitalIO()) {
            return false;
        }
        running = false;
    }
    
    return true;
}


bool FirmataDevice::connect() {
    // Open the serial port read/write, with no controlling terminal, and don't wait for a connection.
    // The O_NONBLOCK flag also causes subsequent I/O on the device to be non-blocking.
    if (-1 == (fd = ::open(serialPort.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK))) {
        serialError("Cannot open serial port");
        return false;
    }
    
    bool shouldClose = true;
    BOOST_SCOPE_EXIT(&shouldClose, &fd) {
        if (shouldClose) {
            (void)::close(fd);
            fd = -1;
        }
    } BOOST_SCOPE_EXIT_END
    
    // open() follows POSIX semantics: multiple open() calls to the same file will succeed
    // unless the TIOCEXCL ioctl is issued.  This will prevent additional opens except by root-owned
    // processes.
    if (-1 == ioctl(fd, TIOCEXCL)) {
        serialError("Cannot obtain exclusive use of serial port");
        return false;
    }
    
    // Now that the device is open, clear the O_NONBLOCK flag so subsequent I/O will block
    if (-1 == fcntl(fd, F_SETFL, 0)) {
        serialError("Cannot restore blocking I/O on serial port");
        return false;
    }
    
    // Get the current options and save them, so we can restore the default settings later
    if (-1 == tcgetattr(fd, &origAttrs)) {
        serialError("Cannot obtain current serial port attributes");
        return false;
    }
    
    struct termios attrs = origAttrs;
    cfmakeraw(&attrs);            // Set raw input (non-canonical) mode
    attrs.c_cc[VMIN] = 0;         // Reads block until a single byte has been received
    attrs.c_cc[VTIME] = 5;        //   or a 500ms timeout expires
    cfsetspeed(&attrs, B57600);   // Set speed to 57600 baud
    attrs.c_cflag |= CS8;         // Use 8-bit words
    attrs.c_cflag &= ~PARENB;     // No parity
    attrs.c_cflag &= ~CSTOPB;     // 1 stop bit
    attrs.c_cflag |= CLOCAL;      // Ignore modem status lines
    
    // Cause the new options to take effect immediately
    if (-1 == tcsetattr(fd, TCSANOW, &attrs)) {
        serialError("Cannot set serial port attributes");
        return false;
    }
    
    shouldClose = false;
    
    return true;
}


void FirmataDevice::disconnect() {
    // Block until all written output has been sent to the device
    if (-1 == tcdrain(fd)) {
        serialError("Serial port drain failed");
    }
    
    // Restore original options
    if (-1 == tcsetattr(fd, TCSANOW, &origAttrs)) {
        serialError("Cannot restore previous serial port attributes");
    }
    
    if (-1 == ::close(fd)) {
        serialError("Cannot close serial port");
    }
    
    fd = -1;
}


bool FirmataDevice::checkProtocolVersion(unique_lock &lock) {
    // Wait for receipt of protocol version
    if (!condition.wait_for(lock, std::chrono::seconds(2), [this]() { return deviceProtocolVersionReceived; })) {
        merror(M_IODEVICE_MESSAGE_DOMAIN,
               "Firmata device \"%s\" did not respond to request for protocol version",
               getTag().c_str());
        return false;
    }
    
    // Confirm that device uses a compatible protocol version
    if (deviceProtocolVersionMajor != protocolVersionMajor ||
        deviceProtocolVersionMinor < protocolVersionMinor)
    {
        merror(M_IODEVICE_MESSAGE_DOMAIN,
               "Firmata device \"%s\" uses protocol version %hhu.%hhu, which is not compatible with the version "
               "used by MWorks (%hhu.%hhu)",
               getTag().c_str(),
               deviceProtocolVersionMajor,
               deviceProtocolVersionMinor,
               protocolVersionMajor,
               protocolVersionMinor);
        return false;
    }
    
    return true;
}


bool FirmataDevice::configureDigitalPorts() {
    boost::weak_ptr<FirmataDevice> weakThis(component_shared_from_this<FirmataDevice>());
    
    for (const auto &port : ports) {
        for (const auto &channel : port) {
            if (channel) {
                const auto pinNumber = channel->getPinNumber();
                if (!sendData({ SET_PIN_MODE, std::uint8_t(pinNumber), std::uint8_t(channel->getDirection()) })) {
                    merror(M_IODEVICE_MESSAGE_DOMAIN,
                           "Cannot set mode of digital pin %d on Firmata device \"%s\"",
                           pinNumber,
                           getTag().c_str());
                    return false;
                }
                if (channel->isOutput()) {
                    auto callback = [weakThis, pinNumber](const Datum &data, MWTime time) {
                        if (auto sharedThis = weakThis.lock()) {
                            sharedThis->setDigitalOutput(pinNumber, data.getBool());
                        }
                    };
                    channel->getValueVariable()->addNotification(boost::make_shared<VariableCallbackNotification>(callback));
                }
            }
        }
    }
    
    return true;
}


bool FirmataDevice::startDigitalIO() {
    for (std::size_t portNum = 0; portNum < ports.size(); portNum++) {
        const auto &port = ports.at(portNum);
        bool portHasInputs = false;
        bool portHasOutputs = false;
        std::array<std::uint8_t, 2> portState = { 0, 0 };
        
        for (std::size_t bitNum = 0; bitNum < port.size(); bitNum++) {
            const auto &channel = port.at(bitNum);
            if (channel) {
                switch (channel->getDirection()) {
                    case FirmataDigitalChannel::Direction::Input:
                        portHasInputs = true;
                        break;
                        
                    case FirmataDigitalChannel::Direction::Output:
                        portHasOutputs = true;
                        portState.at(bitNum / 7) |= (channel->getValueVariable()->getValue().getBool() << (bitNum % 7));
                        break;
                }
            }
        }
        
        if (portHasInputs &&
            !sendData({ std::uint8_t(REPORT_DIGITAL | portNum), 1 }))
        {
            merror(M_IODEVICE_MESSAGE_DOMAIN,
                   "Cannot enable reporting of digital input pins on Firmata device \"%s\"",
                   getTag().c_str());
            return false;
        }
        
        if (portHasOutputs &&
            !sendData({ std::uint8_t(DIGITAL_MESSAGE | portNum), portState.at(0), portState.at(1) }))
        {
            merror(M_IODEVICE_MESSAGE_DOMAIN,
                   "Cannot set initial values of digital output pins on Firmata device \"%s\"",
                   getTag().c_str());
            return false;
        }
    }
    
    return true;
}


bool FirmataDevice::stopDigitalIO() {
    for (std::size_t portNum = 0; portNum < ports.size(); portNum++) {
        const auto &port = ports.at(portNum);
        bool portHasInputs = false;
        bool portHasOutputs = false;
        
        for (const auto &channel : port) {
            if (channel) {
                switch (channel->getDirection()) {
                    case FirmataDigitalChannel::Direction::Input:
                        portHasInputs = true;
                        break;
                        
                    case FirmataDigitalChannel::Direction::Output:
                        portHasOutputs = true;
                        break;
                }
            }
        }
        
        if (portHasInputs &&
            !sendData({ std::uint8_t(REPORT_DIGITAL | portNum), 0 }))
        {
            merror(M_IODEVICE_MESSAGE_DOMAIN,
                   "Cannot disable reporting of digital input pins on Firmata device \"%s\"",
                   getTag().c_str());
            return false;
        }
        
        if (portHasOutputs &&
            !sendData({ std::uint8_t(DIGITAL_MESSAGE | portNum), 0, 0 }))
        {
            merror(M_IODEVICE_MESSAGE_DOMAIN,
                   "Cannot reset values of digital output pins on Firmata device \"%s\"",
                   getTag().c_str());
            return false;
        }
    }
    
    return true;
}


void FirmataDevice::setDigitalOutput(int pinNumber, bool value) {
    unique_lock lock(mutex);
    
    if (running &&
        !sendData({ SET_DIGITAL_PIN_VALUE, std::uint8_t(pinNumber), value }))
    {
        merror(M_IODEVICE_MESSAGE_DOMAIN,
               "Cannot set value of digital output pin %d on Firmata device \"%s\"",
               pinNumber,
               getTag().c_str());
    }
}


bool FirmataDevice::sendData(const std::vector<std::uint8_t> &data) {
    if (-1 == ::write(fd, data.data(), data.size())) {
        serialError("Write to serial port failed");
        return false;
    }
    return true;
}


void FirmataDevice::receiveData() {
    std::array<std::uint8_t, 3> message;
    std::size_t bytesReceived = 0;
    std::size_t bytesExpected = 1;
    MWTime currentCommandTime = 0;
    std::uint8_t currentCommand = 0;
    
    while (continueReceivingData.test_and_set()) {
        // Since this is the only thread that reads from fd, we don't need a lock here
        auto result = ::read(fd, message.data() + bytesReceived, bytesExpected - bytesReceived);
        
        if (-1 == result) {
            
            serialError("Read from serial port failed");
            
            //
            // It would be nice if we tried to re-connect.  However, most Arduinos will reset the
            // running sketch upon connection, meaning we'd have to re-initialize everything.
            // Just give up.
            //
            merror(M_IODEVICE_MESSAGE_DOMAIN,
                   "Aborting all attempts to receive data from Firmata device \"%s\"",
                   getTag().c_str());
            return;
            
        } else if (result > 0) {
            
            bytesReceived += result;
            
            if (bytesReceived == 1) {
                
                //
                // Start processing new message
                //
                
                currentCommandTime = Clock::instance()->getCurrentTimeUS();
                
                currentCommand = message.at(0);
                if (currentCommand < 0xF0) {
                    currentCommand &= 0xF0;
                }
                
                switch (currentCommand) {
                        
                    case DIGITAL_MESSAGE:
                        bytesExpected = 3;
                        break;
                        
                    case REPORT_VERSION:
                        bytesExpected = 3;
                        break;
                        
                    case START_SYSEX:
                        bytesExpected = 2;
                        break;
                        
                    default:
                        mwarning(M_IODEVICE_MESSAGE_DOMAIN,
                                 "Received unexpected command (0x%02hhX) from Firmata device \"%s\"",
                                 currentCommand,
                                 getTag().c_str());
                        bytesReceived = 0;
                        break;
                        
                }
                
            } else if (currentCommand == START_SYSEX) {
                
                //
                // Handle SysEx message
                //
                
                if (bytesExpected == 2) {
                    
                    if (message.at(1) != REPORT_FIRMWARE) {
                        mwarning(M_IODEVICE_MESSAGE_DOMAIN,
                                 "Received unexpected SysEx command (0x%02hhX) from Firmata device \"%s\"",
                                 message.at(1),
                                 getTag().c_str());
                    }
                    
                    bytesExpected = 3;
                    
                } else if (message.at(2) == END_SYSEX) {
                    
                    bytesReceived = 0;
                    bytesExpected = 1;
                    
                } else {
                    
                    // Still waiting for END_SYSEX
                    bytesReceived--;
                    
                }
            
            } else if (bytesReceived == bytesExpected) {
                
                //
                // Finish processing current message
                //
                
                switch (currentCommand) {
                        
                    case DIGITAL_MESSAGE: {
                        const std::size_t portNum = (message.at(0) & 0x0F);
                        const std::array<std::uint8_t, 2> portState = { message.at(1), message.at(2) };
                        const auto &port = ports.at(portNum);
                        
                        for (std::size_t bitNum = 0; bitNum < port.size(); bitNum++) {
                            const auto &channel = port.at(bitNum);
                            if (channel && channel->isInput()) {
                                const bool value = portState.at(bitNum / 7) & (1 << (bitNum % 7));
                                if (channel->getValueVariable()->getValue().getBool() != value) {
                                    channel->getValueVariable()->setValue(value, currentCommandTime);
                                }
                            }
                        }
                        
                        break;
                    }
                        
                    case REPORT_VERSION:
                        if (!deviceProtocolVersionReceived) {
                            {
                                unique_lock lock(mutex);
                                deviceProtocolVersionMajor = message.at(1);
                                deviceProtocolVersionMinor = message.at(2);
                                deviceProtocolVersionReceived = true;
                            }
                            condition.notify_all();
                        }
                        break;
                        
                    default:
                        break;
                        
                }
                
                bytesReceived = 0;
                bytesExpected = 1;
                
            }
            
        }
    }
}


inline void FirmataDevice::serialError(const std::string &msg) const {
    merror(M_IODEVICE_MESSAGE_DOMAIN, "%s (%s): %s", msg.c_str(), serialPort.c_str(), strerror(errno));
}


END_NAMESPACE_MW





























#pragma once

#include <stdexcept>

#include <dynamixel_sdk/dynamixel_sdk.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>



// Control table address
constexpr int ADDR_OPERATING_MODE      = 11;
constexpr int ADDR_TORQUE_ENABLE[2]    = {24, 64};
constexpr int ADDR_GOAL_CURRENT        = 102;
constexpr int ADDR_GOAL_VELOCITY       = 104;
constexpr int ADDR_GOAL_POSITION[2]    = {30, 116};
constexpr int ADDR_PRESENT_CURRENT[2]  = {40, 126};
constexpr int ADDR_PRESENT_VELOCITY[2] = {38, 128};
constexpr int ADDR_PRESENT_POSITION[2] = {36, 132};


class DynamixelInterface {

public:

    enum PROTOCOL {
        VERSION_1 = 1,
        VERSION_2 = 2
    };

    enum OPERATING_MODE {
        CURRENT = 0,
        VELOCITY = 1,
        POSITION = 3,
        EXTENDED_POSITION = 4,
        CURRENT_BASED_POSITION = 5,
        PWM = 16
    };

    DynamixelInterface() :
        packetHandler{dynamixel::PacketHandler::getPacketHandler(PROTOCOL::VERSION_1), 
                      dynamixel::PacketHandler::getPacketHandler(PROTOCOL::VERSION_2)} {}
    ~DynamixelInterface() { this->close(); }

    void open(std::string device_name, int baudrate) {
        // this->packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        // check if dynamixel port open. If you cannot open port, try "sudo chmod 666 /dev/ttyUSB0"
        this->portHandler = dynamixel::PortHandler::getPortHandler(device_name.c_str());
        if (!this->portHandler->openPort()) {
            throw std::runtime_error("Failed to open the port!");
        }
        if (!this->portHandler->setBaudRate(baudrate)) {
            throw std::runtime_error("Failed to set the baudrate!");
        }
    }

    void close() {
        for (auto& dxl_id: this->dynamixelIDs) {
            this->setTorqueDisable(dxl_id.first);
        }
        this->portHandler->closePort();
    }

    void addMotor(int dxl_id, PROTOCOL version) {
        dynamixelIDs.insert(std::pair<int,PROTOCOL>(dxl_id, version));
    }

    void setOperationgMode(int dxl_id, OPERATING_MODE mode) {
        if (dynamixelIDs.at(dxl_id) == PROTOCOL::VERSION_1) {
            ROS_WARN("Dynamixel ID %d cannot change operating mode (PROTOCOL VERSION 1)", dxl_id);
        } else {
            _writeValue<uint8_t>(dxl_id, ADDR_OPERATING_MODE, mode, "change mode");
        }
    }

    void setTorqueEnable(int dxl_id) {
        _writeValue<uint8_t>(dxl_id, ADDR_TORQUE_ENABLE[dynamixelIDs.at(dxl_id)-1], 1, "enable torque");
    }

    void setTorqueEnableAll() {
        for (auto& dxl: dynamixelIDs) {
            setTorqueEnable(dxl.first);
        }
    }

    void setTorqueDisable(int dxl_id) {
        _writeValue<uint8_t>(dxl_id, ADDR_TORQUE_ENABLE[dynamixelIDs.at(dxl_id)-1], 0, "disable torque");
    }

    void setTorqueDisableAll() {
        for (auto& dxl: dynamixelIDs) {
            setTorqueDisable(dxl.first);
        }
    }

    void setPosition(int dxl_id, int position) {
        if (dynamixelIDs.at(dxl_id) == PROTOCOL::VERSION_1) {
            _writeValue<int16_t>(dxl_id, ADDR_GOAL_POSITION[0], position, "set position");
        } else {
            _writeValue<int32_t>(dxl_id, ADDR_GOAL_POSITION[1], position, "set position");
        }
    }

    void setVelocity(int dxl_id, int velocity) {
        if (dynamixelIDs.at(dxl_id) == PROTOCOL::VERSION_1) {
            ROS_WARN("Dynamixel ID %d cannot set velocity (PROTOCOL VERSION 1)", dxl_id);
        } else {
            _writeValue<int32_t>(dxl_id, ADDR_GOAL_VELOCITY, velocity, "set velocity");
        }
    }

    int getPosition(int dxl_id) {
        int val = 0;
        if (dynamixelIDs.at(dxl_id) == PROTOCOL::VERSION_1) {
            val = _readValue<int16_t>(dxl_id, ADDR_PRESENT_POSITION[0], "get position");
        } else {
            val = _readValue<int32_t>(dxl_id, ADDR_PRESENT_POSITION[1], "get position");
        }
        return val;
    }

    int getVelocity(int dxl_id) {
        int val = 0;
        if (dynamixelIDs.at(dxl_id) == PROTOCOL::VERSION_1) {
            val = _readValue<int16_t>(dxl_id, ADDR_PRESENT_VELOCITY[0], "get velocity");
        } else {
            val = _readValue<int32_t>(dxl_id, ADDR_PRESENT_VELOCITY[1], "get velocity");
        }
        return val;
    }

    int getCurrent(int dxl_id) {
        return _readValue<int16_t>(dxl_id, ADDR_PRESENT_CURRENT[dynamixelIDs.at(dxl_id)-1], "get current");
    }


private:

    dynamixel::PortHandler * portHandler;
    dynamixel::PacketHandler * packetHandler[2];

    std::map<int, PROTOCOL> dynamixelIDs;
    std::map<int, uint16_t> currentLimit;

    template<typename T>
    void _writeValue(int dxl_id, int address, T value, std::string description) {
        uint8_t dxl_error = 0;
        int dxl_comm_result = this->packetHandler[dynamixelIDs.at(dxl_id)-1]->writeTxRx(this->portHandler, dxl_id, address, sizeof(T), (uint8_t*)&value, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_INFO("Failed to %s for Dynamixel ID %d; Communication result: %d", description.c_str(), dxl_id, dxl_comm_result); 
        } else {
            ROS_INFO("Success to %s for Dynamixel ID %d", description.c_str(), dxl_id); 
        }
    }

    template<typename T>
    T _readValue(int dxl_id, int address, std::string description) {
        T value = 0;
        uint8_t dxl_error = 0;
        int dxl_comm_result = this->packetHandler[dynamixelIDs.at(dxl_id)-1]->readTxRx(this->portHandler, dxl_id, address, sizeof(T), (uint8_t*)&value, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_INFO("Failed to %s for Dynamixel ID %d; Communication result: %d", description.c_str(), dxl_id, dxl_comm_result); 
        } else {
            ROS_INFO("Success to %s for Dynamixel ID %d", description.c_str(), dxl_id); 
        }
        return value;
    }
};
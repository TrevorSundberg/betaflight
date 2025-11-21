#pragma once

#define UART_CHANNELS 3

typedef struct {
    uint8_t cameraAngleDegrees; // set to -1 to leave it as it is
    void* uartDataInConfigurator;
    int uartDataInSizeConfigurator;
    float channels[SIMULATOR_MAX_RC_CHANNELS]; // [-1, 1]
    double imu_angular_velocity_rpy[3]; // rad/s -> range: +/- 8192; +/- 2000 deg/se
    double imu_linear_acceleration_xyz[3];    // m/s/s NED, body frame -> sim 1G = 9.80665, FC 1G = 256
    double imu_orientation_quat[4];     //w, x, y, z
    double velocity_xyz[3];             // m/s, earth frame. ENU (Ve, Vn, Vup) for virtual GPS mode (USE_VIRTUAL_GPS)!
    double position_xyz[3];             // meters, NED from origin. Longitude, Latitude, Altitude (ENU) for virtual GPS mode (USE_VIRTUAL_GPS)!
    double pressure;
} iterationInput;

typedef struct {
    uint8_t* uartDataOut;
    uint32_t uartDataSize;
} uartBufferOutput;

typedef struct {
    uint8_t cameraAngleDegrees;
    uint8_t motorCount;
    float motorSpeeds[SIMULATOR_MAX_PWM_CHANNELS];   // normal: [0.0, 1.0], 3D: [-1.0, 1.0]
    uint8_t* eeprom;
    uint32_t eepromSize;
    int32_t rebootRequest;
    uartBufferOutput uartBuffers[UART_CHANNELS];
} iterationOutput;

PLUGIN_EXPORT void initialize(const void* eepromInData, const int eepromInSize);
PLUGIN_EXPORT void iteration(iterationInput* input, iterationOutput* output);


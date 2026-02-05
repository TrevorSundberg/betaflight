/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <errno.h>
#include <time.h>

#include "common/maths.h"

#include "build/debug.h"

#include "drivers/adc_impl.h"
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/motor_impl.h"
#include "drivers/serial.h"
/*
#include "drivers/serial_tcp.h"
*/
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_output_impl.h"
#include "drivers/light_led.h"

#include "drivers/timer.h"
#include "timer_def.h"

#include "drivers/accgyro/accgyro_virtual.h"
#include "drivers/barometer/barometer_virtual.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"

#include "config/feature.h"
#include "config/config.h"
#include "config/config_streamer.h"
#include "config/config_streamer_impl.h"
#include "config/config_eeprom_impl.h"

#include "scheduler/scheduler.h"

#include "pg/rx.h"
#include "pg/motor.h"
#include "pg/adc.h"
#include "sensors/adcinternal.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "io/gps.h"
#include "io/gps_virtual.h"
#include "fc/init.h"
#include "drivers/serial_direct.h"

/*
#include <pthread.h>
*/

/*
#include "dyad.h"
#include "udplink.h"
*/

uint32_t SystemCoreClock;

/*
static fdm_packet fdmPkt;
*/
static rc_packet rcPkt;
static servo_packet pwmPkt;
static servo_packet_raw pwmRawPkt;

static bool rc_received = false;
static bool fdm_received = false;

static struct timespec start_time;
static double simRate = 1.0;
/*
static pthread_t tcpWorker, udpWorker, udpWorkerRC;
*/
static bool workerRunning = true;
/*
static udpLink_t stateLink, pwmLink, pwmRawLink, rcLink;
*/
static pthread_mutex_t updateLock;
static pthread_mutex_t mainLoopLock;
static char simulator_ip[32] = "127.0.0.1";

#define PORT_PWM_RAW    9001    // Out
#define PORT_PWM        9002    // Out
#define PORT_STATE      9003    // In
#define PORT_RC         9004    // In

bootloaderRequestType_e rebootRequest = (bootloaderRequestType_e)KF_REBOOT_REQUEST_NONE;

int targetParseArgs(int argc, char * argv[])
{
    //The first argument should be target IP.
    if (argc > 1) {
        strcpy(simulator_ip, argv[1]);
    }

    printf("[SITL] The SITL will output to IP %s:%d (Gazebo) and %s:%d (RealFlightBridge)\n",
           simulator_ip, PORT_PWM, simulator_ip, PORT_PWM_RAW);
    return 0;
}

int timeval_sub(struct timespec *result, struct timespec *x, struct timespec *y);

int lockMainPID(void)
{
    return pthread_mutex_trylock(&mainLoopLock);
}

#define RAD2DEG (180.0 / M_PI)
#define ACC_SCALE (256 / 9.80665)
#define GYRO_SCALE (16.4)

static void sendMotorUpdate(void)
{
/*
    udpSend(&pwmLink, &pwmPkt, sizeof(servo_packet));
*/
}

void updateState(const fdm_packet* pkt)
{
    if (!fdm_received) {
        printf("[SITL] new fdm t:%f\n", pkt->timestamp);
        fdm_received = true;
    }

    static double last_timestamp = 0; // in seconds
    static uint64_t last_realtime = 0; // in uS
    static struct timespec last_ts; // last packet

    struct timespec now_ts;
    clock_gettime(CLOCK_MONOTONIC, &now_ts);

    const uint64_t realtime_now = micros64_real();
    if (realtime_now > last_realtime + 500*1e3) { // 500ms timeout
        last_timestamp = pkt->timestamp;
        last_realtime = realtime_now;
        sendMotorUpdate();
        return;
    }

    const double deltaSim = pkt->timestamp - last_timestamp;  // in seconds
    if (deltaSim < 0) { // don't use old packet
        return;
    }

    int16_t x,y,z;
    x = constrain(-pkt->imu_linear_acceleration_xyz[0] * ACC_SCALE, -32767, 32767);
    y = constrain(-pkt->imu_linear_acceleration_xyz[1] * ACC_SCALE, -32767, 32767);
    z = constrain(-pkt->imu_linear_acceleration_xyz[2] * ACC_SCALE, -32767, 32767);
    virtualAccSet(virtualAccDev, x, y, z);
//    printf("[acc]%lf,%lf,%lf\n", pkt->imu_linear_acceleration_xyz[0], pkt->imu_linear_acceleration_xyz[1], pkt->imu_linear_acceleration_xyz[2]);

    x = constrain(pkt->imu_angular_velocity_rpy[0] * GYRO_SCALE * RAD2DEG, -32767, 32767);
    y = constrain(-pkt->imu_angular_velocity_rpy[1] * GYRO_SCALE * RAD2DEG, -32767, 32767);
    z = constrain(-pkt->imu_angular_velocity_rpy[2] * GYRO_SCALE * RAD2DEG, -32767, 32767);
    virtualGyroSet(virtualGyroDev, x, y, z);
//    printf("[gyr]%lf,%lf,%lf\n", pkt->imu_angular_velocity_rpy[0], pkt->imu_angular_velocity_rpy[1], pkt->imu_angular_velocity_rpy[2]);

    // temperature in 0.01 C = 25 deg
    virtualBaroSet(pkt->pressure, 2500);
#if !defined(USE_IMU_CALC)
#if defined(SET_IMU_FROM_EULER)
    // set from Euler
    double qw = pkt->imu_orientation_quat[0];
    double qx = pkt->imu_orientation_quat[1];
    double qy = pkt->imu_orientation_quat[2];
    double qz = pkt->imu_orientation_quat[3];
    double ysqr = qy * qy;
    double xf, yf, zf;

    // roll (x-axis rotation)
    double t0 = +2.0 * (qw * qx + qy * qz);
    double t1 = +1.0 - 2.0 * (qx * qx + ysqr);
    xf = atan2(t0, t1) * RAD2DEG;

    // pitch (y-axis rotation)
    double t2 = +2.0 * (qw * qy - qz * qx);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    yf = asin(t2) * RAD2DEG; // from wiki

    // yaw (z-axis rotation)
    double t3 = +2.0 * (qw * qz + qx * qy);
    double t4 = +1.0 - 2.0 * (ysqr + qz * qz);
    zf = atan2(t3, t4) * RAD2DEG;
    imuSetAttitudeRPY(xf, -yf, zf); // yes! pitch was inverted!!
#else
    imuSetAttitudeQuat(pkt->imu_orientation_quat[0], pkt->imu_orientation_quat[1], pkt->imu_orientation_quat[2], pkt->imu_orientation_quat[3]);
#endif
#endif

#if defined(USE_VIRTUAL_GPS)
    const double longitude = pkt->position_xyz[0];
    const double latitude = pkt->position_xyz[1];
    const double altitude = pkt->position_xyz[2];
    const double speed = sqrt(sq(pkt->velocity_xyz[0]) + sq(pkt->velocity_xyz[1]));
    const double speed3D = sqrt(sq(pkt->velocity_xyz[0]) + sq(pkt->velocity_xyz[1]) + sq(pkt->velocity_xyz[2]));
    double course = atan2(pkt->velocity_xyz[0], pkt->velocity_xyz[1]) * RAD2DEG;
    if (course < 0.0) {
        course += 360.0;
    }
    setVirtualGPS(latitude, longitude, altitude, speed, speed3D, course);
#endif

#if defined(SIMULATOR_IMU_SYNC)
    imuSetHasNewData(deltaSim*1e6);
    imuUpdateAttitude(micros());
#endif

    if (deltaSim < 0.02 && deltaSim > 0) { // simulator should run faster than 50Hz
//        simRate = simRate * 0.5 + (1e6 * deltaSim / (realtime_now - last_realtime)) * 0.5;
        struct timespec out_ts;
        timeval_sub(&out_ts, &now_ts, &last_ts);
        simRate = deltaSim / (out_ts.tv_sec + 1e-9*out_ts.tv_nsec);
    }
//    printf("simRate = %lf, millis64 = %lu, millis64_real = %lu, deltaSim = %lf\n", simRate, millis64(), millis64_real(), deltaSim*1e6);

    last_timestamp = pkt->timestamp;
    last_realtime = micros64_real();

    last_ts.tv_sec = now_ts.tv_sec;
    last_ts.tv_nsec = now_ts.tv_nsec;

    pthread_mutex_unlock(&updateLock); // can send PWM output now

#if defined(SIMULATOR_GYROPID_SYNC)
    pthread_mutex_unlock(&mainLoopLock); // can run main loop
#endif
}

/*
static void* udpThread(void* data)
{
    UNUSED(data);
    int n = 0;

    while (workerRunning) {
        n = udpRecv(&stateLink, &fdmPkt, sizeof(fdm_packet), 100);
        if (n == sizeof(fdm_packet)) {
            if (!fdm_received) {
                printf("[SITL] new fdm %d t:%f from %s:%d\n", n, fdmPkt.timestamp, inet_ntoa(stateLink.recv.sin_addr), stateLink.recv.sin_port);
                fdm_received = true;
            }
            updateState(&fdmPkt);
        }
    }

    printf("udpThread end!!\n");
    return NULL;
}
*/

static float readRCSITL(const rxRuntimeState_t *rxRuntimeState, uint8_t channel)
{
    UNUSED(rxRuntimeState);
    return rcPkt.channels[channel];
}

static uint8_t rxRCFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);
    return RX_FRAME_COMPLETE;
}

/*
static void *udpRCThread(void *data)
{
    UNUSED(data);
    int n = 0;

    while (workerRunning) {
        n = udpRecv(&rcLink, &rcPkt, sizeof(rc_packet), 100);
        if (n == sizeof(rc_packet)) {
            if (!rc_received) {
                printf("[SITL] new rc %d: t:%f AETR: %d %d %d %d AUX1-4: %d %d %d %d\n", n, rcPkt.timestamp,
                    rcPkt.channels[0], rcPkt.channels[1],rcPkt.channels[2],rcPkt.channels[3],
                    rcPkt.channels[4], rcPkt.channels[5],rcPkt.channels[6],rcPkt.channels[7]);

                rxRuntimeState.channelCount = SIMULATOR_MAX_RC_CHANNELS;
                rxRuntimeState.rcReadRawFn = readRCSITL;
                rxRuntimeState.rcFrameStatusFn = rxRCFrameStatus;

                rxRuntimeState.rxProvider = RX_PROVIDER_UDP;
                rc_received = true;
            }
        }
    }

    printf("udpRCThread end!!\n");
    return NULL;
}

*/

void updateRCInput(const rc_packet* data)
{
    memcpy(&rcPkt, data, sizeof(rc_packet));

    if (!rc_received) {
        printf("[SITL] new rc t:%f AETR: %d %d %d %d AUX1-4: %d %d %d %d\n", rcPkt.timestamp,
            rcPkt.channels[0], rcPkt.channels[1],rcPkt.channels[2],rcPkt.channels[3],
            rcPkt.channels[4], rcPkt.channels[5],rcPkt.channels[6],rcPkt.channels[7]);

        rxRuntimeState.channelCount = SIMULATOR_MAX_RC_CHANNELS;
        rxRuntimeState.rcReadRawFn = readRCSITL;
        rxRuntimeState.rcFrameStatusFn = rxRCFrameStatus;

        rxRuntimeState.rxProvider = RX_PROVIDER_UDP;
        rc_received = true;
    }
}

/*
static void* tcpThread(void* data)
{
    UNUSED(data);

    dyad_init();
    dyad_setTickInterval(0.2f);
    dyad_setUpdateTimeout(0.01f);

    while (workerRunning) {
        dyad_update();
    }

    dyad_shutdown();
    printf("tcpThread end!!\n");
    return NULL;
}
*/

// system
void systemInit(void)
{
    int ret;

    clock_gettime(CLOCK_MONOTONIC, &start_time);
    printf("[system]Init...\n");

    SystemCoreClock = 500 * 1e6; // virtual 500MHz

    if (pthread_mutex_init(&updateLock, NULL) != 0) {
        printf("Create updateLock error!\n");
        exit(1);
    }

    if (pthread_mutex_init(&mainLoopLock, NULL) != 0) {
        printf("Create mainLoopLock error!\n");
        exit(1);
    }

    (void)ret;
/*
    ret = pthread_create(&tcpWorker, NULL, tcpThread, NULL);
    if (ret != 0) {
        printf("Create tcpWorker error!\n");
        exit(1);
    }

    ret = udpInit(&pwmLink, simulator_ip, PORT_PWM, false);
    printf("[SITL] init PwmOut UDP link to gazebo %s:%d...%d\n", simulator_ip, PORT_PWM, ret);

    ret = udpInit(&pwmRawLink, simulator_ip, PORT_PWM_RAW, false);
    printf("[SITL] init PwmOut UDP link to RF9 %s:%d...%d\n", simulator_ip, PORT_PWM_RAW, ret);

    ret = udpInit(&stateLink, NULL, PORT_STATE, true);
    printf("[SITL] start UDP server @%d...%d\n", PORT_STATE, ret);

    ret = udpInit(&rcLink, NULL, PORT_RC, true);
    printf("[SITL] start UDP server for RC input @%d...%d\n", PORT_RC, ret);

    ret = pthread_create(&udpWorker, NULL, udpThread, NULL);
    if (ret != 0) {
        printf("Create udpWorker error!\n");
        exit(1);
    }

    ret = pthread_create(&udpWorkerRC, NULL, udpRCThread, NULL);
    if (ret != 0) {
        printf("Create udpRCThread error!\n");
        exit(1);
    }
*/
}

void systemReset(void)
{
    rebootRequest = (bootloaderRequestType_e)KF_REBOOT_REQUEST_NORMAL;
    /*
    printf("[system]Reset!\n");
    workerRunning = false;
    pthread_join(tcpWorker, NULL);
    pthread_join(udpWorker, NULL);
    exit(0);
    */
}
void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    rebootRequest = requestType;
    /*
    printf("[system]ResetToBootloader!\n");
    UNUSED(requestType);

    printf("[system]ResetToBootloader!\n");
    workerRunning = false;
    pthread_join(tcpWorker, NULL);
    pthread_join(udpWorker, NULL);
    exit(0);
    */
}

void timerInit(void)
{
    printf("[timer]Init...\n");
}

void failureMode(failureMode_e mode)
{
    printf("[failureMode]!!! %d\n", mode);
    while (1);
}

void indicateFailure(failureMode_e mode, int repeatCount)
{
    UNUSED(repeatCount);
    printf("Failure LED flash for: [failureMode]!!! %d\n", mode);
}

// Time part
// Thanks ArduPilot
uint64_t nanos64_real(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec*1e9 + ts.tv_nsec) - (start_time.tv_sec*1e9 + start_time.tv_nsec);
}

uint64_t micros64_real(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e6*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec*1.0e-9)));
}

uint64_t millis64_real(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e3*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec*1.0e-9)));
}

uint64_t micros64(void)
{
    static uint64_t last = 0;
    static uint64_t out = 0;
    uint64_t now = nanos64_real();

    out += (now - last) * simRate;
    last = now;

    return out / 1000;
}

uint64_t millis64(void)
{
    static uint64_t last = 0;
    static uint64_t out = 0;
    uint64_t now = nanos64_real();

    out += (now - last) * simRate;
    last = now;

    return out / (1000 * 1000);
}

uint32_t micros(void)
{
    return micros64() & 0xFFFFFFFF;
}

uint32_t millis(void)
{
    return millis64() & 0xFFFFFFFF;
}

int32_t clockCyclesToMicros(int32_t clockCycles)
{
    return clockCycles;
}

int32_t clockCyclesTo10thMicros(int32_t clockCycles)
{
    return clockCycles;
}

int32_t clockCyclesTo100thMicros(int32_t clockCycles)
{
    return clockCycles;
}

uint32_t clockMicrosToCycles(uint32_t micros)
{
    return micros;
}

uint32_t getCycleCounter(void)
{
    return (uint32_t) (micros64() & 0xFFFFFFFF);
}

static void microsleep(uint32_t usec)
{
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = usec*1000UL;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) ;
}

void delayMicroseconds(uint32_t us)
{
    microsleep(us / simRate);
}

void delayMicroseconds_real(uint32_t us)
{
    microsleep(us);
}

void delay(uint32_t ms)
{
    uint64_t start = millis64();

    while ((millis64() - start) < ms) {
        microsleep(1000);
    }
}

// Subtract the ‘struct timespec’ values X and Y,  storing the result in RESULT.
// Return 1 if the difference is negative, otherwise 0.
// result = x - y
// from: http://www.gnu.org/software/libc/manual/html_node/Elapsed-Time.html
int timeval_sub(struct timespec *result, struct timespec *x, struct timespec *y)
{
    unsigned int s_carry = 0;
    unsigned int ns_carry = 0;
    // Perform the carry for the later subtraction by updating y.
    if (x->tv_nsec < y->tv_nsec) {
        int nsec = (y->tv_nsec - x->tv_nsec) / 1000000000 + 1;
        ns_carry += 1000000000 * nsec;
        s_carry += nsec;
    }

    // Compute the time remaining to wait. tv_usec is certainly positive.
    result->tv_sec = x->tv_sec - y->tv_sec - s_carry;
    result->tv_nsec = x->tv_nsec - y->tv_nsec + ns_carry;

    // Return 1 if result is negative.
    return x->tv_sec < y->tv_sec;
}

// PWM part
static pwmOutputPort_t servos[MAX_SUPPORTED_SERVOS];

// real value to send
static int16_t motorsPwm[MAX_SUPPORTED_MOTORS];
static int16_t servosPwm[MAX_SUPPORTED_SERVOS];
static int16_t idlePulse;

void servoDevInit(const servoDevConfig_t *servoConfig)
{
    printf("[SITL] Init servos num %d rate %d center %d\n", MAX_SUPPORTED_SERVOS,
           servoConfig->servoPwmRate, servoConfig->servoCenterPulse);
    for (uint8_t servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
        servos[servoIndex].enabled = true;
    }
}

pwmOutputPort_t *pwmGetMotors(void)
{
    return pwmMotors;
}

static float pwmConvertFromExternal(uint16_t externalValue)
{
    return (float)externalValue;
}

static uint16_t pwmConvertToExternal(float motorValue)
{
    return (uint16_t)motorValue;
}

static void pwmDisableMotors(void)
{
    // NOOP
}

static void pwmWriteMotor(uint8_t index, float value)
{
    if (pthread_mutex_trylock(&updateLock) != 0) return;

    if (index < MAX_SUPPORTED_MOTORS) {
        motorsPwm[index] = value - idlePulse;
    }

    if (index < pwmRawPkt.motorCount) {
        pwmRawPkt.pwm_output_raw[index] = value;
    }

    pthread_mutex_unlock(&updateLock); // can send PWM output now
}

static void pwmWriteMotorInt(uint8_t index, uint16_t value)
{
    pwmWriteMotor(index, (float)value);
}

static void pwmShutdownPulsesForAllMotors(void)
{
    // NOOP
}

static void pwmCompleteMotorUpdate(void)
{
    // send to simulator
    // for gazebo8 ArduCopterPlugin remap, normal range = [0.0, 1.0], 3D rang = [-1.0, 1.0]

    double outScale = 1000.0;
    if (featureIsEnabled(FEATURE_3D)) {
        outScale = 500.0;
    }

    pwmPkt.motor_speed[3] = motorsPwm[0] / outScale;
    pwmPkt.motor_speed[0] = motorsPwm[1] / outScale;
    pwmPkt.motor_speed[1] = motorsPwm[2] / outScale;
    pwmPkt.motor_speed[2] = motorsPwm[3] / outScale;

    // get one "fdm_packet" can only send one "servo_packet"!!
    if (pthread_mutex_trylock(&updateLock) != 0) return;
    sendMotorUpdate();
/*
    udpSend(&pwmLink, &pwmPkt, sizeof(servo_packet));
//    printf("[pwm]%u:%u,%u,%u,%u\n", idlePulse, motorsPwm[0], motorsPwm[1], motorsPwm[2], motorsPwm[3]);
    udpSend(&pwmRawLink, &pwmRawPkt, sizeof(servo_packet_raw));
*/
}

void pwmWriteServo(uint8_t index, float value)
{
    servosPwm[index] = value;
    if (index + pwmRawPkt.motorCount < SIMULATOR_MAX_PWM_CHANNELS) {
        // In pwmRawPkt, we put servo right after the motors.
        pwmRawPkt.pwm_output_raw[index + pwmRawPkt.motorCount] = value;
    }
}

static const motorVTable_t vTable = {
    .postInit = motorPostInitNull,
    .convertExternalToMotor = pwmConvertFromExternal,
    .convertMotorToExternal = pwmConvertToExternal,
    .enable = pwmEnableMotors,
    .disable = pwmDisableMotors,
    .isMotorEnabled = pwmIsMotorEnabled,
    .decodeTelemetry = motorDecodeTelemetryNull,
    .write = pwmWriteMotor,
    .writeInt = pwmWriteMotorInt,
    .updateComplete = pwmCompleteMotorUpdate,
    .shutdown = pwmShutdownPulsesForAllMotors,
    .requestTelemetry = NULL,
    .isMotorIdle = NULL,
    .getMotorIO = NULL,
};

bool motorPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig, uint16_t _idlePulse)
{
    UNUSED(motorConfig);

    if (!device) {
        return false;
    }

    pwmMotorCount = device->count;
    device->vTable = &vTable;
    
    printf("Initialized motor count %d\n", pwmMotorCount);
    pwmRawPkt.motorCount = pwmMotorCount;

    idlePulse = _idlePulse;

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < pwmMotorCount; motorIndex++) {
        pwmMotors[motorIndex].enabled = true;
    }

    return true;
}

// ADC part
uint16_t adcGetChannel(uint8_t channel)
{
    UNUSED(channel);
    switch (channel) {
        case ADC_BATTERY: {
            const float volts = 4.3f;
            uint16_t centiVolts = (uint16_t)roundf(volts * 100);
            return centiVolts;
        }
        case ADC_CURRENT: {
            const float amps = 0.05f;
            uint16_t centiAmps = (uint16_t)roundf(amps * 100);
            return centiAmps;
        }
    }
    return 0;
}
void adcInit(const adcConfig_t *config) {}
const adcTagMap_t adcTagMap[] = {};

// stack part
char _estack;
char _Min_Stack_Size;

/*
// virtual EEPROM
static FILE *eepromFd = NULL;
*/

#define UART_BUFFER_SIZE (1<<15)

typedef struct {
    uint8_t uartDataOut[UART_BUFFER_SIZE];
    uint32_t uartDataSize;
} uartBuffer;

STATIC_ASSERT(sizeof(int32_t) == sizeof(bootloaderRequestType_e), "bootloaderRequestType_e must be int32_t sized");

uint64_t externalFrame = 0;
void updateRCInput(const rc_packet* data);
void updateState(const fdm_packet* pkt);
uint8_t eepromInitialData[EEPROM_SIZE];
bool saveEEPROM = false;
uartBuffer uartBuffers[KF_MAX_UART_CHANNELS];

extern quaternion_t headfree;
extern quaternion_t offset;

#ifdef USE_THROTTLE_BOOST
extern float throttleBoost;
extern pt1Filter_t throttleLpf;
#endif

#define KF_COPY_FIELD(dst, src, field) ((dst)->field = (src)->field)
#define KF_COPY_ARRAY(dst, src, field) memcpy((dst)->field, (src)->field, sizeof((dst)->field))

typedef struct kf_pid_runtime_state_s {
    float dT;
    float pidFrequency;
    bool pidStabilisationEnabled;
    float previousPidSetpoint[XYZ_AXIS_COUNT];
    biquadFilter_t dtermNotch[XYZ_AXIS_COUNT];
    dtermLowpass_t dtermLowpass[XYZ_AXIS_COUNT];
    dtermLowpass_t dtermLowpass2[XYZ_AXIS_COUNT];
    pt1Filter_t ptermYawLowpass;
    bool antiGravityEnabled;
    pt2Filter_t antiGravityLpf;
    float antiGravityOsdCutoff;
    float antiGravityThrottleD;
    float itermAccelerator;
    uint8_t antiGravityGain;
    float antiGravityPGain;
    pidCoefficient_t pidCoefficient[XYZ_AXIS_COUNT];
    float angleGain;
    float angleFeedforwardGain;
    float horizonGain;
    float horizonLimitSticks;
    float horizonLimitSticksInv;
    float horizonLimitDegrees;
    float horizonLimitDegreesInv;
    float horizonIgnoreSticks;
    float maxVelocity[XYZ_AXIS_COUNT];
    bool inCrashRecoveryMode;
    timeUs_t crashDetectedAtUs;
    timeDelta_t crashTimeLimitUs;
    timeDelta_t crashTimeDelayUs;
    int32_t crashRecoveryAngleDeciDegrees;
    float crashRecoveryRate;
    float crashGyroThreshold;
    float crashDtermThreshold;
    float crashSetpointThreshold;
    float crashLimitYaw;
    float itermLimit;
    float itermLimitYaw;
    bool itermRotation;
    bool zeroThrottleItermReset;
    bool levelRaceMode;
    float tpaFactor;
    float tpaBreakpoint;
    float tpaMultiplier;
    float tpaLowBreakpoint;
    float tpaLowMultiplier;
    bool tpaLowAlways;
    bool useEzDisarm;
    float landingDisarmThreshold;

#ifdef USE_ITERM_RELAX
    pt1Filter_t windupLpf[XYZ_AXIS_COUNT];
    uint8_t itermRelax;
    uint8_t itermRelaxType;
    uint8_t itermRelaxCutoff;
#endif

#ifdef USE_ABSOLUTE_CONTROL
    float acCutoff;
    float acGain;
    float acLimit;
    float acErrorLimit;
    pt1Filter_t acLpf[XYZ_AXIS_COUNT];
    float oldSetpointCorrection[XYZ_AXIS_COUNT];
#endif

#ifdef USE_D_MAX
    pt2Filter_t dMaxRange[XYZ_AXIS_COUNT];
    pt2Filter_t dMaxLowpass[XYZ_AXIS_COUNT];
    float dMaxPercent[XYZ_AXIS_COUNT];
    uint8_t dMax[XYZ_AXIS_COUNT];
    float dMaxGyroGain;
    float dMaxSetpointGain;
#endif

#ifdef USE_AIRMODE_LPF
    pt1Filter_t airmodeThrottleLpf1;
    pt1Filter_t airmodeThrottleLpf2;
    float airmodeThrottleOffsetLimit;
#endif

#ifdef USE_ACRO_TRAINER
    float acroTrainerAngleLimit;
    float acroTrainerLookaheadTime;
    uint8_t acroTrainerDebugAxis;
    float acroTrainerGain;
    bool acroTrainerActive;
    int acroTrainerAxisState[RP_AXIS_COUNT];
#endif

#ifdef USE_DYN_LPF
    uint8_t dynLpfFilter;
    uint16_t dynLpfMin;
    uint16_t dynLpfMax;
    uint8_t dynLpfCurveExpo;
#endif

#ifdef USE_LAUNCH_CONTROL
    uint8_t launchControlMode;
    uint8_t launchControlAngleLimit;
    float launchControlKi;
#endif

#ifdef USE_INTEGRATED_YAW_CONTROL
    bool useIntegratedYaw;
    uint8_t integratedYawRelax;
#endif

#ifdef USE_THRUST_LINEARIZATION
    float thrustLinearization;
    float throttleCompensateAmount;
#endif

#ifdef USE_FEEDFORWARD
    feedforwardAveraging_t feedforwardAveraging;
    float feedforwardSmoothFactor;
    uint8_t feedforwardJitterFactor;
    float feedforwardJitterFactorInv;
    float feedforwardBoostFactor;
    float feedforwardTransition;
    float feedforwardTransitionInv;
    uint8_t feedforwardMaxRateLimit;
    float feedforwardYawHoldGain;
    float feedforwardYawHoldTime;
    bool feedforwardInterpolate;
    pt3Filter_t angleFeedforwardPt3[XYZ_AXIS_COUNT];
#endif

#ifdef USE_ACC
    pt3Filter_t attitudeFilter[RP_AXIS_COUNT];
    pt1Filter_t horizonSmoothingPt1;
    uint16_t horizonDelayMs;
    float angleYawSetpoint;
    float angleEarthRef;
    float angleTarget[RP_AXIS_COUNT];
    bool axisInAngleMode[3];
#endif

#ifdef USE_WING
    float spa[XYZ_AXIS_COUNT];
    tpaSpeedParams_t tpaSpeed;
    float tpaFactorYaw;
    float tpaFactorSterm[XYZ_AXIS_COUNT];
#endif

#ifdef USE_ADVANCED_TPA
    pwl_t tpaCurvePwl;
    float tpaCurvePwl_yValues[TPA_CURVE_PWL_SIZE];
    tpaCurveType_t tpaCurveType;
#endif

#ifdef USE_CHIRP
    chirp_t chirp;
    phaseComp_t chirpFilter;
    float chirpLagFreqHz;
    float chirpLeadFreqHz;
    float chirpAmplitude[3];
    float chirpFrequencyStartHz;
    float chirpFrequencyEndHz;
    float chirpTimeSeconds;
#endif
} kf_pid_runtime_state_t;

typedef struct kf_gyro_state_s {
    float gyroADC[XYZ_AXIS_COUNT];
    float gyroADCf[XYZ_AXIS_COUNT];
    uint8_t sampleCount;
    float sampleSum[XYZ_AXIS_COUNT];
    bool downsampleFilterEnabled;
    gyroCalibration_t calibration[GYRO_COUNT];
    gyroLowpassFilter_t lowpassFilter[XYZ_AXIS_COUNT];
    gyroLowpassFilter_t lowpass2Filter[XYZ_AXIS_COUNT];
    biquadFilter_t notchFilter1[XYZ_AXIS_COUNT];
    biquadFilter_t notchFilter2[XYZ_AXIS_COUNT];
    pt1Filter_t imuGyroFilter[XYZ_AXIS_COUNT];
    uint8_t gyroEnabledBitmask;
    uint8_t gyroDebugMode;
    bool gyroHasOverflowProtection;
    bool useMultiGyroDebugging;
    flight_dynamics_index_t gyroDebugAxis;
#ifdef USE_DYN_LPF
    uint8_t dynLpfFilter;
    uint16_t dynLpfMin;
    uint16_t dynLpfMax;
    uint8_t dynLpfCurveExpo;
#endif
#ifdef USE_GYRO_OVERFLOW_CHECK
    uint8_t overflowAxisMask;
#endif
} kf_gyro_state_t;

typedef struct kf_acc_state_s {
    vector3_t accADC;
    vector3_t jerk;
    float accMagnitude;
    float jerkMagnitude;
    bool isAccelUpdatedAtLeastOnce;
} kf_acc_state_t;

typedef struct kf_reconcile_state_s {
    uint64_t externalFrame;
    uint32_t targetPidLooptime;
    uint8_t activePidLoopDenom;
    pidAxisData_t pidData[XYZ_AXIS_COUNT];
    kf_pid_runtime_state_t pidRuntime;
    kf_gyro_state_t gyro;
    kf_acc_state_t acc;
    attitudeEulerAngles_t attitude;
    quaternion_t imuAttitudeQuaternion;
    quaternion_t headfree;
    quaternion_t offset;
    bool canUseGPSHeading;
    float rcCommand[PRIMARY_CHANNEL_COUNT];
    rcSmoothingFilter_t rcSmoothing;
#ifdef USE_THROTTLE_BOOST
    float throttleBoost;
    pt1Filter_t throttleLpf;
#endif
} kf_reconcile_state_t;

static void kf_write_pid_runtime_state(kf_pid_runtime_state_t *const dst)
{
    KF_COPY_FIELD(dst, &pidRuntime, dT);
    KF_COPY_FIELD(dst, &pidRuntime, pidFrequency);
    KF_COPY_FIELD(dst, &pidRuntime, pidStabilisationEnabled);
    KF_COPY_ARRAY(dst, &pidRuntime, previousPidSetpoint);
    KF_COPY_ARRAY(dst, &pidRuntime, dtermNotch);
    KF_COPY_ARRAY(dst, &pidRuntime, dtermLowpass);
    KF_COPY_ARRAY(dst, &pidRuntime, dtermLowpass2);
    KF_COPY_FIELD(dst, &pidRuntime, ptermYawLowpass);
    KF_COPY_FIELD(dst, &pidRuntime, antiGravityEnabled);
    KF_COPY_FIELD(dst, &pidRuntime, antiGravityLpf);
    KF_COPY_FIELD(dst, &pidRuntime, antiGravityOsdCutoff);
    KF_COPY_FIELD(dst, &pidRuntime, antiGravityThrottleD);
    KF_COPY_FIELD(dst, &pidRuntime, itermAccelerator);
    KF_COPY_FIELD(dst, &pidRuntime, antiGravityGain);
    KF_COPY_FIELD(dst, &pidRuntime, antiGravityPGain);
    KF_COPY_ARRAY(dst, &pidRuntime, pidCoefficient);
    KF_COPY_FIELD(dst, &pidRuntime, angleGain);
    KF_COPY_FIELD(dst, &pidRuntime, angleFeedforwardGain);
    KF_COPY_FIELD(dst, &pidRuntime, horizonGain);
    KF_COPY_FIELD(dst, &pidRuntime, horizonLimitSticks);
    KF_COPY_FIELD(dst, &pidRuntime, horizonLimitSticksInv);
    KF_COPY_FIELD(dst, &pidRuntime, horizonLimitDegrees);
    KF_COPY_FIELD(dst, &pidRuntime, horizonLimitDegreesInv);
    KF_COPY_FIELD(dst, &pidRuntime, horizonIgnoreSticks);
    KF_COPY_ARRAY(dst, &pidRuntime, maxVelocity);
    KF_COPY_FIELD(dst, &pidRuntime, inCrashRecoveryMode);
    KF_COPY_FIELD(dst, &pidRuntime, crashDetectedAtUs);
    KF_COPY_FIELD(dst, &pidRuntime, crashTimeLimitUs);
    KF_COPY_FIELD(dst, &pidRuntime, crashTimeDelayUs);
    KF_COPY_FIELD(dst, &pidRuntime, crashRecoveryAngleDeciDegrees);
    KF_COPY_FIELD(dst, &pidRuntime, crashRecoveryRate);
    KF_COPY_FIELD(dst, &pidRuntime, crashGyroThreshold);
    KF_COPY_FIELD(dst, &pidRuntime, crashDtermThreshold);
    KF_COPY_FIELD(dst, &pidRuntime, crashSetpointThreshold);
    KF_COPY_FIELD(dst, &pidRuntime, crashLimitYaw);
    KF_COPY_FIELD(dst, &pidRuntime, itermLimit);
    KF_COPY_FIELD(dst, &pidRuntime, itermLimitYaw);
    KF_COPY_FIELD(dst, &pidRuntime, itermRotation);
    KF_COPY_FIELD(dst, &pidRuntime, zeroThrottleItermReset);
    KF_COPY_FIELD(dst, &pidRuntime, levelRaceMode);
    KF_COPY_FIELD(dst, &pidRuntime, tpaFactor);
    KF_COPY_FIELD(dst, &pidRuntime, tpaBreakpoint);
    KF_COPY_FIELD(dst, &pidRuntime, tpaMultiplier);
    KF_COPY_FIELD(dst, &pidRuntime, tpaLowBreakpoint);
    KF_COPY_FIELD(dst, &pidRuntime, tpaLowMultiplier);
    KF_COPY_FIELD(dst, &pidRuntime, tpaLowAlways);
    KF_COPY_FIELD(dst, &pidRuntime, useEzDisarm);
    KF_COPY_FIELD(dst, &pidRuntime, landingDisarmThreshold);

#ifdef USE_ITERM_RELAX
    KF_COPY_ARRAY(dst, &pidRuntime, windupLpf);
    KF_COPY_FIELD(dst, &pidRuntime, itermRelax);
    KF_COPY_FIELD(dst, &pidRuntime, itermRelaxType);
    KF_COPY_FIELD(dst, &pidRuntime, itermRelaxCutoff);
#endif

#ifdef USE_ABSOLUTE_CONTROL
    KF_COPY_FIELD(dst, &pidRuntime, acCutoff);
    KF_COPY_FIELD(dst, &pidRuntime, acGain);
    KF_COPY_FIELD(dst, &pidRuntime, acLimit);
    KF_COPY_FIELD(dst, &pidRuntime, acErrorLimit);
    KF_COPY_ARRAY(dst, &pidRuntime, acLpf);
    KF_COPY_ARRAY(dst, &pidRuntime, oldSetpointCorrection);
#endif

#ifdef USE_D_MAX
    KF_COPY_ARRAY(dst, &pidRuntime, dMaxRange);
    KF_COPY_ARRAY(dst, &pidRuntime, dMaxLowpass);
    KF_COPY_ARRAY(dst, &pidRuntime, dMaxPercent);
    KF_COPY_ARRAY(dst, &pidRuntime, dMax);
    KF_COPY_FIELD(dst, &pidRuntime, dMaxGyroGain);
    KF_COPY_FIELD(dst, &pidRuntime, dMaxSetpointGain);
#endif

#ifdef USE_AIRMODE_LPF
    KF_COPY_FIELD(dst, &pidRuntime, airmodeThrottleLpf1);
    KF_COPY_FIELD(dst, &pidRuntime, airmodeThrottleLpf2);
    KF_COPY_FIELD(dst, &pidRuntime, airmodeThrottleOffsetLimit);
#endif

#ifdef USE_ACRO_TRAINER
    KF_COPY_FIELD(dst, &pidRuntime, acroTrainerAngleLimit);
    KF_COPY_FIELD(dst, &pidRuntime, acroTrainerLookaheadTime);
    KF_COPY_FIELD(dst, &pidRuntime, acroTrainerDebugAxis);
    KF_COPY_FIELD(dst, &pidRuntime, acroTrainerGain);
    KF_COPY_FIELD(dst, &pidRuntime, acroTrainerActive);
    KF_COPY_ARRAY(dst, &pidRuntime, acroTrainerAxisState);
#endif

#ifdef USE_DYN_LPF
    KF_COPY_FIELD(dst, &pidRuntime, dynLpfFilter);
    KF_COPY_FIELD(dst, &pidRuntime, dynLpfMin);
    KF_COPY_FIELD(dst, &pidRuntime, dynLpfMax);
    KF_COPY_FIELD(dst, &pidRuntime, dynLpfCurveExpo);
#endif

#ifdef USE_LAUNCH_CONTROL
    KF_COPY_FIELD(dst, &pidRuntime, launchControlMode);
    KF_COPY_FIELD(dst, &pidRuntime, launchControlAngleLimit);
    KF_COPY_FIELD(dst, &pidRuntime, launchControlKi);
#endif

#ifdef USE_INTEGRATED_YAW_CONTROL
    KF_COPY_FIELD(dst, &pidRuntime, useIntegratedYaw);
    KF_COPY_FIELD(dst, &pidRuntime, integratedYawRelax);
#endif

#ifdef USE_THRUST_LINEARIZATION
    KF_COPY_FIELD(dst, &pidRuntime, thrustLinearization);
    KF_COPY_FIELD(dst, &pidRuntime, throttleCompensateAmount);
#endif

#ifdef USE_FEEDFORWARD
    KF_COPY_FIELD(dst, &pidRuntime, feedforwardAveraging);
    KF_COPY_FIELD(dst, &pidRuntime, feedforwardSmoothFactor);
    KF_COPY_FIELD(dst, &pidRuntime, feedforwardJitterFactor);
    KF_COPY_FIELD(dst, &pidRuntime, feedforwardJitterFactorInv);
    KF_COPY_FIELD(dst, &pidRuntime, feedforwardBoostFactor);
    KF_COPY_FIELD(dst, &pidRuntime, feedforwardTransition);
    KF_COPY_FIELD(dst, &pidRuntime, feedforwardTransitionInv);
    KF_COPY_FIELD(dst, &pidRuntime, feedforwardMaxRateLimit);
    KF_COPY_FIELD(dst, &pidRuntime, feedforwardYawHoldGain);
    KF_COPY_FIELD(dst, &pidRuntime, feedforwardYawHoldTime);
    KF_COPY_FIELD(dst, &pidRuntime, feedforwardInterpolate);
    KF_COPY_ARRAY(dst, &pidRuntime, angleFeedforwardPt3);
#endif

#ifdef USE_ACC
    KF_COPY_ARRAY(dst, &pidRuntime, attitudeFilter);
    KF_COPY_FIELD(dst, &pidRuntime, horizonSmoothingPt1);
    KF_COPY_FIELD(dst, &pidRuntime, horizonDelayMs);
    KF_COPY_FIELD(dst, &pidRuntime, angleYawSetpoint);
    KF_COPY_FIELD(dst, &pidRuntime, angleEarthRef);
    KF_COPY_ARRAY(dst, &pidRuntime, angleTarget);
    KF_COPY_ARRAY(dst, &pidRuntime, axisInAngleMode);
#endif

#ifdef USE_WING
    KF_COPY_ARRAY(dst, &pidRuntime, spa);
    KF_COPY_FIELD(dst, &pidRuntime, tpaSpeed);
    KF_COPY_FIELD(dst, &pidRuntime, tpaFactorYaw);
    KF_COPY_ARRAY(dst, &pidRuntime, tpaFactorSterm);
#endif

#ifdef USE_ADVANCED_TPA
    KF_COPY_FIELD(dst, &pidRuntime, tpaCurvePwl);
    KF_COPY_ARRAY(dst, &pidRuntime, tpaCurvePwl_yValues);
    KF_COPY_FIELD(dst, &pidRuntime, tpaCurveType);
#endif

#ifdef USE_CHIRP
    KF_COPY_FIELD(dst, &pidRuntime, chirp);
    KF_COPY_FIELD(dst, &pidRuntime, chirpFilter);
    KF_COPY_FIELD(dst, &pidRuntime, chirpLagFreqHz);
    KF_COPY_FIELD(dst, &pidRuntime, chirpLeadFreqHz);
    KF_COPY_ARRAY(dst, &pidRuntime, chirpAmplitude);
    KF_COPY_FIELD(dst, &pidRuntime, chirpFrequencyStartHz);
    KF_COPY_FIELD(dst, &pidRuntime, chirpFrequencyEndHz);
    KF_COPY_FIELD(dst, &pidRuntime, chirpTimeSeconds);
#endif
}

static void kf_read_pid_runtime_state(const kf_pid_runtime_state_t *const src)
{
    KF_COPY_FIELD(&pidRuntime, src, dT);
    KF_COPY_FIELD(&pidRuntime, src, pidFrequency);
    KF_COPY_FIELD(&pidRuntime, src, pidStabilisationEnabled);
    KF_COPY_ARRAY(&pidRuntime, src, previousPidSetpoint);
    KF_COPY_ARRAY(&pidRuntime, src, dtermNotch);
    KF_COPY_ARRAY(&pidRuntime, src, dtermLowpass);
    KF_COPY_ARRAY(&pidRuntime, src, dtermLowpass2);
    KF_COPY_FIELD(&pidRuntime, src, ptermYawLowpass);
    KF_COPY_FIELD(&pidRuntime, src, antiGravityEnabled);
    KF_COPY_FIELD(&pidRuntime, src, antiGravityLpf);
    KF_COPY_FIELD(&pidRuntime, src, antiGravityOsdCutoff);
    KF_COPY_FIELD(&pidRuntime, src, antiGravityThrottleD);
    KF_COPY_FIELD(&pidRuntime, src, itermAccelerator);
    KF_COPY_FIELD(&pidRuntime, src, antiGravityGain);
    KF_COPY_FIELD(&pidRuntime, src, antiGravityPGain);
    KF_COPY_ARRAY(&pidRuntime, src, pidCoefficient);
    KF_COPY_FIELD(&pidRuntime, src, angleGain);
    KF_COPY_FIELD(&pidRuntime, src, angleFeedforwardGain);
    KF_COPY_FIELD(&pidRuntime, src, horizonGain);
    KF_COPY_FIELD(&pidRuntime, src, horizonLimitSticks);
    KF_COPY_FIELD(&pidRuntime, src, horizonLimitSticksInv);
    KF_COPY_FIELD(&pidRuntime, src, horizonLimitDegrees);
    KF_COPY_FIELD(&pidRuntime, src, horizonLimitDegreesInv);
    KF_COPY_FIELD(&pidRuntime, src, horizonIgnoreSticks);
    KF_COPY_ARRAY(&pidRuntime, src, maxVelocity);
    KF_COPY_FIELD(&pidRuntime, src, inCrashRecoveryMode);
    KF_COPY_FIELD(&pidRuntime, src, crashDetectedAtUs);
    KF_COPY_FIELD(&pidRuntime, src, crashTimeLimitUs);
    KF_COPY_FIELD(&pidRuntime, src, crashTimeDelayUs);
    KF_COPY_FIELD(&pidRuntime, src, crashRecoveryAngleDeciDegrees);
    KF_COPY_FIELD(&pidRuntime, src, crashRecoveryRate);
    KF_COPY_FIELD(&pidRuntime, src, crashGyroThreshold);
    KF_COPY_FIELD(&pidRuntime, src, crashDtermThreshold);
    KF_COPY_FIELD(&pidRuntime, src, crashSetpointThreshold);
    KF_COPY_FIELD(&pidRuntime, src, crashLimitYaw);
    KF_COPY_FIELD(&pidRuntime, src, itermLimit);
    KF_COPY_FIELD(&pidRuntime, src, itermLimitYaw);
    KF_COPY_FIELD(&pidRuntime, src, itermRotation);
    KF_COPY_FIELD(&pidRuntime, src, zeroThrottleItermReset);
    KF_COPY_FIELD(&pidRuntime, src, levelRaceMode);
    KF_COPY_FIELD(&pidRuntime, src, tpaFactor);
    KF_COPY_FIELD(&pidRuntime, src, tpaBreakpoint);
    KF_COPY_FIELD(&pidRuntime, src, tpaMultiplier);
    KF_COPY_FIELD(&pidRuntime, src, tpaLowBreakpoint);
    KF_COPY_FIELD(&pidRuntime, src, tpaLowMultiplier);
    KF_COPY_FIELD(&pidRuntime, src, tpaLowAlways);
    KF_COPY_FIELD(&pidRuntime, src, useEzDisarm);
    KF_COPY_FIELD(&pidRuntime, src, landingDisarmThreshold);

#ifdef USE_ITERM_RELAX
    KF_COPY_ARRAY(&pidRuntime, src, windupLpf);
    KF_COPY_FIELD(&pidRuntime, src, itermRelax);
    KF_COPY_FIELD(&pidRuntime, src, itermRelaxType);
    KF_COPY_FIELD(&pidRuntime, src, itermRelaxCutoff);
#endif

#ifdef USE_ABSOLUTE_CONTROL
    KF_COPY_FIELD(&pidRuntime, src, acCutoff);
    KF_COPY_FIELD(&pidRuntime, src, acGain);
    KF_COPY_FIELD(&pidRuntime, src, acLimit);
    KF_COPY_FIELD(&pidRuntime, src, acErrorLimit);
    KF_COPY_ARRAY(&pidRuntime, src, acLpf);
    KF_COPY_ARRAY(&pidRuntime, src, oldSetpointCorrection);
#endif

#ifdef USE_D_MAX
    KF_COPY_ARRAY(&pidRuntime, src, dMaxRange);
    KF_COPY_ARRAY(&pidRuntime, src, dMaxLowpass);
    KF_COPY_ARRAY(&pidRuntime, src, dMaxPercent);
    KF_COPY_ARRAY(&pidRuntime, src, dMax);
    KF_COPY_FIELD(&pidRuntime, src, dMaxGyroGain);
    KF_COPY_FIELD(&pidRuntime, src, dMaxSetpointGain);
#endif

#ifdef USE_AIRMODE_LPF
    KF_COPY_FIELD(&pidRuntime, src, airmodeThrottleLpf1);
    KF_COPY_FIELD(&pidRuntime, src, airmodeThrottleLpf2);
    KF_COPY_FIELD(&pidRuntime, src, airmodeThrottleOffsetLimit);
#endif

#ifdef USE_ACRO_TRAINER
    KF_COPY_FIELD(&pidRuntime, src, acroTrainerAngleLimit);
    KF_COPY_FIELD(&pidRuntime, src, acroTrainerLookaheadTime);
    KF_COPY_FIELD(&pidRuntime, src, acroTrainerDebugAxis);
    KF_COPY_FIELD(&pidRuntime, src, acroTrainerGain);
    KF_COPY_FIELD(&pidRuntime, src, acroTrainerActive);
    KF_COPY_ARRAY(&pidRuntime, src, acroTrainerAxisState);
#endif

#ifdef USE_DYN_LPF
    KF_COPY_FIELD(&pidRuntime, src, dynLpfFilter);
    KF_COPY_FIELD(&pidRuntime, src, dynLpfMin);
    KF_COPY_FIELD(&pidRuntime, src, dynLpfMax);
    KF_COPY_FIELD(&pidRuntime, src, dynLpfCurveExpo);
#endif

#ifdef USE_LAUNCH_CONTROL
    KF_COPY_FIELD(&pidRuntime, src, launchControlMode);
    KF_COPY_FIELD(&pidRuntime, src, launchControlAngleLimit);
    KF_COPY_FIELD(&pidRuntime, src, launchControlKi);
#endif

#ifdef USE_INTEGRATED_YAW_CONTROL
    KF_COPY_FIELD(&pidRuntime, src, useIntegratedYaw);
    KF_COPY_FIELD(&pidRuntime, src, integratedYawRelax);
#endif

#ifdef USE_THRUST_LINEARIZATION
    KF_COPY_FIELD(&pidRuntime, src, thrustLinearization);
    KF_COPY_FIELD(&pidRuntime, src, throttleCompensateAmount);
#endif

#ifdef USE_FEEDFORWARD
    KF_COPY_FIELD(&pidRuntime, src, feedforwardAveraging);
    KF_COPY_FIELD(&pidRuntime, src, feedforwardSmoothFactor);
    KF_COPY_FIELD(&pidRuntime, src, feedforwardJitterFactor);
    KF_COPY_FIELD(&pidRuntime, src, feedforwardJitterFactorInv);
    KF_COPY_FIELD(&pidRuntime, src, feedforwardBoostFactor);
    KF_COPY_FIELD(&pidRuntime, src, feedforwardTransition);
    KF_COPY_FIELD(&pidRuntime, src, feedforwardTransitionInv);
    KF_COPY_FIELD(&pidRuntime, src, feedforwardMaxRateLimit);
    KF_COPY_FIELD(&pidRuntime, src, feedforwardYawHoldGain);
    KF_COPY_FIELD(&pidRuntime, src, feedforwardYawHoldTime);
    KF_COPY_FIELD(&pidRuntime, src, feedforwardInterpolate);
    KF_COPY_ARRAY(&pidRuntime, src, angleFeedforwardPt3);
#endif

#ifdef USE_ACC
    KF_COPY_ARRAY(&pidRuntime, src, attitudeFilter);
    KF_COPY_FIELD(&pidRuntime, src, horizonSmoothingPt1);
    KF_COPY_FIELD(&pidRuntime, src, horizonDelayMs);
    KF_COPY_FIELD(&pidRuntime, src, angleYawSetpoint);
    KF_COPY_FIELD(&pidRuntime, src, angleEarthRef);
    KF_COPY_ARRAY(&pidRuntime, src, angleTarget);
    KF_COPY_ARRAY(&pidRuntime, src, axisInAngleMode);
#endif

#ifdef USE_WING
    KF_COPY_ARRAY(&pidRuntime, src, spa);
    KF_COPY_FIELD(&pidRuntime, src, tpaSpeed);
    KF_COPY_FIELD(&pidRuntime, src, tpaFactorYaw);
    KF_COPY_ARRAY(&pidRuntime, src, tpaFactorSterm);
#endif

#ifdef USE_ADVANCED_TPA
    KF_COPY_FIELD(&pidRuntime, src, tpaCurvePwl);
    KF_COPY_ARRAY(&pidRuntime, src, tpaCurvePwl_yValues);
    KF_COPY_FIELD(&pidRuntime, src, tpaCurveType);
#endif

#ifdef USE_CHIRP
    KF_COPY_FIELD(&pidRuntime, src, chirp);
    KF_COPY_FIELD(&pidRuntime, src, chirpFilter);
    KF_COPY_FIELD(&pidRuntime, src, chirpLagFreqHz);
    KF_COPY_FIELD(&pidRuntime, src, chirpLeadFreqHz);
    KF_COPY_ARRAY(&pidRuntime, src, chirpAmplitude);
    KF_COPY_FIELD(&pidRuntime, src, chirpFrequencyStartHz);
    KF_COPY_FIELD(&pidRuntime, src, chirpFrequencyEndHz);
    KF_COPY_FIELD(&pidRuntime, src, chirpTimeSeconds);
#endif
}

static void kf_write_gyro_state(kf_gyro_state_t *const dst)
{
    KF_COPY_ARRAY(dst, &gyro, gyroADC);
    KF_COPY_ARRAY(dst, &gyro, gyroADCf);
    KF_COPY_FIELD(dst, &gyro, sampleCount);
    KF_COPY_ARRAY(dst, &gyro, sampleSum);
    KF_COPY_FIELD(dst, &gyro, downsampleFilterEnabled);
    for (int i = 0; i < GYRO_COUNT; ++i) {
        dst->calibration[i] = gyro.gyroSensor[i].calibration;
    }
    KF_COPY_ARRAY(dst, &gyro, lowpassFilter);
    KF_COPY_ARRAY(dst, &gyro, lowpass2Filter);
    KF_COPY_ARRAY(dst, &gyro, notchFilter1);
    KF_COPY_ARRAY(dst, &gyro, notchFilter2);
    KF_COPY_ARRAY(dst, &gyro, imuGyroFilter);
    KF_COPY_FIELD(dst, &gyro, gyroEnabledBitmask);
    KF_COPY_FIELD(dst, &gyro, gyroDebugMode);
    KF_COPY_FIELD(dst, &gyro, gyroHasOverflowProtection);
    KF_COPY_FIELD(dst, &gyro, useMultiGyroDebugging);
    KF_COPY_FIELD(dst, &gyro, gyroDebugAxis);
#ifdef USE_DYN_LPF
    KF_COPY_FIELD(dst, &gyro, dynLpfFilter);
    KF_COPY_FIELD(dst, &gyro, dynLpfMin);
    KF_COPY_FIELD(dst, &gyro, dynLpfMax);
    KF_COPY_FIELD(dst, &gyro, dynLpfCurveExpo);
#endif
#ifdef USE_GYRO_OVERFLOW_CHECK
    KF_COPY_FIELD(dst, &gyro, overflowAxisMask);
#endif
}

static void kf_read_gyro_state(const kf_gyro_state_t *const src)
{
    KF_COPY_ARRAY(&gyro, src, gyroADC);
    KF_COPY_ARRAY(&gyro, src, gyroADCf);
    KF_COPY_FIELD(&gyro, src, sampleCount);
    KF_COPY_ARRAY(&gyro, src, sampleSum);
    KF_COPY_FIELD(&gyro, src, downsampleFilterEnabled);
    for (int i = 0; i < GYRO_COUNT; ++i) {
        gyro.gyroSensor[i].calibration = src->calibration[i];
    }
    KF_COPY_ARRAY(&gyro, src, lowpassFilter);
    KF_COPY_ARRAY(&gyro, src, lowpass2Filter);
    KF_COPY_ARRAY(&gyro, src, notchFilter1);
    KF_COPY_ARRAY(&gyro, src, notchFilter2);
    KF_COPY_ARRAY(&gyro, src, imuGyroFilter);
    KF_COPY_FIELD(&gyro, src, gyroEnabledBitmask);
    KF_COPY_FIELD(&gyro, src, gyroDebugMode);
    KF_COPY_FIELD(&gyro, src, gyroHasOverflowProtection);
    KF_COPY_FIELD(&gyro, src, useMultiGyroDebugging);
    KF_COPY_FIELD(&gyro, src, gyroDebugAxis);
#ifdef USE_DYN_LPF
    KF_COPY_FIELD(&gyro, src, dynLpfFilter);
    KF_COPY_FIELD(&gyro, src, dynLpfMin);
    KF_COPY_FIELD(&gyro, src, dynLpfMax);
    KF_COPY_FIELD(&gyro, src, dynLpfCurveExpo);
#endif
#ifdef USE_GYRO_OVERFLOW_CHECK
    KF_COPY_FIELD(&gyro, src, overflowAxisMask);
#endif
}

static void kf_write_acc_state(kf_acc_state_t *const dst)
{
    KF_COPY_FIELD(dst, &acc, accADC);
    KF_COPY_FIELD(dst, &acc, jerk);
    KF_COPY_FIELD(dst, &acc, accMagnitude);
    KF_COPY_FIELD(dst, &acc, jerkMagnitude);
    KF_COPY_FIELD(dst, &acc, isAccelUpdatedAtLeastOnce);
}

static void kf_read_acc_state(const kf_acc_state_t *const src)
{
    KF_COPY_FIELD(&acc, src, accADC);
    KF_COPY_FIELD(&acc, src, jerk);
    KF_COPY_FIELD(&acc, src, accMagnitude);
    KF_COPY_FIELD(&acc, src, jerkMagnitude);
    KF_COPY_FIELD(&acc, src, isAccelUpdatedAtLeastOnce);
}

KF_EXPORT uint32_t kf_reconcile_output(void *out_state, const uint32_t out_size)
{
    const uint32_t required = (uint32_t)sizeof(kf_reconcile_state_t);

    // Passing NULL gives us the required size
    if (!out_state) {
        return required;
    }
    KF_ASSERT_BASIC(out_size == required);

    kf_reconcile_state_t *state = (kf_reconcile_state_t *)out_state;
    memset(state, 0, required);

    state->externalFrame = externalFrame;
    state->targetPidLooptime = targetPidLooptime;
    state->activePidLoopDenom = activePidLoopDenom;
    memcpy(state->pidData, pidData, sizeof(state->pidData));
    kf_write_pid_runtime_state(&state->pidRuntime);
    kf_write_gyro_state(&state->gyro);
    kf_write_acc_state(&state->acc);
    state->attitude = attitude;
    state->imuAttitudeQuaternion = imuAttitudeQuaternion;
    state->headfree = headfree;
    state->offset = offset;
    state->canUseGPSHeading = canUseGPSHeading;
    memcpy(state->rcCommand, rcCommand, sizeof(state->rcCommand));

    rcSmoothingFilter_t *smoothing = getRcSmoothingData();
    if (smoothing) {
        state->rcSmoothing = *smoothing;
    }

#ifdef USE_THROTTLE_BOOST
    state->throttleBoost = throttleBoost;
    state->throttleLpf = throttleLpf;
#endif

    return required;
}

KF_EXPORT uint32_t kf_reconcile_input(const void *in_state, const uint32_t in_size)
{
    KF_ASSERT_BASIC(in_state);

    const kf_reconcile_state_t *state = (const kf_reconcile_state_t *)in_state;
    KF_ASSERT_BASIC(in_size == sizeof(*state));

    externalFrame = state->externalFrame;
    targetPidLooptime = state->targetPidLooptime;
    activePidLoopDenom = state->activePidLoopDenom;
    memcpy(pidData, state->pidData, sizeof(state->pidData));
    kf_read_pid_runtime_state(&state->pidRuntime);
    kf_read_gyro_state(&state->gyro);
    kf_read_acc_state(&state->acc);

    imuSetAttitudeQuat(state->imuAttitudeQuaternion.w,
                       state->imuAttitudeQuaternion.x,
                       state->imuAttitudeQuaternion.y,
                       state->imuAttitudeQuaternion.z);
    attitude = state->attitude;
    imuAttitudeQuaternion = state->imuAttitudeQuaternion;
    headfree = state->headfree;
    offset = state->offset;
    canUseGPSHeading = state->canUseGPSHeading;

    memcpy(rcCommand, state->rcCommand, sizeof(state->rcCommand));
    rcSmoothingFilter_t *smoothing = getRcSmoothingData();
    if (smoothing) {
        *smoothing = state->rcSmoothing;
    }

#ifdef USE_THROTTLE_BOOST
    throttleBoost = state->throttleBoost;
    throttleLpf = state->throttleLpf;
#endif

    return 1;
}

KF_EXPORT void kf_initialize(const void* eepromInData, const uint32_t eepromInSize) {
#ifdef USE_FLUSH_IO
    FILE* err = freopen("stdout.log", "w", stdout);
    FILE* out = freopen("stderr.log", "w", stderr);
    (void)err;
    (void)out;
#endif
    if (eepromInSize == EEPROM_SIZE) {
        memcpy(eepromInitialData, eepromInData, EEPROM_SIZE);
    }
    init();
}

void set_channel(int channel, rc_packet* rcpkt, double value) {
    rxConfig_t* config = rxConfig();
    uint16_t pwm = value >= 0.0f
        ? config->midrc + (uint16_t)((config->rx_max_usec - config->midrc) * value)
        : config->rx_min_usec + (uint16_t)((config->midrc - config->rx_min_usec) * (1.0f + value));
    rcpkt->channels[channel] = pwm;
}

KF_EXPORT void kf_iteration(const uint32_t iterations, const uint32_t flags, struct flight_states *const flight_states, const uint32_t state_index) {
    // Timestamp is only used for timeout and adjust sim speed which we don't want
    const double timestamp = 0;

    rc_packet rcpkt;
    rcpkt.timestamp = timestamp;

    // AETR1234
    set_channel(0, &rcpkt, controller_states_get_axis_command(&flight_states->controller_states, state_index, KF_AXIS_ROLL));
    set_channel(1, &rcpkt, controller_states_get_axis_command(&flight_states->controller_states, state_index, KF_AXIS_PITCH));
    set_channel(2, &rcpkt, controller_states_get_axis_command(&flight_states->controller_states, state_index, KF_AXIS_THROTTLE));
    set_channel(3, &rcpkt, controller_states_get_axis_command(&flight_states->controller_states, state_index, KF_AXIS_YAW));
    set_channel(4, &rcpkt, controller_states_get_arm_command(&flight_states->controller_states, state_index));
    set_channel(5, &rcpkt, controller_states_get_mode_command(&flight_states->controller_states, state_index));
    for (int i = 6; i < SIMULATOR_MAX_RC_CHANNELS; ++i) {
        rcpkt.channels[i] = rxConfig()->rx_min_usec;
    }
    updateRCInput(&rcpkt);

    // BetaFlight:
    // X - Forward
    // Y - Left
    // Z - Up

    // Unity:
    // X - Right
    // Y - Up
    // Z - Forward

    // BetaFlight X =  Unity Z
    // BetaFlight Y = -Unity X
    // BetaFlight Z =  Unity Y

    // Quaternion W is the same

    fdm_packet fdmpkt;
    fdmpkt.timestamp = timestamp;
    fdmpkt.imu_angular_velocity_rpy[0] = -physics_states_get_angular_velocity_local(&flight_states->physics_states, state_index, KF_Z);
    fdmpkt.imu_angular_velocity_rpy[1] = -physics_states_get_angular_velocity_local(&flight_states->physics_states, state_index, KF_X);
    fdmpkt.imu_angular_velocity_rpy[2] = +physics_states_get_angular_velocity_local(&flight_states->physics_states, state_index, KF_Y);

    fdmpkt.imu_linear_acceleration_xyz[0] = -physics_states_get_linear_acceleration_world(&flight_states->physics_states, state_index, KF_Z);
    fdmpkt.imu_linear_acceleration_xyz[1] = -physics_states_get_linear_acceleration_world(&flight_states->physics_states, state_index, KF_X);
    fdmpkt.imu_linear_acceleration_xyz[2] = +physics_states_get_linear_acceleration_world(&flight_states->physics_states, state_index, KF_Y);

    fdmpkt.imu_orientation_quat[0] = +physics_states_get_orientation_world(&flight_states->physics_states, state_index, KF_W);
    fdmpkt.imu_orientation_quat[1] = -physics_states_get_orientation_world(&flight_states->physics_states, state_index, KF_Z);
    fdmpkt.imu_orientation_quat[2] = -physics_states_get_orientation_world(&flight_states->physics_states, state_index, KF_X);
    fdmpkt.imu_orientation_quat[3] = +physics_states_get_orientation_world(&flight_states->physics_states, state_index, KF_Y);

    fdmpkt.velocity_xyz[0] = -physics_states_get_linear_velocity_world(&flight_states->physics_states, state_index, KF_Z);
    fdmpkt.velocity_xyz[1] = -physics_states_get_linear_velocity_world(&flight_states->physics_states, state_index, KF_X);
    fdmpkt.velocity_xyz[2] = +physics_states_get_linear_velocity_world(&flight_states->physics_states, state_index, KF_Y);

    fdmpkt.position_xyz[0] = -physics_states_get_position_world(&flight_states->physics_states, state_index, KF_Z);
    fdmpkt.position_xyz[1] = -physics_states_get_position_world(&flight_states->physics_states, state_index, KF_X);
    fdmpkt.position_xyz[2] = +physics_states_get_position_world(&flight_states->physics_states, state_index, KF_Y);

    fdmpkt.pressure = drone_states_get_barometer_pressure_pa(&flight_states->drone_states, state_index);
    updateState(&fdmpkt);
    
    uartDataIn(
        0,
        firmware_states_get_uart_data_in_configurator(&flight_states->firmware_states, state_index),
        firmware_states_get_uart_data_in_size_configurator(&flight_states->firmware_states, state_index));
    firmware_states_set_uart_data_in_configurator(&flight_states->firmware_states, state_index, NULL);
    firmware_states_set_uart_data_in_size_configurator(&flight_states->firmware_states, state_index, 0);

    char cameraAngle = firmware_states_get_camera_angle_in(&flight_states->firmware_states, state_index);
    if (cameraAngle != KF_INVALID_CAMERA_ANGLE) {
        rxConfigMutable()->fpvCamAngleDegrees = cameraAngle;
    }

    for (uint32_t i = 0; i < iterations; ++i) {
        scheduler();
        ++externalFrame;
    }

    double outScale = 1000.0;
    if (featureIsEnabled(FEATURE_3D)) {
        outScale = 500.0;
    }

    drone_states_set_motor_count(&flight_states->drone_states, state_index, (char)pwmRawPkt.motorCount);
    for (uint32_t i = 0; i < pwmRawPkt.motorCount; ++i) {
        drone_states_set_motor_outputs(&flight_states->drone_states, state_index, i,  motorsPwm[i] / outScale);
    }

    if ((flags & KF_FLAGS_FINAL_ITERATION) != 0) {
        firmware_states_set_camera_angle_out(&flight_states->firmware_states, state_index, (char)rxConfig()->fpvCamAngleDegrees);

        if (saveEEPROM) {
            firmware_states_set_eeprom_out(&flight_states->firmware_states, state_index, eepromData);
            firmware_states_set_eeprom_out_size(&flight_states->firmware_states, state_index, EEPROM_SIZE);
            saveEEPROM = false;
        }

        firmware_states_set_reboot_request(&flight_states->firmware_states, state_index, (uint32_t)rebootRequest);
        rebootRequest = (bootloaderRequestType_e)KF_REBOOT_REQUEST_NONE;

        for (int i = 0; i < KF_MAX_UART_CHANNELS; ++i) {
            uartBuffer* src = &uartBuffers[i];
            firmware_states_set_uart_data_out(&flight_states->firmware_states, state_index, i, src->uartDataOut);
            firmware_states_set_uart_data_out_size(&flight_states->firmware_states, state_index, i, src->uartDataSize);
            src->uartDataSize = 0;
        }
    }

#ifdef USE_FLUSH_IO
    fflush(stdout);
    fflush(stderr);
#endif
}

void uartDataOut(int id, const void* data, const int size) {
    KF_ASSERT_FORMAT(id >= 0 && id < KF_MAX_UART_CHANNELS, "Unhandled UART id, increase channels if needed: %d", id);

    uartBuffer* buffer = &uartBuffers[id];
    uint32_t newSize = buffer->uartDataSize + size;
    KF_ASSERT_FORMAT(newSize < UART_BUFFER_SIZE, "UART data buffer overflow: newSize(%d) uartDataSize(%d) size(%d)",
            newSize, buffer->uartDataSize, size);

    memcpy(buffer->uartDataOut + buffer->uartDataSize, data, size);
    buffer->uartDataSize = newSize;
}

bool loadEEPROMFromFile(void)
{
    memcpy(eepromData, eepromInitialData, EEPROM_SIZE);
/*
    if (eepromFd != NULL) {
        fprintf(stderr, "[FLASH_Unlock] eepromFd != NULL\n");
        return false;
    }

    // open or create
    eepromFd = fopen(EEPROM_FILENAME, "r+");
    if (eepromFd != NULL) {
        // obtain file size:
        fseek(eepromFd, 0, SEEK_END);
        size_t lSize = ftell(eepromFd);
        rewind(eepromFd);

        size_t n = fread(eepromData, 1, sizeof(eepromData), eepromFd);
        if (n == lSize) {
            printf("[FLASH_Unlock] loaded '%s', size = %ld / %ld\n", EEPROM_FILENAME, lSize, sizeof(eepromData));
        } else {
            fprintf(stderr, "[FLASH_Unlock] failed to load '%s'\n", EEPROM_FILENAME);
            return false;
        }
    } else {
        printf("[FLASH_Unlock] created '%s', size = %ld\n", EEPROM_FILENAME, sizeof(eepromData));
        if ((eepromFd = fopen(EEPROM_FILENAME, "w+")) == NULL) {
            fprintf(stderr, "[FLASH_Unlock] failed to create '%s'\n", EEPROM_FILENAME);
            return false;
        }

        if (fwrite(eepromData, sizeof(eepromData), 1, eepromFd) != 1) {
            fprintf(stderr, "[FLASH_Unlock] write failed: %s\n", strerror(errno));
            return false;
        }
    }
*/
    return true;
}

void configUnlock(void)
{
    loadEEPROMFromFile();
}

void configLock(void)
{
    saveEEPROM = true;
/*
    // flush & close
    if (eepromFd != NULL) {
        fseek(eepromFd, 0, SEEK_SET);
        fwrite(eepromData, 1, sizeof(eepromData), eepromFd);
        fclose(eepromFd);
        eepromFd = NULL;
        printf("[FLASH_Lock] saved '%s'\n", EEPROM_FILENAME);
    } else {
        fprintf(stderr, "[FLASH_Lock] eeprom is not unlocked\n");
    }
*/
}

configStreamerResult_e configWriteWord(uintptr_t address, config_streamer_buffer_type_t *buffer)
{
    STATIC_ASSERT(CONFIG_STREAMER_BUFFER_SIZE == sizeof(uint32_t), "CONFIG_STREAMER_BUFFER_SIZE does not match written size");

    if ((address >= (uintptr_t)eepromData) && (address + sizeof(uint32_t) <= (uintptr_t)ARRAYEND(eepromData))) {
        memcpy((void*)address, buffer, sizeof(config_streamer_buffer_type_t));
        printf("[FLASH_ProgramWord]%p = %08x\n", (void*)address, *((uint32_t*)address));
    } else {
        printf("[FLASH_ProgramWord]%p out of range!\n", (void*)address);
    }
    return CONFIG_RESULT_SUCCESS;
}

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    UNUSED(io);
    UNUSED(cfg);
    printf("IOConfigGPIO\n");
}

void spektrumBind(rxConfig_t *rxConfig)
{
    UNUSED(rxConfig);
    printf("spektrumBind\n");
}

void debugInit(void)
{
    printf("debugInit\n");
}

void unusedPinsInit(void)
{
    printf("unusedPinsInit\n");
}

void IOHi(IO_t io)
{
    UNUSED(io);
}

void IOLo(IO_t io)
{
    UNUSED(io);
}

void IOInitGlobal(void)
{
    // NOOP
}

IO_t IOGetByTag(ioTag_t tag)
{
    UNUSED(tag);
    return NULL;
}

const mcuTypeInfo_t *getMcuTypeInfo(void)
{
    static const mcuTypeInfo_t info = { .id = MCU_TYPE_SIMULATOR, .name = "SIMULATOR" };
    return &info;
}

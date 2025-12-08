/*
* apds9960.h
*
*  Created on: Nov 19, 2025
*      Author: ShaniyaThomas
*/
#ifndef INC_APDS9960_H_
#define INC_APDS9960_H_
#include <stdint.h>
#include <stm32l4xx_hal.h>
enum Gesture {
 GESTURE_NONE = -1,
 GESTURE_UP = 0,
 GESTURE_DOWN = 1,
 GESTURE_LEFT = 2,
 GESTURE_RIGHT = 3
};

typedef struct {
    I2C_HandleTypeDef *i2c;   // which I2C bus this sensor uses
    volatile int Gesture_Flag;
    int gestureEnabled;
    int proximityEnabled;
    int colorEnabled;

    int gestureIn;
    int gestureDirectionX;
    int gestureDirectionY;
    int gestureDirInX;
    int gestureDirInY;
    int gestureSensitivity;
    int detectedGesture;
} APDS9960_t;


//extern int _gestureEnabled;
//extern int _proximityEnabled;
//extern int _colorEnabled;
//extern int _gestureIn;
//extern int _gestureDirectionX;
//extern int _gestureDirectionY;
//extern int _gestureDirInX;
//extern int _gestureDirInY;
//extern int _gestureSensitivity;
//extern int _detectedGesture;
int i2c1_write(APDS9960_t *dev, uint8_t reg, uint8_t value);
int i2c1_read(APDS9960_t *dev, uint8_t reg, uint8_t *buffer, uint8_t quantity);
void sensor_init(APDS9960_t *dev, I2C_HandleTypeDef *hi2c);
void i2c1_readBlock(APDS9960_t *dev, uint8_t reg, uint8_t *val, unsigned int len);
int begin(APDS9960_t *dev);
int end(APDS9960_t *dev);
int gestureAvailable(APDS9960_t *dev);
enum Gesture readGesture(APDS9960_t *dev);
int colorAvailable(APDS9960_t *dev);
//int readColor(int& r, int& g, int& b);
int proximityAvailable(APDS9960_t *dev);
int readProximity(APDS9960_t *dev);
void setGestureSensitivity(APDS9960_t *dev, uint8_t sensitivity);
void setInterruptPin(APDS9960_t *dev, int pin);
int setLEDBoost(APDS9960_t *dev, uint8_t boost);
int setGestureIntEnable(APDS9960_t *dev, int en);
int setGestureMode(APDS9960_t *dev, int en);
int gestureFIFOAvailable(APDS9960_t *dev);
int handleGesture(APDS9960_t *dev);
int enablePower(APDS9960_t *dev);
int disablePower(APDS9960_t *dev);
int enableColor(APDS9960_t *dev);
int disableColor(APDS9960_t *dev);
int enableProximity(APDS9960_t *dev);
int disableProximity(APDS9960_t *dev);
int enableWait(APDS9960_t *dev);
int disableWait(APDS9960_t *dev);
int enableGesture(APDS9960_t *dev);
int disableGesture(APDS9960_t *dev);
//HELPER FUNCTIONS
int setENABLE(APDS9960_t *dev, uint8_t val);
int setWTIME(APDS9960_t *dev, uint8_t val);
int setGPULSE(APDS9960_t *dev, uint8_t val);
int setPPULSE(APDS9960_t *dev, uint8_t val);
int setATIME(APDS9960_t *dev, uint8_t val);
int setCONTROL(APDS9960_t *dev, uint8_t val);
int getENABLE(APDS9960_t *dev, uint8_t* val);
int getGCONF4(APDS9960_t *dev, uint8_t* val);
int setGCONF4(APDS9960_t *dev, uint8_t val);
int getCONFIG2(APDS9960_t *dev, uint8_t* val);
int setCONFIG2(APDS9960_t *dev, uint8_t val);
int getSTATUS(APDS9960_t *dev, uint8_t* val);
int getPDATA(APDS9960_t *dev, uint8_t* val);
int getGSTATUS(APDS9960_t *dev, uint8_t* val);
int getGFLVL(APDS9960_t *dev, uint8_t* val);
int readGFIFO_U(APDS9960_t *dev, uint8_t *val, uint8_t len);
//Addresses
/* APDS-9960 I2C address */
#define APDS9960_I2C_ADDR       0x39<<1
/* Gesture parameters */
#define GESTURE_THRESHOLD_OUT   10
#define GESTURE_SENSITIVITY_1   50
#define GESTURE_SENSITIVITY_2   20
/* Error code for returned values */
#define APDS_ERROR 					0xFF
/* Acceptable device IDs */
#define APDS9960_ID_1           0xAB
#define APDS9960_ID_2           0x9C
/* Misc parameters */
#define FIFO_PAUSE_TIME         30      // Wait period (ms) between FIFO reads
/* APDS-9960 register addresses */
#define APDS9960_ENABLE         0x80
#define APDS9960_ATIME          0x81
#define APDS9960_WTIME          0x83
#define APDS9960_AILTL          0x84
#define APDS9960_AILTH          0x85
#define APDS9960_AIHTL          0x86
#define APDS9960_AIHTH          0x87
#define APDS9960_PILT           0x89
#define APDS9960_PIHT           0x8B
#define APDS9960_PERS           0x8C
#define APDS9960_CONFIG1        0x8D
#define APDS9960_PPULSE         0x8E
#define APDS9960_CONTROL        0x8F
#define APDS9960_CONFIG2        0x90
#define APDS9960_ID             0x92
#define APDS9960_STATUS         0x93
#define APDS9960_CDATAL         0x94
#define APDS9960_CDATAH         0x95
#define APDS9960_RDATAL         0x96
#define APDS9960_RDATAH         0x97
#define APDS9960_GDATAL         0x98
#define APDS9960_GDATAH         0x99
#define APDS9960_BDATAL         0x9A
#define APDS9960_BDATAH         0x9B
#define APDS9960_PDATA          0x9C
#define APDS9960_POFFSET_UR     0x9D
#define APDS9960_POFFSET_DL     0x9E
#define APDS9960_CONFIG3        0x9F
#define APDS9960_GPENTH         0xA0
#define APDS9960_GEXTH          0xA1
#define APDS9960_GCONF1         0xA2
#define APDS9960_GCONF2         0xA3
#define APDS9960_GOFFSET_U      0xA4
#define APDS9960_GOFFSET_D      0xA5
#define APDS9960_GOFFSET_L      0xA7
#define APDS9960_GOFFSET_R      0xA9
#define APDS9960_GPULSE         0xA6
#define APDS9960_GCONF3         0xAA
#define APDS9960_GCONF4         0xAB
#define APDS9960_GFLVL          0xAE
#define APDS9960_GSTATUS        0xAF
#define APDS9960_IFORCE         0xE4
#define APDS9960_PICLEAR        0xE5
#define APDS9960_CICLEAR        0xE6
#define APDS9960_AICLEAR        0xE7
#define APDS9960_GFIFO_U        0xFC
#define APDS9960_GFIFO_D        0xFD
#define APDS9960_GFIFO_L        0xFE
#define APDS9960_GFIFO_R        0xFF
#endif /* INC_APDS9960_H_ */


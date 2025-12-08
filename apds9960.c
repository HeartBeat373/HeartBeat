/*
* apds9960.c
*
*  Created on: Nov 19, 2025
*      Author: ShaniyaThomas
*/
#include "apds9960.h"
#include "stm32l4xx_hal.h"
//extern I2C_HandleTypeDef hi2c1;
//extern I2C_HandleTypeDef hi2c2;
//int _gestureEnabled = 0;
//int _proximityEnabled = 0;
//int _colorEnabled = 0;
//int _gestureIn = 0;
//int _gestureDirectionX = 0;
//int _gestureDirectionY = 0;
//int _gestureDirInX = 0;
//int _gestureDirInY = 0;
//int _gestureSensitivity = 10;
//int _detectedGesture = 0;

extern volatile int Pause_Skip_Gesture_Flag = 0;
extern volatile int Volume_Speed_Gesture_Flag = 0;

int i2c1_write(APDS9960_t *dev, uint8_t reg, uint8_t value)
{
   uint8_t data[2] = {reg, value};
   if(HAL_I2C_Master_Transmit(dev->i2c, APDS9960_I2C_ADDR, data, 2, 1000) == HAL_OK) {
   	return 1;
   }
   return 0;

   // HAL_Delay(500);
}
int i2c1_read(APDS9960_t *dev, uint8_t reg, uint8_t *buffer, uint8_t quantity)
{
   // send register address
   if (HAL_I2C_Master_Transmit(dev->i2c, APDS9960_I2C_ADDR, &reg, 1, 1000) != HAL_OK) {
   	return 0;
   }
   // read bytes and store in buffer
   if(HAL_I2C_Master_Receive(dev->i2c, APDS9960_I2C_ADDR, buffer, quantity, 1000) == HAL_OK) {
   	return 1;
   }
   return 0;
}
void sensor_init(APDS9960_t *dev, I2C_HandleTypeDef *hi2c) {
	dev->i2c = hi2c;
	dev->Gesture_Flag        = 0;
	dev->gestureEnabled     = 0;
	dev->proximityEnabled   = 0;
	dev->colorEnabled       = 0;

	dev->gestureIn          = 0;
	dev->gestureDirectionX  = 0;
	dev->gestureDirectionY  = 0;
	dev->gestureDirInX      = 0;
	dev->gestureDirInY      = 0;

	dev->gestureSensitivity = 10;
	dev->detectedGesture    = GESTURE_NONE;
}

int begin(APDS9960_t *dev) {
	uint8_t id;
	i2c1_read(dev, APDS9960_ID, &id, 1);
//	if (id!=0xAB) { //our address is different
//		return 0;
//	}
	//disable everything
	if (!setENABLE(dev, 0x00)) return 0;
	if (!setWTIME(dev, 0xFF)) return 0;
	if (!setGPULSE(dev, 0x8F)) return 0; // 16us, 16 pulses // default is: 0x40 = 8us, 1 pulse
	if (!setPPULSE(dev, 0x8F)) return 0; // 16us, 16 pulses // default is: 0x40 = 8us, 1 pulse
	if (!setGestureIntEnable(dev, 1)) return 0;
//	if(!f) return 0;
	if (!setGestureMode(dev, 1)) return 0;
	if (!enablePower(dev)) return 0;
	if (!enableWait(dev)) return 0;
	// set ADC integration time to 10 ms
	if (!setATIME(dev, 256 - (10 / 2.78))) return 0;
	// set ADC gain 4x (0x00 => 1x, 0x01 => 4x, 0x02 => 16x, 0x03 => 64x)
	if (!setCONTROL(dev, 0x02)) return 0;
	HAL_Delay(10);
	  // enable power
	if (!enablePower(dev)) return 0;
	return 1;
}
int end(APDS9960_t *dev) {
	 return setENABLE(dev, 0x00);
}
//int gestureAvailable() { //used different logic
//	 if (!_gestureEnabled) enableGesture();
////	 if (digitalRead(_intPin) != LOW) { //TODO: how to tell when interrupt line is low
////	      return 0;
////	  }
////	 else if (gestureFIFOAvailable() <= 0) {
////	    return 0;
////	  }
//   // Check GSTATUS -> GVALID
//   uint8_t r;
//   if (!getGSTATUS(&r)) return 0;
//   if ((r & 0x01) == 0) return 0; // no data
//   if(!Gesture_Flag) return 0;
//    //data present: process FIFO
//   if (gestureFIFOAvailable() <= 0) return 0;
//   handleGesture();
//   if (_proximityEnabled) setGestureMode(0);
//   return (_detectedGesture == GESTURE_NONE) ? 0 : 1;
////	 uint8_t val = getGSTATUS();
////
////	    /* Shift and mask out GVALID bit */
////	 val &= 0x01;
////
////	    /* Return true/false based on GVALID bit */
////	 if(val == 1) {
////		return 1;
////	 }
////	 else {
////	    return 0;
////	 }
////
////	 if(!val) {
////		return 0;
////	 }
////	 else if (gestureFIFOAvailable() <= 0) {
////		return 0;
////	 }
////
////	 handleGesture();
////
////	 if (_proximityEnabled) {
////	    setGestureMode(0);
////	  }
////
////	 return (_detectedGesture == GESTURE_NONE) ? 0 : 1;
//}

int gestureAvailable(APDS9960_t *dev)
{
	  if (!dev->gestureEnabled) enableGesture(dev);

	  if(!dev->Gesture_Flag) return 0;

	  if (gestureFIFOAvailable(dev) <= 0) {
	    return 0;
	  }

	  handleGesture(dev);
	  if (dev->proximityEnabled) {
	    setGestureMode(dev, 0);
	  }
	  return (dev->detectedGesture == GESTURE_NONE) ? 0 : 1;
}


enum Gesture readGesture(APDS9960_t *dev) {
	  int gesture = dev->detectedGesture;
	  dev->detectedGesture = GESTURE_NONE;
	  return gesture;
}
int colorAvailable(APDS9960_t *dev) {
	  uint8_t r;
	  enableColor(dev);
	  if (!getSTATUS(dev, &r)) {
	    return 0;
	  }
	  if (r & 0b00000001) {
	    return 1;
	  }
	  return 0;
}
//int readColor(int& r, int& g, int& b) {
//
//}
int proximityAvailable(APDS9960_t *dev) {
	  uint8_t r;
	  enableProximity(dev);
	  if (!getSTATUS(dev, &r)) {
	    return 0;
	  }
	  if (r & 0b00000010) {
	    return 1;
	  }
	  return 0;
}
int readProximity(APDS9960_t *dev) {
	  uint8_t r;
	  if (!getPDATA(dev, &r)) {
	    return -1;
	  }
	  disableProximity(dev);
	  return (255 - r);
}
//void setGestureSensitivity(uint8_t sensitivity) {
//
//}
//
//void setInterruptPin(int pin) {
//
//}
int setLEDBoost(APDS9960_t *dev, uint8_t boost) {
	uint8_t r;
	if (!getCONFIG2(dev, &r)) return 0;
	r &= 0b11001111;
	r |= (boost << 4) & 0b00110000;
	return setCONFIG2(dev, r);
}
int setGestureIntEnable(APDS9960_t *dev, int en) {
	uint8_t r;
	if (!getGCONF4(dev, &r)) return 0;
	if (en) {
	    r |= 0b00000010;
	} else {
	    r &= 0b11111101;
	}
	return setGCONF4(dev, r);
}
int setGestureMode(APDS9960_t *dev, int en) {
	uint8_t r;
	if (!getGCONF4(dev, &r)) return 0;
	if (en) {
	    r |= 0b00000001;
	} else {
	    r &= 0b11111110;
	}
	return setGCONF4(dev, r);
}
int gestureFIFOAvailable(APDS9960_t *dev) {
	uint8_t r;
	if (!getGSTATUS(dev, &r)) return -1;
	if ((r & 0x01) == 0x00) return -2;
	if (!getGFLVL(dev, &r)) return -3;
	return r;
}
int handleGesture(APDS9960_t *dev) {
	const int gestureThreshold = 30;
	  while (1) {
	    int available = gestureFIFOAvailable(dev);
	    if (available <= 0) return 0;
	    uint8_t fifo_data[128];
	    uint8_t bytes_read = readGFIFO_U(dev, fifo_data, available * 4);
	    if (bytes_read == 0) return 0;
	    for (int i = 0; i+3 < bytes_read; i+=4) {
	      uint8_t u,d,l,r;
	      u = fifo_data[i];
	      d = fifo_data[i+1];
	      l = fifo_data[i+2];
	      r = fifo_data[i+3];
	      // Serial.print(u);
	      // Serial.print(",");
	      // Serial.print(d);
	      // Serial.print(",");
	      // Serial.print(l);
	      // Serial.print(",");
	      // Serial.println(r);
	      if (u<gestureThreshold && d<gestureThreshold && l<gestureThreshold && r<gestureThreshold) {
	    	  dev->gestureIn = 1;
	        if (dev->gestureDirInX != 0 || dev->gestureDirInY != 0) {
	          int totalX = dev->gestureDirInX - dev->gestureDirectionX;
	          int totalY = dev->gestureDirInY - dev->gestureDirectionY;
	          // Serial.print("OUT ");
	          // Serial.print(totalX);
	          // Serial.print(",");
	          // Serial.println(totalY);
	          if (totalX < -dev->gestureSensitivity) { dev->detectedGesture = GESTURE_LEFT; }
	          if (totalX > dev->gestureSensitivity) { dev->detectedGesture = GESTURE_RIGHT; }
	          if (totalY < -dev->gestureSensitivity) { dev->detectedGesture = GESTURE_DOWN; }
	          if (totalY > dev->gestureSensitivity) { dev->detectedGesture = GESTURE_UP; }
	          dev->gestureDirectionX = 0;
	          dev->gestureDirectionY = 0;
	          dev->gestureDirInX = 0;
	          dev->gestureDirInY = 0;
	        }
	        continue;
	      }
	      dev->gestureDirectionX = r - l;
	      dev->gestureDirectionY = u - d;
	      if (dev->gestureIn) {
	    	  dev->gestureIn = 0;
	    	  dev->gestureDirInX = dev->gestureDirectionX;
	    	  dev->gestureDirInY = dev->gestureDirectionY;
	        // Serial.print("IN ");
	        // Serial.print(_gestureDirInX);
	        // Serial.print(",");
	        // Serial.print(_gestureDirInY);
	        // Serial.print(" ");
	      }
	    }
	  }
}
int enablePower(APDS9960_t *dev) {
	  uint8_t r;
	  if (!getENABLE(dev, &r)) return 0;
	  if ((r & 0b00000001) != 0) return 1;
	  r |= 0b00000001;
	  return setENABLE(dev, r);
}
int disablePower(APDS9960_t *dev) {
	uint8_t r;
	if (!getENABLE(dev, &r)) return 0;
	if ((r & 0b00000001) == 0) return 1;
	r &= 0b11111110;
	return setENABLE(dev, r);
}
int enableColor(APDS9960_t *dev) {
	  uint8_t r;
	  if (!getENABLE(dev, &r)) return 0;
	  if ((r & 0b00000010) != 0) {
		  dev->colorEnabled = 1;
	    return 1;
	  }
	  r |= 0b00000010;
	  int res = setENABLE(dev, r);
	  dev->colorEnabled = res;
	  return res;
}
int disableColor(APDS9960_t *dev) {
	  uint8_t r;
	  if (!getENABLE(dev, &r)) return 0;
	  if ((r & 0b00000010) == 0) {
		  dev->colorEnabled = 0;
	    return 1;
	  }
	  r &= 0b11111101;
	  int res = setENABLE(dev, r);
	  dev->colorEnabled = !res; // (res == true) if successfully disabled
	  return res;
}
int enableProximity(APDS9960_t *dev) {
	  uint8_t r;
	  if (!getENABLE(dev, &r)) return 0;
	  if ((r & 0b00000100) != 0) {
		  dev->proximityEnabled = 1;
	    return 1;
	  }
	  r |= 0b00000100;
	  int res = setENABLE(dev, r);
	  dev->proximityEnabled = res;
	  return res;
}
int disableProximity(APDS9960_t *dev) {
	  uint8_t r;
	  if (!getENABLE(dev, &r)) return 0;
	  if ((r & 0b00000100) == 0) {
		  dev->proximityEnabled = 0;
	    return 1;
	  }
	  r &= 0b11111011;
	  int res = setENABLE(dev, r);
	  dev->proximityEnabled = !res; // (res == true) if successfully disabled
	  return res;
}
int enableWait(APDS9960_t *dev) {
	  uint8_t r;
	  if (!getENABLE(dev, &r)) return 0;
	  if ((r & 0b00001000) != 0) return 1;
	  r |= 0b00001000;
	  return setENABLE(dev, r);
}
int disableWait(APDS9960_t *dev) {
	  uint8_t r;
	  if (!getENABLE(dev, &r)) return 0;
	  if ((r & 0b00001000) == 0) return 1;
	  r &= 0b11110111;
	  return setENABLE(dev, r);
}
int enableGesture(APDS9960_t *dev) {
	uint8_t r;
	if (!getENABLE(dev, &r)) return 0;
	if ((r & 0b01000000) != 0) {
		dev->gestureEnabled = 1;
	    return 1;
	}
	r |= 0b01000000;
	int res = setENABLE(dev, r);
	dev->gestureEnabled = res;
	return res;
}
int disableGesture(APDS9960_t *dev) {
	  uint8_t r;
	  if (!getENABLE(dev, &r)) return 0;
	  if ((r & 0b01000000) == 0) {
		  dev->gestureEnabled = 0;
	    return 1;
	  }
	  r &= 0b10111111;
	  int res = setENABLE(dev, r);
	  dev->gestureEnabled = !res; // (res == true) if successfully disabled
	  return res;
}
//HELPER FUNCTIONS
int setENABLE(APDS9960_t *dev, uint8_t val) {
	return i2c1_write(dev, APDS9960_ENABLE, val);
}
int getENABLE(APDS9960_t *dev, uint8_t* val) {
	return i2c1_read(dev, APDS9960_ENABLE, val, 1);
}
int setWTIME(APDS9960_t *dev, uint8_t val) {
	return i2c1_write(dev, APDS9960_WTIME, val);
}
int setGPULSE(APDS9960_t *dev, uint8_t val) {
	return i2c1_write(dev, APDS9960_GPULSE, val);
}
int setPPULSE(APDS9960_t *dev, uint8_t val) {
	return i2c1_write(dev, APDS9960_PPULSE, val);
}
int setATIME(APDS9960_t *dev, uint8_t val) {
	return i2c1_write(dev, APDS9960_ATIME, val);
}
int setCONTROL(APDS9960_t *dev, uint8_t val) {
	return i2c1_write(dev, APDS9960_CONTROL, val);
}
int getGCONF4(APDS9960_t *dev, uint8_t* val) {
	return i2c1_read(dev, APDS9960_GCONF4, val, 1);
}
int setGCONF4(APDS9960_t *dev, uint8_t val) {
	return i2c1_write(dev, APDS9960_GCONF4, val);
}
int getCONFIG2(APDS9960_t *dev, uint8_t* val) {
	return i2c1_read(dev, APDS9960_CONFIG2, val, 1);
}
int setCONFIG2(APDS9960_t *dev, uint8_t val) {
	return i2c1_write(dev, APDS9960_CONFIG2, val);
}
int getSTATUS(APDS9960_t *dev, uint8_t* val) {
	return i2c1_read(dev, APDS9960_STATUS, val, 1);
}
int getPDATA(APDS9960_t *dev, uint8_t* val) {
	return i2c1_read(dev, APDS9960_PDATA, val, 1);
}
int getGSTATUS(APDS9960_t *dev, uint8_t* val) {
	return i2c1_read(dev, APDS9960_GSTATUS, val, 1);
}
int getGFLVL(APDS9960_t *dev, uint8_t* val) {
	return i2c1_read(dev, APDS9960_GFLVL, val, 1);
}
int readGFIFO_U(APDS9960_t *dev, uint8_t *val, uint8_t len) {
	if (i2c1_read(dev, APDS9960_GFIFO_U, val, len) == 0) {
		return 0;
	}
	return len;//TODO: figure out what to return
}


#ifndef _DEV_CONFIG_H_
#define _DEV_CONFIG_H_

/*
 * DEV_Config.h — Backend de hardware para Raspberry Pi 5 (Ubuntu 24.04)
 * Sustituye WiringPi por:
 *  - libgpiod v2  -> GPIO (/dev/gpiochip*)
 *  - i2c-dev      -> I2C  (/dev/i2c-*)
 *  - spidev (opt) -> SPI  (/dev/spidev*)
 */

// ======================= SELECCIÓN DE BACKENDS =======================
// Activa (1) / Desactiva (0) cada backend
#define USE_WIRINGPI_LIB 0     // WiringPi (OBSOLETO en Pi 5)
#define USE_GPIOD_LIB    1     // GPIO con libgpiod v2
#define USE_I2CDEV_LIB   1     // I2C con /dev/i2c-* (linux/i2c-dev.h)
#define USE_SPIDEV_LIB   0     // SPI con /dev/spidev* (linux/spi/spidev.h)

// ======================= INCLUDES DEL SISTEMA ========================
#include <stdint.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

// Debug
#include "Debug.h"

// ======================= PROTOCOLOS SOPORTADOS =======================
#define DEV_SPI 0
#define DEV_I2C 1

// ======================= TIPOS PROPIOS ===============================
#define UBYTE    uint8_t
#define UWORD    uint16_t
#define UDOUBLE  uint32_t

// ======================= HEADERS POR BACKEND =========================
#if USE_WIRINGPI_LIB
  #include <wiringPi.h>
  #include <wiringPiI2C.h>
#endif

#if USE_GPIOD_LIB
  // libgpiod v2 (GPIO character device)
  #include <gpiod.h>
#endif

#if USE_I2CDEV_LIB
  // I2C por interfaz de kernel (/dev/i2c-*)
  #include <linux/i2c-dev.h>
  #include <sys/ioctl.h>
  #include <fcntl.h>
#endif

#if USE_SPIDEV_LIB
  // SPI por interfaz de kernel (/dev/spidev*)
  #include <linux/spi/spidev.h>
  #include <sys/ioctl.h>
  #include <fcntl.h>
#endif

// ======================= VARIABLES EXTERNAS ==========================
extern int INT_PIN;   // Pin de interrupción (definido en el .c)

// ======================= API DEL MÓDULO ==============================
// Inicialización / salida
#ifdef __cplusplus
extern "C" {
#endif

uint8_t DEV_ModuleInit(void);
void    DEV_ModuleExit(void);

// I2C
void DEV_I2C_Init(uint8_t Add);
void I2C_Write_Byte(uint8_t Cmd, uint8_t value);
int  I2C_Read_Byte(uint8_t Cmd);
int  I2C_Read_Word(uint8_t Cmd);

// GPIO
void  DEV_GPIO_Mode(UWORD Pin, UWORD Mode);
void  DEV_Digital_Write(UWORD Pin, UBYTE Value);
UBYTE DEV_Digital_Read(UWORD Pin);

// Timing
void DEV_Delay_ms(UDOUBLE xms);

// SPI (si lo usas)
void DEV_SPI_WriteByte(UBYTE Value);
void DEV_SPI_Write_nByte(uint8_t *pData, uint32_t Len);

#ifdef __cplusplus
}
#endif

#endif // _DEV_CONFIG_H_



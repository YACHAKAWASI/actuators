#ifndef _DEV_CONFIG_H_  // Protección contra inclusión múltiple
#define _DEV_CONFIG_H_  // Definición del símbolo guardián

// ===== CONFIGURACIÓN DE BIBLIOTECA DE HARDWARE =====
// Seleccionar qué biblioteca de hardware usar para GPIO e I2C
#define USE_WIRINGPI_LIB 1  // Usar biblioteca WiringPi (1 = habilitado, 0 = deshabilitado)

/***********************************************************************************************************************
			------------------------------------------------------------------------
			|\\\																///|
			|\\\					Hardware interface							///|
			------------------------------------------------------------------------
***********************************************************************************************************************/

// ===== SELECCIÓN CONDICIONAL DE BIBLIOTECAS DE HARDWARE =====
// Incluir diferentes bibliotecas según la configuración seleccionada

#ifdef USE_BCM2835_LIB  // Si se selecciona la biblioteca BCM2835 (Raspberry Pi)
    #include <bcm2835.h>  // Incluir biblioteca BCM2835 para acceso directo al hardware
#elif USE_WIRINGPI_LIB  // Si se selecciona la biblioteca WiringPi
    #include <wiringPi.h>  // Incluir biblioteca WiringPi para GPIO
    #include <wiringPiI2C.h>  // Incluir biblioteca WiringPi para I2C
#elif USE_DEV_LIB  // Si se selecciona la biblioteca de desarrollo personalizada
    #include "sysfs_gpio.h"  // Incluir biblioteca personalizada para GPIO
    #include "dev_hardware_i2c.h"  // Incluir biblioteca personalizada para I2C
    // #include "dev_hardware_SPI.h"  // Biblioteca SPI comentada (no utilizada)
    
#endif

// ===== BIBLIOTECAS ESTÁNDAR DEL SISTEMA =====
// Incluir bibliotecas del sistema necesarias para el funcionamiento
#include <stdint.h>  // Tipos de datos enteros de tamaño fijo
#include "Debug.h"  // Incluir archivo de debug personalizado

#include <errno.h>  // Códigos de error del sistema
#include <stdio.h>  // Funciones de entrada/salida estándar
#include <string.h>  // Funciones de manipulación de cadenas
#include <unistd.h>  // Funciones del sistema operativo POSIX

// ===== DEFINICIÓN DE PROTOCOLOS DE COMUNICACIÓN =====
// Constantes para identificar el tipo de protocolo de comunicación
#define DEV_SPI 0  // Protocolo SPI (Serial Peripheral Interface)
#define DEV_I2C 1  // Protocolo I2C (Inter-Integrated Circuit)

// ===== DEFINICIÓN DE TIPOS DE DATOS PERSONALIZADOS =====
// Crear alias para tipos de datos enteros sin signo para mayor claridad
/**
 * data
**/
#define UBYTE   uint8_t   // Tipo de dato: byte sin signo (8 bits)
#define UWORD   uint16_t  // Tipo de dato: palabra sin signo (16 bits)
#define UDOUBLE uint32_t  // Tipo de dato: doble palabra sin signo (32 bits)

// ===== VARIABLES EXTERNAS =====
// Variables declaradas externamente (definidas en otro archivo)
extern int INT_PIN;  // Pin de interrupción (valor 4)

/*------------------------------------------------------------------------------------------------------*/

// ===== FUNCIONES DE INICIALIZACIÓN Y FINALIZACIÓN DEL MÓDULO =====
// Funciones para configurar y limpiar el módulo de dispositivos
uint8_t DEV_ModuleInit(void);  // Inicializar el módulo de dispositivos
void    DEV_ModuleExit(void);  // Finalizar y limpiar el módulo de dispositivos

// ===== FUNCIONES DE COMUNICACIÓN I2C =====
// Funciones para comunicación I2C con dispositivos externos
void DEV_I2C_Init(uint8_t Add);  // Inicializar comunicación I2C con dirección específica
void I2C_Write_Byte(uint8_t Cmd, uint8_t value);  // Escribir un byte por I2C
int I2C_Read_Byte(uint8_t Cmd);  // Leer un byte por I2C
int I2C_Read_Word(uint8_t Cmd);  // Leer una palabra (16 bits) por I2C

// ===== FUNCIONES DE CONTROL GPIO =====
// Funciones para controlar pines de entrada/salida de propósito general
void DEV_GPIO_Mode(UWORD Pin, UWORD Mode);  // Configurar modo de un pin GPIO
void DEV_Digital_Write(UWORD Pin, UBYTE Value);  // Escribir valor digital en un pin
UBYTE DEV_Digital_Read(UWORD Pin);  // Leer valor digital de un pin

// ===== FUNCIONES DE TEMPORIZACIÓN =====
// Funciones para crear retardos en la ejecución del programa
void DEV_Delay_ms(UDOUBLE xms);  // Crear retardo en milisegundos

// ===== FUNCIONES DE COMUNICACIÓN SPI =====
// Funciones para comunicación SPI con dispositivos externos
void DEV_SPI_WriteByte(UBYTE Value);  // Escribir un byte por SPI
void DEV_SPI_Write_nByte(uint8_t *pData, uint32_t Len);  // Escribir múltiples bytes por SPI

#endif  // Fin de la protección contra inclusión múltiple

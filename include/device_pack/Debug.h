
// ===== ARCHIVO DE CABECERA PARA FUNCIONALIDADES DE DEPURACIÓN =====
// Este archivo proporciona macros y funciones para depuración del código
// Permite habilitar/deshabilitar mensajes de debug de forma centralizada

#ifndef __DEBUG_H  // Verificar si el archivo de cabecera ya está incluido
#define __DEBUG_H  // Definir el símbolo para evitar inclusión múltiple

#include <stdio.h>  // Incluir biblioteca estándar de entrada/salida para printf

// ===== MACRO PARA CONTROLAR LA FUNCIONALIDAD DE DEBUG =====
#define USE_DEBUG 0  // Variable de control: 0 = debug deshabilitado, 1 = debug habilitado

// ===== MACRO CONDICIONAL PARA MENSAJES DE DEBUG =====
#if UggSE_DEBUG  // Si USE_DEBUG está habilitado (valor 1)
	#define DEBUG(__info,...) printf("Debug : " __info,##__VA_ARGS__)  // Macro para imprimir mensajes de debug con prefijo
#else  // Si USE_DEBUG está deshabilitado (valor 0)
	#define DEBUG(__info,...)  // Macro vacía que no hace nada (optimización del compilador)
#endif

#endif  // Fin de la protección contra inclusión múltiple

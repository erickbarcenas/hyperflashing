/***************************************************************************
      Archivo:    main.c
      Fecha:      12-02-2020

      Tarjeta:    TM4C1294XL Connected LaunchPad Evaluation Kit
      IDE:        Code Composer Studio
      Autores:    Galarza Martínez Abel         |   314070171  |  fsae.unam.abelgm@gmail.com
                  Nieto Lara Aldo               |   315183601  |  aladar_156@hotmail.com
                  Bárcenas Martínez Erick Iván  |   417092331  |  erick3arcenas7@gmail.com

      Resumen:    Sistema de monitoreo inteligente en plaza comercial de alta concurrencia

*/

#include <stdint.h>
#include <stdbool.h>
#include "hyperflashing.h"


/** ========================================================================
                    FUNCIÓN PRINCIPAL
    ========================================================================**/


void main (void){
    sysClock();

    initializeUARTs();  // Config. Terminal.exe - bluetooth
    initializePORTs();
    initializeTIMER(); // Config. ultrasónico

    while(1){
        //lcd();
        //leds();
        //keyboard4x4();
        ultrasonic2(17); //
        ultrasonic1(17); // 17 is the minimum distance the sensor will read to turn on the LED.


    }

    return 0;
 }

#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"


uint32_t d_uint_distance;
uint32_t int32_distance;
uint32_t ui32Loop;



#define   PORTA SYSCTL_RCGCGPIO_R0  // Bluetooth HC-05        | Entrada  ->
#define   PORTD SYSCTL_RCGCGPIO_R3  // LEDS                   | Salida
#define   PORTF SYSCTL_RCGCGPIO_R5  // Ultrasónico (salida)   | Entrada  ->
#define   PORTH SYSCTL_RCGCGPIO_R7  // Teclado matricial      | Entrada  ->
#define   PORTK SYSCTL_RCGCGPIO_R9  // LCD (D0,D1,D2,D3..D7)  | Salida
#define   PORTM SYSCTL_RCGCGPIO_R11 // LCD (RS, RW Y E)       | Salida
#define   PORTN SYSCTL_RCGCGPIO_R12 // Ultrasónico (entrada)  | Entrada  ->
#define   PORTQ SYSCTL_RCGCGPIO_R14 // Teclado matricial      | Entrada  ->




/*
offset DATA 0x3fc
offset DIR 0x400  | 0: input         1: output
offset DEN 0x51c  | 0: disable       1: enable
*/



// SYSCLOCK
void sysClock(void);

void initializeTIMER(void);
// HUART
void initializeUARTs(void);

void TRIGGER_N04 (void);
void TRIGGER_N02 (void);

// Bluetooth
char readDataUART0(void);
char writeDataUART0(char);
// PORTs
void initializePORTs(void);
// LCD
void cronometro(float);
void clearPorts(void);
void writeCommand(void);
void startDisplay(void);
void writeData(void);
void writeMessage(void);
void cronos(void);

void ledsOn(uint8_t, uint32_t, uint32_t);

void lcd(void);
void leds(void);
void ultrasonic1(uint32_t);
void ultrasonic2(uint32_t);
void keyboard4x4(void);




/** ========================================================================
                    DEFINICIÓN DE LAS FUNCIONES
    ========================================================================**/
//    ......... ULTRASÓNICO 1 (entrada) .................
void TRIGGER_N04 (void){
     GPIO_PORTN_DATA_R |= 0B00010000;  // PONER A 1 PN4
     SysCtlDelay(168);  //me quedo 10 microsegundos    3 * (1/f)
                        // 50 MHz T=20ns 20x3 = 60 ns ¿cuánto vale x para 10 us con 60 ns de retardo básico?
                        // x= 10us/60ns = 168
     GPIO_PORTN_DATA_R &= ~(0B00010000);
}
//    ......... ULTRASÓNICO 2 (salida) .................
void TRIGGER_N02 (void){
     GPIO_PORTN_DATA_R |= 0B00000100;  // PONER A 1 PN2
     SysCtlDelay(168);
     GPIO_PORTN_DATA_R &= ~(0B00000100);
}

void sysClock(void){
    uint32_t g_ui32SysClock;
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 50000000);
}


void initializePORTs(void){

    /** ====================================
         *       1. Encender puertos a usar en
         *        RGCGCPIO con máscaras
        ==================================== **/



    SYSCTL_RCGCGPIO_R |=PORTA|PORTD|PORTF|PORTH|PORTK|PORTM|PORTN|PORTQ; // Utilizamos máscaras
    while((SYSCTL_PRGPIO_R&(PORTA|PORTD|PORTF|PORTH|PORTK|PORTM|PORTN|PORTQ))==0); // Esperamos a que se habiliten todos los puertos




    // PORTA

    GPIO_PORTA_AHB_AMSEL_R &= ~0X33; //DESHABILITAR FUNCION ANLOGICA EN PA0-1 0011 0011
    GPIO_PORTA_AHB_AFSEL_R |= 0X33; //HABILITAR FUNCION ALTERNA EN PA0-1
    GPIO_PORTA_AHB_PCTL_R = (GPIO_PORTA_AHB_PCTL_R&0XFF00FF00)+0X00110011;
    GPIO_PORTA_AHB_DEN_R |= 0X33; //HABILITAR FINCION I/O DIGITAL
    GPIO_PORTA_AHB_PUR_R |= 0X10; //HABILITAR PULL UP EN PA4
    // PORTD
    GPIO_PORTD_AHB_DIR_R = 0B00000011; // Digital en PD7 y PD0
    GPIO_PORTD_AHB_DEN_R = 0B00000011; // Salidas en PD7 y PD0
    GPIO_PORTD_AHB_DATA_R = 0X00;
    // PORTF
    //GPIO_PORTF_AHB_DIR_R = 0B01000; //0 entrada
    //GPIO_PORTF_AHB_DEN_R = 0B01100; // 1 digital
    //GPIO_PORTF_AHB_DATA_R = 0X00;
    // PORTH
    // GPIO_PORTH_AHB_DIR_R = ;
    // GPIO_PORTH_AHB_DEN_R =  ;
    GPIO_PORTH_AHB_DATA_R = 0X00;
    // PORTK
    GPIO_PORTK_DIR_R = 0B11111111;
    GPIO_PORTK_DEN_R = 0B11111111;
    GPIO_PORTK_DATA_R = 0X00;
    // PORTM
    GPIO_PORTM_DIR_R = 0B00000101;
    GPIO_PORTM_DEN_R = 0B00000101;
    GPIO_PORTM_DATA_R = 0X00;
    // PORTN
    GPIO_PORTN_DIR_R = 0B00010100; //  entrada (bit 5) y 1 salida (bit 4) 0     // salida PN2 y entrada PN0
    GPIO_PORTN_DEN_R = 0B00111100; // bit 5 y bit 4 son digitales               // bit 2 y 3 son digitales
    GPIO_PORTN_DATA_R = 0X00;
    // PORTQ
    // GPIO_PORTQ_DIR_R = ;
    // GPIO_PORTQ_DEN_R =  ;
    GPIO_PORTQ_DATA_R = 0X00;
 }

/** ..........................................................................
                     TIMER
    ..........................................................................**/
void initializeTIMER(void){
    SYSCTL_RCGCTIMER_R |= 0B00001100; // Encendemos el timer 2 y 3 (ultrasónico HCSR04)   0000 1100
    //retardo para que el reloj alcance el PORTN, PORTF y TIMER 2,3
    ui32Loop = SYSCTL_RCGCTIMER_R;

    TIMER3_CTL_R=0X00000000; //DESHABILITA TIMER EN LA CONFIGURACION
    TIMER3_CFG_R= 0X00000000; //CONFIGURAR PARA 32 BITS
    TIMER3_TAMR_R= 0X00000012; //CONFIGURAR PARA MODO PERIODICO CUENTA HACIA ARRIBA
    TIMER3_TAILR_R= 0X00FFFFFF; // VALOR DE RECARGA  --> Número mayor a 1160000
    TIMER3_ICR_R= 0X00000001 ; //LIMPIA PISOBLE BANDERA PENDIENTE DE TIMER3
    //TIMER3_IMR_R |= 0X00000000; //ACTIVA INTRRUPCION DE TIMEOUT
   // NVIC_EN1_R= 1<<(35-32); //HABILITA LA INTERRUPCION DE  TIMER3
   // TIMER3_CTL_R |= 0X00000001; //HABILITA TIMER EN LA CONFIGURACION


    TIMER2_CTL_R=0X00000000; //DESHABILITA TIMER EN LA CONFIGURACION
    TIMER2_CFG_R= 0X00000000; //CONFIGURAR PARA 32 BITS
    TIMER2_TAMR_R= 0X00000012; //CONFIGURAR PARA MODO PERIODICO CUENTA HACIA ARRIBA
    TIMER2_TAILR_R= 0X00FFFFFF; // VALOR DE RECARGA  --> Número mayor a 1160000
    TIMER2_ICR_R= 0X00000001 ; //
    //TIMER3_IMR_R |= 0X00000000; //ACTIVA INTRRUPCION DE TIMEOUT
    // NVIC_EN1_R= 1<<(35-32); //HABILITA LA INTERRUPCION DE  TIMER3
    // TIMER3_CTL_R |= 0X00000001; //HABILITA TIMER EN LA CONFIGURACION
}

/** ..........................................................................
                     UART
    ..........................................................................**/
void initializeUARTs(void){

    SYSCTL_RCGCUART_R |=0X09; // Encendemos UART0 (terminal.exe), UART3 (bluetooth HC-05) 0000 1001 =>
// Configuración del UART0
    UART0_CTL_R &=~0X0001;  //DESHABILITAR UART0
    UART0_IBRD_R = 27 ; //IBDR=int(50000000/16*115200))= int(27.1267)
    UART0_FBRD_R =8 ; //FBRD= round(0.1267*64 =8)
    UART0_LCRH_R =0X0070; //8 BITS, HABILITAR FIFO
    UART0_CTL_R= 0X0301 ; //HABILITAR RXE, TXE Y UART
    // Configuración del UART3
    UART3_CTL_R &=~0X0001;  //DESHABILITAR UART
    UART3_IBRD_R = 325 ; //IBDR=int(50000000/16*9600))= int(325.5208)
    UART3_FBRD_R =33 ; //FBRD= round(0.5208*64 =33.3311)
    UART3_LCRH_R =0X0070; //8 BITS, HABILITAR FIFO
    UART3_CTL_R= 0X0301 ; //HABILITAR RXE, TXE Y UART
}

/** ..........................................................................
                    ULTRASONIC
                 (RW common function)
    ..........................................................................**/
/** ..........................................................................
                    BLUETOOTH
    ..........................................................................**/
    //esperar hasta que se reciba un dato

char readDataUART0(void){
    while((UART0_FR_R&0X0010)!=0); //ESPERAR A QUE RXFE SEA CERO
    d_uint_distance=((char)(UART0_DR_R&0xff));
    //return((char)(UART0_DR_R&0xff));
}

char writeDataUART0(char character){
    while((UART0_FR_R&0X0020)!=0); // espera a que TXFF sea cero
    UART0_DR_R=character;
}

/** ..........................................................................
                    DISPLAY LCD
    ..........................................................................**/

void cronometro(float seg){
    GPIO_PORTK_DATA_R |=0X00;
    float VelCPU = 62.5*(10E-10);
    int TICKS = seg/VelCPU;
    NVIC_ST_RELOAD_R=TICKS;
    NVIC_ST_CTRL_R=0X05;
    while (( NVIC_ST_CTRL_R&0x10000)==0);
}

void clearPorts(void){
    GPIO_PORTK_DATA_R = 0X00;
    GPIO_PORTM_DATA_R = 0X00;
}

void writeCommand(void){
    GPIO_PORTM_DATA_R |= 0x04;
    cronometro(0.02);
    GPIO_PORTM_DATA_R &= 0xfb;
}

void startDisplay(){
  unsigned char comandos[] = {0x0f, 0x38, 0x38, 0x38, 0x38, 0x0c, 0x01, 0x06};
     int size = sizeof(comandos)/sizeof(comandos[0]);
     int i=0;
     for(i; i<size;i++){
           if(i<3){
               GPIO_PORTK_DATA_R = comandos[i];
               writeCommand();
               cronometro(0.02);
               writeCommand();
           }else{
                GPIO_PORTK_DATA_R = comandos[i];
                writeCommand();
           }
     }
}

void writeData(){
    int brs = 0;
    int brw = 0;
    int benable = 0;
    GPIO_PORTM_DATA_R |= brs;
    GPIO_PORTM_DATA_R |= benable;
    cronometro(0.02);
    GPIO_PORTM_DATA_R &= 0xfa;
}

void writeMessage(){
     unsigned char letras[] = {0x85, 0x54, 0x49, 0x45, 0x4D, 0x50, 0x4F};
     int size = sizeof(letras)/sizeof(letras[0]);
     int i=0;
     for(i; i<size; i++){
           if(i==0){
                GPIO_PORTK_DATA_R = letras[i];
                writeCommand();
           }else{
               GPIO_PORTK_DATA_R = letras[i];
               writeData();
           }
     }
}

void cronos(){
    int output =0;
    int timeUnits = 0x00;
    int timeDec = 0x00;
    while(output != 1){
        GPIO_PORTK_DATA_R = 0xc7;
        writeCommand();
        GPIO_PORTK_DATA_R = timeDec + 0x30;
        writeData();
        GPIO_PORTK_DATA_R = timeUnits + 0x30;
        writeData();
        GPIO_PORTK_DATA_R = 0xc7;
        writeCommand();
        cronometro(0.1);
        if(timeUnits <9){
            timeUnits++;
        }else if(timeDec == 10)
        {
            output=1;
        }else{
            timeDec++;
            timeUnits=0;
        }
    }
}


void ledsOn(uint8_t ultraNum, uint32_t int32_distance, uint32_t maxDistance){


    if(int32_distance <= maxDistance){

       if(ultraNum==1){
           GPIO_PORTD_AHB_DATA_R=0B00000001; // GPIO_PORTD_AHB_DATA_R  PD0
       }else{
          GPIO_PORTD_AHB_DATA_R=0B00000010; // GPIO_PORTD_AHB_DATA_R   PD1
       }

   }else{
       if(ultraNum==1){
                  GPIO_PORTD_AHB_DATA_R=0B00000000; // GPIO_PORTD_AHB_DATA_R
              }else{
                  GPIO_PORTD_AHB_DATA_R=0B00000000; // GPIO_PORTD_AHB_DATA_R
              }
   }
    return 0;
}

/** ..........................................................................
                    DEVICES
    ..........................................................................**/

void lcd(void){
    clearPorts();
    startDisplay();
    writeMessage();
    while(1){
        cronos();
    }
}
// ...........................................
void leds(void){}
// ...........................................
void keyboard4x4(void){}
// ...........................................
void ultrasonic1(uint32_t maxDistance){

    //while(1){

        TIMER3_TAV_R=0X00; // ponemos en cero para que cuente hacia arriba
        TRIGGER_N04();

        // 0 a 1
        while((GPIO_PORTN_DATA_R&0X20)==0);
        // Arrancar el temporizador

        // Inicia el conteo
        TIMER3_CTL_R |= 0X00000001; //HABILITA TIMER EN LA CONFIGURACION CUANDO PN5 SEA 1


        while((GPIO_PORTN_DATA_R&0X20)==0X20); // echo PN5: 0010 0000
        TIMER3_CTL_R &= ~(0X00000001); // Cuando cambie de 1 a 0 detener el timer correspondiente

        int32_distance = TIMER3_TAV_R/(58*50);
        writeDataUART0(int32_distance);

        ledsOn(1, int32_distance, maxDistance);

        SysCtlDelay(1050000);   //xnS

      //  }
    return 0;
}

void ultrasonic2(uint32_t maxDistance){

   // while(1){

        TIMER2_TAV_R=0X00; // ponemos en cero para que cuente hacia arriba
        TRIGGER_N02();

        while((GPIO_PORTN_DATA_R&0X08)==0); // PN1:0000 0010   // Echo

        TIMER2_CTL_R |= 0X00000001;

        while((GPIO_PORTN_DATA_R&0X08)==0X08); //    PF2:0000 1000     // Echo
        TIMER2_CTL_R &= ~(0X00000001); // Cuando cambie de 1 a 0 detener el timer correspondiente

        int32_distance = TIMER2_TAV_R/(58*50);
        writeDataUART0(int32_distance);


        ledsOn(2, int32_distance, maxDistance);


        SysCtlDelay(1050000);

      //  }
    return 0;
}


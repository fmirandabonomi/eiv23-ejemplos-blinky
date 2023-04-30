#ifndef SOPORTE_BLUEPILL_H
#define SOPORTE_BLUEPILL_H

// https://www.doxygen.nl/manual/commands.html

/** @brief Handles de los pines de entrada/salida disponibles
 *
 * @note Notar la diferencia entre ***handle*** y ***puntero***. Un ***handle*** es un
 * tipo "opaco" (sus detalles no son visibles), normalmente un ***índice en
 * una tabla (arreglo)***. Un ***puntero*** contiene una ***dirección de memoria***.
 */
typedef enum HPin{
    PA0,    ///< Solo 3V. Función Alternativa primaria: `[ADC0 | USART2 CTS | TIM2 CH1 | TIM2 ETR | WKUP]`
    PA1,    ///< Solo 3V. Función Alternativa primaria: `[ADC1 | USART2 RTS | TIM2 CH2]`
    PA2,    ///< Solo 3V. Función Alternativa primaria: `[ADC2 | USART2 TX  | TIM2 CH3]`
    PA3,    ///< Solo 3V. Función Alternativa primaria: `[ADC3 | USART2 RX  | TIM2 CH4]`
    PA4,    ///< Solo 3V. Función Alternativa primaria: `[ADC4 | USART2 CK  | SPI1 NSS]`
    PA5,    ///< Solo 3V. Función Alternativa primaria: `[ADC5 | SPI1 SCK]`
    PA6,    ///< Solo 3V. Función Alternativa primaria: `[ADC6 | SPI1 MISO | TIM3 CH1]`. Fn.Alt. remapeada: `[TIM1 BKIN]`
    PA7,    ///< Solo 3V. Función Alternativa primaria: `[ADC7 | SPI1 MOSI | TIM3 CH2]`. Fn.Alt. remapeada: `[TIM1 CH1N]`
    PA8,    ///< Tolera 5V. Función Alternativa primaria: `[USART1 CK  | TIM1 CH1 | MCO]`.
    PA9,    ///< Tolera 5V. Función Alternativa primaria: `[USART1 TX  | TIM1 CH2]`
    PA10,   ///< Tolera 5V. Función Alternativa primaria: `[USART1 RX  | TIM1 CH3]`
    PA11,   ///< Tolera 5V. Función Alternativa primaria: `[USART1 CTS | TIM1 CH4 | CAN RX | USBDM]`
    PA12,   ///< Tolera 5V. Función Alternativa primaria: `[USART1 RTX | TIM1 ETR | CAN TX | USBDP]`
    /* PA13, Ocupado con SWDIO. Otra función: JTAG: JTMS (no usada) */
    /* PA14, Ocupado con SWCLK. Otra función: JTAG: JTCK (no usada) */
    PA15,   ///< Tolera 5V. JTAG: `JTDI`. Función Alternativa remapeada: `[TIM2 CH1 | TIM2 ETR | SPI1 NSS]`
    PB0,    ///< Solo 3V. Función Alternativa primaria: `[ADC8 | TIM3 CH3]`. Fn.Alt. remapeada: `[TIM1 CH2N]`
    PB1,    ///< Solo 3V. Función Alternativa primaria: `[ADC9 | TIM3 CH4]`. Fn.Alt. remapeada: `[TIM1 CH3N]`
    /* PB2, Ocupado con BOOT1 */
    PB3,    ///< Tolera 5V. JTAG: `JTDO`. Función Alternativa remapeada: `[TIM2 CH2 | TRACESWO | SPI SCK]`
    PB4,    ///< Tolera 5V. JTAG: `JNTRST`. Función Alternativa remapeada: `[TIM3 CH1 | SPI1 MISO]`
    PB5,    ///< Solo 3V.   Función Alternativa primaria: `[I2C1 SMBAI]`. Fn.Alt. remapeada: `[TIM3 CH2 | SPI1 MOSI]`
    PB6,    ///< Tolera 5V. Función Alternativa primaria: `[I2C1 SCL | TIM4 CH1]`. Fn.Alt. remapeada: `[USART1 TX]`
    PB7,    ///< Tolera 5V. Función Alternativa primaria: `[I2C1 SDA | TIM4 CH2]`. Fn.Alt. remapeada: `[USART1 RX]`
    PB8,    ///< Tolera 5V. Función Alternativa primaria: `[TIM4 CH3]`. Fn.Alt. remapeada: `[I2C1 SCL | CAN RX]`
    PB9,    ///< Tolera 5V. Función Alternativa primaria: `[TIM4 CH4]`. Fn.Alt. remapeada: `[I2C1 SDA | CAN TX]`
    PB10,   ///< Tolera 5V. Función Alternativa primaria: `[I2C2 SCL | USART3 TX]`. Fn.Alt. remapeada: `[TIM2 CH3]`
    PB11,   ///< Tolera 5V. Función Alternativa primaria: `[I2C2 SDA | USART3 RX]`. Fn.Alt. remapeada: `[TIM2 CH4]`
    PB12,   ///< Tolera 5V. Función Alternativa primaria: `[SPI2 NSS  | TIM1 BKIN  | USART3 CK | I2C2 SMBAI  ]`.
    PB13,   ///< Tolera 5V. Función Alternativa primaria: `[SPI2 SCK  | TIM1 CH1N  | USART3 CTS]`.
    PB14,   ///< Tolera 5V. Función Alternativa primaria: `[SPI2 MISO | TIM1 CH2N  | USART3 RTS]`.
    PB15,   ///< Tolera 5V. Función Alternativa primaria: `[SPI2 MOSI | TIM1 CH3N]`.
    PC13,   ///< Solo 3V. Conectado al led de la placa. `TAMPER RTC`
    PC14,   ///< Solo 3V. Conectado al cristal de 8 MHz. Función Alternativa primaria: `OSC32 IN`
    PC15,   ///< Solo 3V. Conectado al cristal de 8 MHz. Función Alternativa primaria: `OSC32 OUT`
    P_LED = PC13    ///< Led integrado en la placa. Ver PC13
} HPin;

/**
 * @brief Establece la configuración inicial. 
 * 
 */
void SBP_init();

typedef enum Pin_ModoPull{
    PIN_FLOTANTE,
    PIN_PULLUP,
    PIN_PULLDOWN
}Pin_ModoPull;
/**
 * @brief Configura un pin como entrada. Si es necesario activa el reloj del puerto.
 * 
 * @param hpin Handle del pin.
 * @param pull Configuración de resistencia de pull-up/pull-down 
 */
void Pin_modoEntrada(HPin hpin, Pin_ModoPull pull);

typedef enum Pin_Velocidad{
    PIN_2MHz,
    PIN_10MHz,
    PIN_50MHz
}Pin_Velocidad;

/**
 * @brief Configura un pin como salida. Si es necesario activa el reloj del puerto.
 * 
 * @param hpin Handle del pin.
 * @param velocidad Velocidad del driver.
 * @param drenadorAbierto [true: Drenador abierto, false: Push-pull (normal)]
 */
void Pin_modoSalida(HPin hpin, Pin_Velocidad velocidad, bool drenadorAbierto);

typedef enum Pin_FlancoInterrupcion{
    PIN_INT_ASCENDENTE  = 0b01, ///< Interrupción en flanco ascendente
    PIN_INT_DESCENDENTE = 0b10, ///< Interrupción en flanco descendente
    PIN_INT_ASCENDENTE_Y_DESCENDENTE = PIN_INT_ASCENDENTE | PIN_INT_DESCENDENTE ///< Interrupción en ambos flancos
}Pin_FlancoInterrupcion;

/**
 * @typedef Pin_ExtInt_Handler 
 * @note No es un callback común, pues se ejecuta en modo handler.
 */
typedef void Pin_ExtInt_Handler(void);

/**
 * @brief Configura la interrupción externa en un pin, ya sea para flanco ascendente, descendente
 * o ascendente y descendente. Activa relojes según sea necesario. Reemplaza la configuración
 * previa. Los detalles son solucionados por la rutina de servicio de interrupción antes de llamar
 * al handler provisto.
 * 
 * @param hpin Handle del pin 
 * @param handler Puntero a rutina de handler.
 * @param flanco Flanco al que es sensible la interrupción
 */
void Pin_configuraInterrupcionExterna(HPin hpin, Pin_ExtInt_Handler *handler, Pin_FlancoInterrupcion flanco);

/**
 * @brief Remueve cualquier configuración de interrupción externa en un pin. Falla si los periféricos
 * fueron apagados antes.
 * 
 * @param hpin Handle del pin
 * @return true: Configuración removida
 * @return false: Periférico está apagado
 * 
 */
bool Pin_desactivaInterrupcionExterna(HPin hpin);

/**
 * @brief Lee el buffer de entrada de un pin (previamente configurado).
 * 
 * @param hpin Handle del pin.
 * @return true Nivel ALTO
 * @return false Nivel BAJO
 */
bool Pin_lee(HPin hpin);

/**
 * @brief Escribe en el buffer de salida de un pin. El pin debe haber sido configurado.
 * 
 * @param hpin Handle del pin
 * @param valor true: Nivel ***alto***, false: Nivel ***bajo***
 */
void Pin_escribe(HPin hpin, bool valor);

/**
 * @brief Lee el estado del buffer de salida del pin. El pin debe haber sido configurado.
 *
 * @note El estado del buffer de salida solo es relevante cuando el pin
 * está configurado en modo salida o como entrada con pull-up/pull-down.
 * Si el pin está configurado con una función distinta de GPIO el buffer
 * está desconectado del pin.
 * 
 * @param hpin Handle del pin
 * @return true: Nivel ***alto*** o entrada pull-up
 * @return false: Nivel ***bajo*** o entrada pull-down
 */
bool Pin_estadoSalida(HPin hpin);


#endif
#include <soporte_bluepill.h>
#include <stm32f1xx.h>
#include <system_stm32f1xx.h>

/* Vectores de interrupción */

void SysTick_Handler(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

/* General */

void BP_init(void){
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock/1000);  
}

/* SysTick */

static uint32_t volatile ticks;

void SysTick_Handler(void)
{
    ++ticks;
}

uint32_t BP_get_ticks(void){
    return ticks; // La lectura es atómica para enteros de 32 bits.
}
/* Pines y exti */

typedef struct PinDescriptor{
    uint8_t hGpio;
    uint8_t nrPin;
}PinDescriptor;

#define HANDLE_GPIO(base) ((uint8_t)(((intptr_t)(base))>>10 & 0xFFU)) // 0x42..0x46
#define GET_GPIO(h) ((GPIO_TypeDef *)(PERIPH_BASE | ((h)<<10)))

#define MAX_PINS 32
#define TWOS_POWER(x) (!((x)&((x)-1)))
#if !TWOS_POWER(x)
#error MAX_PINS debe ser potencia de dos
#endif

static PinDescriptor const pines[MAX_PINS]={
[PA0]  = {HANDLE_GPIO(GPIOA), 0},
[PA1]  = {HANDLE_GPIO(GPIOA), 1},
[PA2]  = {HANDLE_GPIO(GPIOA), 2},
[PA3]  = {HANDLE_GPIO(GPIOA), 3},
[PA4]  = {HANDLE_GPIO(GPIOA), 4},
[PA5]  = {HANDLE_GPIO(GPIOA), 5},
[PA6]  = {HANDLE_GPIO(GPIOA), 6},
[PA7]  = {HANDLE_GPIO(GPIOA), 7},
[PA8]  = {HANDLE_GPIO(GPIOA), 8},
[PA9]  = {HANDLE_GPIO(GPIOA), 9},
[PA10] = {HANDLE_GPIO(GPIOA),10},
[PA11] = {HANDLE_GPIO(GPIOA),11},
[PA12] = {HANDLE_GPIO(GPIOA),12},
[PA15] = {HANDLE_GPIO(GPIOA),15},
[PB0]  = {HANDLE_GPIO(GPIOB), 0},
[PB1]  = {HANDLE_GPIO(GPIOB), 1},
[PB3]  = {HANDLE_GPIO(GPIOB), 3},
[PB4]  = {HANDLE_GPIO(GPIOB), 4},
[PB5]  = {HANDLE_GPIO(GPIOB), 5},
[PB6]  = {HANDLE_GPIO(GPIOB), 6},
[PB7]  = {HANDLE_GPIO(GPIOB), 7},
[PB8]  = {HANDLE_GPIO(GPIOB), 8},
[PB9]  = {HANDLE_GPIO(GPIOB), 9},
[PB10] = {HANDLE_GPIO(GPIOB),10},
[PB11] = {HANDLE_GPIO(GPIOB),11},
[PB12] = {HANDLE_GPIO(GPIOB),12},
[PB13] = {HANDLE_GPIO(GPIOB),13},
[PB14] = {HANDLE_GPIO(GPIOB),14},
[PB15] = {HANDLE_GPIO(GPIOB),15},
[PC13] = {HANDLE_GPIO(GPIOC),13},
[PC14] = {HANDLE_GPIO(GPIOC),14},
[PC15] = {HANDLE_GPIO(GPIOC),15}
};

#define GET_PIN(h) (pines[(h)%MAX_PINS])


#define MAX_GPIOS 8
#if !TWOS_POWER(MAX_GPIOS)
#error MAX_GPIOS debe ser potencia de dos
#endif


#define GET_GPIO_ENR_MASK(hGpio) (1UL<<((hGpio)&0xF)) 

typedef struct ExtiHandlerDescriptor{
    BP_Pin_ExtInt_Handler * handler;
    BP_Pin_FlancoInterrupcion flanco;
    bool valido;
}ExtiHandlerDescriptor;

static ExtiHandlerDescriptor exti_handlers[MAX_PINS]={0};

#define GET_EXTI_HANDLER(hpin) (exti_handlers[hpin % MAX_PINS])

static uint64_t exti_use_counter = 0;

static int exti_get_count(int const nrPin){
    return (exti_use_counter >> (4*nrPin))&0xF;
}
static void exti_count_up(int const nrPin){
    if (exti_get_count(nrPin) < 15){
        exti_use_counter += 1 << (4*nrPin);
    }
}
static void exti_count_down(int const nrPin){
    if(exti_get_count(nrPin)){
        exti_use_counter -= 1 << (4*nrPin);
    }
}

typedef struct ExtiIrqnDescriptor{
    bool valid;
    IRQn_Type irqn;
    uint32_t extiMask;
}ExtiIrqnDescriptor;
#define ExtiIrqnDescriptor_VALID(irqn_,extiMask_) ((ExtiIrqnDescriptor){\
    .valid=true,.irqn=(irqn_),.extiMask=(extiMask_)})
#define ExtiIrqnDescriptor_INVALID ((ExtiIrqnDescriptor){0})

static ExtiIrqnDescriptor exti_irq(int const nrPin){
    ExtiIrqnDescriptor irq = ExtiIrqnDescriptor_INVALID;
    switch(nrPin){
    case 0:       irq = ExtiIrqnDescriptor_VALID(EXTI0_IRQn,1UL<<0);
    break;case 1: irq = ExtiIrqnDescriptor_VALID(EXTI1_IRQn,1UL<<1);
    break;case 2: irq = ExtiIrqnDescriptor_VALID(EXTI2_IRQn,1UL<<2);
    break;case 3: irq = ExtiIrqnDescriptor_VALID(EXTI3_IRQn,1UL<<3);
    break;case 4: irq = ExtiIrqnDescriptor_VALID(EXTI4_IRQn,1UL<<4);
    break;case 5:
    /*fallthrough*/case 6: 
    /*fallthrough*/case 7: 
    /*fallthrough*/case 8: 
    /*fallthrough*/case 9: 
        irq = ExtiIrqnDescriptor_VALID(EXTI9_5_IRQn,0x1FUL<<5);
    break;case 10:
    /*fallthrough*/case 11:
    /*fallthrough*/case 12:
    /*fallthrough*/case 13:
    /*fallthrough*/case 14:
    /*fallthrough*/case 15:
        irq = ExtiIrqnDescriptor_VALID(EXTI15_10_IRQn,0x3FUL<<10);
    break;default:
    break;
    }
    return irq;
}

#define GET_CR(pin,puerto) ((pin.nrPin < 8) ? &puerto->CRL : &puerto->CRH)
#define GET_CR_OFFSET(pin) ((pin.nrPin % 8)*4)
#define MASCARA_MODO 0xF
void BP_Pin_modoEntrada(BP_HPin const hpin, BP_Pin_ModoPull const pull){
    uint32_t const modo = (pull != PIN_FLOTANTE) ? 0x8 : 0x4;
    PinDescriptor const pin = GET_PIN(hpin);
    if (pin.hGpio){
        GPIO_TypeDef *const puerto = GET_GPIO(pin.hGpio);
        uint32_t volatile *const CR = GET_CR(pin,puerto);
        uint8_t const offset = GET_CR_OFFSET(pin); 
        __disable_irq();
        RCC->APB2ENR |= GET_GPIO_ENR_MASK(pin.hGpio);
        *CR = (*CR & ~(MASCARA_MODO << offset)) | (modo << offset);
        __enable_irq();
        if (pull != PIN_FLOTANTE){
            if (pull == PIN_PULLUP)
                puerto->BSRR = 1<<pin.nrPin;
            else
                puerto->BRR = 1<<pin.nrPin;
        }
    }
}

void BP_Pin_modoSalida(BP_HPin hpin, BP_Pin_Velocidad velocidad, bool drenadorAbierto){
    static uint8_t const modo_salida[4]={[PIN_2MHz]=0x2,[PIN_10MHz]=0x1,[PIN_50MHz]=0x3};
    uint32_t const modo = modo_salida[velocidad % 4] | ((drenadorAbierto)? 0x4 : 0x0);
    PinDescriptor const pin = GET_PIN(hpin);
    if (pin.hGpio){
        GPIO_TypeDef *const puerto = GET_GPIO(pin.hGpio);
        uint32_t volatile * const CR = GET_CR(pin,puerto);
        uint8_t const offset = GET_CR_OFFSET(pin);
        __disable_irq();
        RCC->APB2ENR |= GET_GPIO_ENR_MASK(pin.hGpio);
        *CR = (*CR & ~(MASCARA_MODO << offset)) | (modo << offset);
        __enable_irq();
    }
}
void BP_Pin_configuraInterrupcionExterna(BP_HPin hpin, BP_Pin_ExtInt_Handler *handler, BP_Pin_FlancoInterrupcion flanco){
    PinDescriptor const pin = GET_PIN(hpin);
    ExtiIrqnDescriptor const irqDescriptor = exti_irq(pin.nrPin);
    if (   pin.hGpio
        && irqDescriptor.valid 
        && handler != (BP_Pin_ExtInt_Handler*) 0
        && (flanco & (PIN_INT_ASCENDENTE | PIN_INT_DESCENDENTE))){
        ExtiHandlerDescriptor *const handlerDescriptor = &GET_EXTI_HANDLER(hpin);
        __disable_irq();
        handlerDescriptor->flanco = flanco;
        handlerDescriptor->handler = handler;
        if (!handlerDescriptor->valido){
            exti_count_up(pin.nrPin);
        }
        handlerDescriptor->valido = true;
        uint32_t const mascara = 1UL << pin.nrPin; 
        if (flanco & PIN_INT_ASCENDENTE)  EXTI->RTSR |= mascara;
        if (flanco & PIN_INT_DESCENDENTE) EXTI->FTSR |= mascara;
        EXTI->PR = mascara;   // ignora IRQ pendiente anterior
        EXTI->EMR |= mascara;
        NVIC_EnableIRQ(irqDescriptor.irqn);
        __enable_irq();
    }
}
bool BP_Pin_desactivaInterrupcionExterna(BP_HPin hpin){
    PinDescriptor const pin = GET_PIN(hpin);
    ExtiIrqnDescriptor const irqDescriptor = exti_irq(pin.nrPin); 
    if (   pin.hGpio
        && irqDescriptor.valid){
        ExtiHandlerDescriptor *const handlerDescriptor = &GET_EXTI_HANDLER(hpin);
        __disable_irq();
        if (handlerDescriptor->valido){
            handlerDescriptor->valido = false;
            exti_count_down(pin.nrPin);
            if (!exti_get_count(pin.nrPin)){
                uint32_t const mascara = 1UL << pin.nrPin;
                EXTI->IMR  &= ~mascara;
                if (!(EXTI->EMR & mascara)){
                    EXTI->FTSR &= ~mascara;
                    EXTI->RTSR &= ~mascara;
                }
                if (!(EXTI->IMR & irqDescriptor.extiMask))
                    NVIC_DisableIRQ(irqDescriptor.irqn);
            }
        }
        __enable_irq();
    }
    return false;
}
bool BP_Pin_lee(BP_HPin hpin){
    bool resultado = false;
    PinDescriptor const pin = GET_PIN(hpin);
    if(pin.hGpio){
        GPIO_TypeDef *const puerto = GET_GPIO(pin.hGpio);
        resultado = puerto->IDR & (1 << pin.nrPin);
    }
    return resultado;
}
void BP_Pin_escribe(BP_HPin hpin, bool valor){
    PinDescriptor const pin = GET_PIN(hpin);
    if (pin.hGpio){
        GPIO_TypeDef *const puerto = GET_GPIO(pin.hGpio);
        if (valor)
            puerto->BSRR = 1<<pin.nrPin;
        else
            puerto->BRR  = 1<<pin.nrPin;
    }
}
bool BP_Pin_estadoSalida(BP_HPin hpin){
    bool resultado = false;
    PinDescriptor const pin = GET_PIN(hpin);
    if(pin.hGpio){
        GPIO_TypeDef *const puerto = GET_GPIO(pin.hGpio);
        resultado = puerto->ODR & (1 << pin.nrPin);
    }
    return resultado;
}


static void despacha_exti_pin(BP_HPin hPin){
    ExtiHandlerDescriptor const hd = GET_EXTI_HANDLER(hPin);
    if (hd.valido){
        BP_Pin_FlancoInterrupcion const flanco = (BP_Pin_lee(hPin)) ? PIN_INT_ASCENDENTE : 
                                                                PIN_INT_DESCENDENTE;
        if (hd.flanco & flanco)
            hd.handler();
    }
}

#define PR_MASK(nrPin) (1UL<<(nrPin))

void EXTI0_IRQHandler(void){
    EXTI->PR = PR_MASK(0);
    despacha_exti_pin(PA0);
    despacha_exti_pin(PA1);
}
void EXTI1_IRQHandler(void){
    EXTI->PR = PR_MASK(1);
    despacha_exti_pin(PA1);
    despacha_exti_pin(PB1);
}
void EXTI2_IRQHandler(void){
    EXTI->PR = PR_MASK(2);
    despacha_exti_pin(PA2);
}
void EXTI3_IRQHandler(void){
    EXTI->PR = PR_MASK(3);
    despacha_exti_pin(PA3);
    despacha_exti_pin(PB3);
}
void EXTI4_IRQHandler(void){
    EXTI->PR = PR_MASK(4);
    despacha_exti_pin(PA4);
    despacha_exti_pin(PB4);
}
void EXTI9_5_IRQHandler(void){
    // Fuentes de interrupción según número de ceros a la izquierda luego de alinear
    enum{EXTI_9,EXTI_8,EXTI_7,EXTI_6,EXTI_5};
    switch(__CLZ(EXTI->PR << (31-9))){
    case EXTI_9:
        EXTI->PR = PR_MASK(9);
        despacha_exti_pin(PA9);
        despacha_exti_pin(PB9);
    break;case EXTI_8:
        EXTI->PR = PR_MASK(8);
        despacha_exti_pin(PA8);
        despacha_exti_pin(PB8);
    break;case EXTI_7:
        EXTI->PR = PR_MASK(7);
        despacha_exti_pin(PA7);
        despacha_exti_pin(PB7);
    break;case EXTI_6:
        EXTI->PR = PR_MASK(6);
        despacha_exti_pin(PA6);
        despacha_exti_pin(PB6);
    break;case EXTI_5:
        EXTI->PR = PR_MASK(5);
        despacha_exti_pin(PA5);
        despacha_exti_pin(PB5);
    break;default:
    break;
    }
}
void EXTI15_10_IRQHandler(void){
    enum{EXTI_15,EXTI_14,EXTI_13,EXTI_12,EXTI_11,EXTI_10};
    switch(__CLZ(EXTI->PR << (31-15))){
    case EXTI_15:
        EXTI->PR = PR_MASK(15);
        despacha_exti_pin(PA15);
        despacha_exti_pin(PB15);
        despacha_exti_pin(PC15);
    break;case EXTI_14:
        EXTI->PR = PR_MASK(14);
        despacha_exti_pin(PB14);
        despacha_exti_pin(PC14);
    break;case EXTI_13:
        EXTI->PR = PR_MASK(13);
        despacha_exti_pin(PB13);
        despacha_exti_pin(PC13);
    break;case EXTI_12:
        EXTI->PR = PR_MASK(12);
        despacha_exti_pin(PA12);
        despacha_exti_pin(PB12);
    break;case EXTI_11:
        EXTI->PR = PR_MASK(11);
        despacha_exti_pin(PA11);
        despacha_exti_pin(PB11);
    break;case EXTI_10:
        EXTI->PR = PR_MASK(10);
        despacha_exti_pin(PA10);
        despacha_exti_pin(PB10);
    break;default:
    break;
    }
}
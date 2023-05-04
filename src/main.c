#include <soporte_bluepill.h>
#include <stdint.h>

static bool volatile pulsado;
static bool volatile estado_led;
static void handler_exti_proc(BP_HandlerObject*self){
    (void)self;
    pulsado = true;
}
static void handler_temp2_proc(BP_HandlerObject*self){
    (void)self;
    pulsado = false;
}
static void handler_temp1_proc(BP_HandlerObject *self){
    (void)self;
    estado_led = !estado_led;
    BP_Pin_escribe(P_LED,estado_led);
}
BP_HandlerObject const handler_exti = {.handler=handler_exti_proc};
BP_HandlerObject const handler_temp1 = {.handler=handler_temp1_proc};
BP_HandlerObject const handler_temp2 = {.handler=handler_temp2_proc};

int main(void){
    BP_init();
    BP_Pin_modoSalida(P_LED,PIN_2MHz,false);
    estado_led = BP_P_LED_OFF;
    BP_Pin_escribe(P_LED,estado_led);
    BP_Pin_modoEntrada(PB9,PIN_PULLUP);
    BP_Pin_configuraInterrupcionExterna(PB9,&handler_exti,PIN_INT_DESCENDENTE);
    BP_Pin_modoSalida(PA5,PIN_50MHz,false);
    BP_Pin_modoSalida(PA6,PIN_50MHz,false);
    enum {ESPERA,PULSADO} estado = ESPERA;
    for(;;){
        switch(estado){
        case ESPERA:
            if(pulsado){
                BP_retardo(500,&handler_temp1);
                BP_retardo(100,&handler_temp1);
                BP_retardo(400,&handler_temp1);
                BP_retardo(1,&handler_temp1);
                BP_retardo(10,&handler_temp1);
                BP_retardo(30,&handler_temp1);
                BP_retardo(500,&handler_temp2);
                estado = PULSADO;
            }
        break;case PULSADO:
            if(!pulsado){
                estado = ESPERA;
            }
        break;default:
            estado = ESPERA;
        break;
        }
        BP_esperaInterrupcion();
    }
    return 0;
}
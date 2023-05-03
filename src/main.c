#include <soporte_bluepill.h>
#include <stdint.h>

int main(void){
    BP_init();
    uint32_t ticks = BP_getTicks();
    BP_Pin_modoSalida(P_LED,PIN_2MHz,false);
    bool estado_led = BP_P_LED_OFF;
    BP_Pin_escribe(P_LED,estado_led);
    BP_Pin_modoSalida(PB9,PIN_2MHz,false);
    BP_Pin_escribe(PB9,0);
    for(;;){
        const uint32_t nticks = BP_getTicks();
        if (nticks-ticks >= 500){
            ticks = nticks;
            estado_led = !estado_led;
            BP_Pin_escribe(P_LED,estado_led);
        }
        BP_Pin_escribe(PB9,1);
        BP_esperaInterrupcion();
        BP_Pin_escribe(PB9,0);
    }
    return 0;
}
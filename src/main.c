#include <soporte_bluepill.h>
#include <stdint.h>

int main(void){
    BP_init();
    uint32_t ticks = BP_get_ticks();
    BP_Pin_modoSalida(P_LED,PIN_2MHz,false);
    bool estado_led = BP_P_LED_OFF;
    BP_Pin_escribe(P_LED,estado_led);
    for(;;){
        const uint32_t nticks = BP_get_ticks();
        if (nticks-ticks >= 500){
            ticks = nticks;
            estado_led = !estado_led;
            BP_Pin_escribe(P_LED,estado_led);
        }
    }
    return 0;
}
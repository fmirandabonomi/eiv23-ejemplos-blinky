#include <soporte_bluepill.h>
#include <stdint.h>

int main(void){
    SBP_init();
    uint32_t ticks = SBP_get_ticks();
    SBP_Pin_modoSalida(P_LED,PIN_2MHz,false);
    bool estado_led = SBP_P_LED_OFF;
    SBP_Pin_escribe(P_LED,estado_led);
    for(;;){
        const uint32_t nticks = SBP_get_ticks();
        if (nticks-ticks >= 500){
            ticks = nticks;
            estado_led = !estado_led;
            SBP_Pin_escribe(P_LED,estado_led);
        }
    }
    return 0;
}
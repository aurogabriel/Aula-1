/*########################################################
 * Título:
 * !!!!!!!!!!!!!! Estudos da DSP- Pisca LED !!!!!!!!!!!!!
 *
 * Autor: Auro Gabriel - 08/2022
 *
 * Mapa das portas usadas:
 *
 *      GPIO0 - Pino 40 => 10 do J4
 *
 * #######################################################
 */

#include "F28x_Project.h"
Uint32 i;

int main(void)
{
    InitSysCtrl();
    EALLOW; //Registradores protegidos (Não é o caso do GPIO0 mas coloquei pra exemplificar o uso)
    GpioCtrlRegs.GPAMUX1.bit.GPIO0=0; //Configuro o MUX da porta
    //###########################################################
    //Sempre GPyGMUX depois de GPyMUX, nunca o contrário
    //###########################################################
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0=0;  //Configuro o grupo de MUX da porta
    GpioCtrlRegs.GPADIR.bit.GPIO0=1;   //Configuro Se é input ou output
    GpioCtrlRegs.GPAPUD.bit.GPIO0=1;   //Configuro se o pull up ta ligado ou desligado PUD= PullUp Disable
    EDIS;//Fecha "Registradores protegidos (Não é o caso do GPIO0 mas coloquei pra exemplificar o uso)

    GpioDataRegs.GPADAT.bit.GPIO0=1;
    while(1){
            for(i=0;i<0x000FFFFF;i++){} //Delay que faz ele piscar -- Uma contagem muito grande que não faz nada
            GpioDataRegs.GPATOGGLE.bit.GPIO0=1; // GPATOGGLE -> Regs que faz o Toggle.
        }
    return;
}

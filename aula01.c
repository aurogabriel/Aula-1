
#include "F28x_Project.h"
Uint32 i;

int main(void)
{
    InitSysCtrl();
    EALLOW; //Registradores protegidos (Não é o caso do GPIO0 mas coloquei pra exemplificar o uso)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0=0;  //Configuro o MUX da porta
    GpioCtrlRegs.GPAMUX1.bit.GPIO0=0; //Configuro o MUX da porta
    GpioCtrlRegs.GPADIR.bit.GPIO0=1;   //Configuro Se é input ou output
    GpioCtrlRegs.GPAPUD.bit.GPIO0=1;   //Configuro se o pull up ta ligado ou desligado PUD= PullUp Disable
    EDIS;//Fecha "Registradores protegidos (Não é o caso do GPIO0 mas coloquei pra exemplificar o uso)

    GpioDataRegs.GPADAT.bit.GPIO0=1;
    while(1){
            for(i=0;i<0x000FFFFF;i++){} //Delay que faz ele piscar -- Uma contagem muito grande que não faz nada
            GpioDataRegs.GPATOGGLE.bit.GPIO0=1;

        }

}

//CÓDIGO PARA CONTROLE DE VELOCIDADE DE UM MOTOR DE INDUÇÃO TRIFÁSICO DE 0.25CV.
//TRABALHO DE CONCLUSÃO DE CURSO

//ALUNO: LEONARDO DUARTE MILFONT MATRÍCULA: 378771
//PROFESSORES ORIENTADORES : WILKLEY BEZERRA E DALTON HONÓRIO

// INCLUDE FILES.
#include <math.h>
#include <stdio.h>
#include "F28x_Project.h"
#include "params.h"
#include "datastruct.h"


#define DPI 6.28318530717958647692

msg_data *data1 = (void *)(uint32_t)0x03FC00;
//VARIÁVEIS GLOBAIS.

float t = 0;//Variável para medição de tempo.
int saida = 0; //Variável para controle do loop principal.
float b,c,d = 0; // Valores de moduladora para PWM em malha aberta.
float theta_atual_ma = 0; //Ângulo para aplicação das transformadas de eixo em malha aberta.
float theta_ant_ma = 0; // Armazena valor anterior do ângulo para aplicação das transformadsa de eixo em malha aberta.
float theta_ant = 0; // Armazena valor anterior do ângulo para aplicação das transformadsa de eixo em malha fechada.
float theta_atual = 0; // Armazena valor anterior do ângulo para aplicação das transformadsa de eixo em malha feechada.

//Variáveis de auxílio para aplicação das transformadas de eixo.
unsigned int pin12 = 0,cont_zero = 0; //Variáveis de controle gerias.
//####################################################


int malha=1; //Auro: mudança do pin12 (para mudar malha aberta e malha fechada)


//##############################################################################
//      Auro: Coloquei aqui pra poder usar a lógica do vinicius
//##############################################################################

#define PORTADORA_FREQ 6000
#define MODULADORA_FREQ 60
#define NOS 200 //Number of Samples
#define Mi 0.8   // Índice de modulação
#define pi 3.14159265358979323846



Uint16 AlarmCount=0;                        // Alarm Counter
Uint16 index= 0;
Uint16 w1,w2,w3;
Uint16 TB_Prd;
Uint16 TB_Prescale;
Uint16 Comando_L_D;
Uint16 aux = 0;
Uint16 SPWM_State = 0;

//#############################################################################

//  -> Auro

//#############################################################################


int Ia_med, Ib_med, Ic_med, Ia_med0, Ib_med0, Ic_med0, I_d_AD, I_q_AD; //Tratamento das variáveis medidas mo conversor AD.
float  ia, ib, ic;
float va, vb, vc;

//Correntes de referência nos eixos d/q.
float I_d =0;
float I_q =0;
float erro_cd =0; //Erro para o controlador (eixo d).
float erro_cq =0; //Erro para o controlador (eixo q).
float erro_cd_ant1 =0; //Erro para o controlador uma amostra anterior (eixo d).
float erro_cq_ant1 =0; //Erro para o controladoruyma amostra anterior (eixo q).
float v_atual_d =0; //Saída de tensão atual do controlador (eixo d).
float v_atual_q =0; //Saída de tensão atual do controlador (eixo q).
float v_d_ant =0; //Saída de tensão  do controlador uma amostra anteriorr (eixo d).
float v_q_ant =0; //Saída de tensão  do controlador uma amostra anterior (eixo q).
float I_atual_d=0; //Saída de corrente da planta controlada (eixo d).
float I_atual_q=0; //Saída de corrente da planta controlada (eixo q).
float I_d_ant =0; //Saída de corrente  do controlador uma amostra anteriorr (eixo d).
float I_q_ant =0; //Saída de corrente  do controlador uma amostra anterior (eixo q).


int Va_med = 0, Vb_med = 0, Vc_med = 0;

float Va =0; //tensões de saída do modulador.
float Vb =0;
float Vc =0;
float theta_rad; // Ângulo calculado no integrador.
float ref_kd =0; //Valores de referência para os eixos d e q.
float ref_kq =0;
unsigned int ref_kd_AD =0; //Valores de referência para os eixos d e q em valores digitais.
unsigned int ref_kq_AD =0;
float ref_kq_ant =0;//Valores passados para a referência de corrente no eixo q.

//Variáveis gerais.

float T = 0; //Constante de tempo do rotor.
float w=0;   //Velocidade mecânica do rotor em rad/s.
float wa=0; // Velocidade angular elétrica medida em rpm

//float w_eletrica=0;  // Velocidade angular elétrica medida em rad/s. // -->Auro: Não ta sendo Usada

unsigned int Posicao_ADC = 0; //Leitura da velocidade no módulo eqep.
float Rotor_Posicao = 0; //Posição angular do rotor.
float Rotor_Posicao_Ant = 0; //Memória da posição angular do rotor.
float delta_posicao =0; //Variação da posição angular do rotor.
float delta_posicao_1 =0; //Variação da posição angular do rotor (variável auxiliar).
float Velo = 0; //Medida de velocidade mecânica do rotor em RPM.
float Velo_avg =0; //Leitura filtrada de velocidade.
unsigned long Velo_ADC =0 ; //Medida digital da velocidade.
float Velo_aux = 0; //Variável auxiliar para controle de velocidade.
float wsl=0;  //Velocidade de escorregamento do rotor.
float w_tot =0; //Velocidade elétrica do rotor em rad/s.
float w_avg =0; //Velocidade angular depois do filtro. (Auro: essa velocidade está em radianos por minuto)
float ref_Velo = 800; //Referência de velocidade para o controle do motor.

//unsigned int ref_Velo_AD =0; //Referência digital de velocidade.// -->Auro: Não ta sendo Usada

unsigned int ref = 5;   //Escolha da referência de velocidade.
float ref_Velo1 =0; //Referência de velocidade para o controle do motor.
float cont_velo =0; //Contador para a malha de velocidade.
float cont_velo_aux =0; //Contador para a referência de velocidade.
float erro_Velo =0; // Erro para a amlha de velocidade.
float erro_Velo_ant1 =0; //Valores passados para o erro da malha de velocidade.
float Velo_ant1 =0;//Valores passados para a velociddae do motor.


float V_a = 0, V_b = 0, V_c = 0;
float va_obs = 0, vb_obs = 0, vc_obs = 0, ia_obs = 0, ib_obs = 0, ic_obs = 0, refobs = 0, iq = 0.5;
int n_degrau = 0;




void SetupTimers(void);
void ConfigureDAC(void);
void SetupADC(void);




void zero_sensores(void);
void SetupEQEP1(void);
//__interrupt void adca1_isr(void);// -->Auro: Não ta sendo Usada
__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
__interrupt void epwm3_isr(void);
__interrupt void timer0_isr(void);
__interrupt void eqep1_isr(void);

float ref_OBS(float, float, float, float, float, float);


//#############################################################################################


void Setup_GPIO(void);
void Setup_INTERRUPT(void);
void Setup_ePWM(void);
void Setup_ADC(void);

void Liga_Bancada(void);
void Desliga_Bancada(void);
void Stop_SPWM(void);

void Set_ePWM_Frequency(uint32_t freq_pwm);


//__interrupt void alarm_handler_isr(void);     // Alarm Handler interrupt service routine function prototype.

__interrupt void adca_isr(void);




//#############################################################################################




// MAIN
void main(void){



     InitSysCtrl();                          // PLL, WatchDog, enable Peripheral Clocks
     InitGpio();                             // Inicialização do GPIO


     EALLOW;

         CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =0;
         CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =1;

      EDIS;


    //INICIALIZAÇÃO DO CONTROLE DO SISTEMA.
    EALLOW;

     DevCfgRegs.CPUSEL5.bit.SCI_A = 1; // Handoff do SCIA para a CPU02

    //EQEP.
    InitEQep1Gpio();
    GPIO_SetupPinMux(43, GPIO_MUX_CPU2, 0xF);
    GPIO_SetupPinOptions(43, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(42, GPIO_MUX_CPU2, 0xF);
    GPIO_SetupPinOptions(42, GPIO_OUTPUT, GPIO_ASYNC);

    EDIS;


    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;
    EALLOW;

    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    // AS INTERRUPÇÕES QUE SÃO USADAS SÃO REMAPEADAS.
    PieVectTable.ADCA1_INT = &adca_isr;
    PieVectTable.TIMER0_INT = &timer0_isr;
    PieVectTable.EQEP1_INT = &eqep1_isr;

    EDIS;


    //##########__CONFIGURAÇÕES INICIAIS__#######################################################################

      Set_ePWM_Frequency(PORTADORA_FREQ);                // Set the ePWM frequence in Hz. Min 193 hz. A frequência da portadora deve ser um múltiplo da moduladora.
                                                         //Para garantir um número inteiro de pulsos por semiciclo. (RASHID).

      Setup_GPIO();                                      // Configuração dos GPIOs
      Setup_ePWM();                                      // Abre todas as chaves
      Setup_ADC();                                        // Configuração das interrupções
      ConfigureDAC();
      SetupEQEP1();
      SetupTimers();


    while(1){



        //Auro: Mudei aqui so pra manter a lógica do vinicius

        Comando_L_D != 0 ? Liga_Bancada():Desliga_Bancada(); // Uiliza o debug em tempo real para ligar ou desligar a bancada
                                                                    // Alterando o valor da variável Comando_L_D na janela de expressões
                                                                    // do code composer studio.


        //MODULADORA SENOIDAL PARA PWM EM MALHA ABERTA.
               b=4000 + 3200*__sin(theta_atual_ma);
               c=4000 + 3200*__sin(theta_atual_ma  -  ((DPI)/3));
               d=4000 + 3200*__sin(theta_atual_ma  -  ((2*DPI)/3));

    }
}


//Calculo da tensão usando as pwms:

const int vpeak = 250;
int epwm1_high = 0;
__interrupt void epwm1_isr(){

       if(epwm1_high == 0 ){
            vc_obs = vpeak;
            epwm1_high = 1;
            EPwm4Regs.ETSEL.bit.INTSEL = 0b101;
        }

       else{
            vc_obs = 0;
            epwm1_high = 0;
            EPwm4Regs.ETSEL.bit.INTSEL = 0b100;
        }
        EPwm4Regs.ETCLR.bit.INT = 1;
        PieCtrlRegs.PIEACK.all |= PIEACK_GROUP3;

}

//Calculo da tensão usando as pwms:

int epwm2_high = 0;
__interrupt void epwm2_isr(){
    if(epwm2_high == 0){
        vb_obs = vpeak;
        epwm2_high = 1;
        EPwm5Regs.ETSEL.bit.INTSEL = 0b101;
    }else{
        vb_obs = 0;
        epwm2_high = 0;
        EPwm5Regs.ETSEL.bit.INTSEL = 0b100;
    }
    EPwm5Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP3;
}

//Calculo da tensão usando as pwms:

int epwm3_high = 0;
__interrupt void epwm3_isr(){
    if(epwm3_high == 0){
        va_obs = vpeak;
        epwm3_high = 1;
        EPwm6Regs.ETSEL.bit.INTSEL = 0b101;
    }else{
        va_obs = 0;
        epwm3_high = 0;
        EPwm6Regs.ETSEL.bit.INTSEL = 0b100;
    }
    EPwm6Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP3;
}




//CONFIGURAÇÃO DO TIMER 1.
void SetupTimers(void){
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer1, 200, Ts*1e6); //200MHz, 80us
    CpuTimer1Regs.TCR.all = 0x4000;

    ConfigCpuTimer(&CpuTimer0, 200, 160); //200MHz, 80us
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer0Regs.TCR.bit.TIE = 1;
    CpuTimer0Regs.TCR.bit.TIF = 1;
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;
}






    //CONFIGURAÇÃO DO DAC.
void ConfigureDAC(){
        EALLOW;

        //DAC-B
        DacbRegs.DACCTL.bit.DACREFSEL = 1;          // Use ADC references
        DacbRegs.DACCTL.bit.LOADMODE = 0;           // Load on next SYSCLK
        DacbRegs.DACVALS.all = 0x0800;              // Set mid-range

        //DAC -A
        DacaRegs.DACCTL.bit.DACREFSEL = 1;          // Use ADC references
        DacaRegs.DACCTL.bit.LOADMODE = 0;           // Load on next SYSCLK
        DacaRegs.DACVALS.all = 0x0800;              // Set mid-range
        DacaRegs.DACOUTEN.bit.DACOUTEN = 1;         // Enable DAC

        EDIS;
    }


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    unsigned long i_amostra = 0;
    float pos;
    int cont_controle = 0;
    unsigned int tc = 0;

    __interrupt void timer0_isr(){

                /*####################################################
                 *
                 *              Usado Malha Aberta
                 *
                 * ###################################################
                 */

                PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

                theta_atual_ma = ((0.000160)*DPI*20) + theta_ant_ma; // Integrador discreto.
                theta_ant_ma = theta_atual_ma;                      // 0.000160 = 160us -> tempo de amostragem do integrador

                if(theta_atual_ma > DPI){
                    theta_atual_ma = theta_atual_ma -DPI;
                    theta_ant_ma = theta_atual_ma;// Controle para manter Theta
                }
                if(theta_atual_ma < 0){
                    theta_atual_ma = theta_atual_ma +DPI;
                    theta_ant_ma = theta_atual_ma;// Controle para manter Theta
                }

                //###################################################


                //LEITURA EQEP -POSIÇÃO.
                Posicao_ADC  = EQep1Regs.QPOSCNT;



                //Cálculo da velocidade para o erro com o encoder para malha fechada

                if(EQep1Regs.QEPSTS.bit.COEF == 0 && EQep1Regs.QEPSTS.bit.CDEF == 0){

                    /*########################################################################################
                     *   !!!!!!!!!! FÓRMULA USADA, OLHAR EXEMPLO DA TI, EM ANEXO NO MOP!!!!!!!!!!!!!!
                     *
                     *                                        upps/4*ppr
                     *                              wa= -------------------------
                     *                                    (t2-t1)/(200Mhz/ccps)
                     *
                     *########################################################################################
                     */

                    wa = (32.0*60.0/2048.0)/(EQep1Regs.QCPRD*128.0/200.0e6);        // teoricamente, está em rotações por segundo

                }
                else{
                    EQep1Regs.QEPSTS.bit.COEF = 0;
                    EQep1Regs.QEPSTS.bit.CDEF = 0;
                }

                //POSIÇAO ANGULAR.
                Rotor_Posicao = ((float)Posicao_ADC)*DPI/2048.0; // DoisPi/2048 = Ângulo por pulso de quadratura    -->Auro: O produto desses dois dá o ângulo que já rodou
                                                                //  Posicao_ADC = Quantidade de pulsos                        # "Rotor_Posicao" é em radianos

                delta_posicao = Rotor_Posicao - Rotor_Posicao_Ant; // -->Auro:Atualiza Rotor_Posicao_Ant la no final.

                //ROTOR PARADO
                if(delta_posicao == 0||Velo == 0){
                    Velo_aux = Velo;
                }

                //VELOCIDADE NEGATIVA
                if (delta_posicao <0 || Velo<0){
                    delta_posicao = delta_posicao_1;
                }

                //VELOCIDADE POSITIVA.
                if (delta_posicao>0 || Velo>0){


                   //DERIVADA DISCRETA DA POSIÇÃO = VELOCIDADE.
                    Velo = wa;



                    // Filtro passa baixa para f32f
                    Velo_avg = Velo_avg + 0.000160*100.0*(Velo - Velo_avg); // -->0.000160=tempo de amostragem do integrador // 100.0 Teria que ter unidade de frequencia para que "Velo - Velo_avg" tenha sentido dimensioalmente
                                                                                                                             // 100Hz seria frequencia do que ? Do integrador?

                    w_avg = (Velo_avg*DPI)/(60); // --> DoisPi para tornar em radianos
                                                 //     dividido por 60 para voltar para minutos ( Perceba que Velo_avg tem a msm unidade de wa {?} )


                }


                Rotor_Posicao_Ant = Rotor_Posicao; //Atualiza a variável
                Velo_ant1 = Velo_avg;   //Atualiza a variável


            //INÍCIO DA MALHA DE CONTROLE.

/*########################################################################
 * ##################! MALHA FECHADA !######################################
 * #######################################################################
 */
           if (malha==1){

 //Análogo ao que foi feito para a malha aberta, mas agora levando em conta a malha fechada e a velocidade do encoder.
 //     CAMPO ORIENTADO INDIRETO.
                float v_controle = Velo_avg;

                  T = ((Llr+Lm)/Rr);
                  wsl = (ref_kq)/(T*ref_kd);
                  w_tot = wsl + v_controle*DPI/60.0;

                theta_atual = ((160E-006)*w_tot) + theta_ant; // Integrador discreto. 160E-006 = 160us -tempo de amostragem do integrador
                theta_ant = theta_atual;

                if(theta_atual > DPI){
                    theta_atual = theta_atual -DPI;
                    theta_ant = theta_atual;// Controle para manter Theta
                }
                if(theta_atual < 0){
                    theta_atual = theta_atual +DPI;
                    theta_ant = theta_atual;// Controle para manter Theta
                }

                // APLICAÇÃO DAS TRANSFORMADAS DE EIXO DE CLARKE E PARK.
                // 2 - park.
                theta_rad = theta_atual;

                if(theta_rad > DPI){
                    theta_rad = theta_rad - DPI;
                }
                if(theta_rad < -DPI){
                    theta_rad = theta_rad + DPI;
                }

                I_d = 0.81649658092772603273242802490196*(ic*__cos(theta_rad) + ib*__cos(theta_rad - ((DPI)/3)) + ia*__cos(theta_rad -((2*DPI)/3)));
                I_q = 0.81649658092772603273242802490196*(-ic*__sin(theta_rad) - ib*__sin(theta_rad - ((DPI)/3)) - ia*__sin(theta_rad -((2*DPI)/3)));

                I_atual_d = I_d;
                I_atual_q = I_q;

                data1->id = I_d;
                data1->iq = I_q;


                //MALHA DE VELOCIDADE.
                cont_velo ++;
                if(cont_velo==1225){

                    //Referência degrau
                    if(ref==1){
                        ref_Velo = 1000;
                    }

                    //Referência triangular
                    if (ref==2){
                        t = 0.096*cont_velo_aux;
                        if(t<2){
                            ref_Velo = 100*(t) + 600 ;
                            cont_velo_aux ++;
                        }
                        if(t>=2){
                            ref_Velo = -100*(t) + 1000 ;
                            cont_velo_aux ++;
                        }
                        if(t>=4){
                            cont_velo_aux = 0;
                        }
                    }

                    //Referência trapezoidal
                    if (ref==3){
                        t = 0.096*cont_velo_aux;
                        if(t<1){
                            ref_Velo = 100*(t) + 600 ;
                            cont_velo_aux ++;
                        }
                        if(t>=1 && t<2){
                            ref_Velo = 800;
                            cont_velo_aux ++;
                        }
                        if(t>=2){
                            ref_Velo = -100*(t) + 1100 ;
                            cont_velo_aux ++;
                        }
                        if(t>=5 && t<6){
                            ref_Velo = 600;
                            cont_velo_aux ++;
                        }
                        if(t>=5 && t<6){
                            cont_velo_aux = 0;
                        }
                    }

                    //Referência senoidal
                    if(ref==4){
                        t = 0.096*cont_velo_aux;
                        ref_Velo = 600 + 200*__sin(DPI*0.25*t);
                        cont_velo_aux++;
                        if(t>=4){
                            cont_velo_aux = 0;
                        }
                    }

                    //Referência Quadrada
                    if(ref==5){
                        t = 0.096*cont_velo_aux;
                        if(t >= 5){
                            ref_Velo = 1000;
                        }else{
                            ref_Velo = 500;
                        }
                        cont_velo_aux++;
                        if(t>=10){
                            cont_velo_aux = 0;
                        }
                    }
                    cont_velo = 0;

              //PI malha de velocidade
                    double kp = 6.6667e-05;
                    double ki = 6.6334e-05;

                    erro_Velo = ref_Velo - v_controle;
                    ref_kq = ref_kq_ant + ki*erro_Velo_ant1 + kp*erro_Velo;
                    erro_Velo_ant1 = erro_Velo;
                    ref_kq_ant = ref_kq;
                }
                ref_kd = 0.4;
                n_degrau++;
                if(n_degrau >= 6250*3){
                    n_degrau = 0;
                    if(iq == 0.5){
                        iq = 0.7;
                    }else{
                        iq = 0.5;
                    }
                }

                //CÁLCULO DOS ERROS
                erro_cd = ref_kd - I_d;
                erro_cq =  ref_kq - I_q;

                float kpi = 103, kii = 100.4;
                v_atual_d = v_d_ant + kpi*erro_cd - kii*erro_cd_ant1;
                v_atual_q = v_q_ant + kpi*erro_cq - kii*erro_cq_ant1;


                //Atualização das variáveis.
                erro_cd_ant1 = erro_cd;
                erro_cq_ant1 = erro_cq;

                I_q_ant = I_atual_q;
                v_d_ant = v_atual_d;
                v_q_ant = v_atual_q;

                //APLICAÇÃO DAS TRANSFORMADAS INVERSAS DE CLARKE E PARK:
                Va =  0.81649658092772603273242802490196*(v_atual_d*__cos(theta_rad) - v_atual_q*__sin(theta_rad));
                Vb =  0.81649658092772603273242802490196*(v_atual_d*__cos(theta_rad - ((DPI)/3.0)) - v_atual_q*__sin(theta_rad - ((DPI)/3.0)));
                Vc =  0.81649658092772603273242802490196* (v_atual_d*__cos(theta_rad - ((2*DPI)/3.0)) - v_atual_q*__sin(theta_rad - ((2*DPI)/3.0)));

                //GANHO DE MODULAÇÃO:
                Va = Va*16;
                Vb = Vb*16;
                Vc = Vc*16;

                //OFFSET:
                Va = Va + 4000;
                Vb = Vb + 4000;
                Vc = Vc + 4000;

                //SATURADOR:
                if (Va >= 8000){
                    Va = 7950;
                }
                if (Va <= 0){
                    Va = 50;
                }
                if (Vb >= 8000){
                    Vb = 7950;
                }
                if (Vb <= 0 ){
                    Vb = 50;
                }
                if (Vc >= 8000){
                    Vc = 7950;
                }
                if (Vc <= 0){
                    Vc = 50;
                }


                V_a = (Va - 4000)/16.0;
                V_b = (Vb - 4000)/16.0;
                V_c = (Vc - 4000)/16.0;


        /*#################################################################################################################
        *                      Leituras do Osciloscopio  1
        * #################################################################################################################
        */
                //   DacaRegs.DACVALS.all = (V_a*1024.0/1000.0) + 1024;
               //    DacbRegs.DACVALS.all = (V_b*1024.0/1000.0) + 1024;

        //#################################################################################################################


                //MODULAÇÃO:
                EPwm4Regs.CMPA.bit.CMPA = Vc; // adjust duty for output EPWM4A
                EPwm5Regs.CMPA.bit.CMPA = Vb; // adjust duty for output EPWM5A
                EPwm6Regs.CMPA.bit.CMPA = Va; // adjust duty for output EPWM6A

               }

           i_amostra++;
           data1->j = i_amostra;


/*########################################################################
 * ##################! MALHA ABERTA !######################################
 * #######################################################################
 */
            if (malha==0){

                EPwm4Regs.CMPA.bit.CMPA = b; // adjust duty for output EPWM4A
                EPwm5Regs.CMPA.bit.CMPA = c; // adjust duty for output EPWM5A
                EPwm6Regs.CMPA.bit.CMPA = d; // adjust duty for output EPWM6A

                }
        }





float ia_filt = 0, ib_filt = 0, ic_filt = 0;
float ga = 680, gb = 560, gc = 500;


__interrupt void adca_isr(void){

    DINT;
   // AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
  //  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;


    //LEITURA DO CONVERSOR AD.



      va =  __divf32(3.0*AdcaResultRegs.ADCRESULT0,4096.0)-1.65;  // Tensão lida na porta (o equivalente à leitura digital)
      vb =  __divf32(3.0*AdcbResultRegs.ADCRESULT1,4096.0)-1.65;
      vc =  __divf32(3.0*AdccResultRegs.ADCRESULT2,4096.0)-1.65;


      ic =  __divf32(vc ,0.0184);
      ib =  __divf32(vb ,0.0184);
      ia = __divf32(va,0.0184);



    float kk1 = 9.809831e-01, kk2 = 1.901685e-02;
    ia_filt = kk1*ia_filt + kk2*ia;
    ib_filt = kk1*ib_filt + kk2*ib;
    ic_filt = kk1*ic_filt + kk2*ic;

    /*#################################################################################################################
     *                      Leituras do Osciloscopio  2
     * #################################################################################################################
     */
         //  DacaRegs.DACVALS.all = (ib_filt*2048.0/3.0) + 1024;
         //   DacbRegs.DACVALS.all = (ib*2048.0/3.0) + 1024;
     //##################################################################################################################


    ia = ia_filt;
    ib = ib_filt;
    ic = ic_filt;


        refobs = ref_OBS(vc_obs, vb_obs, va_obs, ia, ib, ic);


        /*#################################################################################################################
         *                      Leituras do Osciloscopio  3
         * #################################################################################################################
         */

         DacaRegs.DACVALS.all = Velo_avg * 2.048;
         // DacaRegs.DACVALS.all = refobs * 2.048;

        //#################################################################################################################


        data1->med_vel = Velo_avg;
        data1->obs_vel = refobs;
        data1->ref = ref_Velo;





    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    EINT;


}

//Encoder - Fonseca
__interrupt void eqep1_isr(){

/*
    EQep1Regs.QCLR.bit.UTO = 1;
    EQep1Regs.QCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
*/

}




void SetupEQEP1 (){
    EALLOW;

    EQep1Regs.QUPRD = 1;            // Unit Timer for 100Hz at 200 MHz
                                              // SYSCLKOUT
    EQep1Regs.QDECCTL.bit.QSRC = 00;      // QEP quadrature count mode
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;
    EQep1Regs.QEPCTL.bit.PCRM = 1;         // PCRM=01 mode - QPOSCNT reset on maximum position
    EQep1Regs.QEPCTL.bit.UTE = 1;         // Unit Timeout Enable
    EQep1Regs.QEPCTL.bit.QCLM = 1;        // Latch on unit time out

    EQep1Regs.QPOSMAX = 4*600;               // 2400 contagens

    EQep1Regs.QDECCTL.bit.SWAP = 1;        // troca o sentido da contagem
    EQep1Regs.QEPCTL.bit.QPEN = 1;        // QEP enable



    EQep1Regs.QCAPCTL.bit.UPPS = 5;       // 1/32 for unit position
    EQep1Regs.QCAPCTL.bit.CCPS = 7;       // 1/128 for CAP clock



    EQep1Regs.QCAPCTL.bit.CEN = 1;        // QEP Capture Enable

    //EQep1Regs.QEINT.bit.UTO = 1; // 400 Hz interrupt for speed estimation

    EDIS;
}




/*##################################################################################
 *
 *
                     Auro: Veio da lógica do vinícius, para ativar a
                      bancada com a lógica nova:
 *
 *
 * ##################################################################################
 */

void Setup_GPIO(void){
EALLOW;

//##############################__FONTE 3V3__##############################

    GpioCtrlRegs.GPAGMUX1.bit.GPIO14 = 0;        // GPIO = GPIO (default)
    GpioCtrlRegs.GPAMUX1.bit.GPIO14  = 0;        // GPIO = GPIO (default)
    GpioCtrlRegs.GPADIR.bit.GPIO14   = 1;        // OUTPUT
    GpioDataRegs.GPASET.bit.GPIO14    = 1;        // HIGH

//##############################_GND CABO FLAT_##############################

    //GPIO26
    GpioCtrlRegs.GPAGMUX2.bit.GPIO26 = 0;        // GPIO = GPIO (default)
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;        // GPIO = GPIO (default)
    GpioCtrlRegs.GPADIR.bit.GPIO26   = 1;        // OUTPUT
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;        // LOW

    //GPIO66
    GpioCtrlRegs.GPCGMUX1.bit.GPIO66 = 0;        // GPIO = GPIO (default)
    GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 0;        // GPIO = GPIO (default)
    GpioCtrlRegs.GPCDIR.bit.GPIO66   = 1;        // OUTPUT
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;        // LOW

    //GPIO130
     GpioCtrlRegs.GPEGMUX1.bit.GPIO130 = 0;        // GPIO = GPIO (default)
     GpioCtrlRegs.GPEMUX1.bit.GPIO130 = 0;        // GPIO = GPIO (default)
     GpioCtrlRegs.GPEDIR.bit.GPIO130   = 1;        // OUTPUT
     GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;        // LOW

     //GPIO131
     GpioCtrlRegs.GPEGMUX1.bit.GPIO131 = 0;        // GPIO = GPIO (default)
     GpioCtrlRegs.GPEMUX1.bit.GPIO131 = 0;        // GPIO = GPIO (default)
     GpioCtrlRegs.GPEDIR.bit.GPIO131   = 1;        // OUTPUT
     GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;        // LOW

     //GPIO63
     GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 0;        // GPIO = GPIO (default)
     GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 0;        // GPIO = GPIO (default)
     GpioCtrlRegs.GPBDIR.bit.GPIO63   = 1;        // OUTPUT
     GpioDataRegs.GPBCLEAR.bit.GPIO63 = 1;        // LOW

     //GPIO64
     GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 0;        // GPIO = GPIO (default)
     GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 0;        // GPIO = GPIO (default)
     GpioCtrlRegs.GPCDIR.bit.GPIO64   = 1;        // OUTPUT
     GpioDataRegs.GPCCLEAR.bit.GPIO64 = 1;        // LOW

     //GPIO27
     GpioCtrlRegs.GPAGMUX2.bit.GPIO27= 0;        // GPIO = GPIO (default)
     GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;        // GPIO = GPIO (default)
     GpioCtrlRegs.GPADIR.bit.GPIO27   = 1;        // OUTPUT
     GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

     //GPIO25
      GpioCtrlRegs.GPAGMUX2.bit.GPIO25= 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPADIR.bit.GPIO25   = 1;        // OUTPUT
      GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

      //GPIO32
      GpioCtrlRegs.GPBGMUX1.bit.GPIO32 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPBDIR.bit.GPIO32   = 1;        // OUTPUT
      GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;        // LOW

      //GPIO19
      GpioCtrlRegs.GPAGMUX2.bit.GPIO19= 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPADIR.bit.GPIO19  = 1;        // OUTPUT
      GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;

      //GPIO18
      GpioCtrlRegs.GPAGMUX2.bit.GPIO18= 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPADIR.bit.GPIO18   = 1;        // OUTPUT
      GpioDataRegs.GPACLEAR.bit.GPIO18 = 1;

      //GPIO67
      GpioCtrlRegs.GPCGMUX1.bit.GPIO67 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPCDIR.bit.GPIO67   = 1;        // OUTPUT
      GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;        // LOW

      //GPIO111
      GpioCtrlRegs.GPDGMUX1.bit.GPIO111 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPDMUX1.bit.GPIO111 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPDDIR.bit.GPIO111   = 1;        // OUTPUT
      GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;        // LOW

      //GPIO60
      GpioCtrlRegs.GPBGMUX2.bit.GPIO60 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPBDIR.bit.GPIO60   = 1;        // OUTPUT
      GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;        // LOW

      //GPIO22
      GpioCtrlRegs.GPAGMUX2.bit.GPIO22= 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;        // GPIO = GPIO (default)
      GpioCtrlRegs.GPADIR.bit.GPIO22   = 1;        // OUTPUT
      GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

//##############################__ePWM 4, 5 e 6__###########################


       GpioCtrlRegs.GPAGMUX1.bit.GPIO6 = 0;
       GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;
       GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;

       GpioCtrlRegs.GPAGMUX1.bit.GPIO7 = 0;
       GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;
       GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;


      GpioCtrlRegs.GPAGMUX1.bit.GPIO8 = 0;
      GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;
      GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;

      GpioCtrlRegs.GPAGMUX1.bit.GPIO9 = 0;
      GpioCtrlRegs.GPAMUX1.bit.GPIO9= 1;
      GpioCtrlRegs.GPAPUD.bit.GPIO9= 1;


      GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 0;
      GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;
      GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;

      GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = 0;
      GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;
      GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;


//##############################__LEDS do DSP : ALARME__##############################

    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31  = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO31  = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO31 =1;


    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 0;

    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO34  = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO34  = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO34 =1;

//##############################__ALARME__##############################

   GpioCtrlRegs.GPAGMUX1.bit.GPIO15 = 0;        // GPIO = GPIO (default)
   GpioCtrlRegs.GPAMUX1.bit.GPIO15  = 0;        // GPIO = GPIO (default)
   GpioCtrlRegs.GPADIR.bit.GPIO15   = 0;        // INPUT
   GpioCtrlRegs.GPAQSEL1.bit.GPIO15   = 2;          // XINT2 Qual using 6 samples
   GpioCtrlRegs.GPACTRL.bit.QUALPRD1 = 0xFF;      // Each sampling window
                                                  // is 510*SYSCLKOUT
  // InputXbarRegs.INPUT5SELECT = 15;               // GPIO 15 is XINT2


//#############################__Acionamento e desligamento da bancada__###########

   //Base do transistor que aciona o relé da fonte de controle.
   GpioCtrlRegs.GPDGMUX1.bit.GPIO105 = 0;
   GpioCtrlRegs.GPDGMUX1.bit.GPIO105 = 0;
   GpioCtrlRegs.GPDDIR.bit.GPIO105 = 1;
   //Base do transistor que aciona o relé da fonte de potência.
   GpioCtrlRegs.GPDGMUX1.bit.GPIO104 = 0;
   GpioCtrlRegs.GPDGMUX1.bit.GPIO104 = 0;
   GpioCtrlRegs.GPDDIR.bit.GPIO104 = 1;


//##################################__ADC__##########

   GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 0;



EDIS;


}


void Liga_Bancada(void)
{

  if(aux == 0 )
  {
      EALLOW;

        Stop_SPWM();

        GpioDataRegs.GPDSET.bit.GPIO105 = 1;     // Liga fonte de controle.
        DELAY_US(2000000);
        GpioDataRegs.GPDSET.bit.GPIO104 = 1;    //Liga fonte de potÊncia.


        while(1){if(SPWM_State){break;} }       // Espera até que SPWM_State seja diferente de zero
                                               // Para liberar o PWM. Isso é feito alterando a variável
                                               // SPWM_Satte em tempo real através da aba de exprssões do DSP.


        // Enable CPU INT3 which is connected to EPWM1-3 INT:
        IER |= M_INT1 | M_INT3 | M_INT5;
        PieCtrlRegs.PIEIER3.bit.INTx4 = 1;
        PieCtrlRegs.PIEIER3.bit.INTx5 = 1;
        PieCtrlRegs.PIEIER3.bit.INTx6 = 1;

        PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
        PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

        PieCtrlRegs.PIEIER5.bit.INTx1 = 1;
        // Enable global Interrupts and higher priority real-time debug events:
        EINT;  // Enable Global interrupt INTM
        ERTM;  // Enable Global realtime interrupt DBGM


        Setup_GPIO();                         //Libera PWM.
        IER |= M_INT1;

        aux++;
        EDIS;
  }


}

void Desliga_Bancada(void)
{

    EALLOW;

    GpioDataRegs.GPDCLEAR.bit.GPIO104 = 1;  // Desliga fonte de potÊncia

    DELAY_US(2000000);

    GpioDataRegs.GPDCLEAR.bit.GPIO105 = 1; // Desliga fonte de controle

    aux = 0;

    EDIS;
}


void Stop_SPWM(void){

    EALLOW;

    IER &= M_INT1;                          //Desabilita PWM assocaido a interrupção do ADC.
    GpioCtrlRegs.GPAMUX1.all = 0;           // GPIO = GPIO
    GpioCtrlRegs.GPADIR.all = 0x00000FC0;
    GpioDataRegs.GPACLEAR.all = 0x00000FC0; // LOW (GPIO 6, 7, 8, 9, 10 e 11)
                                           // Abre todas as chaves.

    EDIS;
}



void Setup_ePWM(void){

    EALLOW;                    //###### LEMBRAR DE AJUSTAR O FED E RED PARA OS PREESCALES DO TBCLOCK.

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;           // DISABLE TBCLK for ePWM configuration.

//##########__EPWM4__###################################################################

    EPwm4Regs.TBPRD = TB_Prd;                       // Set timer period TBPR = (EPWMCLK ) / ( 2 x 2 x freq_pwm) for up-down count mode.
    EPwm4Regs.CMPA.bit.CMPA = 0;                    // Clear CMPA
    EPwm4Regs.TBPHS.bit.TBPHS = 0;                  // Regsitrador de fase zerado.
    EPwm4Regs.TBCTL.bit.SYNCOSEL =TB_CTR_ZERO  ;    // Sincroniza as fases em TBPRD = 0.
    EPwm4Regs.TBCTR = 0x0000;                       // Limpa o contador.
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Configura a portadora para o modo simétrico(up-down).
    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Desabilita o carregamento das fases.

    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        //TBCLK = EPWMCLK/(HSPCLKDIV*CLKDIV)
    if(TB_Prescale == 1)         EPwm4Regs.TBCTL.bit.CLKDIV = 0;      // EPWMCLK/1.
    else if (TB_Prescale == 128) EPwm4Regs.TBCTL.bit.CLKDIV = 7;      // EPWMCLK/128.

    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;       //Habilita o shadow como um buffer duplo.
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD; /* Carrega os registradores nos eventos TBCTR = ZERO  e TBCTR = PRD.
                                                         Necessário pois a senoide apresenta valores diferentes em CAU e CAD.
                                                      */

    // Configuração do Action Qualifier para geração do SPWM.
    EPwm4Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm4Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // Active Low complementary to use EPWM4A complementary to EMPW4B.
    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // Habilita o módulo de Dead-Band.
    EPwm4Regs.DBFED.bit.DBFED = 400;                // FED = 350 TBCLKs (4.0 uS)
    EPwm4Regs.DBRED.bit.DBRED = 400;                // RED = 350 TBCLKs (4.0 uS)


    //##########__EPWM5__##########################################################################

    EPwm5Regs.TBPRD = TB_Prd;
    EPwm5Regs.CMPA.bit.CMPA = 0;
    EPwm5Regs.TBPHS.bit.TBPHS = 0;
    EPwm5Regs.TBCTL.bit.SYNCOSEL =TB_CTR_ZERO ;
    EPwm5Regs.TBCTR = 0x0000;
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;

    if (TB_Prescale == 1)         EPwm5Regs.TBCTL.bit.CLKDIV = 0;      // EPWMCLK/1
    else if (TB_Prescale == 128) EPwm5Regs.TBCTL.bit.CLKDIV = 7;       // EPWMCLK/128

    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;


    EPwm5Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm5Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm5Regs.DBFED.bit.DBFED = 400;
    EPwm5Regs.DBRED.bit.DBRED =400;

//##########__EPWM6__##########################################################################

    EPwm6Regs.TBPRD = TB_Prd;
    EPwm6Regs.CMPA.bit.CMPA = 0;
    EPwm6Regs.TBPHS.bit.TBPHS = 0;
    EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
    EPwm6Regs.TBCTR = 0x0000;
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;

    if(TB_Prescale == 1)        EPwm6Regs.TBCTL.bit.CLKDIV = 0;
    else if (TB_Prescale == 128)EPwm6Regs.TBCTL.bit.CLKDIV = 7;

    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    EPwm6Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm6Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm6Regs.DBFED.bit.DBFED = 400;
    EPwm6Regs.DBRED.bit.DBRED = 400;

//##########__EPWM1__##########################################################################

        // PWM sendo utilizado como TIMER , gerando SOC a 12 Khz.
       // Frequencia de amostragem deve ser um multiplo de 60.

     EPwm1Regs.TBPRD = 4165 ;  // Sampling at 12 Khz / TBPRD = TPWM/TBCLK - 1 -> FAZER O CALCULO COM A FREQ !!
     EPwm1Regs.TBPHS.bit.TBPHS = 0;
     EPwm1Regs.TBCTR = 0X0000;
     EPwm1Regs.TBCTL.bit.CTRMODE= TB_COUNT_UP; // Configura como up count mode.

     // Condigurações padrões para gerar TBCLK = 50 Mhz  ( EPWMCLK = SYSCLKOUT/2 = 100 Mhz , TBCLK = EPWMCLK/(HSPCLKDIV * CLKDIV) );
     EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;
     EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1 ;

     EPwm1Regs.ETSEL.bit.SOCAEN =1;  // Enable SOC on A group
     EPwm1Regs.ETSEL.bit.SOCASEL=ET_CTR_PRD ;  // Dispara o SOC em CTR = PRD.
     EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;     // Dispara o SOC no primeiro evento.

     CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;                // ENABLE TBCLKs

       EDIS;

}

void Setup_ADC(){

        Uint16 acqps;

        // Configurações mínimas, consultar datasheet pag 105.

        if( ADC_RESOLUTION_12BIT  == AdcaRegs.ADCCTL2.bit.RESOLUTION)
            acqps = 14;
        else
            acqps = 63;

        EALLOW;

      //##################################################################### ADC A ################################################

        // POWER UP SEQUENCE
        CpuSysRegs.PCLKCR13.bit.ADC_A =1;   // Habilita o clock do módulo A do ADC.
        AdcaRegs.ADCCTL2.bit.PRESCALE = 6; // RECOMENDADO NO DATASHEET , ADCCLK = 50 Mhz


        AdcSetMode(ADC_ADCA,ADC_RESOLUTION_12BIT,ADC_SIGNALMODE_SINGLE);

        AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1; // Gera interrupção um ciclo de clock antes do EOC.
        AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;   // Energiza o ADC A .
        DELAY_US(1000);  // 1ms de delay para ligar o módulo do ADC.

        // SOC and INTERRUPT config
        AdcaRegs.ADCSOC0CTL.bit.CHSEL = 3; // ADCINA3 - PINO 26 (J3).
        AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // SOCA Epwm1.
        AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; // 64 SYSCLK cycles to charge the capacitor. Recomendado no Datasheet , pag 105.



        AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // EOC DISPARA o  ADCINT1;
        AdcaRegs.ADCINTSEL1N2.bit.INT1E =0;   // Desabilita interrupções do ADC;
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 =1; //Make sure the INT1 flag is cleared.

        //##################################################################### ADC B ################################################

        // POWER UP SEQUENCE
        CpuSysRegs.PCLKCR13.bit.ADC_B = 1;   // Habilita o clock do módulo B do ADC.
        AdcbRegs.ADCCTL2.bit.PRESCALE = 6;  // RECOMENDADO NO DATASHEET , ADCCLK = 50 Mhz

        AdcSetMode(ADC_ADCB,ADC_RESOLUTION_12BIT,ADC_SIGNALMODE_SINGLE);

        AdcbRegs.ADCCTL1.bit.INTPULSEPOS=1; // Gera interrupção um ciclo de clock antes do EOC.
        AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;  // Energiza o ADC B.
        DELAY_US(1000);  // 1ms de delay para ligar o módulo do ADC.

        // SOC config
        AdcbRegs.ADCSOC1CTL.bit.CHSEL =3 ; // ADCINB3 - PINO 25 (J3).
        AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 5; // SOCA Epwm1.
        AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps;


        //##################################################################### ADC C ################################################

        // POWER UP SEQUENCE
        CpuSysRegs.PCLKCR13.bit.ADC_C = 1;
        AdccRegs.ADCCTL2.bit.PRESCALE = 6;  // RECOMENDADO NO DATASHEET , ADCCLK = 50 Mhz

        AdcSetMode(ADC_ADCC,ADC_RESOLUTION_12BIT,ADC_SIGNALMODE_SINGLE);

        AdccRegs.ADCCTL1.bit.INTPULSEPOS=1; // Gera interrupção um ciclo de clock antes do EOC.
        AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;  // Energiza o ADC C.
        DELAY_US(1000);  // 1ms de delay para ligar o módulo do ADC.

        // SOC config
        AdccRegs.ADCSOC2CTL.bit.CHSEL =3 ; // ADCINC3 - PINO 24 (J3).
        AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 5; // SOCA Epwm1.
        AdccRegs.ADCSOC2CTL.bit.ACQPS = acqps;



        EDIS;

}


void Set_ePWM_Frequency(uint32_t freq_pwm){
    if(freq_pwm < 763){                      //Minimum Value of frequency without dividing the EPWMCLK . Thus is necessary to divide it if we want less freq.
        TB_Prd =  (0x17D784)/(2*2*freq_pwm); //0x17D784 = 200 MHz / 128.
        TB_Prescale = 128;                   //Control variable to choose witch value will divide EPWMCLK.
    }
    else{                                    // Not necessary to divide EPWMCLK.
        TB_Prd =  (0xBEBC200)/(2*2*freq_pwm);//0xBEBC200 = 200 Mhz = EPWMCLk/1;
        TB_Prescale = 1;                     // Divide EPWMCLK by 1.
    }
}


#include "S32K144.h" /* include peripheral declarations S32K144 */
//A lo largo de codigo los pines y puertos estan ordenados alfabeticamente y de menor a mayor, no por la función que cumplen.
//variables necesarias para el programa de la ventana, antipinch, movimiento puerta.
unsigned char SubiendoMotorVentana=0; //variable binaria indica lo de su nombre
unsigned char BajandoMotorVentana=0; //variable binaria indica lo de su nombre
unsigned long int dato_ADC = 0; //variable que indica el V en la entrada del ADC
unsigned long int dato_I=0; //variable que indica la I que pasa por el sensor de corriente
unsigned char contadorparaventanaAutomaticaU=0; //variable para contar el tiempo que se preciona subir ventana
unsigned char contadorparaventanaAutomaticaD=0; //variable para contar el tiempo que se preciona bajar ventana
unsigned char automaticaUP=0; //variable binaria indica lo de su nombre, movimiento automatico de la ventana
unsigned char automaticaDOWN=0; //variable binaria indica lo de su nombre, movimiento automatico de la ventana
unsigned char movComputadora=0; //variable binaria indica cuando la computadora del carro activa movimiento de la ventana
unsigned char seguro=0; //variable binaria indica lo de su nombre
unsigned char blutu=0;
unsigned int i;

void clock_can (void)
{
	//CLOCK CONFIG
		SCG->SOSCDIV=0x00000101;   			//SOSCDIV1 & SOSCDIV2 =1: divide by 1
		SCG->SOSCCFG=0x00000024;  			//Range=2: Medium freq (SOSC betw 1MHz-8MHz)
			                             	//HGO=0:   Config xtal osc for low power
											//EREFS=1: Input is external XTAL
		while(SCG->SOSCCSR & (1<<23)); /* Ensure SOSCCSR unlocked */
		SCG->SOSCCSR=0x00000001;
		while(!(SCG->SOSCCSR & (1<<24)));
}

void can_init (void)
{
	#define BUFFER_SIZE 4
	i=0;
	PCC->PCCn[PCC_FlexCAN0_INDEX] |= 1<<30;

	//SE DESHABILITA EL CAN PARA CAMBIAR EL CLOCK SOURCE
	CAN0->MCR|=(1<<31); 								//DESHABILITAMOS EL CAN PARA CAMBIAR CLK
	CAN0->CTRL1 &=~(1<<13);							//PONEMOS EL CLKSRC (CLOCK) PARA EL SOSDIV
	CAN0->MCR&=~(1<<31); 								//HABILITAMOS DE NUEVO EL CAN

	//CONFIGURAMOS EL TIME QUANTA

	while (!((CAN0->MCR & (1<<24)) >> 24))  {}			//CHECAMOS QUE ESTE EN FREEZE MODE
	CAN0->CTRL1|=(3<<19)+(3<<16)+(6<<0)+(3<<22);	//CONFIGURAMOS PARA 500KHz BIT TIME
		  	  	  	  	  	  	  	  	  	  	  	// TIME QUANTO =16 POR 8MHz/500KHz=16
		  	  	  	  	  	  	  	  	  	  		// PESG1 Y 2 SON 3 PORQUE VAN DEL 25%DEL TIME QUANTA-1
		  	  	  	  	  	  	  	  	  	  		//RWJ=3 AUN NO SE PORQUE BIEN
		  	  	  	  	  	  	  	  	  	  		//SMP=1 NO SE PONE NADA EN EL BIT}

	//CLEREAMOS NUESTROS BUFFERS
	for(i=0;i<128;i++){
		CAN0->RAMn[i]=0; 				// TENEMOS 128 BYTES PARA ESCRIBIR 4 BYTES DE MENSAGE*32 BUFFERS=128
	}
	 CAN0->MCR = 0x0000001F;       // INICIALIZAMOS LOS BUFFERS
	 CAN0->RAMn[4*BUFFER_SIZE]=0x0C000000;

	 	                   // Good practice: wait for FRZACK to clear (not in freeze mode)
	 while ((CAN0->MCR && (1<<24)) >> 24)  {}
	 	                   // Good practice: wait for NOTRDY to clear (module ready)
	  while ((CAN0->MCR && (1<<27)) >> 27)  {}
}

void can_transmit (unsigned int ACCION)
{
	PTD->PTOR=(1<<0);				// CADA QUE SE TRANSFIERA, TOGGLE DEL LED
	CAN0->IFLAG1|=(1<<0);			// The corresponding buffer has successfully completed transmission
	CAN0->RAMn[0*BUFFER_SIZE+2]=(ACCION<<0); 			// PAYLOAD 0
	CAN0->RAMn[0*BUFFER_SIZE+1]=(0x1FF<<18); 			// ID
	CAN0->RAMn[0*BUFFER_SIZE+0]=(12<<24)+(1<<22)+(4<<16); 	// CÓDIGO Tx = 0b1100,
															// SRR = 1
															// DLC = 0b0100 (4 BYTE)
}

void delay (void)
{
	unsigned long i;
	for(i=0; i<=5000000; i++);
}

void UART_init (void) {
	PCC->PCCn[PCC_LPUART0_INDEX] &= ~(1<<30);
	PCC->PCCn[PCC_LPUART0_INDEX] |= (3<<24); //activar reloj firc
	PCC->PCCn[PCC_LPUART0_INDEX] |= (1<<30); //activar reloj de interfaz

    LPUART0->BAUD=156; // pagina 1538
	LPUART0->CTRL |= (3<<18);//TEN=REN=1
	LPUART0->CTRL|=(1<<21);//hab interrupcion UART
}

void ADC_init (void)
{
	PCC->PCCn[PCC_ADC0_INDEX] |= (1<<30)+(3<<24);	//Enable clock ADC	//PCS=3: Select FIRCCDIV2
	ADC0->SC1[0] = 1<<6;	//Enable interruption
}

void LPT_init (void)
{
    PCC-> PCCn[PCC_LPTMR0_INDEX]=0x40000000; //habilitar reloj del LPT
    LPTMR0-> PSR=5;
    LPTMR0-> CMR=50-1; //tiempo de referencia -1
    LPTMR0-> CSR=(1<<6)+1; //Enable LPT y hab intr de LPT
}

void PORT_init (void) {
	PCC->PCCn[PCC_PORTA_INDEX]=0x40000000;  //Hab clk interfaz para PORTA
	PCC->PCCn[PCC_PORTB_INDEX]=0x40000000;  //Hab clk interfaz para PORTB
	PCC->PCCn[PCC_PORTC_INDEX]=0x40000000; //Hab clk interfaz para PORTC
    PCC->PCCn[PCC_PORTD_INDEX]=0x40000000; //Hab clk interfaz para PORTD
    PCC->PCCn[PCC_PORTE_INDEX]=0x40000000; //Hab clk interfaz para PORTE
    PORTA->PCR[0]=0x00000100+(12<<16); // input, activar seguro. Interrupción cuando nivel logico 1.
    PORTA->PCR[1]=0x00000000; //ADC 10 ADC0_SE1, su alternativa 0 es la que necesitamos. Input desde el sensor de corriente
    PORTA->PCR[7]=0x00000100+(12<<16); //input, desactivar seguro. Interrupción cuando nivel logico 1.
    //PORTA->PCR[4]=0x00000100; // Output Switch Mica
    PORTB->PCR[0]|=(2<<8); //Blutu RX
    PORTB->PCR[1]|=(2<<8); //Blutu TX
    PORTB->PCR[12]=0x00000100;	//GPIO Output Seguro Puerta
    PORTB->PCR[16]=0x00000100+(12<<16);//input, final de carrera (Ventana abajo). Interrupción cuando nivel logico 1.
    PORTC->PCR[1]=0x00000100+(12<<16); //input, final de carrera (puerta abierta). Interrupción cuando nivel logico 1.
    PORTC->PCR[2]=0x00000100+(12<<16); //input, señal del seguro de la puerta (puerta cerrada), por ahora es un final de carrera. Interrupción cuando nivel logico 1.
    //PORTC->PCR[7]=0x00000100+(12<<16); //input, final de carrera (ventana arriba). Interrupción cuando nivel logico 1.
    PORTC->PCR[15]=0x00000100+(12<<16); //input, SEÑAL ABRIR PUERTA. Interrupción cuando nivel logico 1.
    //PORTD->PCR[6]=0x00000100+(11<<16); //input, SEÑAL SUBIR VENTANA. Interrupción en ambos raising and falling edge.
    PORTD->PCR[4]=0x00000100+(12<<16);
    PORTD->PCR[7]=0x00000100+(11<<16); //input, SEÑAL BAJAR VENTANA. Interrupción en ambos raising and falling edge.
    PORTE->PCR[1]=0x00000100+(12<<16); //input, SEÑAL CERRAR PUERTA. Interrupción cuando nivel logico 1.
	PORTE->PCR[4]|=(5<<8); 					//PORT E4: MUX = ALT5, EXCEL -> CAN0_Rx
	PORTE->PCR[5]|=(5<<8); 					//PORT E5: MUX = ALT5, EXCEL -> CAN0_Tx
	PORTE->PCR[6]=0x00000100+(11<<16);
    //PORTE->PCR[2]=0x00000100; //output, DIRECCION ABRIENDO M.LINEAL
	PORTB->PCR[13]=0x00000100; //output, DIRECCION ABRIENDO M.LINEAL
    PORTE->PCR[14]=0x00000100; //output, DIRECCION CERRANDO M.LINEAL
    PORTE->PCR[15]=0x00000100; //output, DIRECCION SUBIENDO M.VENTANA
    PORTE->PCR[16]=0x00000100; //output, DIRECCION BAJANDO M.VENTANA

    //PTA->PDDR=(1<<4);	//Output Switch Mica
    PTB->PDDR=(1<<12)+(1<<13);	//Output Seguro Puerta
    PTE->PDDR=(1<<14)+(1<<15)+(1<<16); //SEÑALES OUTPUT ENABLE DE LOS MOTORES y comienzan en cero
    //Como los demas son señales los necesitamos como inputs, por reset todos son inputs.
}

void ADC0_IRQHandler (void)
{
	dato_ADC = 5000*(ADC0->R[0])/0xFF;	//Get channel's conversion results in mv. Leer resultados, apagar COCO.
	//dato_I=(1000*(dato_ADC-2618))/185; //para el de 5 A . Formula para calcular la corriente del sensor de corriente, según su hoja de datos
	dato_I=(1000*(dato_ADC-2500))/100; //para el de 20 A
}

void anti_pinch_puerta (void)
{
    if (dato_I>2000&&((PTE->PDOR&(1<<14))!=0)) //si la I es mayor a 2 A y el output de DIRECCION CERRANDO M.LINEAL es 1
    {
    	PTE->PCOR=(1<<14); // output 0 a DIRECCION CERRANDO M.LINEAL. Dejar de cerrar.
    	PTB->PSOR=(1<<13); // output 1 a DIRECCION ABRIENDO M.LINEAL. Comenzar a abrir.
    }
}

void anti_pinch_ventana (void)
{
	if (dato_I>3999&&SubiendoMotorVentana==1) { //si se esta subiendo la ventana y la corriente es mayor a 3.999 A
	PTE->PCOR=(1<<15); //output 0 a DIRECCION SUBIENDO M.VENTANA. Dejar de subir.
	automaticaUP=0; //indicar que el modo automatico se desactivo.
	PTE->PSOR=(1<<16); //output 1 a DIRECCION BAJANDO M.VENTANA. Comenzar a bajar.
	movComputadora=1; //activar movimiento indicado por el carro, el cual nadie lo puede interrumpir.
	}
}

void movimiento_automatico_ventana(void)
{
    //MOVIMIENTO AUTOMATICO DE VENTANA
    if (SubiendoMotorVentana==1&&BajandoMotorVentana==0) { //solamente si la ventaja esta subiendo
    	contadorparaventanaAutomaticaU++; //sumar 1 a la variable
    } else if ((SubiendoMotorVentana==0&&BajandoMotorVentana==0)||(SubiendoMotorVentana==1&&BajandoMotorVentana==1)) {//si no, si no se esta moviendo la ventana o se activaron las dos direcciónes
    	contadorparaventanaAutomaticaU=0; //reiniciar el conteo.
    }
    if (SubiendoMotorVentana==0&&BajandoMotorVentana==1) { //solamente si la ventaja esta bajando
            	contadorparaventanaAutomaticaD++; //sumar 1 a la variable
            } else if ((SubiendoMotorVentana==0&&BajandoMotorVentana==0)||(SubiendoMotorVentana==1&&BajandoMotorVentana==1)) {//si no, si no se esta moviendo la ventana o se activaron las dos direcciónes
            	contadorparaventanaAutomaticaD=0; //reiniciar el conteo.
            }
    if (contadorparaventanaAutomaticaU==40) //si el conteo llega a 6
    {
    	contadorparaventanaAutomaticaU=0;  //reiniciar el conteo.
    	automaticaUP=1; //activat movimiento automatico hacia arriba
    }
    if (contadorparaventanaAutomaticaD==40) //si el conteo llega a 6
    {
    	contadorparaventanaAutomaticaD=0;  //reiniciar el conteo.
    	automaticaDOWN=1; //activat movimiento automatico hacia abajo
    }
}

void LPTMR0_IRQHandler (void)
{
        LPTMR0->CSR|=(1<<7); //borrar la bandera
        ADC0->SC1[0]|= 1;	//Canal 1 PIN PTA 1 Y SE LLAMA ADC 10 en el quick guide
        //ADC0->SC1[0] |= 12;	//Canal 12 (001100b) de ADC EVB tiene potenciometro
        //anti_pinch_puerta();
        //anti_pinch_ventana();
        movimiento_automatico_ventana();
}


void activar_seguro (void)
{
	PTB->PCOR=(1<<13); // output 0 a DIRECCION ABRIENDO M.LINEAL. Dejar de abrir.
	PTE->PSOR=(1<<14); // output 1 a DIRECCION CERRANDO M.LINEAL. Comenzar a cerrar.
	seguro=1; //activar el seguro, bloquear la puerta, no permitirle abrirse al motor lineal
}

void desactivar_seguro (void)
{
	seguro=0; //quitar el seguro
}

void detener_bajar(void)
{
	PTE->PCOR=(1<<16); //output 0 a DIRECCION BAJANDO M.VENTANA. Dejar a bajar.
    automaticaDOWN=0; //desactivar el movimiento automatico (por si llego automaticamente)
    movComputadora=0; //desactivar el moComputadora ordenado por el carro (por si el carro lo activó)
    BajandoMotorVentana=0;
}

void detener_apertura(void)
{
	PTB->PCOR=(1<<13);  // output 0 a DIRECCION ABRIENDO M.LINEAL. Dejar de abrir.
}

void detener_cerradura(void)
{
	PTE->PCOR=(1<<14);  // output 0 a DIRECCION CERRANDO M.LINEAL. Dejar de cerrar.
}

void detener_subir(void)
{
    PTE->PCOR=(1<<15); //output 0 a DIRECCION SUBIENDO M.VENTANA. Dejar de subir.
    automaticaUP=0; //desactivar el movimiento automatico (por si llego automaticamente)
    movComputadora=0; //desactivar el moComputadora ordenado por el carro (por si el carro lo activó)
    SubiendoMotorVentana=0;
}

void abrir_puerta(void)
{
    if(((PTE->PDOR&(1<<15))==0)&&((PTE->PDOR&(1<<16))==0)){ //si no se esta moviendo el motor de la ventana
    	if (seguro) {} //si el seguro es 1 no hagas nada
    			else //si no es 1, abre la puerta
    			{
    				PTE->PCOR=(1<<14); // output 0 a DIRECCION CERRANDO M.LINEAL. Dejar de cerrar.
    				PTB->PSOR=(1<<12);	//Se libera el seguro de puerta
    				//delay();
    				PTB->PSOR=(1<<13); // output 1 a DIRECCION ABRIENDO M.LINEAL. Comenzar a abrir.
    				delay();
    				delay();
    				PTB->PCOR=(1<<12);	//Se apaga el seguro de puerta
    			}
    }
}

void subir_ventana(void)
{
	if (movComputadora) { //si el movCoputadora esta activado, no muevas los outputs
	  if(SubiendoMotorVentana) //de aqui
    	{
		  SubiendoMotorVentana=0;
    	} else {
    		SubiendoMotorVentana=1;
    	}
	  	if((PTE->PDIR&(1<<6))==1)
	  	{
	  		SubiendoMotorVentana=1;
	  	} //hasta aca, son unas condiciones necesarias para que no se causen bugs en el codigo
	}
	else if(((movComputadora==0)&&((PTB->PDOR&(1<<13))==0))&&((PTE->PDOR&(1<<14))==0)) { //si no hay movComputadora y no se esta moviendo el motor lineal ejecutar funcion de movimiento de ventanas
			if (SubiendoMotorVentana==1&&automaticaUP==0) //si el motor esta subiendo la ventana, y no hay automatico, dejar de subir
            	{
            	SubiendoMotorVentana=0; //"no subir con el motor"
            	PTE->PCOR=(1<<15); //output 0 a DIRECCION SUBIENDO M.VENTANA. Dejar de subir.
            	}
            else if (SubiendoMotorVentana==0&&BajandoMotorVentana==0) //si el motor de ventana no tiene movimiento, comenzar a subir
            	{
            	SubiendoMotorVentana=1; //"subir con el motor"
            	PTE->PSOR=(1<<15); //output 1 a DIRECCION SUBIENDO M.VENTANA. Comenzar a subir.
            	}
            else if (automaticaDOWN==1) //si esta activo el modo automatico de bajar, detenerlo
            	{
            	SubiendoMotorVentana=1; //"subir con el motor"
            	PTE->PCOR=(1<<16); //output 0 a DIRECCION BAJANDO M.VENTANA. Dejar de bajar.
            	automaticaDOWN=0; //desactivar modo automatico
            	if((PTD->PDIR&(1<<7))==0) //de aqui
            	{
    			BajandoMotorVentana=0;
            	} else {
            	BajandoMotorVentana=1;
            	} //hasta aca, son unas condiciones necesarias para que no se causen bugs en el codigo
            	}
            else if (SubiendoMotorVentana==0&&BajandoMotorVentana==1) //si la ventana esta bajando, detenerla
            	{
            	SubiendoMotorVentana=1; //"subir con el motor"
            	PTE->PCOR=(1<<16); //output 0 a DIRECCION BAJANDO M.VENTANA. Dejar de bajar.
            	}
	}
}

void bajar_ventana(void)
{
    if (movComputadora) { //si el movCoputadora esta activado, no muevas los outputs
    		  if(BajandoMotorVentana) //de aqui
            	{
    			  BajandoMotorVentana=0;
            	} else {
            		BajandoMotorVentana=1;
            	}
    		  	if((PTD->PDIR&(1<<7))==1)
    		  	{
    		  		BajandoMotorVentana=1;
    		  	} //hasta aca, son unas condiciones necesarias para que no se causen bugs en el codigo
    		}
    else if(((movComputadora==0)&&((PTB->PDOR&(1<<13))==0))&&((PTE->PDOR&(1<<14))==0)) { //si no hay movComputadora y no se esta moviendo el motor lineal ejecutar funcion de movimiento de ventanas
    if (BajandoMotorVentana==1&&automaticaDOWN==0) //si el motor esta bajando la ventana, y no hay automatico, dejar de bajar
    {
            BajandoMotorVentana=0;  //"no bajar con el motor"
            	PTE->PCOR=(1<<16); //output 0 en DIRECCION BAJANDO M.VENTANA. Dejar de bajar.
    }
            else if (BajandoMotorVentana==0&&SubiendoMotorVentana==0) //si el motor de ventana no tiene movimiento, comenzar a bajar
            {
            	BajandoMotorVentana=1; //"bajar con el motor"
            	PTE->PSOR=(1<<16); //output 1 en DIRECCION BAJANDO M.VENTANA. Comenzar a bajar.
            }
            else if (automaticaUP==1) //si esta activo el modo automatico de subir, detenerlo
            {
            	BajandoMotorVentana=1; //"bajar con el motor"
            	PTE->PCOR=(1<<15); //output 0 a DIRECCION SUBIENDO M.VENTANA. Dejar de subir.
            	automaticaUP=0; //desactivar modo automatico
             	if((PTE->PDIR&(1<<6))==0) //de aqui
            	{
            		SubiendoMotorVentana=0;
            	} else {
            		SubiendoMotorVentana=1;
            	} //hasta aca, son unas condiciones necesarias para que no se causen bugs en el codigo
            }
            else if (BajandoMotorVentana==0&&SubiendoMotorVentana==1) //si la ventana esta subiendo, detenerla
                {
            	BajandoMotorVentana=1;  //"bajar con el motor"
            	PTE->PCOR=(1<<15);  //output 0 a DIRECCION SUBIENDO M.VENTANA. Dejar de subir.
                }
    }
}

void cerrar_puerta(void)
{
    if(((PTE->PDOR&(1<<15))==0)&&((PTE->PDOR&(1<<16))==0))
    { //si no se esta moviendo el motor de la ventana
    PTB->PCOR=(1<<13); // output 0 a DIRECCION CERRANDO M.LINEAL. Dejar de abrir.
	PTE->PSOR=(1<<14); // output 1 a DIRECCION ABRIENDO M.LINEAL. Comenzar a cerrar.
    }
}

void LPUART0_RxTx_IRQHandler(void)
{
		blutu = (LPUART0 -> DATA);
    	if (blutu=='1')
    	{
    		subir_ventana();
    		blutu=0;
    	}
    	else if (blutu=='2')
    	{
    		bajar_ventana();
    		blutu=0;
    	}
    	else if (blutu=='3')
    	{
    		desactivar_seguro();
    		blutu=0;
    	}
    	else if (blutu=='4')
    	{
    		activar_seguro();
    		blutu=0;
    	}
    	else if (blutu=='5')
    	{
    		abrir_puerta();
    		blutu=0;
    	}
    	else if (blutu=='6')
    	{
    		cerrar_puerta();
    		blutu=0;
    	}
    	else if(blutu=='7')					//ABRIR PUERTA
    	{
    		can_transmit(0);
    	}
    	else if (blutu=='8')			//CERRAR PUERTA
    	{
    		can_transmit(1);
    	}
    	else if (blutu=='9')			//SUBIR VENTANA
    	{
    		can_transmit(2);
    	}
    	else if (blutu=='A')			//BAJAR VENTANA
    	{
    		can_transmit(3);
    	}
    	/*else if (blutu=='B')			//MICA OPACA
		{
			PTA->PCOR=(1<<4);
		}
		else if (blutu=='C')			//MICA TRANSPARENTE
		{
			PTA->PSOR=(1<<4);
		}*/
}

void PORTA_IRQHandler (void)
{
	if ((PORTA->PCR[0]&(1<<24))==(1<<24)){ //entra la interrupcion cuando se presiona el boton de activar seguro
		PORTA->PCR[0]|=1<<24; //apagar la bandera
		activar_seguro();
	}

	if ((PORTA->PCR[7]&(1<<24))==(1<<24)){ //entra la interrupcion cuando se presiona el boton de desactivar seguro
		PORTA->PCR[7]|=1<<24; //apagar la bandera
		desactivar_seguro ();
	}
}

void PORTB_IRQHandler (void)
{
	if ((PORTB->PCR[16]&(1<<24))==(1<<24)){ //entra la interrupcion cuando se baja completamente la ventana
		PORTB->PCR[16]|=1<<24; //apagar bandera
		detener_bajar();
	}

}

void PORTC_IRQHandler (void)
{
	if ((PORTC->PCR[1]&(1<<24))==(1<<24)) //entra la interrupcion cuando la puerta se abre totalmente
    {
        PORTC->PCR[1]|=1<<24;  //apagar bandera
        detener_apertura();
    }

	if ((PORTC->PCR[2]&(1<<24))==(1<<24)) //entra la interrupcion cuando la puerta se cierra totalmente
    {
        PORTC->PCR[2]|=1<<24;  //apagar bandera
        detener_cerradura();
    }



	if ((PORTC->PCR[15]&(1<<24))==(1<<24)) //entra la interrupcion cuando se presiona el boton para abrir la puerta
    {
        PORTC->PCR[15]|=1<<24;  //apagar bandera
        abrir_puerta();
    }
}

void PORTD_IRQHandler (void)
{

	if ((PORTD->PCR[4]&(1<<24))==(1<<24)) //entra la interrupcion cuando se sube completamente la ventana
    {
        PORTD->PCR[4]|=1<<24;  //apagar bandera
        detener_subir();
    }

    if ((PORTD->PCR[7]&(1<<24))==(1<<24)) //entra la interrupcion cuando se presiona y se suelta el boton de bajar ventana
        {
            PORTD->PCR[7]|=1<<24;  //apagar bandera
            bajar_ventana();
        }
}

void PORTE_IRQHandler (void)
{
    if ((PORTE->PCR[1]&(1<<24))==(1<<24)) //entra la interrupcion cuando se presiona el boton de cerrar puerta
    {
        PORTE->PCR[1]|=1<<24;  //apagar bandera
        cerrar_puerta();
    }

    if ((PORTE->PCR[6]&(1<<24))==(1<<24)) //entra la interrupcion cuando se presiona y se suelta el boton de subir ventana
    {
        PORTE->PCR[6]|=1<<24;  //apagar bandera
        subir_ventana();
    }
}

int main(void)
{
		SCG->FIRCDIV=1<<9;
		SCG->FIRCCFG=0;
		SCG->FIRCCSR = 1;
		//SCG->FIRCDIV=0x00000100;

		PORT_init();
		UART_init();
		clock_can();
		can_init();
		ADC_init();
		LPT_init();

        S32_NVIC->ISER[0]|= (1<<(31%32)); //Activar la interrupción de uart
    	S32_NVIC->ISER[1]|=(1<<(39%32)); //Hab interrupcion NVIC ADC0
        S32_NVIC->ISER[1]|=(1<<(58%32)); //lpt
        S32_NVIC->ISER[1]|=(1<<(59%32)); //port a
        S32_NVIC->ISER[1]|=(1<<(60%32)); //port b
        S32_NVIC->ISER[1]|=(1<<(61%32)); //port cb
        S32_NVIC->ISER[1]|=(1<<(62%32)); //port d
        S32_NVIC->ISER[1]|=(1<<(63%32)); //port e

        for(;;)
        {
        }
return 0;
}

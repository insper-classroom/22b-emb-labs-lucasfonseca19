/************************************************************************
* 5 semestre - Eng. da Computao - Insper
* Rafael Corsi - rafael.corsi@insper.edu.br
*
* Material:
*  - Kit: ATMEL SAME70-XPLD - ARM CORTEX M7
*
* Objetivo:
*  - Demonstrar configuraçao do PIO
*
* Periféricos:
*  - PIO
*  - PMC
*
* Log:
*  - 10/2018: Criação
************************************************************************/

/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include "asf.h"

/* -------------------------------------------------------------------------- */
/*                                   DEFINE                                   */
/* -------------------------------------------------------------------------- */
/* ---------------------------- LEDs Pin Numbers ---------------------------- */
// PINO NO OLED           PIO NO SAME70
// LED1 -> 7 			  PIO_PA0
// LED2 -> 8			  PIO_PC30
// LED3 -> 6			  PIO_PB2
/* ---------------------------- Bttms Pin Numbers --------------------------- */
// Bttm1 -> 9 			  PIO_PD28
// Bttm2 -> 3             PIO_PC31
// Bttm3 -> 4             PIO_PA19


// LEDs
#define LED_PIO1      PIOA
#define LED_PIO_ID1  ID_PIOA
#define LED_IDX1      0
#define LED_IDX_MASK1 (1 << LED_IDX1)

#define LED_PIO2      PIOC
#define LED_PIO_ID2   ID_PIOC
#define LED_IDX2      30
#define LED_IDX_MASK2 (1 << LED_IDX2)

#define LED_PIO3      PIOB
#define LED_PIO_ID3   ID_PIOB
#define LED_IDX3      2
#define LED_IDX_MASK3 (1 << LED_IDX3)


// Botões
#define BUT_PIO1      PIOD
#define BUT_PIO_ID1   ID_PIOD
#define BUT_IDX1      28
#define BUT_IDX_MASK1 (1 << BUT_IDX1)

#define BUT_PIO2      PIOC
#define BUT_PIO_ID2   ID_PIOC
#define BUT_IDX2      31
#define BUT_IDX_MASK2 (1 << BUT_IDX2)

#define BUT_PIO3      PIOA
#define BUT_PIO_ID3   ID_PIOA
#define BUT_IDX3      19
#define BUT_IDX_MASK3 (1 << BUT_IDX3)

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/


/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

/************************************************************************/
/* funções                                                              */
/************************************************************************/

/************************************************************************/
/* Main                                                                 */
/************************************************************************/
/* Funcao principal chamada na inicalizacao do uC.                      */
int main(void)
{
	// Inicializa clock
	sysclk_init();

	// Desativa watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;

	// Ativa PIOs
	pmc_enable_periph_clk(LED_PIO_ID1);
	pmc_enable_periph_clk(BUT_PIO_ID1);
	pmc_enable_periph_clk(LED_PIO_ID2);
	pmc_enable_periph_clk(BUT_PIO_ID2);
    pmc_enable_periph_clk(LED_PIO_ID3);
	pmc_enable_periph_clk(BUT_PIO_ID3);

	// Configura Pinos
	//Inicializa PC8 como saída
	pio_set_output(LED_PIO1, LED_PIO_IDX_MASK1, 0, 0, 0);
	

	// SUPER LOOP
	// aplicacoes embarcadas no devem sair do while(1).
	while(1) {
		// Verifica valor do pino que o botão está conectado
		if(!pio_get(1, PIO_INPUT, BUT_IDX_MASK1)) {
			// Pisca LED
			for (int i=0; i<10; i++) {
				pio_clear(LED_PIO1, LED_IDX_MASK1);  // Limpa o pino LED_PIO_PIN
				delay_ms(100);                         // delay
				pio_set(LED_PIO1, LED_IDX_MASK1);    // Ativa o pino LED_PIO_PIN
				delay_ms(100);                         // delay
			}
			} else  {
			// Ativa o pino LED_IDX (par apagar)
			pio_set(LED_PIO1, LED_IDX_MASK1);
		}
	}
	// Nunca devemos chegar aqui !
	return 0;
}
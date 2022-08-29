/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

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
#define LED_PIO1 PIOA
#define LED_PIO_ID1 ID_PIOA
#define LED_IDX1 0
#define LED_IDX_MASK1 (1 << LED_IDX1)

#define LED_PIO2 PIOC
#define LED_PIO_ID2 ID_PIOC
#define LED_IDX2 30
#define LED_IDX_MASK2 (1 << LED_IDX2)

#define LED_PIO3 PIOB
#define LED_PIO_ID3 ID_PIOB
#define LED_IDX3 2
#define LED_IDX_MASK3 (1 << LED_IDX3)

// Botões
#define BUT_PIO1 PIOD
#define BUT_PIO_ID1 ID_PIOD
#define BUT_IDX1 28
#define BUT_IDX_MASK1 (1 << BUT_IDX1)

#define BUT_PIO2 PIOC
#define BUT_PIO_ID2 ID_PIOC
#define BUT_IDX2 31
#define BUT_IDX_MASK2 (1 << BUT_IDX2)

#define BUT_PIO3 PIOA
#define BUT_PIO_ID3 ID_PIOA
#define BUT_IDX3 19
#define BUT_IDX_MASK3 (1 << BUT_IDX3)

// Defaults usados no _pio_set_input(...)

/*  Default pin configuration (no attribute). */
#define _PIO_DEFAULT (0u << 0)
/*  The internal pin pull-up is active. */
#define _PIO_PULLUP (1u << 0)
/*  The internal glitch filter is active. */
#define _PIO_DEGLITCH (1u << 1)
/*  The internal debouncing filter is active. */
#define _PIO_DEBOUNCE (1u << 3)

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

/*
 * \brief Set a high output level on all the PIOs defined in ul_mask.
 * This has no immediate effects on PIOs that are not output, but the PIO
 * controller will save the value if they are changed to outputs.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 */
void _pio_set(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_SODR = ul_mask;
}

/*
 * \brief Set a low output level on all the PIOs defined in ul_mask.
 * This has no immediate effects on PIOs that are not output, but the PIO
 * controller will save the value if they are changed to outputs.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 */
void _pio_clear(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_CODR = ul_mask;
}
/*
 * \brief Configure PIO internal pull-up.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 * \param ul_pull_up_enable Indicates if the pin(s) internal pull-up shall be
 * configured.
 */
void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_pull_up_enable)
{
	if (ul_pull_up_enable)
	{
		p_pio->PIO_PUER = ul_mask;
	}
	else
	{
		p_pio->PIO_PUDR = ul_mask;
	}
}
/*
 * \brief Configure one or more pin(s) or a PIO controller as inputs.
 * Optionally, the corresponding internal pull-up(s) and glitch filter(s) can
 * be enabled.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask indicating which pin(s) to configure as input(s).
 * \param ul_attribute PIO attribute(s).
 */
void _pio_set_input(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_attribute)
{
	_pio_pull_up
}

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
	// Saídas
	pio_set_output(LED_PIO1, LED_IDX_MASK1, 0, 0, 0);
	pio_set_output(LED_PIO2, LED_IDX_MASK2, 0, 0, 0);
	pio_set_output(LED_PIO3, LED_IDX_MASK3, 0, 0, 0);
	// Entradas
	pio_set_input(BUT_PIO1, BUT_IDX_MASK1, PIO_DEFAULT);
	pio_set_input(BUT_PIO2, BUT_IDX_MASK2, PIO_DEFAULT);
	pio_set_input(BUT_PIO3, BUT_IDX_MASK3, PIO_DEFAULT);
	_pio_pull_up(BUT_PIO1, BUT_IDX_MASK1, 1);
	_pio_pull_up(BUT_PIO2, BUT_IDX_MASK2, 1);
	_pio_pull_up(BUT_PIO3, BUT_IDX_MASK3, 1);
	// SUPER LOOP
	// aplicacoes embarcadas no devem sair do while(1).
	while (1)
	{
		if (!pio_get(BUT_PIO1, PIO_INPUT, BUT_IDX_MASK1))
		{
			for (int i = 0; i < 10; i++)
			{
				_pio_clear(LED_PIO1, LED_IDX_MASK1); // Limpa o pino LED_PIO_PIN
				delay_ms(100);						 // delay
				_pio_set(LED_PIO1, LED_IDX_MASK1);	 // Ativa o pino LED_PIO_PIN
				delay_ms(100);						 // delay
			}
		}
		if (!pio_get(BUT_PIO2, PIO_INPUT, BUT_IDX_MASK2))
		{
			for (int i = 0; i < 10; i++)
			{
				_pio_clear(LED_PIO2, LED_IDX_MASK2); // Limpa o pino LED_PIO_PIN
				delay_ms(100);						 // delay
				_pio_set(LED_PIO2, LED_IDX_MASK2);	 // Ativa o pino LED_PIO_PIN
				delay_ms(100);						 // delay
			}
		}
		if (!pio_get(BUT_PIO3, PIO_INPUT, BUT_IDX_MASK3))
		{
			for (int i = 0; i < 10; i++)
			{
				_pio_clear(LED_PIO3, LED_IDX_MASK3); // Limpa o pino LED_PIO_PIN
				delay_ms(100);						 // delay
				_pio_set(LED_PIO3, LED_IDX_MASK3);	 // Ativa o pino LED_PIO_PIN
				delay_ms(100);						 // delay
			}
		}
		else
		{
			_pio_set(LED_PIO1, LED_IDX_MASK1);
			_pio_set(LED_PIO2, LED_IDX_MASK2);
			_pio_set(LED_PIO3, LED_IDX_MASK3);
		}
	}

	// Nunca devemos chegar aqui !
	return 0;
}
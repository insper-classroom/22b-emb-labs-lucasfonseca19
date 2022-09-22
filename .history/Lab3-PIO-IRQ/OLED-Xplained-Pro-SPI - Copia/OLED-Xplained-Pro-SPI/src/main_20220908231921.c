/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */
#include <asf.h>
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"
/* -------------------------------------------------------------------------- */
/*                                   Defines                                  */
/* -------------------------------------------------------------------------- */

// LED
#define LED_PIO PIOC
#define LED_PIO_ID ID_PIOC
#define LED_IDX 8
#define LED_IDX_MASK (1 << LED_IDX)
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

/* -------------------------------------------------------------------------- */
/*                              Variávei Globais                              */
/* -------------------------------------------------------------------------- */
// Flag de interrupção
volatile char but_flag;
char long_press_flag = 0;
int delay = 100;
char str[128];
/* -------------------------------------------------------------------------- */
/*                                  Prototype                                 */
/* -------------------------------------------------------------------------- */
void io_init(void);
void pisca_led(int n, int t);

/* -------------------------------------------------------------------------- */
/*                            Handlers e Callbacks                            */
/* -------------------------------------------------------------------------- */
void but_callback(void)
{
	if (pio_get(BUT_PIO1, PIO_INPUT, BUT_IDX_MASK1))
	{
		but_flag = 0; // subida
	}
	else
	{
		but_flag = 1; // descida
	}
}

/* -------------------------------------------------------------------------- */
/*                                   Funções                                  */
/* -------------------------------------------------------------------------- */
void pisca_led(int n, int t)
{
	for (int i = 0; i < n; i++)
	{
		pio_clear(LED_PIO, LED_IDX_MASK);
		delay_ms(t);
		pio_set(LED_PIO, LED_IDX_MASK);
		delay_ms(t);
	}
}

// Inicializa botao SW0 do kit com interrupcao
void io_init(void)
{

	// Configura led
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);

	// Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT_PIO_ID1);

	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	pio_configure(BUT_PIO1, PIO_INPUT, BUT_IDX_MASK1, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO1, BUT_IDX_MASK1, 60);

	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUT_PIO1,
					BUT_PIO_ID1,
					BUT_IDX_MASK1,
					PIO_IT_EDGE,
					but_callback);

	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT_PIO1, BUT_IDX_MASK1);
	pio_get_interrupt_status(BUT_PIO1);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT_PIO_ID1);
	NVIC_SetPriority(BUT_PIO_ID1, 4); // Prioridade 4
}

int main(void)
{
	board_init();
	sysclk_init();
	delay_init();
	io_init();

	// Init OLED
	gfx_mono_ssd1306_init();
	gfx_mono_draw_string("Delay:", 0, 16, &sysfont);
	/* Insert application code here, after the board has been initialized. */
	while (1)
	{

		if (but_flag)
		{
			delay_ms(1000);
			if (but_flag)
			{
				delay += 100;
				but_flag
				sprintf(str, "%d", delay);
				gfx_mono_draw_string(str, 64, 16, &sysfont);
				long_press_flag = 1;
			}
			else
			{
				if (!long_press_flag)
				{
					delay -= 100;
					sprintf(str, "%d", delay);
					gfx_mono_draw_string(str, 64, 16, &sysfont);
					long_press_flag = 0;
				}
			}
		}
		long_press_flag = 0;
	}
}

#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* -------------------------------------------------------------------------- */
/*                                   Defines                                  */
/* -------------------------------------------------------------------------- */

/* --------------------------------- Botões --------------------------------- */
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



/* ---------------------------------- MAIN ---------------------------------- */
int main (void)
{
	board_init();
	sysclk_init();
	delay_init();

  // Init OLED
	gfx_mono_ssd1306_init();
  
  
	gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
  gfx_mono_draw_string("mundo", 50,16, &sysfont);
  
  

  /* Insert application code here, after the board has been initialized. */
	while(1) {



			// Escreve na tela um circulo e um texto
			
			for(int i=70;i<=120;i+=2){
				
				gfx_mono_draw_rect(i, 5, 2, 10, GFX_PIXEL_SET);
				delay_ms(10);
				
			}
			
			for(int i=120;i>=70;i-=2){
				
				gfx_mono_draw_rect(i, 5, 2, 10, GFX_PIXEL_CLR);
				delay_ms(10);
				
			}
			
			
	}
}

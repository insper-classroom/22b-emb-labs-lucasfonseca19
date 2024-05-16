#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* tmin e tmax */
#define tmin 58e-6
#define tmax 0.011764

#define USART_COM_ID ID_USART1
#define USART_COM USART1
/* Botao da placa */
#define BUT_PIO PIOD
#define BUT_PIO_ID ID_PIOD
#define BUT_PIO_PIN 28
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

/* ------------------------------ Pinos HC-SR04 ----------------------------- */

// Echo 	
#define ECHO_PIO PIOA
#define ECHO_PIO_ID ID_PIOA
#define ECHO_PIO_PIN 5
#define ECHO_PIO_PIN_MASK (1 << ECHO_PIO_PIN)

// Trig
#define TRIG_PIO PIOD
#define TRIG_PIO_ID ID_PIOD
#define TRIG_PIO_PIN 11
#define TRIG_PIO_PIN_MASK (1 << TRIG_PIO_PIN)

/** RTOS  */

/* -------------------------------------------------------------------------- */
/*                                 Prototypes                                 */
/* -------------------------------------------------------------------------- */
/* ----------------------------- Protótipo tasks ---------------------------- */

#define TASK_OLED_STACK_SIZE (1024 * 6 / sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY (tskIDLE_PRIORITY)

// task trigger
#define TASK_TRIGGER_STACK_SIZE (1024 * 6 / sizeof(portSTACK_TYPE))
#define TASK_TRIGGER_STACK_PRIORITY (tskIDLE_PRIORITY)

/* --------------------------- Queues e Semaphores -------------------------- */

SemaphoreHandle_t xSemaphore;

/* ------------------------ Protótipo Funções Padrão ------------------------ */

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/* ----------------------- Protótipos Callbacks/Funcs ----------------------- */

void callback_echo(void);
void callback_but(void);
void HC_SR04_init(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

// #region

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;)
	{
	}
}

extern void vApplicationIdleHook(void) {}

extern void vApplicationTickHook(void) {}

extern void vApplicationMallocFailedHook(void)
{
	configASSERT((volatile void *)NULL);
}
// #endregion

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void callback_echo(void)
{

	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
}
void RTT_Handler(void)
{

	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS)
	{
		// Printa que passou do tempo máximo
		printf("Tempo máximo ultrapassado");
	}
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters)
{
	/* Inicializa o OLED */
	gfx_mono_ssd1306_init();
	/* Inicializa o HC-SR04 */

	uint32_t tempo_rtt;

	for (;;)
	{
		if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
		{
			// verifica se é subida ou descida
			if (pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK))
			{
				// subida
				RTT_init(1 / (tmin * 2), 0, 0);
			}
			else
			{
				// descida
				tempo_rtt = rtt_read_timer_value(RTT);
				float tempo = tmin * 2 * tempo_rtt;
				float distancia = tempo * 340 / 2;
				// Printa a distancia no oled
				char str[20];
				sprintf(str, "Distancia: %.2f", distancia);
				gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
				gfx_mono_draw_string(str, 0, 0, &sysfont);
			}
		}
	}
}

static void task_trigger(void *pvParameters)
{

	for (;;)
	{
		if (!pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK))
		{
			// se o echo estiver em 0
			// envia um pulso de 10us

			pio_set(TRIG_PIO, TRIG_PIO_PIN_MASK);
			delay_us(10);
			pio_clear(TRIG_PIO, TRIG_PIO_PIN_MASK);
		}
	}

	/************************************************************************/
	/* funcoes                                                              */
	/************************************************************************/

	static void configure_console(void)
	{
		const usart_serial_options_t uart_serial_options = {
			.baudrate = CONF_UART_BAUDRATE,
			.charlength = CONF_UART_CHAR_LENGTH,
			.paritytype = CONF_UART_PARITY,
			.stopbits = CONF_UART_STOP_BITS,
		};

		/* Configure console UART. */
		stdio_serial_init(CONF_UART, &uart_serial_options);

		/* Specify that stdout should not be buffered. */
		setbuf(stdout, NULL);
	}

	static void HC_SR04_init(void)
	{
		pmc_enable_periph_clk(ECHO_PIO_ID);
		pmc_enable_periph_clk(TRIG_PIO_ID);

		// Echo (input)
		pio_configure(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK, PIO_DEFAULT);
		// Trig (output)
		pio_configure(TRIG_PIO, PIO_OUTPUT_0, TRIG_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

		pio_handler_set(ECHO_PIO, ECHO_PIO_ID, ECHO_PIO_PIN_MASK, PIO_IT_EDGE, callback_echo);

		pio_enable_interrupt(ECHO_PIO, ECHO_PIO_PIN_MASK);
		pio_get_interrupt_status(ECHO_PIO);

		NVIC_EnableIRQ(ECHO_PIO_ID);

		NVIC_SetPriority(ECHO_PIO_ID, 4);
	}

	static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource)
	{

		uint16_t pllPreScale = (int)(((float)32768) / freqPrescale);

		rtt_sel_source(RTT, false);
		rtt_init(RTT, pllPreScale);

		if (rttIRQSource & RTT_MR_ALMIEN)
		{
			uint32_t ul_previous_time;
			ul_previous_time = rtt_read_timer_value(RTT);
			while (ul_previous_time == rtt_read_timer_value(RTT))
				;
			rtt_write_alarm_time(RTT, IrqNPulses + ul_previous_time);
		}

		/* config NVIC */
		NVIC_DisableIRQ(RTT_IRQn);
		NVIC_ClearPendingIRQ(RTT_IRQn);
		NVIC_SetPriority(RTT_IRQn, 4);
		NVIC_EnableIRQ(RTT_IRQn);

		/* Enable RTT interrupt */
		if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
			rtt_enable_interrupt(RTT, rttIRQSource);
		else
			rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	}

	/************************************************************************/
	/* main                                                                 */
	/************************************************************************/

	int main(void)
	{
		/* Initialize the SAM system */
		sysclk_init();
		board_init();
		HC_SR04_init();
		/* Initialize the console uart */
		configure_console();
		xSemaphore = xSemaphoreCreateBinary();
		if (xSemaphore == NULL)
		{
			printf("Erro ao criar semaforo");
		}

		/* Create task to control oled */
		if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS)
		{
			printf("Failed to create oled task\r\n");
		}
		/* Create task to control trigger */
		if (xTaskCreate(task_trigger, "trigger", TASK_TRIGGER_STACK_SIZE, NULL, TASK_TRIGGER_STACK_PRIORITY, NULL) != pdPASS)
		{
			printf("Failed to create trigger task\r\n");
		}

		/* Start the scheduler. */
		vTaskStartScheduler();

		/* RTOS n�o deve chegar aqui !! */
		while (1)
		{
		}

		/* Will only get here if there was insufficient memory to create the idle task. */
		return 0;
	}

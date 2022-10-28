#include <asf.h>
#include "conf_board.h"
#include <math.h>
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"
#include "mcu6050.h"
#include "Fusion-1.0.6/Fusion/Fusion.h"
#include "math.h"

#define USART_COM_ID ID_USART1
#define USART_COM USART1
/* Botao da placa */
#define BUT_PIO PIOD
#define BUT_PIO_ID ID_PIOD
#define BUT_PIO_PIN 28
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

/* LED da placa */
#define LED_PIO PIOC
#define LED_PIO_ID ID_PIOC
#define LED_IDX 8
#define LED_IDX_MASK (1 << LED_IDX)

/* -------------------------------- Leds Oled ------------------------------- */
// LED 1 OLED
#define LED1_PIO PIOA
#define LED1_PIO_ID ID_PIOA
#define LED1_PIO_IDX 0
#define LED1_PIO_IDX_MASK (1 << LED1_PIO_IDX)

// LED 2 OLED
#define LED2_PIO PIOC
#define LED2_PIO_ID ID_PIOC
#define LED2_PIO_IDX 30
#define LED2_PIO_IDX_MASK (1 << LED2_PIO_IDX)

// LED 3 OLED
#define LED3_PIO PIOB
#define LED3_PIO_ID ID_PIOB
#define LED3_PIO_IDX 2
#define LED3_PIO_IDX_MASK (1 << LED3_PIO_IDX)

/** RTOS  */

/* -------------------------------------------------------------------------- */
/*                                 Prototypes                                 */
/* -------------------------------------------------------------------------- */
/* ----------------------------- Protótipo tasks ---------------------------- */

#define TASK_IMU_STACK_SIZE (1024 * 6 / sizeof(portSTACK_TYPE))
#define TASK_IMU_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_HOUSE_DOWN_STACK_SIZE (1024 * 6 / sizeof(portSTACK_TYPE))
#define TASK_HOUSE_DOWN_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_ORIENTACAO_STACK_SIZE (1024 * 6 / sizeof(portSTACK_TYPE))
#define TASK_ORIENTACAO_STACK_PRIORITY (tskIDLE_PRIORITY)

/* --------------------------- Queues e Semaphores -------------------------- */

SemaphoreHandle_t xSemaphore;
// Semáforo de queda
SemaphoreHandle_t xSemaphoreHouseDown;
// Fila da orientação
QueueHandle_t xQueueOrientacao;
enum orientacao
{
	ESQUERDA,
	FRENTE,
	DIREITA
};
/* ------------------------ Protótipo Funções Padrão ------------------------ */

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/* ----------------------- Protótipos Callbacks/Funcs ----------------------- */

int8_t mcu6050_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t mcu6050_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
void mcu6050_i2c_bus_init(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void pisca_led(Pio *pio, uint32_t mask, int nvezes, int delay);
void io_init(void);
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
/* TASKS                                                                */
/************************************************************************/

static void task_imu(void *pvParameters)
{
	mcu6050_i2c_bus_init();
	/* Inicializa Função de fusão */
	FusionAhrs ahrs;
	FusionAhrsInitialise(&ahrs);

	/*Variáveis*/
	int16_t raw_acc_x, raw_acc_y, raw_acc_z;
	volatile uint8_t raw_acc_xHigh, raw_acc_yHigh, raw_acc_zHigh;
	volatile uint8_t raw_acc_xLow, raw_acc_yLow, raw_acc_zLow;
	float proc_acc_x, proc_acc_y, proc_acc_z;

	int16_t raw_gyr_x, raw_gyr_y, raw_gyr_z;
	volatile uint8_t raw_gyr_xHigh, raw_gyr_yHigh, raw_gyr_zHigh;
	volatile uint8_t raw_gyr_xLow, raw_gyr_yLow, raw_gyr_zLow;
	float proc_gyr_x, proc_gyr_y, proc_gyr_z;

	float pitch, roll, yaw;
	float prev_yaw = 0;
	int orientacao;

	/* buffer para recebimento de dados */
	uint8_t bufferRX[10];
	uint8_t bufferTX[10];
	/* resultado da função */
	uint8_t rtn;

	rtn = twihs_probe(TWIHS2, MPU6050_DEFAULT_ADDRESS);
	if (rtn != TWIHS_SUCCESS)
	{
		printf("[ERRO] [i2c] [probe] \n");
	}
	else
	{
		printf("[DADO] [i2c] probe OK\n");
	}

	rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, bufferRX, 1);
	if (rtn != TWIHS_SUCCESS)
	{
		printf("[ERRO] [i2c] [read] \n");
	}
	else
	{
		printf("[DADO] [i2c] %x:%x \n", MPU6050_RA_WHO_AM_I, bufferRX[0]);
		// verifiar se o valor do buffer esta correto
		if (bufferRX[0] == 0x68)
		{
			printf("Sucesso na leitura do sensor");
		}
		else
		{
			printf("Falha na leitura do sensor");
		}
	}
	// Set Clock source
	bufferTX[0] = MPU6050_CLOCK_PLL_XGYRO;
	rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, bufferTX, 1);
	if (rtn != TWIHS_SUCCESS)
		printf("[ERRO] [i2c] [write] \n");

	// Aceletromtro em 2G
	bufferTX[0] = MPU6050_ACCEL_FS_2 << MPU6050_ACONFIG_AFS_SEL_BIT;
	rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, bufferTX, 1);
	if (rtn != TWIHS_SUCCESS)
		printf("[ERRO] [i2c] [write] \n");

	// Configura range giroscopio para operar com 250 °/s
	bufferTX[0] = 0x00; // 250 °/s
	rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, bufferTX, 1);
	if (rtn != TWIHS_SUCCESS)
		printf("[ERRO] [i2c] [write] \n");

	while (1)
	{
		// Le valor do acc X High e Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, &raw_acc_xHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_L, &raw_acc_xLow, 1);

		// Le valor do acc y High e  Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_YOUT_H, &raw_acc_yHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, &raw_acc_yLow, 1);

		// Le valor do acc z HIGH e Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H, &raw_acc_zHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, &raw_acc_zLow, 1);

		// Dados são do tipo complemento de dois
		raw_acc_x = (raw_acc_xHigh << 8) | (raw_acc_xLow << 0);
		raw_acc_y = (raw_acc_yHigh << 8) | (raw_acc_yLow << 0);
		raw_acc_z = (raw_acc_zHigh << 8) | (raw_acc_zLow << 0);

		// Le valor do gyr X High e Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_H, &raw_gyr_xHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_L, &raw_gyr_xLow, 1);

		// Le valor do gyr y High e  Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_YOUT_H, &raw_gyr_yHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, &raw_gyr_yLow, 1);

		// Le valor do gyr z HIGH e Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, &raw_gyr_zHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, &raw_gyr_zLow, 1);

		// Dados são do tipo complemento de dois
		raw_gyr_x = (raw_gyr_xHigh << 8) | (raw_gyr_xLow << 0);
		raw_gyr_y = (raw_gyr_yHigh << 8) | (raw_gyr_yLow << 0);
		raw_gyr_z = (raw_gyr_zHigh << 8) | (raw_gyr_zLow << 0);

		// Dados em escala real
		proc_acc_x = (float)raw_acc_x / 16384;
		proc_acc_y = (float)raw_acc_y / 16384;
		proc_acc_z = (float)raw_acc_z / 16384;

		proc_gyr_x = (float)raw_gyr_x / 131;
		proc_gyr_y = (float)raw_gyr_y / 131;
		proc_gyr_z = (float)raw_gyr_z / 131;

		// print proc_acc_ e proc_gyr_
		printf("acc_x: %f | acc_y: %f | acc_z: %f \n gyr_x: %f | gyr_y: %f | gyr_z: %f \n", proc_acc_x, proc_acc_y, proc_acc_z, proc_gyr_x, proc_gyr_y, proc_gyr_z);
		// uma amostra a cada 1ms
		// Free fall detection using accelerometer data
		float acc_total_vector = sqrt((proc_acc_x * proc_acc_x) + (proc_acc_y * proc_acc_y) + (proc_acc_z * proc_acc_z));
		printf("vector: %f \n", acc_total_vector);
		if (acc_total_vector < 0.35)
		{
			printf("CAIU");
			xSemaphoreGive(xSemaphoreHouseDown);
		}

		// Orientação da sensor

		const FusionVector gyroscope = {proc_gyr_x, proc_gyr_y, proc_gyr_z};
		const FusionVector accelerometer = {proc_acc_x, proc_acc_y, proc_acc_z};
		// Tempo entre amostras
		float dT = 0.1;

		// aplica o algoritmo
		FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, dT);

		// dados em pitch roll e yaw
		const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

		printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

		roll = euler.angle.roll;
		pitch = euler.angle.pitch;
		yaw = euler.angle.yaw;

		// TODO

		if ((yaw - sqrt(pow(prev_yaw, 2)) > 50) && pitch < 20)
		{
			orientacao = 0;
			xQueueSend(xQueueOrientacao, &orientacao, 0);
		}

		else if (roll < -40 && pitch <50)
		{
			orientacao = 1;
			xQueueSend(xQueueOrientacao, &orientacao, 0);
		}
		else if (yaw < -50 && pitch < 20)
		{
			orientacao = 2;
			xQueueSend(xQueueOrientacao, &orientacao, 0);
		}
		else
		{
			orientacao = 3;
			xQueueSend(xQueueOrientacao, &orientacao, 0);
		}
		vTaskDelay(10);
	}
}
static void task_house_down(void *pvParameters)
{
	while (1)
	{
		// Se o semáforo xSemaphoreHouseDown for liberado, piscamos o led
		if (xSemaphoreTake(xSemaphoreHouseDown, portMAX_DELAY) == pdTRUE)
		{
			// Piscamos o led
			pisca_led(LED_PIO, LED_IDX_MASK, 5, 100);
		}
	}
}
static void task_orientacao(void *pvParameters)
{
	gfx_mono_ssd1306_init();
	int orientacao;
	while (1)
	{
		// Lemos a queue de orientacao
		if (xQueueReceive(xQueueOrientacao, &orientacao, portMAX_DELAY) == pdTRUE)
		{
			// Se a queue for liberada, mostramos a orientação no display
			gfx_mono_draw_string("Orientacao:", 0, 0, &sysfont);
			if (orientacao == 0)
			{
				gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
				gfx_mono_draw_string("Esquerda", 0, 10, &sysfont);
				pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
				pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
				pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
			}
			else if (orientacao == 1)
			{
				gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
				gfx_mono_draw_string("Frente", 0, 10, &sysfont);
				pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
				pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
				pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
			}
			else if (orientacao == 2)
			{
				gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
				gfx_mono_draw_string("Direita", 0, 10, &sysfont);
				pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
				pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
				pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
			}
			else if (orientacao == 3)
			{
				gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
				pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
				pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
				pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
			}
		}
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
void mcu6050_i2c_bus_init(void)
{
	twihs_options_t mcu6050_option;
	pmc_enable_periph_clk(ID_TWIHS2);

	/* Configure the options of TWI driver */
	mcu6050_option.master_clk = sysclk_get_cpu_hz();
	mcu6050_option.speed = 40000;
	twihs_master_init(TWIHS2, &mcu6050_option);

	/* Enable the peripheral and set TWIHS mode. */
	/** Enable TWIHS port to control PIO pins */
	pmc_enable_periph_clk(ID_PIOD);
	pio_set_peripheral(PIOD, PIO_TYPE_PIO_PERIPH_C, 1 << 28);
	pio_set_peripheral(PIOD, PIO_TYPE_PIO_PERIPH_C, 1 << 27);
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
int8_t mcu6050_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	int32_t ierror = 0x00;

	twihs_packet_t p_packet;
	p_packet.chip = dev_addr;
	p_packet.addr[0] = reg_addr;
	p_packet.addr_length = 1;
	p_packet.buffer = reg_data;
	p_packet.length = cnt;

	ierror = twihs_master_write(TWIHS2, &p_packet);

	return (int8_t)ierror;
}

int8_t mcu6050_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	int32_t ierror = 0x00;

	twihs_packet_t p_packet;
	p_packet.chip = dev_addr;
	p_packet.addr[0] = reg_addr;
	p_packet.addr_length = 1;
	p_packet.buffer = reg_data;
	p_packet.length = cnt;

	// TODO: Algum problema no SPI faz com que devemos ler duas vezes o registrador para
	//       conseguirmos pegar o valor correto.
	ierror = twihs_master_read(TWIHS2, &p_packet);

	return (int8_t)ierror;
}
void pisca_led(Pio *pio, uint32_t mask, int nvezes, int delay)
{
	for (int i = 0; i < nvezes; i++)
	{
		pio_clear(pio, mask);
		delay_ms(delay);
		pio_set(pio, mask);
		delay_ms(delay);
	}
}
void io_init(void)
{
	/* Configura o pino do LED como saída */
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_set_output(LED_PIO, LED_IDX_MASK, 1, 0, 0);

	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 1, 0, 0);

	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 1, 0, 0);

	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 1, 0, 0);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void)

{
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	io_init();
	/* Initialize the console uart */
	configure_console();
	xSemaphore = xSemaphoreCreateBinary();
	xSemaphoreHouseDown = xSemaphoreCreateBinary();
	if (xSemaphore == NULL)
	{
		printf("Erro ao criar semaforo");
	}

	if (xTaskCreate(task_imu, "imu", TASK_IMU_STACK_SIZE, NULL, TASK_IMU_STACK_PRIORITY, NULL) != pdPASS)
	{
		printf("Failed to create imu task\r\n");
	}

	// task_house_down
	if (xTaskCreate(task_house_down, "house_down", TASK_HOUSE_DOWN_STACK_SIZE, NULL, TASK_HOUSE_DOWN_STACK_PRIORITY, NULL) != pdPASS)
	{
		printf("Failed to create house_down task\r\n");
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

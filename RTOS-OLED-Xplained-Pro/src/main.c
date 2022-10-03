#include <asf.h>
#include "conf_board.h"

//PROVA RODRIGO MATTAR

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

#define BUT_1_PIO PIOD
#define BUT_1_PIO_ID ID_PIOD
#define BUT_1_IDX 28
#define BUT_1_IDX_MASK (1u << BUT_1_IDX)

#define BUT_2_PIO PIOC
#define BUT_2_PIO_ID ID_PIOC
#define BUT_2_IDX 31
#define BUT_2_IDX_MASK (1u << BUT_2_IDX)

#define BUT_3_PIO PIOA
#define BUT_3_PIO_ID ID_PIOA
#define BUT_3_IDX 19
#define BUT_3_IDX_MASK (1u << BUT_3_IDX)

#define PIO_1		PIOD
#define PIO_1_ID		ID_PIOD
#define PIO_1_IDX	30
#define PIO_1_IDX_MASK (1u << PIO_1_IDX)

#define PIO_2		PIOA
#define PIO_2_ID		ID_PIOA
#define PIO_2_IDX	6
#define PIO_2_IDX_MASK (1u << PIO_2_IDX)

#define PIO_3		PIOC
#define PIO_3_ID		ID_PIOC
#define PIO_3_IDX	19
#define PIO_3_IDX_MASK (1u << PIO_3_IDX)

#define PIO_4		PIOA
#define PIO_4_ID		ID_PIOA
#define PIO_4_IDX	2
#define PIO_4_IDX_MASK (1u << PIO_4_IDX)

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
void but1_callback(void);
void but2_callback(void);
void but3_callback(void);
static void BUT_init(void);
void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

QueueHandle_t xQueueModo;

QueueHandle_t xQueueSteps;

SemaphoreHandle_t xSemaphoreRTT;

typedef struct {
	uint status;
	uint dir;
	uint vel;
} motorData;

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void RTT_Handler(void) {
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);
	
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		xSemaphoreGiveFromISR(xSemaphoreRTT, 0);
	}
}

void but_callback(void) {
	printf("Entrou 0\n");
}

void but1_callback(void){
	printf("Entrou 1\n");
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	int angle= 180;
	xQueueSendFromISR(xQueueModo, (void *)&angle,  &xHigherPriorityTaskWoken);
};
void but2_callback(void){
	printf("Entrou 2\n");
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	int angle= 90;
	xQueueSendFromISR(xQueueModo, (void *)&angle,  &xHigherPriorityTaskWoken);
};
void but3_callback(void){
	printf("Entrou 3\n");
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	int angle= 45;
	xQueueSendFromISR(xQueueModo, (void *)&angle,  &xHigherPriorityTaskWoken);
};

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_modo(void *pvParameters) {
	gfx_mono_ssd1306_init();
	//gfx_mono_draw_string("Exemplo RTOS", 0, 0, &sysfont);
	//gfx_mono_draw_string("oii", 0, 20, &sysfont);
	printf("foi\n");
	BUT_init();
	
	int angle;
	char str[128];
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	for (;;)  {
		if (xQueueReceive(xQueueModo, &angle, (TickType_t) 100)) {
			sprintf(str, "%d  ", angle);
			gfx_mono_draw_string(str, 0,0, &sysfont);
			int valorPassos = angle/0.17578125;
			xQueueSendFromISR(xQueueSteps, (void *)&valorPassos,  &xHigherPriorityTaskWoken);
		}
		

	}
}

static void task_motor(void *pvParameters) {
	int passos;
	int count =0;
	for (;;)  {
		if (xQueueReceive(xQueueSteps, &passos, (TickType_t) 100)) {
			printf("Recebeu Passo: %d\n", passos);
			RTT_init(200, 1, RTT_MR_RTTINCIEN);
		}
		if(passos>0){
			if(xSemaphoreTake(xSemaphoreRTT, 500 / portTICK_PERIOD_MS) == pdTRUE){
				pio_set(PIO_4, PIO_4_IDX_MASK); //D
				pio_clear(PIO_3, PIO_3_IDX_MASK); //C
				pio_clear(PIO_2, PIO_2_IDX_MASK); //B
				pio_clear(PIO_1, PIO_1_IDX_MASK); //A
				passos-=1;
			}
			
			if(xSemaphoreTake(xSemaphoreRTT, 500 / portTICK_PERIOD_MS) == pdTRUE){
				pio_clear(PIO_4, PIO_4_IDX_MASK); //D
				pio_set(PIO_3, PIO_3_IDX_MASK); //C
				pio_clear(PIO_2, PIO_2_IDX_MASK); //B
				pio_clear(PIO_1, PIO_1_IDX_MASK); //A
				passos-=1;
			}
			
			if(xSemaphoreTake(xSemaphoreRTT, 500 / portTICK_PERIOD_MS) == pdTRUE){
				pio_clear(PIO_4, PIO_4_IDX_MASK); //D
				pio_clear(PIO_3, PIO_3_IDX_MASK); //C
				pio_set(PIO_2, PIO_2_IDX_MASK); //B
				pio_clear(PIO_1, PIO_1_IDX_MASK); //A
				passos-=1;
			}
			if(xSemaphoreTake(xSemaphoreRTT, 500 / portTICK_PERIOD_MS) == pdTRUE){
				pio_clear(PIO_4, PIO_4_IDX_MASK); //D
				pio_clear(PIO_3, PIO_3_IDX_MASK); //C
				pio_clear(PIO_2, PIO_2_IDX_MASK); //B
				pio_set(PIO_1, PIO_1_IDX_MASK); //
				passos-=1;
			} ;
		}
		
		
		
		

	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
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

void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);
	
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
}

static void BUT_init(void) {
	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	pio_configure(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK, PIO_PULLUP);
	pio_configure(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK, PIO_PULLUP);
	pio_configure(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK, PIO_PULLUP);

	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUT_2_PIO,
	BUT_2_PIO_ID,
	BUT_2_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but2_callback);
	
	pio_handler_set(BUT_1_PIO,
	BUT_1_PIO_ID,
	BUT_1_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but1_callback);
	
	pio_handler_set(BUT_3_PIO,
	BUT_3_PIO_ID,
	BUT_3_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but3_callback);

	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT_1_PIO, BUT_1_IDX_MASK);
	pio_get_interrupt_status(BUT_1_PIO);
	
	pio_enable_interrupt(BUT_2_PIO, BUT_2_IDX_MASK);
	pio_get_interrupt_status(BUT_2_PIO);
	
	pio_enable_interrupt(BUT_3_PIO, BUT_3_IDX_MASK);
	pio_get_interrupt_status(BUT_3_PIO);
	
	pmc_enable_periph_clk(PIO_1_ID);
	pio_configure(PIO_1, PIO_OUTPUT_1, PIO_1_IDX_MASK, PIO_DEFAULT);
	
	pmc_enable_periph_clk(PIO_2_ID);
	pio_configure(PIO_2, PIO_OUTPUT_1, PIO_2_IDX_MASK, PIO_DEFAULT);
	
	pmc_enable_periph_clk(PIO_3_ID);
	pio_configure(PIO_3, PIO_OUTPUT_1, PIO_3_IDX_MASK, PIO_DEFAULT);
	
	pmc_enable_periph_clk(PIO_4_ID);
	pio_configure(PIO_4, PIO_OUTPUT_1, PIO_4_IDX_MASK, PIO_DEFAULT);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT_1_PIO_ID);
	NVIC_SetPriority(BUT_1_PIO_ID, 4); // Prioridade 4
	NVIC_EnableIRQ(BUT_2_PIO_ID);
	NVIC_SetPriority(BUT_2_PIO_ID, 4); // Prioridade 4
	NVIC_EnableIRQ(BUT_3_PIO_ID);
	NVIC_SetPriority(BUT_3_PIO_ID, 4);
	
	
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();
	
	xQueueModo = xQueueCreate(100, sizeof(motorData));
	if (xQueueModo == NULL)
	printf("falha em criar a queue xQueueADC \n");
	
	xQueueSteps = xQueueCreate(100, sizeof(motorData));
	if (xQueueSteps == NULL)
	printf("falha em criar a queue xQueueADC \n");
	
	xSemaphoreRTT = xSemaphoreCreateBinary();

	/* Create task to control oled */
	if (xTaskCreate(task_modo, "modo", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create modo task\r\n");
	}
	
	if (xTaskCreate(task_motor, "motor", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create motor task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}

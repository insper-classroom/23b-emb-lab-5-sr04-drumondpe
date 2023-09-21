#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

#define TRIGGER_PIO     PIOD     // Pino de Trigger (não tem debounce) Y
#define TRIGGER_PIO_ID  ID_PIOD
#define TRIGGER_PIO_PIN 30
#define TRIGGER_PIO_PIN_MASK (1 << TRIGGER_PIO_PIN)

#define ECHO_PIO     PIOA       // X
#define ECHO_PIO_ID  ID_PIOA
#define ECHO_PIO_PIN 6
#define ECHO_PIO_PIN_MASK (1 << ECHO_PIO_PIN)

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_ECHO_STACK_SIZE                (1024*2/sizeof(portSTACK_TYPE)) // Tamanho da pilha da tarefa Echo
#define TASK_ECHO_STACK_PRIORITY            (tskIDLE_PRIORITY + 1) // Prioridade da tarefa Echo


volatile bool echo_high = false;         // Variável para acompanhar o nível alto do pino Echo
volatile uint32_t echo_high_time_us = 0; // Variável para armazenar o tempo em que o pino Echo fica alto

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
static void BUT_init(void);
void echo_high_callback(const uint32_t id, const uint32_t index);


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

void but_callback(void) {
}

void echo_high_callback(const uint32_t id, const uint32_t index) {
    if (pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK)) {
        RTT_init();

    } else {
		printf("Tempo de ida e volta: %d us\n\r", RTT_get_time_us());
    }
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
	gfx_mono_draw_string("Exemplo RTOS", 0, 0, &sysfont);
	gfx_mono_draw_string("oii", 0, 20, &sysfont);

	for (;;)  {
		pio_set();
		delay_us(10);
		pio_clear();


		vTaskDelay(1000);
	}
}

static void task_echo(void *pvParameters) {

    for (;;) {
        vTaskDelay(1000);
    }
}

static void TRIGGER_init(){
	pmc_enable_periph_clk(TRIGGER_PIO_ID);
	pio_configure(TRIGGER_PIO, PIO_OUTPUT_0, TRIGGER_PIO_PIN_MASK, PIO_DEFAULT);

}

static void ECHO_init(){
	pio_configure(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK, PIO_DEFAULT);
	pio_set_debounce_filter(ECHO_PIO, ECHO_PIO_PIN_MASK, 60);
	pio_handler_set(ECHO_PIO, ECHO_PIO_ID, ECHO_PIO_PIN_MASK, PIO_IT_EDGE, echo_high_callback);
	pio_enable_interrupt(ECHO_PIO, ECHO_PIO_PIN_MASK);

	NVIC_EnableIRQ(ECHO_PIO_ID);
    NVIC_SetPriority(ECHO_PIO_ID, 4);
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

static void BUT_init(void) {
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

	/* conf bot�o como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
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

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}

	if (xTaskCreate(task_echo, "echo", TASK_ECHO_STACK_SIZE, NULL, TASK_ECHO_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create echo task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS n�o deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}

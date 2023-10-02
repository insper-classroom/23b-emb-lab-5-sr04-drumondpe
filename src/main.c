#include <asf.h>
#include "conf_board.h"
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

// Botão da placa
#define BUTTON_PIO            PIOA
#define BUTTON_PIO_ID         ID_PIOA
#define BUTTON_PIN            11
#define BUTTON_PIN_MASK       (1 << BUTTON_PIN)

// ECHO
#define ECHO_PIO              PIOA
#define ECHO_PIO_ID           ID_PIOA
#define ECHO_PIN              6
#define ECHO_PIN_MASK         (1 << ECHO_PIN)

// TRIGGER
#define TRIGGER_PIO           PIOD
#define TRIGGER_PIO_ID        ID_PIOD
#define TRIGGER_PIN           30
#define TRIGGER_PIN_MASK      (1 << TRIGGER_PIN)

// RTOS
#define OLED_TASK_STACK_SIZE          (1024 * 6 / sizeof(portSTACK_TYPE))
#define OLED_TASK_PRIORITY            (tskIDLE_PRIORITY)

// Prototypes
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

void button_callback(void);
void echo_callback(void);
static void BUTTON_init(void);
static void my_rtt_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
static void configure_console(void);

// Funções do RTOS
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
    printf("Stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
    for (;;) { }
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
    configASSERT((volatile void *)NULL);
}

// SEMAPHORES
SemaphoreHandle_t xSemaphore = NULL;

// QUEUES
QueueHandle_t xQueue = NULL;

// HANDLERS / CALLBACKS
void button_callback(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
}

void echo_callback(void) {
    if (pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIN_MASK) == 1) {
        my_rtt_init(8200, 8200, 0);
    } else {
        uint32_t ticks;
        ticks = rtt_read_timer_value(RTT);
        xQueueSendFromISR(xQueue, &ticks, 0);
    }
}

// TASKS
static void oled_task(void *pvParameters) {
    gfx_mono_ssd1306_init();
    float distance;
    uint32_t ticks;
    char str[32];

    for (;;) {
        if (xSemaphoreTake(xSemaphore, (TickType_t)500) == pdTRUE) {
            pio_set(TRIGGER_PIO, TRIGGER_PIN_MASK);
            delay_us(10);
            pio_clear(TRIGGER_PIO, TRIGGER_PIN_MASK);

            if (xQueueReceive(xQueue, &ticks, (TickType_t)500) == pdTRUE) {
                distance = (((float)ticks) * 340.0) / (2.0 * 10000.0); //8200
                distance = distance * 100;
                printf("Distance: %f cm\n", distance);
                sprintf(str, "%6.f", distance);
                gfx_mono_draw_string(str, 25, 12, &sysfont);
                gfx_mono_draw_string(" cm  ", 65, 12, &sysfont);
            }
        }
    }
}

// FUNÇÕES
static void configure_console(void) {
    const usart_serial_options_t uart_serial_options = {
        .baudrate = CONF_UART_BAUDRATE,
        .charlength = CONF_UART_CHAR_LENGTH,
        .paritytype = CONF_UART_PARITY,
        .stopbits = CONF_UART_STOP_BITS,
    };

    /* Configure console UART. */
    stdio_serial_init(CONF_UART, &uart_serial_options);
    setbuf(stdout, NULL);
}

static void BUTTON_init(void) {
    pmc_enable_periph_clk(BUTTON_PIO_ID);
    pio_configure(BUTTON_PIO, PIO_INPUT, BUTTON_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    pio_set_debounce_filter(BUTTON_PIO, BUTTON_PIN_MASK, 60);

    pio_handler_set(BUTTON_PIO, BUTTON_PIO_ID, BUTTON_PIN_MASK, PIO_IT_FALL_EDGE, button_callback);

    pio_enable_interrupt(BUTTON_PIO, BUTTON_PIN_MASK);
    pio_get_interrupt_status(BUTTON_PIO);

    NVIC_EnableIRQ(BUTTON_PIO_ID);
    NVIC_SetPriority(BUTTON_PIO_ID, 4);
}

static void ECHO_init(void) {
    pmc_enable_periph_clk(ECHO_PIO_ID);
    pio_configure(ECHO_PIO, PIO_INPUT, ECHO_PIN_MASK, PIO_DEFAULT);
    pio_handler_set(ECHO_PIO, ECHO_PIO_ID, ECHO_PIN_MASK, PIO_IT_EDGE, echo_callback);
    pio_enable_interrupt(ECHO_PIO, ECHO_PIN_MASK);
    pio_get_interrupt_status(ECHO_PIO);
    NVIC_EnableIRQ(ECHO_PIO_ID);
    NVIC_SetPriority(ECHO_PIO_ID, 4);
}

static void TRIGGER_init(void) {
    pmc_enable_periph_clk(TRIGGER_PIO_ID);
    pio_set_output(TRIGGER_PIO, TRIGGER_PIN_MASK, 0, 0, 0);
}

static void my_rtt_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {
    uint16_t pllPreScale = (int)(((float)32768) / freqPrescale);
    rtt_sel_source(RTT, false);
    rtt_init(RTT, pllPreScale);

    if (rttIRQSource & RTT_MR_ALMIEN) {
        uint32_t ul_previous_time;
        ul_previous_time = rtt_read_timer_value(RTT);
        while (ul_previous_time == rtt_read_timer_value(RTT));
        rtt_write_alarm_time(RTT, IrqNPulses + ul_previous_time);
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

// MAIN
int main(void) {
    sysclk_init();
    board_init();

    xSemaphore = xSemaphoreCreateBinary();
    xQueue = xQueueCreate(32, sizeof(uint32_t));

    if (xQueue == NULL) {
        printf("Failed to create queue\r\n");
    }

    if (xSemaphore == NULL) {
        printf("Failed to create semaphore\r\n");
    }

    BUTTON_init();
    ECHO_init();
    TRIGGER_init();
    configure_console();

    /* Create task to control oled */
    if (xTaskCreate(oled_task, "oled", OLED_TASK_STACK_SIZE, NULL, OLED_TASK_PRIORITY, NULL) != pdPASS) {
        printf("Failed to create oled task\r\n");
    }

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* RTOS não deve chegar aqui !! */
    while (1) {
    }

    /* Will only get here if there was insufficient memory to create the idle task. */
    return 0;
}

#include "board.h"
#include "board_internal.h"
#include "gpio.h"
#include "spi.h"
#include "usart.h"
#include "fdcan.h"
#include "main.h"

extern void SystemClock_Config(void);

/* -----------------------------------------------------------------------
 * EXTI dispatch table
 *
 * Fixed capacity, no dynamic allocation. If the table is full,
 * board_register_exti_callback() silently ignores the registration —
 * add an assert here to catch overflow during development.
 *
 * pin == 0 is the empty-slot sentinel. GPIO_PIN_0 is 0x0001 in the
 * STM32 HAL, so 0 never collides with a real pin value.
 * ----------------------------------------------------------------------- */
#define BOARD_EXTI_TABLE_SIZE 8U

typedef struct {
    uint16_t              pin;
    board_exti_callback_t callback;
} exti_entry_t;

static exti_entry_t s_exti_table[BOARD_EXTI_TABLE_SIZE] = {0};

void board_register_exti_callback(uint16_t gpio_pin, board_exti_callback_t callback) {
    for (uint8_t i = 0; i < BOARD_EXTI_TABLE_SIZE; i++) {
        if (s_exti_table[i].pin == gpio_pin) {
            s_exti_table[i].callback = callback;
            return;
        }
    }
    for (uint8_t i = 0; i < BOARD_EXTI_TABLE_SIZE; i++) {
        if (s_exti_table[i].pin == 0U) {
            s_exti_table[i].pin      = gpio_pin;
            s_exti_table[i].callback = callback;
            return;
        }
    }
}

/* RCC_CSR at start-up, before it is cleared for the next one */
static uint32_t s_reset_flags;

bool board_init(void) {
    s_reset_flags = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_LPUART1_UART_Init();
    MX_FDCAN1_Init();
    MX_FDCAN2_Init();
    return true;
}

/* LSI 32 kHz / 32 = 1 kHz: one count per millisecond (4095 max) */
#define WATCHDOG_PRESCALER_32 3U

void board_watchdog_start(void) {
    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP; /* frozen when halted */
    IWDG->KR = 0xCCCCU; /* start (also turns the LSI on) */
    IWDG->KR = 0x5555U; /* unlock PR and RLR */
    IWDG->PR = WATCHDOG_PRESCALER_32;
    IWDG->RLR = BOARD_WATCHDOG_MS - 1U;
    const uint32_t t0 = HAL_GetTick();
    while ((IWDG->SR != 0U) && ((HAL_GetTick() - t0) < 100U)) {
    }
    IWDG->KR = 0xAAAAU;
}

void board_watchdog_refresh(void) { IWDG->KR = 0xAAAAU; }

const char *board_reset_cause(void) {
    /* Several flags can be set at once: the most specific one wins */
    if ((s_reset_flags & RCC_CSR_IWDGRSTF) != 0U) {
        return "watchdog";
    }
    if ((s_reset_flags & RCC_CSR_WWDGRSTF) != 0U) {
        return "watchdog de ventana";
    }
    if ((s_reset_flags & RCC_CSR_SFTRSTF) != 0U) {
        return "software (fallo o depurador)";
    }
    if ((s_reset_flags & RCC_CSR_BORRSTF) != 0U) {
        return "encendido o caida de tension";
    }
    if ((s_reset_flags & RCC_CSR_PINRSTF) != 0U) {
        return "pin de reset";
    }
    return "desconocido";
}

/*
 * Single HAL entry point for all EXTI interrupts. Dispatches to the
 * registered callback for the firing pin, if any. Unknown pins are
 * silently ignored — no handler registered means no action taken.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    for (uint8_t i = 0; i < BOARD_EXTI_TABLE_SIZE; i++) {
        if (s_exti_table[i].pin == GPIO_Pin && s_exti_table[i].callback != NULL) {
            s_exti_table[i].callback(GPIO_Pin);
            return;
        }
    }
}
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Pines del MUX y UART
#define UART_RX_PIN GPIO_NUM_19
#define MUX_PIN_A GPIO_NUM_18
#define MUX_PIN_B GPIO_NUM_5
#define MUX_PIN_C GPIO_NUM_22

#define UART_NUM UART_NUM_1
#define BUF_SIZE (1024) 

int canal = 0;

// Configura UART para leer un solo byte
void configure_uart() {
    //uart_driver_delete(UART_NUM); // Liberar UART previamente
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
}

// Configuración del MUX
void configure_mux() {
    gpio_set_direction(MUX_PIN_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(MUX_PIN_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(MUX_PIN_C, GPIO_MODE_OUTPUT);
}

// Selecciona el canal del MUX usando un switch
void select_mux_channel(int channel) {
    switch (channel) {
        case 0:
            gpio_set_level(MUX_PIN_A, 0);
            gpio_set_level(MUX_PIN_B, 0);
            gpio_set_level(MUX_PIN_C, 0);
            printf("Canal %d: ", canal);
            break;
        case 1:
            gpio_set_level(MUX_PIN_A, 1);
            gpio_set_level(MUX_PIN_B, 0);
            gpio_set_level(MUX_PIN_C, 0);
            printf("Canal %d: ", canal);
            break;
        case 2:
            gpio_set_level(MUX_PIN_A, 0);
            gpio_set_level(MUX_PIN_B, 1);
            gpio_set_level(MUX_PIN_C, 0);
            printf("Canal %d: ", canal);
            break;
    }
}

// Lee un byte desde el canal activo del MUX
void read_from_mux() {
    uint8_t data;
    int length = uart_read_bytes(UART_NUM, &data, 1, 100 / portTICK_PERIOD_MS);
    if (length > 0) {
        printf(" %d\n", data);
    } else {
        printf("No hay datos disponibles en el canal %d\n", canal);
    }
}

void app_main() {
    configure_uart();
    configure_mux();

    while (1) {
        select_mux_channel(canal);
        read_from_mux();
        canal = (canal + 1) % 2;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

// Definir los pines UART
#define UART_NUM UART_NUM_1
#define UART_TX_PIN GPIO_NUM_18  //CLK (TX del micro)
#define UART_RX_PIN GPIO_NUM_19  //MISO (RX del micro)
#define UART_INH_PIN GPIO_NUM_23 //MOSI 

// Definir los pines de control del MUX
#define PIN_A GPIO_NUM_13        //SENSE1
//#define PIN_B GPIO_NUM_15        //SENSE2
#define PIN_B GPIO_NUM_5         //SD_CS
//#define PIN_C GPIO_NUM_14        //SENSE3
#define PIN_C GPIO_NUM_22        //SCL

// Tamaño del buffer
#define BUF_SIZE (1024)

// Función para procesar un paquete de datos del sensor
bool process_data_packet(uint8_t *data, int length) {
    for (int i = 0; i < length; i += 4) {
        if (data[i] == 0xFF && i + 3 < length) {  // Verificar header y longitud suficiente para el paquete
            uint8_t Data_H = data[i + 1];
            uint8_t Data_L = data[i + 2];
            uint8_t checksum = data[i + 3];

            uint8_t calculated_checksum = (0xFF + Data_H + Data_L) & 0xFF;

            if (calculated_checksum == checksum) {
                uint8_t distance = (Data_H << 8) | Data_L;
                uint8_t umbral = 100;
                bool pig;
                printf("Distancia: %d mm\n", distance);

                if (distance > umbral) {
                    pig = 0;
                } else {
                    pig = 1;
                  }
                printf("Hay cerdo: %d \n", pig);

                return pig; // Devolver la variable que decide si el cerdo está bebiendo
            } else {
                printf("Checksum incorrecto en el paquete %d\n", i / 4);
              }
        } else {
            printf("Paquete de datos incorrecto en el índice %d\n", i);
          }
    }
    return 0; // Devolver 0 si no se pudo procesar ningún paquete correctamente
}

// Función para configurar los pines de control A, B y C del MUX
void configure_multiplexer() {
    gpio_set_direction(PIN_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_C, GPIO_MODE_OUTPUT);
    gpio_set_direction(UART_INH_PIN, GPIO_MODE_OUTPUT);

    // Seleccionar el canal Y0 (A=0, B=0, C=0)
    gpio_set_level(PIN_A, 0);
    gpio_set_level(PIN_B, 0);
    gpio_set_level(PIN_C, 1);
    gpio_set_level(UART_INH_PIN, 0);
}

// Función para configurar el UART
void configure_uart(void) {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        //.source_clk = UART_SCLK_APB, // Especificar el reloj APB como fuente
        .source_clk = UART_SCLK_DEFAULT
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
}

// Función que lee y procesa los datos del sensor
bool read_ultrasonic_sensor(void) {
    // Buffer para almacenar los datos del UART
    uint8_t data[BUF_SIZE];

    // Leer datos del sensor
    int length = uart_read_bytes(UART_NUM, data, BUF_SIZE, 100 / portTICK_PERIOD_MS);
    if (length > 0) {
        // Procesar los datos según el protocolo del sensor y devolver la distancia
        return process_data_packet(data, length);
    } else {
        printf("No data received\n");
      }

    // Si no se reciben datos, devolver 0
    return 0;
}

// Función principal
void app_main(void) {
    // Configurar UART y MUX solo una vez
    configure_uart();
    configure_multiplexer();

    while (1) {
        // Llamar a la función de lectura del sensor y almacenar el resultado
        bool pig_presence = read_ultrasonic_sensor();
        
    /*    //PRUEBA PARA VER SI TX DEL ESP32 FUNCIONA
        uint8_t byte_to_send = 0xAA;  // Byte a enviar
        // Enviar el byte por UART
        uart_write_bytes(UART_NUM, (const char *)&byte_to_send, 1); //Enviar caracter "..." continuamente
    */
        // Esperar un segundo antes de la siguiente lectura
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


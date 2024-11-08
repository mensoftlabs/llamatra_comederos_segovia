#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

// Definir los pines UART
#define UART_NUM UART_NUM_1
#define UART_TX_PIN GPIO_NUM_13  //SENSE1 (TX del micro) 
#define UART_RX_PIN GPIO_NUM_19  //MISO (RX del micro)
#define UART_INH_PIN GPIO_NUM_23 //MOSI 
#define sense2 GPIO_NUM_

// Definir los pines de control del MUX
#define PIN_A GPIO_NUM_18        //CLK
#define PIN_B GPIO_NUM_5         //SD_CS
#define PIN_C GPIO_NUM_22        //SCL

// Tamaño del buffer
#define BUF_SIZE (1024)

// Definir el número de sensores para multiplexar
#define NUM_SENSE 5

//Variables globales
uint8_t canal = 0; 
uint8_t cont1, cont2, cont3, cont4, cont5;

/*
// Función para procesar un paquete de datos del sensor
uint8_t process_data_packet(uint8_t *data, int length) {
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
                printf(" %d mm\n", distance);

                if (distance > umbral) {
                    pig = 0;
                } else {
                    pig = 1;
                    if (distance == 0) {
                      pig = 0;
                    }
                  }
                printf("Hay cerdo: %d \n", pig);
                printf("\n");

                return pig; // Devolver la variable que decide si el cerdo está bebiendo
            } else {
                printf("Checksum incorrecto en el paquete \n");
                printf("\n");
              }
        } else {
            printf("Paquete de datos incorrecto \n");
            printf("\n");
          }
    }
    return 0; // Devolver 0 si no se pudo procesar ningún paquete correctamente
}
*/
// Función para procesar un paquete de datos del sensor
uint8_t process_data_packet(uint8_t *data, int length) {


  uint8_t distance = (data[1] << 8) | data[2];
  uint8_t umbral = 100;
  bool pig;
  printf(" %d mm\n", distance);

  if (distance > umbral) 
  {
      pig = 0;
  } else 
  {
    pig = 1;
    if (distance == 0) {
      pig = 0;
    }
  }
  printf("Hay cerdo: %d \n", pig);
  printf("\n");

  return pig; // Devolver la variable que decide si el cerdo está bebiendo

}




// Función para configurar los pines de control A, B y C del MUX
void configure_multiplexer(uint8_t canal1) {

    switch (canal1) {
        case 0: //Sensor 1 en el pin Y0
          gpio_set_level(PIN_A, 0);
          gpio_set_level(PIN_B, 0);
          gpio_set_level(PIN_C, 0);
          break;
        case 1: //Sensor 2 en el pin Y1
          gpio_set_level(PIN_A, 1);
          gpio_set_level(PIN_B, 0);
          gpio_set_level(PIN_C, 0);
          break;
        case 2: //Sensor 3 en el pin Y2
          gpio_set_level(PIN_A, 0);
          gpio_set_level(PIN_B, 1);
          gpio_set_level(PIN_C, 0);
          break;
        case 3: //Sensor 4 en el pin Y3
          gpio_set_level(PIN_A, 1);
          gpio_set_level(PIN_B, 1);
          gpio_set_level(PIN_C, 0);
          break;
        case 4: //Sensor 5 en el pin Y4
          gpio_set_level(PIN_A, 1);
          gpio_set_level(PIN_B, 1);
          gpio_set_level(PIN_C, 0);
          break;
    }

    printf("Canal actual %d: ", canal);
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
    uint8_t data[4];
    uint8_t vaciar[100];

    // Leer datos del sensor y limpiar buffer
    uart_flush_input(UART_NUM);
    size_t buffered_length;
    uart_get_buffered_data_len(UART_NUM, &buffered_length);
    printf("Bytes en el buffer de recepción: %d\n", buffered_length);
/*
    while(buffered_length < 4)
    {
      vTaskDelay(pdMS_TO_TICKS(90));
      printf("Esperando...");
      uart_get_buffered_data_len(UART_NUM, &buffered_length);
    }*/
    int length = uart_read_bytes(UART_NUM, data, 4, 100 / portTICK_PERIOD_MS);
    
    uart_flush_input(UART_NUM);
    
    if (length > 0) {
        int SUM=(data[0]+data[1]+data[2])&0x00FF;
        if ((data[0] == 0xFF) && (data[3] == SUM)){
          printf("trama correcta");
        }
        // Procesar los datos según el protocolo del sensor y devolver la distancia
        return process_data_packet(data, length);
    } else {
        printf("No data received\n");
        printf("\n");
      }

    // Si no se reciben datos, devolver 0
    return 0;
}

// Función principal
void app_main(void) {

    gpio_set_direction(PIN_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_C, GPIO_MODE_OUTPUT);

    gpio_set_level(UART_TX_PIN, 0); //Fijar tiempo de respuesta a 100ns
    gpio_set_level(UART_INH_PIN, 0); //Habilitar el mux

    // Configurar UART y MUX solo una vez
    configure_uart();

    while (1) {
        configure_multiplexer(canal);
        read_ultrasonic_sensor();

        //Cambia al siguiente canal
        canal = (canal + 1) % NUM_SENSE;
        
        // Esperar antes de la siguiente lectura
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

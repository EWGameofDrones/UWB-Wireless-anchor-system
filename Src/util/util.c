#include <stdio.h>
// #include "boards.h"
#include "nrf_delay.h"
// #include "FreeRTOS.h"

// #include "nrf_drv_uart.h"
// #include "app_error.h"

// // Define UART instance
// static const nrf_drv_uart_t uart_inst = NRF_DRV_UART_INSTANCE(0); // UART0

// // UART event handler
// static void uart_event_handler(nrf_drv_uart_event_t * p_event, void * p_context)
// {
//     // Handle UART events here if needed
// }

// // UART initialization function
// ret_code_t uart_init(void)
// {
//     ret_code_t err_code;
    
//     // Configure UART parameters
//     nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
//     uart_config.pseltxd = UART_0_TX_PIN;  // Define your TX pin
//     uart_config.pselrxd = UART_0_RX_PIN;  // Define your RX pin
//     uart_config.baudrate = NRF_UART_BAUDRATE_115200;
//     uart_config.hwfc = NRF_UART_HWFC_DISABLED;
//     uart_config.parity = NRF_UART_PARITY_EXCLUDED;

//     // Initialize UART
//     err_code = nrf_drv_uart_init(&uart_inst, &uart_config, uart_event_handler);
//     APP_ERROR_CHECK(err_code);

//     return err_code;
// }

void util() {
    unsigned int i = 0;

    while (true) {
        bsp_board_led_invert(i);
        i += 1;
        if (i > 3) {
            i = 0;
        }
        nrf_delay_ms(250);
    }
}

void debug_LED(int led, bool on) {
    if (on == true) {
        bsp_board_led_on(led);
    }
    else if (on == false) {
        bsp_board_led_off(led);
    }
}

void debug_LED_timer(int led, int time) {
    bsp_board_led_on(led);
    nrf_delay_ms(time);
    // vTaskDelay(pdMS_TO_TICKS(time));
    bsp_board_led_off(led);
}

// Compares two frame's common fields, EXCEPT for address source
bool frame_compare(uint8_t *frame1, uint8_t *frame2) {
    int out1 = memcmp(frame1, frame2, 7);
    int out2 = memcmp(frame1 + 9, frame2 + 9, 1);


    return (out1 == 0 && out2 == 0);
}

// Compares frame if it has a valid address source (index 7 and index 8)
bool source_address_compare(uint8_t *frame) {

    if (frame[7] < 'A' || frame[7] > 'C') {
        return false;
    }

    if (frame[8] < 'A' || frame[8] > 'Z') {
        return false;
    }
    return true;
}

void debug_log(char msg[], int val) {
    char str[strlen(msg) + 10];
    snprintf(str, sizeof(str), msg, val);
    test_run_info((unsigned char *)str);
}

void debug_log_f(char msg[], float val) {
    char str[strlen(msg) + 10];
    snprintf(str, sizeof(str), msg, val);
    test_run_info((unsigned char *)str);
}

// Distance Buffer stuff ///////////////////////////

// struct to hold distance from an anchor
struct distance {
    double distance; // 8 bytes
    uint8_t source; // 1 byte, compiler adds 7bytes of padding
};

struct distance distance_create_entry(double distance, uint8_t source) {
    struct distance output;
    output.distance = distance;
    output.source = source;
    return output;
}

// Fifo buffer to hold distances from anchors
struct distance_buffer {
    struct distance buffer[9]; // 9 * 16 = 144 bytes
    unsigned int head;
    unsigned int tail;
    unsigned short int total;
};

void dist_buffer_init(struct distance_buffer *dist_buf) {
    dist_buf->head = 0;
    dist_buf->tail = 0;
    dist_buf->total = 0;
    printf("Distance buffer initialized\n");
    return;
};

bool dist_buffer_is_full(struct distance_buffer *dist_buf) {
    if (dist_buf->total >= 9) {
        printf("Distance buffer is full\n");
        return true;
    }
    printf("Distance buffer is not full\n");
    return false;
};

bool dist_buffer_push(struct distance_buffer *dist_buf, double distance, uint8_t source) {
    if (dist_buffer_is_full(dist_buf)) {
        printf("Distance buffer failed to push\n");
        return false;
    }

    dist_buf->buffer[dist_buf->head].distance = distance;
    dist_buf->buffer[dist_buf->head].source = source;
    dist_buf->head = (dist_buf->head + 1) % 9;
    dist_buf->total += 1;
    printf("Distance buffer pushed\n");
    return true;
};

bool dist_buffer_pop(struct distance_buffer *dist_buf) {
    if (dist_buf->total <= 0) {
        printf("Distance buffer failed to pop\n");
        return false;
    }

    struct distance *ptr_dist = &dist_buf->buffer[dist_buf->tail];
    dist_buf->tail = (dist_buf->tail + 1) % 9;
    dist_buf->total -= 1;
    printf("Distance buffer popped\n");
    return true;
}

void delay_nonblocking(int ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// struct distance dist_buffer_peek(struct distance_buffer *dist_buf) {
//     struct distance *ptr_dist = &dist_buf->buffer[dist_buf->tail];
//     printf("Distance: %f, Source: %c\n", ptr_dist->distance, ptr_dist->source);
//     return *ptr_dist;
// }

bool dist_buffer_peek(struct distance_buffer *dist_buf, struct distance *item) {
    if (dist_buf->total <= 0) {
        printf("Distance buffer failed to peek\n");
        return false;
    }

    item->distance = dist_buf->buffer[dist_buf->tail].distance;
    item->source = dist_buf->buffer[dist_buf->tail].source;
    printf("Distance: %f, Source: %c\n", item->distance, item->source);
    return true;
}

void dist_to_buffer(float distance, uint8_t *rx_buffer) {
    char dist_str[20];
    int length = snprintf(dist_str, sizeof(dist_str), "%3.2f", distance);
    debug_log("dist_str: %s", dist_str);
    if (length > 0) {
        memcpy(rx_buffer + 10, dist_str, length + 1);  // Include null terminator
    }
}

float buffer_to_dist(uint8_t *rx_buffer) {
    char dist_str[20];
    strncpy(dist_str, (char *)(rx_buffer + 10), 19);
    dist_str[19] = '\0';  // Ensure null termination
    // printf("dist_str: %s\n", dist_str);
    return atof(dist_str);
}


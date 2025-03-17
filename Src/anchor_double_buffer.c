/*
This code is to to be a double_buffered, interrupt handling, anchor
*/


#include "deca_probe_interface.h"
#include <deca_device_api.h>
#include <deca_spi.h>
#include <example_selection.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nrf_drv_uart.h"
#include "nrf_uart.h"
#include "nrf.h"
// #include "../dependencies/app_uart.h"

#define DIST_MSG_TYPE 0xE2
#define DIST_MSG_VALUE_IDX 10
#define DIST_MSG_LEN 14

#define QUEUE_SIZE 10
#define UART_MSG_SIZE 32  // Adjust based on your expected message size

typedef struct {
    char messages[QUEUE_SIZE][UART_MSG_SIZE];
    int front;
    int rear;
    int count;
} UARTQueue;

void initQueue(UARTQueue *q) {
    q->front = 0;
    q->rear = 0;
    q->count = 0;
}

int isQueueFull(UARTQueue *q) {
    return q->count == QUEUE_SIZE;
}

int isQueueEmpty(UARTQueue *q) {
    return q->count == 0;
}

void enqueue(UARTQueue *q, const char *message) {
    if (isQueueFull(q)) {
        printf("Queue is full. Message dropped!\n");
        return;
    }
    strncpy(q->messages[q->rear], message, UART_MSG_SIZE - 1);
    q->messages[q->rear][UART_MSG_SIZE - 1] = '\0';  // Ensure null termination
    q->rear = (q->rear + 1) % QUEUE_SIZE;
    q->count++;
}


// Function to extract distance from received message
static int dist_msg_get_value(uint8_t *buffer)
{
    uint32_t dist_int = 
        ((uint32_t)buffer[10] << 24) |
        ((uint32_t)buffer[11] << 16) |
        ((uint32_t)buffer[12] << 8) |
        (uint32_t)buffer[13];

    float result = ((float)dist_int) / 1000.0f;  // Cast to float first, then divide
    // debug_log("Verification int: %u", dist_int);
    // debug_log("Verification float: %f", result);
    return dist_int;
}


#if defined(TEST_ANCHOR_DBL_BUFF)

extern void test_run_info(unsigned char *data);

/* Example application name and version to display on LCD screen. */
#define APP_NAME "DOUBLE_BUFFERED Anchor BETA"

/* The following can be enabled to use manual RX enable instead of auto RX re-enable
 * NOTE: when using DW30xx devices, only the manual RX enable should be used
 *       with DW37xx devices either manual or auto RX enable can be used. */
#define USE_MANUAL_RX_ENABLE 1

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

// TX CONFIG START ////////////////////////////////////////////
// TX CONFIG START ////////////////////////////////////////////

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 500

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Frames used in the ranging process. See NOTE 3 below. */
static uint8_t tx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0 };
static uint8_t rx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'w', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };


/* Buffer to store received frame. See NOTE 1 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX];

#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX          2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN         4
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 400

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 2 below. */
extern dwt_txconfig_t txconfig_options;

// TX CONFIG END ////////////////////////////////////////////
// TX CONFIG END ////////////////////////////////////////////

/* Declaration of static functions. */
static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void rx_err_cb(const dwt_cb_data_t *cb_data);

// UART BEGIN ////////////////////////////////////////////////
// Define UART instance a
static const nrf_drv_uart_t uart_inst = NRF_DRV_UART_INSTANCE(0);

// Add at file top
static volatile bool uart_busy = false;
static volatile bool new_data_ready = false;
static char uart_message[16]; // Uart message to send to host

// Initialize UART
static void uart_init(void)
{
    ret_code_t err_code;

    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
    uart_config.pseltxd = TX_PIN_NUMBER;  // Define your TX pin
    uart_config.pselrxd = RX_PIN_NUMBER;  // Define your RX pin
    uart_config.baudrate = NRF_UART_BAUDRATE_115200;

    err_code = nrf_drv_uart_init(&uart_inst, &uart_config, NULL);
    if (err_code != NRF_SUCCESS) {
        debug_log("UART initialization failed: %d", err_code);
        APP_ERROR_CHECK(err_code);
    } else {
        debug_log("UART initialized successfully");
    }


}

// Modify uart_send to be non-blocking
static void uart_send(const char *data)
{
    if (!uart_busy) {
        uart_busy = true;
        ret_code_t err_code = nrf_drv_uart_tx(&uart_inst, (uint8_t *)data, strlen(data));
        uart_busy= false;
        // if (err_code != NRF_SUCCESS) {
        //      uart_busy= false;
        // }
    } else {
        debug_log("UART is busy");
    }
}

// Modify UART init to include event handler
static void uart_event_handler(nrf_drv_uart_event_t *p_event, void *p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_UART_EVT_TX_DONE:
            uart_busy = false;
            break;
    }
}

// UART END ////////////////////////////////////////////////

int anchor_dbl_buff(void) {
    debug_log("By FAU");
    debug_log(APP_NAME);
    debug_log("entry point");

    /* Configure frame filtering */

    // // Set PAN ID
    // dwt_setpanid(0xDDDD);
    // dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN);

    // // Enable address filtering for short addresses
    // // dwt_seteui("WA");  // Set expected destination address

    // // Configure frame filter
    // uint16_t frame_filter = DWT_FF_ADDR_FILTER;     // Enable address filtering
    //                     // DWT_FF_ADDR_FILTER;   // Enable address filtering
    //                     // DWT_FF_MAC_EN |       // Enable MAC filtering

    // // Mask out all other frame types
    // uint16_t frame_filter_mask = DWT_FF_NOTYPE_EN;

    // dwt_configureframefilter(frame_filter, frame_filter_mask);

    // Additional function code filtering will need to be done in software
    // after receiving the frame, by checking the function code byte (0xE2)    

    uint32_t dev_id;
    uart_init();

    port_set_dw_ic_spi_fastrate();
    reset_DWIC();
    Sleep(2);
    dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

    dev_id = dwt_readdevid();

    while (!dwt_checkidlerc()) { };

    if (dwt_initialise(DWT_DW_IDLE) == DWT_ERROR)
        {
            debug_log("INIT FAILED");
            while (1) { };
        }

    /* Configure DW3xxx */
    if (dwt_configure(&config)) /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
        {
            debug_log("CONFIG FAILED");
            while (1) { };
        }

    // txrf twr --------------------------------------------------
    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);

    /* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
     * Note, in real low power applications the LEDs should not be used. */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    // dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    // dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    // STUFF for txrf --------------------------------------------
    
    /* Register RX call-back. When using automatic RX re-enable is used below the RX error will not be reported */
    dwt_setcallbacks(NULL, rx_ok_cb, NULL, rx_err_cb, NULL, NULL, NULL);

    /*Clearing the SPI ready interrupt*/
    dwt_writesysstatuslo(DWT_INT_RCINIT_BIT_MASK | DWT_INT_SPIRDY_BIT_MASK);

    /* Enable RX interrupts for double buffer (RX good frames and RX errors). */
    dwt_setinterrupt(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR, 0, DWT_ENABLE_INT);
    // dwt_setinterrupt_db(RDB_STATUS_RXOK, DWT_ENABLE_INT);

    /* Install DW IC IRQ handler. */
    port_set_dwic_isr(dwt_isr);

    dwt_setdblrxbuffmode(DBL_BUF_STATE_EN, DBL_BUF_MODE_MAN); // Enable double buffer - manual RX re-enable mode, see NOTE 4.

    dwt_rxenable(DWT_START_RX_IMMEDIATE); // Enable 
    
    // debug_LED_timer(0, 5000);
    // debug_log("Led off");

    while (1) {
    }

    debug_log("End of entry point");
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_ok_cb()
 * @brief Callback to process RX good frame events
 * @param  cb_data  callback data
 * @return  none
 */
int totalReceivedRangings = 0;
static void rx_ok_cb(const dwt_cb_data_t *cb_data) {
    uint8_t temp_byte;
    debug_log("RX OK");

    /* First check byte 7 without copying entire buffer */
    dwt_readrxdata(&temp_byte, 1, 7); // Read just byte at index 7
    
    if (temp_byte == 'D') {
        /* Only copy full frame if we're interested in it */
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        int received_distance = dist_msg_get_value(rx_buffer);
        // snprintf(uart_message, sizeof(uart_message), 
        //         "USING UART: Frame Received: Frame[7, 8]= %c %c : received distance: %3.2f m\n", 
        //         rx_buffer[7], rx_buffer[8], received_distance / 1000.0f);
        snprintf(uart_message, sizeof(uart_message), 
                "%c %3.2f %d\n", 
                rx_buffer[8], received_distance / 1000.0f, rx_buffer[2]);
        uart_send(uart_message);
        // debug_log(uart_message);
    } else {
        /* Clear RX buffer by re-enabling receiver */
        // totalReceivedRangings += 1;
        // debug_log("ranging %d", totalReceivedRangings);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }

}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_err_cb()
 * @brief Callback to process RX error and timeout events
 * @param  cb_data  callback data
 * @return  none
 */
static void rx_err_cb(const dwt_cb_data_t *cb_data) {
    if (cb_data->status & DWT_INT_RXFTO_BIT_MASK) {
        debug_log("RX timeout");
    }
    if (cb_data->status & DWT_INT_RXPTO_BIT_MASK) {
        debug_log("Preamble timeout");
    }
    if (cb_data->status & DWT_INT_RXPHE_BIT_MASK) {
        debug_log("PHY header error");
    }
    if (cb_data->status & DWT_INT_RXFCE_BIT_MASK) {
        debug_log("CRC error");
    }
    if (cb_data->status & DWT_INT_RXOVRR_BIT_MASK) {
        debug_log("RX buffer overflow");
    }
    
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

#endif
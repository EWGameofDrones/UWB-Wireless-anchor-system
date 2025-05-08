/*
 * interrupt_initiator.c - Interrupt-driven TWR initiator
 * Based on anchor_double_buffer.c structure
 */

#include "deca_probe_interface.h"
#include <config_options.h>
#include <deca_device_api.h>
#include <deca_spi.h>
#include <example_selection.h>
#include <port.h>
#include <shared_defines.h>
// #include <shared_functions.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nrf_drv_uart.h"
#include "nrf_uart.h"
#include "nrf.h"

#if defined(INTERRUPT_INITIATOR)

#define RESP_ID_1 'A'
#define RESP_ID_2 'B'

#define __DMB() __asm volatile ("dmb" ::: "memory")


/* Example application name and version */
#define APP_NAME "\n\nINT-INITIATOR BETA"

/* Default communication configuration */
static dwt_config_t config = {
    5,                /* Channel number */
    DWT_PLEN_128,     /* Preamble length */
    DWT_PAC8,         /* Preamble acquisition chunk size */
    9,                /* TX preamble code */
    9,                /* RX preamble code */
    1,                /* Non-standard SFD */
    DWT_BR_6M8,       /* Data rate */
    DWT_PHRMODE_STD,  /* PHY header mode */
    DWT_PHRRATE_STD,  /* PHY header rate */
    (129 + 8 - 8),    /* SFD timeout */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length */
    DWT_PDOA_M0       /* PDOA mode off */
};

/* Constants for ranging and timing */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
#define RESP_RX_TIMEOUT_UUS 2000
#define RNG_DELAY_MS 100
// #define DWT_TIME_UNITS (1.0/499.2e6/128.0)
#define DWT_TIME_UNITS (1.0 / (499.2e6 * 128.0))
#define SPEED_OF_LIGHT 299702547.0
#define POLL_INTERVAL_MS 100  // Fixed polling interval of 100ms

/* Frames used in the ranging process */
static uint8_t tx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'I', 'E', 0xE0, 0, 0 };
static uint8_t rx_resp_msg_1[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', RESP_ID_1, 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static uint8_t rx_resp_msg_2[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', RESP_ID_2, 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

/* Distance message to send to monitor/aggregator */
static uint8_t tx_dist_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'D', 'I', 'S', 'T', 0xE2, 0, 0, 0, 0, 0, 0 };

/* Buffer to store received frames */
#define RX_BUF_LEN 20
static uint8_t rx_buffer[RX_BUF_LEN];

/* Distance calculation related variables */
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
#define DIST_MSG_VALUE_IDX 10

/* State variables */
static uint8_t frame_seq_nb = 0;
static bool resp1_received = false;
static bool resp2_received = false;
static double distance_1 = 0.0;
static double distance_2 = 0.0;
static uint32_t status_reg = 0;
static volatile bool ranging_complete = false;

/* UART related definitions */
static const nrf_drv_uart_t uart_inst = NRF_DRV_UART_INSTANCE(0);
static volatile bool uart_busy = false;
static char uart_message[64];

extern dwt_txconfig_t txconfig_options;

/* Declaration of static functions. */
static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void rx_to_cb(const dwt_cb_data_t *cb_data);
static void rx_err_cb(const dwt_cb_data_t *cb_data);
static void tx_conf_cb(const dwt_cb_data_t *cb_data);
static void tx_done_cb(const dwt_cb_data_t *cb_data);

static void compute_distance(uint8_t *buffer, double *distance, char resp_id);
static void dist_msg_set_value(uint8_t *buffer, float distance, char resp_id);
// void resp_msg_get_ts(uint8_t *ts_field, uint32_t *ts);
static void uart_init(void);
static void uart_send(const char *data);
static void uart_event_handler(nrf_drv_uart_event_t *p_event, void *p_context);
static void process_response(uint8_t *buffer, uint16_t len, double *distance, char resp_id, uint32_t resp_rx_ts);

/* Delay from end of transmission to activation of reception, expressed in UWB microseconds (1 uus is 512/499.2 microseconds). See NOTE 3 below. */
#define TX_TO_RX_DELAY_UUS 60

/* Receive response timeout, expressed in UWB microseconds. See NOTE 4 below. */
#define RX_RESP_TO_UUS 100000

/* Default inter-frame delay period, in milliseconds. */
#define DFLT_TX_DELAY_MS 1000
/* Inter-frame delay period in case of RX timeout, in milliseconds.
 * In case of RX timeout, assume the receiver is not present and lower the rate of blink transmission. */
#define RX_TO_TX_DELAY_MS 3000
/* Inter-frame delay period in case of RX error, in milliseconds.
 * In case of RX error, assume the receiver is present but its response has not been received for any reason and retry blink transmission immediately. */
#define RX_ERR_TX_DELAY_MS 0
/* Current inter-frame delay period.
 * This global static variable is also used as the mechanism to signal events to the background main loop from the interrupt handler callbacks,
 * which set it to positive delay values. */
static int32_t tx_delay_ms = -1;


// STATE MACHINES
typedef enum {
    STATE_IDLE,
    STATE_POLLING,
    STATE_WAITING_FOR_RESPONSES,
    STATE_PROCESSING_RESPONSES
} state_t;

// GLOBAL VARIABLES
volatile state_t current_state = STATE_IDLE; // Ensure volatile for interrupt-safe access
static volatile int32_t ranging_delay_ms = 1000; // Also volatile for interrupt-safe access

// Correct implementation of resp_msg_get_ts
static void resp_msg_get_ts(const uint8_t *ts_field, uint32_t *ts)
{
    *ts = 0;
    for (int i = 0; i < RESP_MSG_TS_LEN; i++) {
        *ts |= ((uint32_t)ts_field[i]) << (8 * i);
    }
}

/* Initialize UART */
static void uart_init(void)
{
    ret_code_t err_code;
    
    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
    uart_config.pseltxd = TX_PIN_NUMBER;
    uart_config.pselrxd = RX_PIN_NUMBER;
    uart_config.baudrate = NRF_UART_BAUDRATE_115200;
    
    err_code = nrf_drv_uart_init(&uart_inst, &uart_config, uart_event_handler);
    if (err_code != NRF_SUCCESS) {
        test_run_info((unsigned char *)"UART INIT FAILED");
        APP_ERROR_CHECK(err_code);
    }
}

/* UART event handler */
static void uart_event_handler(nrf_drv_uart_event_t *p_event, void *p_context)
{
    if (p_event->type == NRF_DRV_UART_EVT_TX_DONE) {
        uart_busy = false;
    }
}

/* Send data over UART */
static void uart_send(const char *data)
{
    if (!uart_busy) {
        uart_busy = true;
        ret_code_t err_code = nrf_drv_uart_tx(&uart_inst, (uint8_t *)data, strlen(data));
        if (err_code != NRF_SUCCESS) {
            uart_busy = false;
        }
    }
}

/* Set distance value in distance message */
static void dist_msg_set_value(uint8_t *buffer, float distance, char resp_id)
{
    /* Set responder ID in the message */
    buffer[8] = resp_id;
    
    /* Convert float distance to fixed-point for transmission */
    uint32_t dist_int = (uint32_t)(distance * 1000.0f);
    
    /* Store the distance value in the message */
    buffer[10] = (uint8_t)((dist_int >> 24) & 0xFF);
    buffer[11] = (uint8_t)((dist_int >> 16) & 0xFF);
    buffer[12] = (uint8_t)((dist_int >> 8) & 0xFF);
    buffer[13] = (uint8_t)(dist_int & 0xFF);
}

/* Compute distance from received response */
static void compute_distance(uint8_t *buffer, double *distance, char resp_id)
{
    printf("compute distance start\n");
    uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
    int32_t rtd_init, rtd_resp;
    float clockOffsetRatio;

    /* Retrieve poll transmission and response reception timestamps */
    poll_tx_ts = dwt_readtxtimestamplo32();
    resp_rx_ts = dwt_readrxtimestamplo32();
    
    printf("compute distance flag1\n");
    
    /* Get clock offset ratio to correct for local and remote clock differences */
    clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);
    
    /* Get timestamps embedded in response message */
    resp_msg_get_ts(&buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
    resp_msg_get_ts(&buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

    printf("compute distance flag2\n");
    
    /* Compute time of flight and distance */
    rtd_init = resp_rx_ts - poll_tx_ts;
    rtd_resp = resp_tx_ts - poll_rx_ts;
    *distance = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS * SPEED_OF_LIGHT;
    
    printf("compute distance flag3\n");
    
    /* Log distance information */
    // snprintf(uart_message, sizeof(uart_message), "DIST_%c: %.2f m\n", resp_id, *distance);
    // uart_send(uart_message);
    printf("Distance from responder %c: %.2f m\n", resp_id, *distance);

    // printf("compute distance flag4\n");
    
    /* Just update the distance value, but don't send a message here */
    /* We'll let the main loop handle distance reporting */
    
    // printf("compute distance end\n");
}


volatile uint32_t * const TEST_VARIABLE_PTR = (volatile uint32_t *)0x20004000;
volatile uint32_t * const tx_poll_timestamp = (volatile uint32_t *)0x20004004;
volatile uint32_t * const rx_poll_timestamp = (volatile uint32_t *)0x20004008;


/* Main interrupt-driven initiator function */
int ss_twr_initiator_interrupt(void)
{
    /* Initialize and configure DW3000 */
    test_run_info((unsigned char *)APP_NAME);
    
    uart_init();
    
    /* Configure SPI rate, DW3000 supports up to 36 MHz */
    port_set_dw_ic_spi_fastrate();

    /* Reset DW IC */
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    
    /* Probe for the correct device driver. */
    dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */ { };

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED     ");
        while (1) { };
    }

    /* Configure DW IC. See NOTE 10 below. */
    /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    if (dwt_configure(&config))
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1) { };
    }

    printf("DW IC configured successfully\n");
    
    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);

    /* Register the call-backs */
    dwt_setcallbacks(&tx_done_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb, NULL, NULL, NULL);

    /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
    dwt_setinterrupt(DWT_INT_TXFRS_BIT_MASK | DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXPTO_BIT_MASK | DWT_INT_RXPHE_BIT_MASK
                         | DWT_INT_RXFCE_BIT_MASK | DWT_INT_RXFSL_BIT_MASK | DWT_INT_RXSTO_BIT_MASK,
        0, DWT_ENABLE_INT);

    /*Clearing the SPI ready interrupt*/
    dwt_writesysstatuslo(DWT_INT_RCINIT_BIT_MASK | DWT_INT_SPIRDY_BIT_MASK);

    /* Install DW IC IRQ handler. */
    port_set_dwic_isr(dwt_isr);

    port_EnableEXT_IRQ();
    printf("Interrupt setup after enable: status=%d\n", port_GetEXT_IRQStatus());

    /* Set delay to turn reception on after transmission of the frame. See NOTE 3 below. */
    dwt_setrxaftertxdelay(TX_TO_RX_DELAY_UUS);

    /* Set response frame timeout. */
    dwt_setrxtimeout(RX_RESP_TO_UUS);

    printf("Before While(1) loop \n");
    
    /* Main ranging loop */

    *TEST_VARIABLE_PTR = 0;

    
    while (1) {
        printf("Starting ranging\n");
        print_state("while(1)");

        /* Reset device state */
        dwt_forcetrxoff();
        
        /* Clear all status flags */
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK | 
                            DWT_INT_RXFCG_BIT_MASK | 
                            SYS_STATUS_ALL_RX_ERR);
        
        /* Prepare frame data */
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb++;
        
        /* Zero out timestamp before TX */
        *tx_poll_timestamp = 0;

        /* Write frame data to DW IC and prepare transmission */
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 0);

        /* Start transmission */
        int ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        int timeout = 3;

        while ((*TEST_VARIABLE_PTR) < 2) {
            // printf("while(1), tx_timestamp: %lu\n", *tx_poll_timestamp);
            Sleep(100);
            printf("Test variable: %lu\n", *TEST_VARIABLE_PTR);

            timeout--;
            if (timeout <= 0) { break; }
        }

        (*TEST_VARIABLE_PTR) = 0;
        // printf("Test variable: %lu\n", *TEST_VARIABLE_PTR);

        Sleep(100);
        
        // printf("time elapsed: %d ms\n", port_GetTick() - start_time);


        // /* Write frame data to DW IC and prepare transmission */
        // dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        // dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 0);
        
        // /* Start transmission */
        // int ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
        // if (ret != DWT_SUCCESS) {
        //     printf("TX start failed\n");
        //     continue;
        // } else {
        //     printf("TX started successfully\n");
        // }
        
        // /* Wait for any RX event. */
        // while (tx_delay_ms == -1) { };

        // /* Execute the defined delay before next transmission. */
        // if (tx_delay_ms > 0)
        // {
        //     printf("delay %d ms \n", tx_delay_ms);
        //     Sleep(tx_delay_ms);
        // }
        
        // /* Manually capture timestamp if callback hasn't */
        // if (tx_poll_timestamp == 0) {
        //     tx_poll_timestamp = dwt_readtxtimestamplo32();
        //     printf("Manual TX timestamp capture: %u\n", tx_poll_timestamp);
        // }

        // tx_delay_ms = -1;
        
        // current_state = STATE_IDLE;
        printf("End of main loop\n\n");
        /* Rest of main loop... */
    }
}

// print out the current state
void print_state(const char *func_name)
{
    const char *state_names[] = { "STATE_IDLE", "STATE_POLLING", "STATE_WAITING_FOR_RESPONSES", "STATE_PROCESSING_RESPONSES" };
    printf("%s: ", func_name);
    printf("Current state: %s\n", state_names[current_state]);
}



/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_ok_cb()
 *
 * @brief Callback to process RX good frame events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    current_state = STATE_PROCESSING_RESPONSES;
    __DMB(); // Add memory barrier after state change

    // dwt_rxenable(DWT_START_RX_IMMEDIATE); // Re-enable RX immediately

    printf("rx_ok_cb\n");

    (*TEST_VARIABLE_PTR)++;
    __DMB(); // Add memory barrier after state change

    // print_state("rx_ok_cb");

    dwt_readrxdata(rx_buffer, cb_data->datalength, 0);

    // printf("rx_ok_cb, frame[8]: %c\n", rx_buffer[8]);

    uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
    float clockOffsetRatio;
    int32_t rtd_init, rtd_resp;
    double distance;
    double tof;

    // /* Clear local RX buffer */
    // memset(rx_buffer, 0, FRAME_LEN_MAX);

    // printf("rx_ok_cb: lengths: %d : %d\n", cb_data->datalength, sizeof(rx_resp_msg_1));
    // /* Copy received frame to buffer */
    if (cb_data->datalength == sizeof(rx_resp_msg_1))
    {
    //     printf("rx_ok_cb: lengths match\n");
    //     dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
        
    //     /* Print raw message for debugging */
    //     printf("Response message bytes: ");
    //     for (int i = 0; i < 20; i++) {
    //         printf("%02X ", rx_buffer[i]);
    //     }
    //     printf("\n");
        printf("rx_ok_cb, frame[8]: %c\n", rx_buffer[8]);

        /* Retrieve timestamps */
        poll_tx_ts = dwt_readtxtimestamplo32();
        resp_rx_ts = dwt_readrxtimestamplo32();
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);
        resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
        resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);
        
        /* Compute time of flight and distance */
        rtd_init = resp_rx_ts - poll_tx_ts;
        rtd_resp = resp_tx_ts - poll_rx_ts;
        tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        distance = tof * SPEED_OF_LIGHT;
        
        /* Print distance information */
        // sprintf(dist_str, "DIST1: %3.2f m", distance);
        printf("DIST: %3.2f m\n", distance);
    }
    dwt_rxenable(DWT_START_RX_IMMEDIATE); // Re-enable RX immediately

    // (*TEST_VARIABLE_PTR)++;
    // __DMB();  // Memory barrier before reading
    
    printf("rx_ok_cb end\n");
    // current_state = STATE_IDLE;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_to_cb()
 *
 * @brief Callback to process RX timeout events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_to_cb(const dwt_cb_data_t *cb_data)
{
    (void)cb_data;
    printf("rx_to_cb\n");
    /* Set corresponding inter-frame delay. */
    tx_delay_ms = RX_TO_TX_DELAY_MS;

    /* TESTING BREAKPOINT LOCATION #2 */
}
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_err_cb()
 *
 * @brief Callback to process RX error events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_err_cb(const dwt_cb_data_t *cb_data)
{
    printf("rx_err_cb\n");
    (void)cb_data;
    /* Set corresponding inter-frame delay. */
    tx_delay_ms = RX_ERR_TX_DELAY_MS;

    /* TESTING BREAKPOINT LOCATION #3 */
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tx_conf_cb()
 *
 * @brief Callback to process TX confirmation events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void tx_conf_cb(const dwt_cb_data_t *cb_data)
{
    (void)cb_data;
    /* This callback has been defined so that a breakpoint can be put here to check it is correctly called but there is actually nothing specific to
     * do on transmission confirmation in this example. Typically, we could activate reception for the response here but this is automatically handled
     * by DW IC using DWT_RESPONSE_EXPECTED parameter when calling dwt_starttx().
     * An actual application that would not need this callback could simply not define it and set the corresponding field to NULL when calling
     * dwt_setcallbacks(). The ISR will not call it which will allow to save some interrupt processing time. */

    /* TESTING BREAKPOINT LOCATION #4 */
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tx_done_cb()
 *
 * @brief Callback to process TX done events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void tx_done_cb(const dwt_cb_data_t *cb_data)
{
    printf("tx_done_cb triggered\n");

    // Clear TX status flag
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

    
    // Save TX timestamp immediately
    *tx_poll_timestamp = dwt_readtxtimestamplo32();
    
    printf("tx_done_cb Timestamp captured: %lu\n",*tx_poll_timestamp);
    current_state = STATE_WAITING_FOR_RESPONSES;
    __DMB(); // Add memory barrier after state change

}

#endif


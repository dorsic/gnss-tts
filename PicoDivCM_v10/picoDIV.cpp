/*
    Divides the input frequency by factor 10^7, 10MHz to 1Hz and other frequencies using PIO.
    System clock can be driven from 10MHz external clock through GPIO.
    Based on the pio_blink example https://github.com/raspberrypi/pico-examples/tree/master/pio/pio_blink

    EXTERNAL CLOCK REFERENCE
    Connect 10 MHz 3.3V input signal to GPIO20 (pin 26).
    By default output 1 Hz, 10 ms pulse is at GPIO21 (and GPIO25, led).
    
    GPIO19 is used to synchronize to the output with external signal.
    Short it to GPIO20 when synchronization not needed.
    Use SYNC_TUNE to fine align generated output to sync signal.
    PIO1 is used for synchronized program, PIO0 for free running unsynchronized divider.
    Synchronization is done only at PIO program start. After that the clock 
    is steered only by reference clock.

    Note if system clock under 48MHz, USB is not working.

    SPDX-License-Identifier: BSD-3-Clause

    Version:
    v1.0 15-Feb-2022  Marek Dorsic (.md)
*/

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/pll.h"
#include "hardware/xosc.h"
#include "hardware/uart.h"
#include "picoDIV.pio.h"
#include "picoDIV_sync.pio.h"
#include "uart.pio.h"
#include "SCPI_parser/Vrekrer_scpi_parser.h"

#define NODEBUG

#define EXT_CLK_GPIO 20           // CAN NOT BE CHANGED, because of RP2040 gpio usages.
#define SYNC_GPIO 19              // DO NOT CHANGE unless you modify the WAIT instruction in picoDIV_Sync PIO
#define SYNC_TUNE 7               // fine align the output PPS to sync PPS, change by 1 is change by 100ns @ 10MHz ExtClk

#define OUT_A_GPIO 21
#define OUT_B_GPIO PICO_DEFAULT_LED_PIN

#define UART_TX 16
#define UART_RX 17


uint32_t sys_freq = 0;
uint32_t ext_clk_freq = 0;
uint32_t pulse_len_ns = 10000000;    // 10 ms
uint32_t out_freq_hz = 1;            // 1 Hz ~ 1 PPS
bool sync_output = false;
bool ext_ref = false;
bool pdiv_ison = false;
bool conf_clocks = true;
int poffset = -1;
int psyncoffset = -1;

SCPI_Parser pdiv_instrument;

void configure_clocks() {
    // clk_sys is driving the PIO
    // clk_peri is driving the UART (baudrate)
    #ifdef DEBUG
        printf("DEBUG> Configuring clocks... ext_ref=%d\n", ext_ref);
    #endif
    if (ext_ref) {
        printf("Configuring external ref clock @ %d\n", ext_clk_freq);
        sys_freq = ext_clk_freq;
        clock_configure_gpin(clk_ref, EXT_CLK_GPIO, sys_freq, sys_freq);
        clock_configure_gpin(clk_sys, EXT_CLK_GPIO, sys_freq, sys_freq);
        clock_configure_gpin(clk_peri, EXT_CLK_GPIO, sys_freq, sys_freq);
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        xosc_disable();
    } else {
        sys_freq = XOSC_MHZ*MHZ;  // 12MHz is the internal XOSC
        printf("Configuring TCXO  @ %d\n", sys_freq);
        xosc_init();
        clock_configure(clk_ref, CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC, 0, sys_freq, sys_freq);
        clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, sys_freq, sys_freq);
        clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, sys_freq, sys_freq);
    }
    // stop the clk_sys PPL
    pll_deinit(pll_sys);
    
    stdio_uart_init_full(uart_default, 9600, UART_TX, UART_RX);
    conf_clocks = false;
    #ifdef DEBUG
        printf("DEBUG> DONE clocks configuration\n", ext_ref);
    #endif
}

void blink_pin_forever(uint sm, uint pin, uint32_t total_clk, uint32_t pulse_clk, bool sync) {
    #ifdef DEBUG
        printf("DEBUG> Blink pin forever pdiv_ison=%d, sm=%d, pin=%d, sync=%d, total_clk=%u\n", pdiv_ison, sm, pin, sync, total_clk);
    #endif

    PIO pio;
    uint32_t st = 0;
    if (pdiv_ison) {
        pio_sm_set_enabled(pio, sm, false);
    }
    if (sync) {
        pio = pio1;
        picodiv_sync_program_init(pio, sm, psyncoffset, pin);
        st = SYNC_TUNE;
    } else {
        pio = pio0;
        picodiv_program_init(pio, sm, poffset, pin);
    }
    pio_sm_set_enabled(pio, sm, true);

    pio->txf[sm] = pulse_clk - 3;                   // write number of HIGH clock cycles to PIO register
    pio->txf[sm] = total_clk - pulse_clk - 3;       // write number off LOW clock cycles to PIO register
    pio->txf[sm] = total_clk - st;                   // write the first sync period clock cycles to PIO register
    #ifdef DEBUG
        printf("DEBUG> DONE Blink pin forever.\n");
    #endif
}

void configure_pios() {
    #ifdef DEBUG
        printf("DEBUG> Configuring PIOs sync_output=%d\n", sync_output);
    #endif

    // assign output pins and outout frequency to PIOs               
    sys_freq = ext_ref ? ext_clk_freq : XOSC_MHZ*MHZ;
    uint32_t tot_clk = div_u32u32(sys_freq, (uint32_t) out_freq_hz);
    uint32_t nomin = (uint32_t)pulse_len_ns * (sys_freq/MHZ);
    uint32_t pls_clk = div_u32u32(nomin, (uint32_t)1000);
    blink_pin_forever(0, OUT_A_GPIO, tot_clk, pls_clk, sync_output);
    blink_pin_forever(1, OUT_B_GPIO, tot_clk, pls_clk, sync_output);
    #ifdef DEBUG
        printf("DEBUG> DONE PIOs configuration.\n");
    #endif
}


void scpi_errorhandler(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
  switch(pdiv_instrument.last_error){
    case pdiv_instrument.ErrorCode::BufferOverflow: 
      printf("Buffer overflow error\n");
      break;
    case pdiv_instrument.ErrorCode::Timeout:
      printf("Communication timeout error\n");
      break;
    case pdiv_instrument.ErrorCode::UnknownCommand:
      printf("Unknown command received\n");
      break;
    case pdiv_instrument.ErrorCode::NoError:
      printf("No Error\n");
      break;
  }
  pdiv_instrument.last_error = pdiv_instrument.ErrorCode::NoError;    
}

void identify(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
  printf("PicoDIV in Counter Module v1.0, v1.0\n");
}

void pdiv_set_freq(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    #ifdef DEBUG
        printf("DEBUG> in command pdiv_set_freq command. Param size=%d\n", parameters.Size());
    #endif

    if (parameters.Size() > 0) {
        sscanf(parameters[0], "%" SCNu32, &out_freq_hz);
        if (pdiv_ison) {
            configure_pios();
        }
    }
    #ifdef DEBUG
        printf("DEBUG> pdiv_set_freq done.\n");
    #endif
}

void pdiv_set_plength(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    if (parameters.Size() > 0) {
        sscanf(parameters[0], "%" SCNu32, &pulse_len_ns);
        if (pdiv_ison) {
            configure_pios();
        }
    }
}

void pdiv_set_reftcxo(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    ext_ref = false;
    conf_clocks = true;
}

void pdiv_set_refext(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    if (parameters.Size() > 0) {
        sscanf(parameters[0], "%" SCNu32, &ext_clk_freq);
    } else {
        ext_clk_freq = 10000000;
    }
    ext_ref = true;
    conf_clocks = true;
    printf("External ref set @%d\n", ext_clk_freq);
}

void pdiv_set_syncon(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    sync_output = true;
}

void pdiv_set_syncoff(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    sync_output = false;
}

void pdiv_onoff(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    #ifdef DEBUG
        printf("DEBUG> in command pdiv_onoff.\n");
    #endif

    if (parameters.Size() > 0) {
        if (strcmp(parameters[0], "ON") == 0) {
            if (conf_clocks) {
                configure_clocks();
            }
            configure_pios();
            pdiv_ison = true;
        } else if (strcmp(parameters[0], "OFF") == 0) {
            pio_sm_set_enabled(pio0, 0, false);
            pio_sm_set_enabled(pio0, 1, false);
            pdiv_ison = false;
        } else {
            pdiv_instrument.last_error = pdiv_instrument.ErrorCode::UnknownCommand;
            scpi_errorhandler(commands, parameters, interface);
        }
    }
    #ifdef DEBUG
        printf("DEBUG> DONE pdiv_onoff command.\n");
    #endif
}

void pdiv_slide(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    #ifdef DEBUG
        printf("DEBUG> in pdiv_slide, pdiv_ison=%d, param.size=%d\n", pdiv_ison, parameters.Size());
    #endif
    if (pdiv_ison && (parameters.Size() > 0)) {
        uint32_t cycl;
        sscanf(parameters[0], "%" SCNu32, &cycl);
        #ifdef DEBUG
            printf("DEBUG> Sliding %u cycles...\n", cycl);
        #endif
        PIO pio = (sync_output)? pio1 : pio0;
        for (uint32_t i=0; i<cycl; i++) {
            pio_sm_exec_wait_blocking(pio, 0, pio_encode_nop());
        }
        for (uint32_t i=0; i<cycl; i++) {
            pio_sm_exec_wait_blocking(pio, 1, pio_encode_nop());
        }
    }
    #ifdef DEBUG
        printf("DEBUG> DONE pdiv_slide.\n");
    #endif
}

void initialize() {
    poffset = pio_add_program(pio0, &picodiv_program);
    psyncoffset = pio_add_program(pio1, &picodiv_sync_program);

    pdiv_instrument.RegisterCommand("*IDN?", &identify);
    pdiv_instrument.SetCommandTreeBase("CONFigure:DIVider");
        pdiv_instrument.RegisterCommand(":FREQuency", &pdiv_set_freq);
        pdiv_instrument.RegisterCommand(":PULSelength", &pdiv_set_plength);
        pdiv_instrument.RegisterCommand(":SYNC:ON", &pdiv_set_syncon);
        pdiv_instrument.RegisterCommand(":SYNC:OFF", &pdiv_set_syncoff);
    pdiv_instrument.SetCommandTreeBase("CONFigure:REFerence");
        pdiv_instrument.RegisterCommand(":TCXO", &pdiv_set_reftcxo);
        pdiv_instrument.RegisterCommand(":EXT", &pdiv_set_refext);    
    pdiv_instrument.SetCommandTreeBase("DIVider");
        pdiv_instrument.RegisterCommand(":ENABle", &pdiv_onoff);
        pdiv_instrument.RegisterCommand(":SLIDe", &pdiv_slide);
    pdiv_instrument.SetErrorHandler(&scpi_errorhandler);
}

int main() {
    
    sleep_ms(2000);
//    configure_clocks();
    // initialize UART
    initialize();
    
    stdio_init_all();
    stdio_uart_init_full(uart_default, 9600, UART_TX, UART_RX);
//    stdio_init_all();

    while (true) {
        pdiv_instrument.ProcessInput(SCPI_Interface(), "\n");
    }
    
}

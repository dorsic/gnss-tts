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
    Custom UART implementation.
    
    PIO SM usage as following
    PIO0 SM0    - unsynchronized divider for OUT_A
    PIO0 SM1    - unsynchronized divider for OUT_REF
    PIO0 SM2    - picoPET counting clock pulses while Gate open
    PIO1 SM0    - synchronized divider for OUT_A
    PIO1 SM1    - synchronized divider for OUT_REF
    PIO1 SM2    - unused
    PIO0 SM3    - UART RX
    PIO1 SM3    - UART TX
    


    SPDX-License-Identifier: BSD-3-Clause

    Version:
    v1.0 15-Feb-2022  Marek Dorsic (.md)
*/

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/pll.h"
#include "hardware/xosc.h"
#include "picoDIV.pio.h"
#include "picoDIV_sync.pio.h"
#include "picoPET_sp.pio.h"
#include "SCPI_parser/Vrekrer_scpi_parser.h"

#define NODEBUG

#define EI_CLK_GPIO 20            // IntCLK or ExtCLK based on settings
#define TIC_CLK_GPIO 22           // TIC clock, NOTE: It may be also set to intercal XO of the Pico.
#define SYNC_GPIO 22              // DO NOT CHANGE unless you modify the WAIT instruction in picoDIV_Sync PIO
#define SYNC_TUNE 7               // fine align the output PPS to sync PPS, change by 1 is change by 100ns @ 10MHz ExtClk

#define OUT_A_GPIO 17
#define OUT_XO_GPIO 21            // TCXO output
#define OUT_TRIG_GPIO 26           // generated 
#define GATE_GPIO 5

#define CH1_IMP_GPIO 11
#define CH2_IMP_GPIO 10
#define START_S0_GPIO 9
#define START_S1_GPIO 8
#define STOP_S0_GPIO 7
#define STOP_S1_GPIO 6

#define CLK_XO 0
#define CLK_EXT 1
#define CLK_INT 2
#define CLK_REF 3
#define CLK_TRIG 4
#define CH1 5
#define CH2 6

// pulse len should be multiplier of the ref clock period
#define PDIV_MIN_PULSE_LENGTH 250   // 1/pdiv_ref_freq * 3 * 1e9  [ns]  = 250ns @ 12MHz, 300ns @ 10MHz
#define PDIV_MAX_FREQ 2000000       // pdiv_ref_freq/6  = 2MHz @ 12MHz, 1.666MHz @ 10MHz

#define PDIV_SM 0
#define TRIG_SM 1
#define PET_SM 2
// pio0 SM 3 is dedicated to UART_TX
// pio1 SM 3 is dedicated to UART_RX

uint32_t pdiv_ref_freq = XOSC_MHZ * MHZ;
uint8_t pdiv_ref_clk = CLK_XO;
uint32_t pdiv_pulse_len = 10000000;
uint32_t pdiv_pulse_len_ns = 10000000;    // 10 ms
uint32_t pdiv_pulse_perc = 0;
uint32_t pdiv_out_freq_hz = 1;            // 1 Hz ~ 1 PPS
uint8_t pdiv_ext_ref = 0;
uint8_t pdiv_enabled = 0;
uint8_t pdiv_sync_output = 0;

uint8_t tic_start = 4;
uint8_t tic_stop = 5;
uint8_t tic_ch1_imp = 0;
uint8_t tic_ch2_imp = 0;
uint8_t tic_pet_enabled = 0;
uint32_t tic_pet_count = -1;

uint32_t trig_pulse_len_ns = 50000;       // 50 us
uint32_t trig_pulse_len = 50000;
uint32_t trig_out_freq_hz = 1000;         // 1 Hz ~ 1 PPS
uint8_t trig_enabled = 0;
uint8_t trig_pulse_perc = 0;
uint8_t trig_sync_output = 0;

uint8_t conf_clocks = 1;
int offset = -1;
int syncoffset = -1;
int petoffset = -1;

SCPI_Parser cm_instrument;

void configure_clocks() {
    // clk_sys is driving the PIO
    // clk_peri is driving the UART (baudrate)
    #ifdef DEBUG
        printf("DEBUG> Configuring clocks... ext_ref=%d\n", ext_ref);
    #endif    
    if (pdiv_ext_ref) {
        uint8_t source_gpio = (pdiv_ref_clk == CLK_EXT || pdiv_ref_clk == CLK_INT)? EI_CLK_GPIO : TIC_CLK_GPIO;
        clock_configure_gpin(clk_ref, source_gpio, pdiv_ref_freq, pdiv_ref_freq);
        clock_configure_gpin(clk_sys, source_gpio, pdiv_ref_freq, pdiv_ref_freq);
        clock_configure_gpin(clk_peri, source_gpio, pdiv_ref_freq, pdiv_ref_freq);
        xosc_disable();
    } else {
        xosc_init();
        clock_configure(clk_ref, CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC, 0, pdiv_ref_freq, pdiv_ref_freq);
        clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, pdiv_ref_freq, pdiv_ref_freq);
        clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, pdiv_ref_freq, pdiv_ref_freq);
        clock_gpio_init(OUT_XO_GPIO, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 1);
    }
    clock_gpio_init(PICO_DEFAULT_LED_PIN, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, MHZ);
    // stop the clk_sys PPL
    pll_deinit(pll_sys);
    
//    stdio_uart_init_full(uart_default, 9600, UART_TX, UART_RX);
    cm_instrument.Initialize();
    conf_clocks = 0;
    #ifdef DEBUG
        printf("DEBUG> DONE clocks configuration\n", ext_ref);
    #endif
}

void _blink_pin_forever(uint sm, uint pin, uint32_t total_clk, uint32_t pulse_clk, uint8_t sync) {
    #ifdef DEBUG
        printf("DEBUG> Blink pin forever pdiv_ison=%d, sm=%d, pin=%d, sync=%d, total_clk=%u\n", pdiv_ison, sm, pin, sync, total_clk);
    #endif

    PIO pio;
    uint32_t st = 0;

    if (sync) {
        pio = pio1;
        pio_sm_init(pio, sm, syncoffset, NULL);
        picodiv_sync_program_init(pio, sm, syncoffset, pin);
        st = SYNC_TUNE;
    } else {
        pio = pio0;
        pio_sm_init(pio, sm, offset, NULL);
        picodiv_program_init(pio, sm, offset, pin);
    }
    pio_sm_set_enabled(pio, sm, 1);

    pio->txf[sm] = pulse_clk - 3;      
    pio->txf[sm] = total_clk - pulse_clk - 3;       // write number off LOW clock cycles to PIO register
    pio->txf[sm] = total_clk - st;                  // write the first sync period clock cycles to PIO register
    #ifdef DEBUG
        printf("DEBUG> DONE Blink pin forever.\n");
    #endif
}

uint32_t multiplier(const char *str) {
    switch (str[strlen(str)-1]) {
        case 'k':
        case 'K':
            return 1000;
        case 'm':
        case 'M':
            return 1000000;
        case '%':
            return 100;
    }
    return 1;
}

uint32_t _pulse_len_ns(uint8_t isperc, uint32_t value, uint32_t clk_hz, uint32_t ref_clk_hz) {
    uint32_t ret = 0;
    if (isperc) {
        ret =  (uint32_t)(((double)1.0E7*(double)value)/(double)clk_hz);
    } else {
        ret = value;
    }
    if (ret < (uint32_t)3.0e9/ref_clk_hz) {
        ret = (uint32_t)(3.0e9/ref_clk_hz);
    }
    return ret;
}

void _pdiv_configure_pios() {
    #ifdef DEBUG
        printf("DEBUG> Configuring divider PIOs sync_output=%d\n", sync_output);
    #endif

    // assign output pins and outout frequency to PIOs               
    uint32_t tot_clk = div_u32u32(pdiv_ref_freq, (uint32_t) pdiv_out_freq_hz);
    uint32_t nomin = (uint32_t)pdiv_pulse_len_ns * (pdiv_ref_freq/MHZ);
    uint32_t pls_clk = div_u32u32(nomin, (uint32_t)1000);
    _blink_pin_forever(PDIV_SM, OUT_A_GPIO, tot_clk, pls_clk, pdiv_sync_output);
    #ifdef DEBUG
        printf("DEBUG> DONE divider PIOs configuration.\n");
    #endif
}

void _trig_configure_pios() {
    #ifdef DEBUG
        printf("DEBUG> Configuring trigger PIOs \n");
    #endif

    // assign output pins and outout frequency to PIOs               
    uint32_t tot_clk = div_u32u32(pdiv_ref_freq, (uint32_t) trig_out_freq_hz);
    uint32_t nomin = (uint32_t)trig_pulse_len_ns * (pdiv_ref_freq/MHZ);
    uint32_t pls_clk = div_u32u32(nomin, (uint32_t)1000);
    _blink_pin_forever(TRIG_SM, OUT_TRIG_GPIO, tot_clk, pls_clk, trig_sync_output);
    #ifdef DEBUG
        printf("DEBUG> DONE trigger PIO configuration.\n");
    #endif
}

void _pdiv_enable(uint8_t on) {
    if (on) {
        if (conf_clocks)
            configure_clocks();
        _pdiv_configure_pios();
        pdiv_enabled = 1;
    } else {
        pio_sm_set_enabled(pio0, PDIV_SM, 0);
        pio_sm_set_enabled(pio1, PDIV_SM, 0);
        pdiv_enabled = 0;
    }
}

void _pdiv_delay(uint32_t n_clk_cycles) {
    if (pdiv_enabled) {
        PIO pio = (pdiv_sync_output)? pio1 : pio0;
        pio_sm_exec(pio, PDIV_SM, pio_encode_delay(n_clk_cycles));
//        for (uint32_t i=0; i<n_clk_cycles; i++) {
//            pio_sm_exec_wait_blocking(pio, PDIV_SM, pio_encode_nop());
//        }
    }
}

void _trig_enable(uint8_t on) {
    if (on) {
        if (conf_clocks)
            configure_clocks();
        _trig_configure_pios();
        trig_enabled = 1;
    } else {
        pio_sm_set_enabled(pio0, TRIG_SM, 0);
        pio_sm_set_enabled(pio1, TRIG_SM, 0);
        trig_enabled = 0;
    }
}

void _trig_delay(uint32_t n_clk_cycles) {
    if (trig_enabled) {
        PIO pio = (trig_sync_output)? pio1 : pio0;
        pio_sm_exec(pio, TRIG_SM, pio_encode_delay(n_clk_cycles));
//        for (uint32_t i=0; i<n_clk_cycles; i++) {
//            pio_sm_exec_wait_blocking(pio, TRIG_SM, pio_encode_nop());
//        }
    }
}

void _pdiv_set_ref_clk(uint8_t clk, uint32_t freq) {
    conf_clocks = (clk != pdiv_ref_clk) || freq != (pdiv_ref_freq);
    if (conf_clocks) {
        pdiv_ext_ref = 1;
        pdiv_ref_clk = clk;
        pdiv_ref_freq = freq;
        if (pdiv_ref_clk == CLK_XO) {
            pdiv_ref_freq = XOSC_MHZ * MHZ;
            pdiv_ext_ref = 0;
        }
        if (pdiv_enabled) {
            pdiv_pulse_len_ns = _pulse_len_ns(pdiv_pulse_perc, pdiv_pulse_len, pdiv_out_freq_hz, pdiv_ref_freq);            
            _pdiv_configure_pios();
        }
        if (trig_enabled) {
            trig_pulse_len_ns = _pulse_len_ns(trig_pulse_perc, trig_pulse_len, trig_out_freq_hz, pdiv_ref_freq);
            _trig_configure_pios();
        }
        configure_clocks();
    }
}

void _tic_set_impedance(uint8_t channel, uint8_t impedance) {
    // impedance either 50 for 50 Ohm or 100 for 1M Ohm
    if (channel == CH1 && impedance == 50) {
        gpio_put(CH1_IMP_GPIO, 1);
        tic_ch1_imp = 50;
    } else if (channel == CH1 && impedance == 100) {
        gpio_put(CH1_IMP_GPIO, 0);
        tic_ch1_imp = 100;
    } else if (channel == CH2 && impedance == 50) {
        gpio_put(CH2_IMP_GPIO, 1);
        tic_ch2_imp = 50;
    } else if (channel == CH2 && impedance == 100) {
        gpio_put(CH2_IMP_GPIO, 0);
        tic_ch2_imp = 100;
    }
}

void _tic_pet_enable(uint8_t on) {
    if (on) {
        picopet_sp_program_init(pio0, PET_SM, petoffset, GATE_GPIO);
        pio_sm_set_enabled(pio0, PET_SM, 1);
    } else {
        pio_sm_set_enabled(pio0, PET_SM, 0);
    }
    tic_pet_enabled = on;
}

void _ack(SCPI_I interface) {
    interface.putchars("OK\n");
}

void _source_name(char* buf, uint8_t source) {
    switch (source)
    {
    case CH1:
        strcpy(buf, "CH1");
        break;
    case CH2:
        strcpy(buf, "CH2");
        break;
    case CLK_TRIG:
        strcpy(buf, "TRIG");
        break;
    case CLK_REF:
        strcpy(buf, "REF");
        break;
    case CLK_XO:
        strcpy(buf, "XO");
        break;
    case CLK_EXT:
        strcpy(buf, "EXT");
        break;
    case CLK_INT:
        strcpy(buf, "INT");
        break;    
    default:
        break;
    }
}

void scpi_errorhandler(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
  switch(cm_instrument.last_error){
    case cm_instrument.ErrorCode::BufferOverflow: 
      interface.putchars("Buffer overflow error\n");
      break;
    case cm_instrument.ErrorCode::Timeout:
      interface.putchars("Communication timeout error\n");
      break;
    case cm_instrument.ErrorCode::UnknownCommand:
      interface.putchars("Unknown command received\n");
      break;
    case cm_instrument.ErrorCode::MissingParameter:
      interface.putchars("Missing parameter\n");
      break;
    case cm_instrument.ErrorCode::InvalidParameter:
      interface.putchars("Invalid parameter value received\n");
      break;  
    case cm_instrument.ErrorCode::UnknownSource:
      interface.putchars("Unknown source for command received\n");
      break;        
    case cm_instrument.ErrorCode::NoError:
      interface.putchars("No Error\n");
      break;
  }
  cm_instrument.last_error = cm_instrument.ErrorCode::NoError;    
}

void identify(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    interface.putchars("PicoDIV in Counter Module v2.0, v1.0\n");
}

void pdiv_set_ref(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    if (parameters.Size() > 1) {
        uint32_t freq = atoi(parameters[1]) * multiplier(parameters[1]);
        if (strcmp(parameters[0], "INT") == 0) {
            _pdiv_set_ref_clk(CLK_INT, freq);
            _ack(interface);
        } else if (strcmp(parameters[0], "EXT") == 0) {
            _pdiv_set_ref_clk(CLK_EXT, freq);
            _ack(interface);
        } else if (strcmp(parameters[0], "XO") == 0) {
            _pdiv_set_ref_clk(CLK_XO, 0);
            _ack(interface);
        } else {
            cm_instrument.last_error = cm_instrument.ErrorCode::UnknownSource;
            scpi_errorhandler(commands, parameters, interface);
        }
    } else if (parameters.Size() > 0) {
        if (strcmp(parameters[0], "XO") == 0) {
            _pdiv_set_ref_clk(CLK_XO, 0);
            _ack(interface);
        } else {
            cm_instrument.last_error = cm_instrument.ErrorCode::UnknownSource;
            scpi_errorhandler(commands, parameters, interface);
        }
    } else {
        cm_instrument.last_error = cm_instrument.ErrorCode::MissingParameter;
        scpi_errorhandler(commands, parameters, interface);
    }
}

void pdiv_get_ref(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    char str[6];
    _source_name(str, pdiv_ref_clk);
    sprintf(str, "%s\n", str);
    interface.putchars(str);
}

void pdiv_get_reffreq(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    char str[12];
    if (conf_clocks) {
        // return negative value, if the freq is waiting to be applied
        sprintf(str, "%d\n", -pdiv_ref_freq);
    } else {
        sprintf(str, "%d\n", pdiv_ref_freq);
    }
    interface.putchars(str);
}

void pdiv_enable(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    #ifdef DEBUG
        printf("DEBUG> in command pdiv_onoff.\n");
    #endif

    if (parameters.Size() > 0) {
        if (strcmp(parameters[0], "ON") == 0 || strcmp(parameters[0], "1") == 0) {
            _pdiv_enable(1);
            _ack(interface);
        } else if (strcmp(parameters[0], "OFF") == 0 || strcmp(parameters[0], "0") == 0) {
            _pdiv_enable(0);
            _ack(interface);
        } else {
            cm_instrument.last_error = cm_instrument.ErrorCode::InvalidParameter;
            scpi_errorhandler(commands, parameters, interface);
        }
    } else {
        cm_instrument.last_error = cm_instrument.ErrorCode::MissingParameter;
        scpi_errorhandler(commands, parameters, interface);
    }
    #ifdef DEBUG
        printf("DEBUG> DONE pdiv_onoff command.\n");
    #endif
}

void pdiv_delay(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    #ifdef DEBUG
        printf("DEBUG> in pdiv_slide, pdiv_enabled=%d, param.size=%d\n", pdiv_enabled, parameters.Size());
    #endif
    if (parameters.Size() > 0) {
        uint32_t cycl = atoi(parameters[0]) * multiplier(parameters[0]);
        #ifdef DEBUG
            printf("DEBUG> Sliding %u cycles...\n", cycl);
        #endif
        _pdiv_delay(cycl);
        _ack(interface);
    } else {
        cm_instrument.last_error = cm_instrument.ErrorCode::MissingParameter;
        scpi_errorhandler(commands, parameters, interface);
    }
    #ifdef DEBUG
        printf("DEBUG> DONE pdiv_slide.\n");
    #endif
}

void pdiv_set_freq(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    #ifdef DEBUG
        printf("DEBUG> in command pdiv_set_freq command. Param size=%d\n", parameters.Size());
    #endif
    if (parameters.Size() > 0) {
        pdiv_out_freq_hz = atoi(parameters[0]) * multiplier(parameters[0]);
        if (pdiv_pulse_perc) {
            pdiv_pulse_len_ns = _pulse_len_ns(pdiv_pulse_perc, pdiv_pulse_len, pdiv_out_freq_hz, pdiv_ref_freq);            
        }
        if (pdiv_enabled) {
            _pdiv_enable(1);
        }
        _ack(interface);
    } else {
        cm_instrument.last_error = cm_instrument.ErrorCode::MissingParameter;
        scpi_errorhandler(commands, parameters, interface);
    }
    #ifdef DEBUG
        printf("DEBUG> pdiv_set_freq done.\n");
    #endif
}

void pdiv_set_plength(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    if (parameters.Size() > 0) {
        pdiv_pulse_perc = (multiplier(parameters[0])==100);
        pdiv_pulse_len = atoi(parameters[0]);
        pdiv_pulse_len_ns = _pulse_len_ns(pdiv_pulse_perc, pdiv_pulse_len, pdiv_out_freq_hz, pdiv_ref_freq);
        if (pdiv_enabled) {
            _pdiv_enable(1);
        }
        _ack(interface);
    } else {
        cm_instrument.last_error = cm_instrument.ErrorCode::MissingParameter;
        scpi_errorhandler(commands, parameters, interface);
    }
}

void pdiv_set_sync_enable(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    if (parameters.Size() > 0) {
        if (strcmp(parameters[0], "ON") == 0 || strcmp(parameters[0], "1") == 0) {
            pdiv_sync_output = 1;
            if (pdiv_enabled) {
                _pdiv_enable(0);
                _pdiv_enable(1);
            }
            _ack(interface);
        } else if (strcmp(parameters[0], "OFF") == 0 || strcmp(parameters[0], "0") == 0) {
            pdiv_sync_output = 0;
            _ack(interface);
        } else {
            cm_instrument.last_error = cm_instrument.ErrorCode::InvalidParameter;
            scpi_errorhandler(commands, parameters, interface);
        }
    } else {
        cm_instrument.last_error = cm_instrument.ErrorCode::MissingParameter;
        scpi_errorhandler(commands, parameters, interface);
    }
}

void trig_enable(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    #ifdef DEBUG
        printf("DEBUG> in command pdiv_onoff.\n");
    #endif

    if (parameters.Size() > 0) {
        if (strcmp(parameters[0], "ON") == 0 || strcmp(parameters[0], "1") == 0) {
            _trig_enable(1);
            _ack(interface);
        } else if (strcmp(parameters[0], "OFF") == 0 || strcmp(parameters[0], "0") == 0) {
            _trig_enable(0);
            _ack(interface);
        } else {
            cm_instrument.last_error = cm_instrument.ErrorCode::InvalidParameter;
            scpi_errorhandler(commands, parameters, interface);
        }
    } else {
        cm_instrument.last_error = cm_instrument.ErrorCode::MissingParameter;
        scpi_errorhandler(commands, parameters, interface);
    }
    #ifdef DEBUG
        printf("DEBUG> DONE pdiv_onoff command.\n");
    #endif
}

void trig_delay(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    #ifdef DEBUG
        printf("DEBUG> in pdiv_slide, pdiv_ison=%d, param.size=%d\n", pdiv_ison, parameters.Size());
    #endif
    if (parameters.Size() > 0) {
        uint32_t cycl = atoi(parameters[0])*multiplier(parameters[0]);
        #ifdef DEBUG
            printf("DEBUG> Sliding %u cycles...\n", cycl);
        #endif
        _trig_delay(cycl);
        _ack(interface);
    } else {
        cm_instrument.last_error = cm_instrument.ErrorCode::MissingParameter;
        scpi_errorhandler(commands, parameters, interface);
    }
    #ifdef DEBUG
        printf("DEBUG> DONE pdiv_slide.\n");
    #endif
}

void trig_set_freq(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    #ifdef DEBUG
        printf("DEBUG> in command pdiv_set_freq command. Param size=%d\n", parameters.Size());
    #endif
    if (parameters.Size() > 0) {
        trig_out_freq_hz = atoi(parameters[0])*multiplier(parameters[0]);
        trig_pulse_len_ns = _pulse_len_ns(trig_pulse_perc, trig_pulse_len, trig_out_freq_hz, pdiv_ref_freq);
        if (trig_enabled) {
            _trig_enable(1);
        }
        _ack(interface);
    } else {
        cm_instrument.last_error = cm_instrument.ErrorCode::MissingParameter;
        scpi_errorhandler(commands, parameters, interface);
    }
    #ifdef DEBUG
        printf("DEBUG> pdiv_set_freq done.\n");
    #endif
}

void trig_set_plength(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    if (parameters.Size() > 0) {
        uint32_t m = multiplier(parameters[0]);
        trig_pulse_perc = m == 100;
        if (!trig_pulse_perc) 
            trig_pulse_len = atoi(parameters[0]) * m;
        else
            trig_pulse_len = atoi(parameters[0]);
        trig_pulse_len_ns = _pulse_len_ns(trig_pulse_perc, trig_pulse_len, trig_out_freq_hz, pdiv_ref_freq);
        if (trig_enabled) {
            _trig_enable(1);
        }
        _ack(interface);
    } else {
        cm_instrument.last_error = cm_instrument.ErrorCode::MissingParameter;
        scpi_errorhandler(commands, parameters, interface);
    }
}

void trig_set_sync_enable(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    if (parameters.Size() > 0) {
        if (strcmp(parameters[0], "ON") == 0 || strcmp(parameters[0], "1") == 0) {
            trig_sync_output = 1;
            if (trig_enabled) {
                _trig_enable(0);
                _trig_enable(1);
            }
            _ack(interface);
        } else if (strcmp(parameters[0], "OFF") == 0 || strcmp(parameters[0], "0") == 0) {
            trig_sync_output = 0;
            _ack(interface);
        } else {
            cm_instrument.last_error = cm_instrument.ErrorCode::InvalidParameter;
            scpi_errorhandler(commands, parameters, interface);
        }
    } else {
        cm_instrument.last_error = cm_instrument.ErrorCode::MissingParameter;
        scpi_errorhandler(commands, parameters, interface);
    }
}

void tic_set_start(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    if (parameters.Size() > 0) {
        if (strcmp(parameters[0], "CH1") == 0) {
//            interface.putchars("setting TIC start to CH1\n");
            tic_start = CH1;
            gpio_put(START_S1_GPIO, 0);
            gpio_put(START_S0_GPIO, 0);
            _ack(interface);
        } else if (strcmp(parameters[0], "CH2") == 0) {
//            interface.putchars("setting TIC start to CH2\n");
            tic_start = CH2;
            gpio_put(START_S1_GPIO, 1);
            gpio_put(START_S0_GPIO, 0);
            _ack(interface);
        } else if (strcmp(parameters[0], "REF") == 0) {
//            interface.putchars("setting TIC start to REF\n");
            tic_start = CLK_REF;
            gpio_put(START_S1_GPIO, 0);
            gpio_put(START_S0_GPIO, 1);
            _ack(interface);
        } else if (strcmp(parameters[0], "TRIG") == 0) {
//            interface.putchars("setting TIC start to TRIG\n");
            tic_start = CLK_TRIG;
            gpio_put(START_S1_GPIO, 1);
            gpio_put(START_S0_GPIO, 1);
            _ack(interface);
        } else {
            cm_instrument.last_error = cm_instrument.ErrorCode::UnknownSource;
            scpi_errorhandler(commands, parameters, interface);
        }
    } else {
        cm_instrument.last_error = cm_instrument.ErrorCode::MissingParameter;
        scpi_errorhandler(commands, parameters, interface);
    }    
}

void tic_set_stop(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    if (parameters.Size() > 0) {
        if (strcmp(parameters[0], "CH1") == 0) {
//            interface.putchars("setting TIC stop to CH1\n");
            tic_stop = CH1;
            gpio_put(STOP_S1_GPIO, 0);
            gpio_put(STOP_S0_GPIO, 0);
            _ack(interface);
        } else if (strcmp(parameters[0], "CH2") == 0) {
//            interface.putchars("setting TIC stop to CH2\n");
            tic_stop = CH2;
            gpio_put(STOP_S1_GPIO, 1);
            gpio_put(STOP_S0_GPIO, 0);
            _ack(interface);
        } else if (strcmp(parameters[0], "TRIG") == 0) {
//            interface.putchars("setting TIC stop to TRIG\n");
            tic_stop = CLK_TRIG;
            gpio_put(STOP_S1_GPIO, 0);
            gpio_put(STOP_S0_GPIO, 1);
            _ack(interface);
        } else if (strcmp(parameters[0], "REF") == 0) {
//            interface.putchars("setting TIC stop to REF\n");
            tic_stop = CLK_REF;
            gpio_put(STOP_S1_GPIO, 1);
            gpio_put(STOP_S0_GPIO, 1);
            _ack(interface);
        } else {
            cm_instrument.last_error = cm_instrument.ErrorCode::UnknownSource;
            scpi_errorhandler(commands, parameters, interface);
        }
    } else {
        cm_instrument.last_error = cm_instrument.ErrorCode::MissingParameter;
        scpi_errorhandler(commands, parameters, interface);
    }    
}

void tic_set_impedance(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    uint8_t imp = 0;
    uint8_t ch = 0;
//    sprintf(buf, "parameters count %d, parameter0 %s", parameters.Size(), parameters[0]);
//    interface.putchars(buf);
    if (parameters.Size() > 1) {
        if (strcmp(parameters[0], "CH1") == 0) {
            ch = CH1;
        } else if (strcmp(parameters[0], "CH2") == 0) {
            ch = CH2;            
        } else {
            cm_instrument.last_error = cm_instrument.ErrorCode::UnknownSource;
            scpi_errorhandler(commands, parameters, interface);
            return;
        }

        if (strcmp(parameters[1], "LOW") == 0 || strcmp(parameters[1], "50") == 0 || strcmp(parameters[1], "0") == 0) {
            imp = 50;
        } else if (strcmp(parameters[1], "HIGH") == 0 || strcmp(parameters[1], "1M") == 0 || strcmp(parameters[1], "1m") == 0 || strcmp(parameters[1], "1") == 0) {
            imp = 100;
        } else {
            cm_instrument.last_error = cm_instrument.ErrorCode::InvalidParameter;
            scpi_errorhandler(commands, parameters, interface);
            return;
        }

        if (ch > 0 && imp > 0) {
            _tic_set_impedance(ch, imp);
            _ack(interface);
        }
    } else {
        cm_instrument.last_error = cm_instrument.ErrorCode::MissingParameter;
        scpi_errorhandler(commands, parameters, interface);
    }
}

void tic_set_pet_enable(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    if (parameters.Size() > 0) {
        if (strcmp(parameters[0], "ON") == 0 || strcmp(parameters[0], "1") == 0) {
            _tic_pet_enable(1);
            _ack(interface);
        } else if (strcmp(parameters[0], "OFF") == 0 || strcmp(parameters[0], "0") == 0) {
            _tic_pet_enable(0);
            _ack(interface);
        } else {
            cm_instrument.last_error = cm_instrument.ErrorCode::InvalidParameter;
            scpi_errorhandler(commands, parameters, interface);
        }
    } else {
        cm_instrument.last_error = cm_instrument.ErrorCode::MissingParameter;
        scpi_errorhandler(commands, parameters, interface);
    }
}

void tic_get_start(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    char str[6];
    _source_name(str, tic_start);
    sprintf(str, "%s\n", str);
    interface.putchars(str);
}

void tic_get_stop(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    char str[6];
    _source_name(str, tic_stop);
    sprintf(str, "%s\n", str);
    interface.putchars(str);
}

void tic_get_pet_count(SCPI_C commands, SCPI_P parameters, SCPI_I interface) {
    char str[16];
    sprintf(str, "%d\n", tic_pet_count);
    interface.putchars(str);
    tic_pet_count = -1;
}

void initialize() {
    if (pio_can_add_program(pio0, &picodiv_program))
        offset = pio_add_program(pio0, &picodiv_program);
    else 
        printf("Unable to load program (picoDIV) to PIO_0");
    if (pio_can_add_program(pio1, &picodiv_sync_program))
        syncoffset = pio_add_program(pio1, &picodiv_sync_program);
    else 
        printf("Unable to load program (picoDIV_sync) to PIO_1");
    if (pio_can_add_program(pio0, &picopet_sp_program))
        petoffset = pio_add_program(pio0, &picopet_sp_program);
    else 
        printf("Unable to load program (picoPET) to PIO_0");

    cm_instrument.RegisterCommand("*IDN?", &identify);
    cm_instrument.SetCommandTreeBase(":DIVider");
        cm_instrument.RegisterCommand(":REFerence", &pdiv_set_ref);         // params <INT | EXT | XO> <freq in Hz>
        cm_instrument.RegisterCommand(":REFerence?", &pdiv_get_ref);
        cm_instrument.RegisterCommand(":REFerence:FREQuency?", &pdiv_get_reffreq);        
        cm_instrument.RegisterCommand(":ENABle", &pdiv_enable);             // param <ON | 1> | <OFF | 0>
        cm_instrument.RegisterCommand(":DELay", &pdiv_delay);               // param delay in clock cycles
        cm_instrument.RegisterCommand(":FREQuency", &pdiv_set_freq);        // param freq in Hz
        cm_instrument.RegisterCommand(":PULSelength", &pdiv_set_plength);   // param <high portion in ns> | <p%>
        cm_instrument.RegisterCommand(":SYNChronization", &pdiv_set_sync_enable);        // param <ON | 1> | <OFF | 0>
    cm_instrument.SetCommandTreeBase(":DIVider:TRIGger");        
        cm_instrument.RegisterCommand(":ENABle", &trig_enable);             // param <ON | 1> | <OFF | 0>
        cm_instrument.RegisterCommand(":DELay", &trig_delay);               // param delay in clock cycles
        cm_instrument.RegisterCommand(":FREQuency", &trig_set_freq);        // param freq in Hz
        cm_instrument.RegisterCommand(":PULSelength", &trig_set_plength);   // param <high portion in ns> | <p%>
        cm_instrument.RegisterCommand(":SYNChronization", &trig_set_sync_enable);        // param <ON | 1> | <OFF | 0>
    cm_instrument.SetCommandTreeBase(":TIC");
        cm_instrument.RegisterCommand(":STARt", &tic_set_start);        // param <CH1 | CH2 | TRIG | REF>
        cm_instrument.RegisterCommand(":STARt?", &tic_get_start);
        cm_instrument.RegisterCommand(":STOP", &tic_set_stop);          // param <CH1 | CH2 | TRIG | REF>
        cm_instrument.RegisterCommand(":STOP?", &tic_get_stop);
        cm_instrument.RegisterCommand(":IMPedance", &tic_set_impedance);      // param <50 | LOW> | <1M | HIGH>
        cm_instrument.RegisterCommand(":PET:ENABle", &tic_set_pet_enable); // param <ON | 1> | <OFF | 0>
        cm_instrument.RegisterCommand(":PET?", &tic_get_pet_count);
    cm_instrument.SetErrorHandler(&scpi_errorhandler);

 //   gpio_init(SYNC_GPIO);
 //   gpio_set_dir(SYNC_GPIO, GPIO_IN);
 //   gpio_pull_down(SYNC_GPIO);

    cm_instrument.Initialize();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, 1);
    gpio_init(GATE_GPIO);
    gpio_set_dir(GATE_GPIO, 0);
    gpio_pull_down(GATE_GPIO);

    gpio_init(OUT_A_GPIO);
    gpio_set_dir(OUT_A_GPIO, 1);
    gpio_init(OUT_TRIG_GPIO);
    gpio_set_dir(OUT_TRIG_GPIO, 1);

    gpio_init(CH1_IMP_GPIO);
    gpio_set_dir(CH1_IMP_GPIO, 1);
    gpio_put(CH1_IMP_GPIO, 0);      // HIGH impedance
    gpio_init(CH2_IMP_GPIO);
    gpio_set_dir(CH2_IMP_GPIO, 1);
    gpio_put(CH2_IMP_GPIO, 0);      // HIGH impedance
    gpio_init(START_S0_GPIO);
    gpio_set_dir(START_S0_GPIO, 1);
    gpio_put(START_S0_GPIO, 0);
    gpio_init(START_S1_GPIO);
    gpio_set_dir(START_S1_GPIO, 1);
    gpio_put(START_S1_GPIO, 0);     // CH1
    gpio_init(STOP_S0_GPIO);
    gpio_set_dir(STOP_S0_GPIO, 1);
    gpio_put(STOP_S0_GPIO, 0);
    gpio_init(STOP_S1_GPIO);
    gpio_set_dir(STOP_S1_GPIO, 1);
    gpio_put(STOP_S1_GPIO, 1);      // CH2
}

void core1_main() {
    //pet_counting
    while (1) {
        while (tic_pet_enabled) {
            if (!pio_sm_is_rx_fifo_empty(pio0, PET_SM)) {
                tic_pet_count = pio_sm_get(pio0, PET_SM);            // read the register from ASM code
                tic_pet_count = (~tic_pet_count)+2;
            }
        }
    }
}

int main() {
    
    sleep_ms(3000);

    printf("Counter Module v2");
    initialize();
    _pdiv_set_ref_clk(CLK_XO, XOSC_MHZ*MHZ);
    configure_clocks();
    stdio_init_all();
    
    multicore_launch_core1(core1_main);
    while (1) {
        cm_instrument.ProcessInput(SCPI_Interface(), "\n");
    }
}

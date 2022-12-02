# Time Transfer System (TTS). ds

Time interval counter based on TI TDC7200 chip(s) witch GNSS modules for precise time standard comparisons (Ublox RCB-F9T, RCB-M8T).

![Image of GpsCounter-components](/GpsCounter-components.svg)

The Frequency counter will have 4 modes:
- Mode 1 for intervals from 12ns to 500ns, when START and STOP are connected directly to TDC7200 which is also setup to mode 1 measurement,
- Mode 2 for intervals from 200ns to 5ms, when START and STOP are connected directly to TDC7200 which is also setup to mode 2 measurement,
- Mode 3 for intervals above 5ms, when 2 interleaved measurements with TDC7200 are taken and combined. First measures time from START impuls to second clock impulse and the second from STOP impulse to to second clock impulse. Inbetween clock impulses are counted by the Pico (see Mode 4)
- Mode 4 for impulse counting with the RPi Pico. With 10MHz clk_sys it has resolution of 100ns (see [PicPET](http://leapsecond.com/pic/picpet2.htm))


![CM v3.0](/PCB/CounterModule/CounterModule%20v3.0/PCB_PicoCounter_v3.svg)

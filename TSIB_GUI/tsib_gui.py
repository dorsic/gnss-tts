from curses import baudrate
import tkinter as tk
import tkinter.font as TkFont
from tkinter import *
from tkinter import ttk

from time import sleep
from serial import Serial
import threading

SERIALPORT = '/dev/ttyS0'
SERIALPORT = '/dev/cu.usbmodem141101'
SERIALPORT = '/dev/cu.usbmodem143101'
BAUDRATE = 115200

class TSIB2_Panel(Frame):

    SELECT_COLOR = '#006000'
    RUN_COLOR = '#600000'
    SELECT_TEXT = '#ffffff'
    BLACK = '#000000'
    BORDERWIDTH = 4

    def __init__(self, serialport=SERIALPORT, baudrate=BAUDRATE):
        tk.Frame.__init__(self)
        self._ref = StringVar()
        self._trig = StringVar()
        self._start = StringVar()
        self._stop = StringVar()
        self._startRising = IntVar()
        self._stopRising = IntVar()
        self._mode = IntVar()
        self._nstops = IntVar()
        self._calPeriods = IntVar()
        self._ticenable = IntVar()
        self.sport = None

        self.grid()
        self.master.title("Time System Integration Board cm 2")     

        self.lblText = Label(self, text='. ... ... ..., ... ns', bg=self.SELECT_COLOR, fg=self.SELECT_TEXT,
            borderwidth=self.BORDERWIDTH, padx=10, anchor=S+E, font=TkFont.Font(family='Courier', size=32, weight='bold', slant='roman'))
        self.lblText.grid(row=0, column=2, rowspan=2, columnspan=12, sticky= W+E+N+S)
        self.lblTextUnits = Label(self, text='s      ms      us      ns      ps       ', anchor=E, bg=self.SELECT_COLOR, fg=self.SELECT_TEXT,
            borderwidth=self.BORDERWIDTH, padx=10, font=TkFont.Font(family='Courier', size=16, weight='normal', slant='roman'))
        self.lblTextUnits.grid(row=2, column=2, columnspan=12, sticky= W+E+N+S)

        self.lfRef = LabelFrame(self, text='Reference', padx=10, pady=5)
        self.lfRef.grid(row=1, column=0, rowspan=2, columnspan=2)
        self.btnExtRef = Checkbutton(self.lfRef, text="External 10MHz", var=self._ref, onvalue="EXT", command=self.set_ref, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=1)
        self.btnIntRef = Checkbutton(self.lfRef, text="Internal 12MHz", var=self._ref, onvalue="XO", command=self.set_ref, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=1)
        self.btnExtRef.grid(row=1, column=0, columnspan=2, sticky = W+E+N+S)
        self.btnIntRef.grid(row=2, column=0, columnspan=2, sticky = W+E+N+S)

        self.lfTrig = LabelFrame(self, text='Trigger', padx=10, pady=5)
        self.btnTrig = Checkbutton(self.lfTrig, text="OFF", var=self._trig, offvalue="OFF", onvalue="ON", command=self.set_trig, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR)
        self.btnTrig.grid(row=5, column=0, columnspan=2, sticky = W+E+N+S)
        self.lbTrigFreqLbl = Label(self.lfTrig, text='f (Hz):')
        self.eTrigFreq = Entry(self.lfTrig, width=6)
        self.lfTrig.grid(row=4, column=0, rowspan=2, columnspan=2)
        self.btnTrig.grid(row=4, column=0, rowspan=1, columnspan=2)
        self.lbTrigFreqLbl.grid(row=5, column=0, rowspan=1, columnspan=1)
        self.eTrigFreq.grid(row=5, column=1, rowspan=1, columnspan=1)
        self.eTrigFreq.insert(0, '1')
    
        self.lfStart = LabelFrame(self, text='Start', padx=10, pady=5)
        self.lfStart.grid(row=0, column=16, rowspan=4, columnspan=4)
        self.btnStartCh1 = Checkbutton(self.lfStart, text="Ch1", var=self._start, onvalue="CH1", command=self.set_start, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=1)
        self.btnStartCh2 = Checkbutton(self.lfStart, text="Ch2", var=self._start, onvalue="CH2", command=self.set_start, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=1)
        self.btnStartTrig = Checkbutton(self.lfStart, text="TRIG", var=self._start, onvalue="TRIG", command=self.set_start, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=1)
        self.btnStartRef = Checkbutton(self.lfStart, text="REF", var=self._start, onvalue="REF", command=self.set_start, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=1)
        self.btnStartRising = Checkbutton(self.lfStart, text="Rising", var=self._startRising, command=self.set_startRising, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=1)

        self.btnStartCh1.grid(row=0, column=16, columnspan=2, sticky = W+E+N+S)
        self.btnStartCh2.grid(row=1, column=16, columnspan=2, sticky = W+E+N+S)
        self.btnStartTrig.grid(row=0, column=18, columnspan=2, sticky = W+E+N+S)
        self.btnStartRef.grid(row=1, column=18, columnspan=2, sticky = W+E+N+S)
        self.btnStartRising.grid(row=2, column=16, columnspan=4, sticky = W+E+N+S)

        self.lfStop = LabelFrame(self, text='Stop', padx=10, pady=5)
        self.lfStop.grid(row=0, column=20, rowspan=4, columnspan=4)
        self.btnStopCh1 = Checkbutton(self.lfStop, text="Ch1", var=self._stop, onvalue="CH1", command=self.set_stop, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=1)
        self.btnStopCh2 = Checkbutton(self.lfStop, text="Ch2", var=self._stop, onvalue="CH2", command=self.set_stop, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=1)
        self.btnStopTrig = Checkbutton(self.lfStop, text="TRIG", var=self._stop, onvalue="TRIG", command=self.set_stop, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=1)
        self.btnStopRef = Checkbutton(self.lfStop, text="REF", var=self._stop, onvalue="REF", command=self.set_stop, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=1)
        self.btnStopRising = Checkbutton(self.lfStop, text="Rising", var=self._stopRising, command=self.set_stopRising, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=1)

        self.btnStopCh1.grid(row=0, column=20, columnspan=2, sticky = W+E+N+S)
        self.btnStopCh2.grid(row=1, column=20, columnspan=2, sticky = W+E+N+S)
        self.btnStopTrig.grid(row=0, column=22, columnspan=2, sticky = W+E+N+S)
        self.btnStopRef.grid(row=1, column=22, columnspan=2, sticky = W+E+N+S)
        self.btnStopRising.grid(row=2, column=20, columnspan=4, sticky = W+E+N+S)

        self.lblMode = Label(self, text="Mode: ", anchor=W)
        self.btnMode1 = Checkbutton(self, text="1 us", var=self._mode, onvalue=1, command=self.set_mode, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=1, padx=5)
        self.btnMode2 = Checkbutton(self, text="5 ms", var=self._mode, onvalue=2, command=self.set_mode, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=1, padx=5)
        self.btnMode3 = Checkbutton(self, text=">5 ms", var=self._mode, onvalue=3, command=self.set_mode, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=1, padx=5)
        self.lblMode.grid(row=3, column=4, columnspan=1, sticky = W+E+N+S)
        self.btnMode1.grid(row=3, column=5, columnspan=2, sticky = W+E+N+S)
        self.btnMode2.grid(row=3, column=7, columnspan=2, sticky = W+E+N+S)
        self.btnMode3.grid(row=3, column=9, columnspan=2, sticky = W+E+N+S)

        self.lblNStops = Label(self, text="Stops: ", anchor=W)
        self.btnNStops1 = Checkbutton(self, text="1", var=self._nstops, onvalue=1, command=self.set_nstops, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=3, padx=5)
        self.btnNStops2 = Checkbutton(self, text="2", var=self._nstops, onvalue=2, command=self.set_nstops, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=3, padx=5)
        self.btnNStops3 = Checkbutton(self, text="3", var=self._nstops, onvalue=3, command=self.set_nstops, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=3, padx=5)
        self.btnNStops4 = Checkbutton(self, text="4", var=self._nstops, onvalue=4, command=self.set_nstops, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=3, padx=5)
        self.btnNStops5 = Checkbutton(self, text="5", var=self._nstops, onvalue=5, command=self.set_nstops, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=3, padx=5)
        self.lblNStops.grid(row=4, column=4, columnspan=1, sticky = W+E+N+S)
        self.btnNStops1.grid(row=4, column=5, columnspan=2, sticky = W+E+N+S)
        self.btnNStops2.grid(row=4, column=7, columnspan=2, sticky = W+E+N+S)
        self.btnNStops3.grid(row=4, column=9, columnspan=2, sticky = W+E+N+S)
        self.btnNStops4.grid(row=4, column=11, columnspan=2, sticky = W+E+N+S)
        self.btnNStops5.grid(row=4, column=13, columnspan=2, sticky = W+E+N+S)

        self.lblCalp = Label(self, text="Calibration periods: ", anchor=W)
        self.btnCalp1 = Checkbutton(self, text="1", var=self._calPeriods, onvalue=1, command=self.set_calPeriods, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=3, padx=5)
        self.btnCalp2 = Checkbutton(self, text="2", var=self._calPeriods, onvalue=2, command=self.set_calPeriods, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=3, padx=5)
        self.btnCalp10 = Checkbutton(self, text="10", var=self._calPeriods, onvalue=10, command=self.set_calPeriods, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=3, padx=5)
        self.btnCalp20 = Checkbutton(self, text="20", var=self._calPeriods, onvalue=20, command=self.set_calPeriods, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=3, padx=5)
        self.btnCalp40 = Checkbutton(self, text="40", var=self._calPeriods, onvalue=40, command=self.set_calPeriods, 
                indicatoron=False, borderwidth=self.BORDERWIDTH, selectcolor=self.SELECT_COLOR, pady=3, padx=5)
        self.lblCalp.grid(row=5, column=4, columnspan=1, sticky = W+E+N+S)
        self.btnCalp1.grid(row=5, column=5, columnspan=2, sticky = W+E+N+S)
        self.btnCalp2.grid(row=5, column=7, columnspan=2, sticky = W+E+N+S)
        self.btnCalp10.grid(row=5, column=9, columnspan=2, sticky = W+E+N+S)
        self.btnCalp20.grid(row=5, column=11, columnspan=2, sticky = W+E+N+S)
        self.btnCalp40.grid(row=5, column=13, columnspan=2, sticky = W+E+N+S)

        self.btnRun = Checkbutton(self, text="RUN", var=self._ticenable, command=self.set_ticenable, 
                indicatoron=False, borderwidth=10, selectcolor=self.RUN_COLOR, pady=5, padx=5)
        self.btnRun.grid(row=4, rowspan=2, column=16, columnspan=8, sticky = W+E+N+S)

        self._serialReadThread = None
        try:
            self.sport = Serial(serialport, baudrate=baudrate, timeout=3)
        except:
            print("Unable to open serial port.")
            self.set_display("No Connection")
            self.sport = None

        if self.sport:
                self._ref.set('XO')
                self.set_ref()
                self._trig.set('OFF')
                self.set_trig() 
                self._start.set('CH1')
                self._startRising.set(1)
                self.set_start()
                self.set_startRising()
                self._stop.set('CH2')
                self._stopRising.set(1)
                self.set_stop()
                self.set_stopRising()
                self._mode.set(3)
                self.set_mode()
                self._nstops.set(1)
                self.set_nstops()
                self._calPeriods.set(10)
                self.set_calPeriods()

    def _switch(self, var):
        return
        for v in vars(self):
            vi = getattr(self, v)
            if isinstance(vi, Checkbutton):
                if vi.cget('onvalue') == var.get():
                    # activate
                    vi.configure(bg=self.SELECT_COLOR)
                    vi.configure(fg=self.SELECT_TEXT)
                else:
                    # deactivate
                    vi.configure(bg=self.cget('bg'))
                    vi.configure(fg=self.BLACK)

    def set_display(self, msg):
        self.lblText.configure(text=msg)

    def set_ticenable(self):
        if (self._ticenable.get()):
            if not self._serialReadThread:
                self._serialReadThread = threading.Thread(target=self._serial_read, daemon=True)
            self._serialReadThread.start()
        else:
            print("Wainting to join the tread...")
            self._serialReadThread.join(timeout=1.0)
            print("Thread joined")
            self._serialReadThread = None

        self._switch(self._ticenable)
        cmd = ":TIC:ENAB " + str(self._ticenable.get())
        self.sendcommand(cmd)

    def set_calPeriods(self):
        self._switch(self._calPeriods)
        cmd = ":TIC:CALP " + str(self._calPeriods.get())
        self.sendcommand(cmd)

    def set_nstops(self):
        self._switch(self._nstops)
        cmd = ":TIC:NSTO " + str(self._nstops.get())
        self.sendcommand(cmd)

    def set_mode(self):
        self._switch(self._mode)
        cmd = ":TIC:MODE " + str(self._mode.get())
        self.sendcommand(cmd)

    def set_startRising(self):
        self._switch(self._startRising)
        cmd = ":TIC:EDGE STAR," + str(self._startRising.get())
        self.sendcommand(cmd)

    def set_start(self):
        self._switch(self._start)
        cmd = ":TIC:STAR " + self._start.get()
        self.sendcommand(cmd)

    def set_stopRising(self):
        self._switch(self._stopRising)
        cmd = ":TIC:EDGE STOP," + str(self._stopRising.get())
        self.sendcommand(cmd)

    def set_stop(self):
        self._switch(self._stop)
        cmd = ":TIC:STOP " + self._stop.get()
        self.sendcommand(cmd)

    def set_trig(self):
        self._switch(self._trig)
        if self._trig.get() == "ON":
            self.btnTrig.configure(text=self._trig.get())
            cmd = ":DIV:TRIG:FREQ " + self.eTrigFreq.get()
            self.sendcommand(cmd)
        cmd = ":DIV:TRIG:ENAB " + self._trig.get()
        self.sendcommand(cmd)

    def set_ref(self):
        self._switch(self._ref)
        cmd = ":CONF:REF " + ("EXT,10M" if self._ref.get() == "EXT" else "XO")
        self.sendcommand(cmd)
        
    def _serial_readavailable(self):
        while self.sport.in_waiting:
             ln = self.sport.readline().decode().strip()
             print(ln)

    def sendcommand(self, cmd):
        print(cmd)
        self.sport.write((cmd + '\n').encode())
        print(' --- ')
        sleep(0.1)
        self._serial_readavailable()

    def _serial_read(self):
        while self._ticenable.get():
            while self.sport.in_waiting:
                msg = self.sport.readline().decode().strip()
                print(msg)
                if not msg.startswith("#"):
                    if msg.replace('.','').isnumeric():
                        self.set_display("{:,.3f} ns".format(float(msg)))
                    else:
                        self.set_display(msg)

def main(): 
    panel = TSIB2_Panel()
    panel.set_display(".,...,...,...;... s")
    panel.mainloop()
    
if __name__ == '__main__':
    main()

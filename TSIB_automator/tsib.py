import argparse
from asyncore import read
import serial
import serial.threaded
import sys
import socket
from mqtt import Mqtt

class SerialToNet(serial.threaded.Protocol):
    """serial->socket"""

    readdata = ""

    def __init__(self, output=None, echo=False, mqttClient=None):
        self.socket = None
        self.output = output
        self.echo = echo
        self.mqttClient = mqttClient            

    def __call__(self):
        return self

    def on_line(self, data):
        self.readdata += data
        line = None
        if '\n' in data:
            line = self.readdata[:self.readdata.find('\n')]
            self.readdata = self.readdata[self.readdata.find('\n')+1:]
            line = None if line.startswith('#') else line.strip()
            if line and (line.replace('.', '').replace('-', '').isnumeric()):
                line = float(line)*1.0e-9
                line = {'indication': line}
        if self.mqttClient and line:
            self.mqttClient.publish(self.mqttClient.topic, line, 1, False)


    def data_received(self, data):
        if self.socket is not None:
            self.socket.sendall(data)
        if self.output:
            self.output.write(data.decode())
        if self.echo:
            print(data.decode(), end='')
        if self.mqttClient:
            self.on_line(data.decode())


class TSIB(object):
    _ackstr = 'OK\n'

    def __init__(self, serialdevice, baudrate=115200, 
                startupfile=None, echo=False, output=None):
        self.echo = echo
        self.port = serialdevice
        self.output = None
        try:
            self.serial = serial.Serial(port=serialdevice, baudrate=baudrate)
        except serial.SerialException as e:
            sys.stderr.write('Could not open serial port {}: {}\n'.format(self.serial, e))
            sys.exit(1)

        if output:
            self.output=open(output, 'w')

        if startupfile:
            self.apply_startup(startupfile)

    def execute_mqttcommand(self, topic, message):
        self.send_command(message, waitack=True, echo=True, response=False)

    def send_command(self, command, waitack=True, echo=False, response=False):
        if not command:
            return True

        self.serial.write((command+'\n').encode())
        if echo or self.echo:
            print(command)
        if waitack or response:
            answ = self.readline(echo=True)
        if response:
            return answ
        if waitack:
            return answ==self._ackstr
        return True

    def readline(self, echo=None):
        line = self.serial.readline().decode().strip('\n\r')
        if echo or self.echo:
            print(line)
        if self.output:
            self.output.write(line+"\n")
        return line

    def apply_startup(self, filename):
        if not filename:
            return False
        with open(filename, 'r') as f:
            for c in f.readlines():
                cmd = c.strip()
                if not cmd.startswith('#'):
                    self.send_command(cmd)
        return True
    
    def read_infinite(self):
        ser_to_net = SerialToNet(self.output, self.echo)
        serial_worker = serial.threaded.ReaderThread(self.serial, ser_to_net)
        serial_worker.start()        
        try:
            pass
        except KeyboardInterrupt:
            pass        
        sys.stderr.write('\n--- exit ---\n')
        serial_worker.stop()

    def start_net_forwarder(self, localport, mqtt, mqtt_topic, mqtt_device):
        mqttClient = None
        if mqtt:
            mqttClient = Mqtt()
            mqttClient.topic = mqtt_topic
            mqttClient.registerActor('rarach/timelab/tsib/execute/' + mqtt_device + '/#', self.execute_mqttcommand)

        ser_to_net = SerialToNet(self.output, self.echo, mqttClient)
        serial_worker = serial.threaded.ReaderThread(self.serial, ser_to_net)
        serial_worker.start()

        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('', localport))
        srv.listen(1)
        try:
            while True:
                sys.stderr.write('Waiting for connection on {}...\n'.format(localport))
                client_socket, addr = srv.accept()
                sys.stderr.write('Connected by {}\n'.format(addr))
                # More quickly detect bad clients who quit without closing the
                # connection: After 1 second of idle, start sending TCP keep-alive
                # packets every 1 second. If 3 consecutive keep-alive packets
                # fail, assume the client is gone and close the connection.
                try:
                    client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 1)
                    client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 1)
                    client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
                    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                except AttributeError:
                    pass # XXX not available on windows
                client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                try:
                    ser_to_net.socket = client_socket
                    # enter network <-> serial loop
                    while True:
                        try:
                            data = client_socket.recv(1024)
                            if not data:
                                break
                            self.serial.write(data)                 # get a bunch of bytes and send them
                        except socket.error as msg:
                            sys.stderr.write('ERROR: {}\n'.format(msg))
                            # probably got disconnected
                            break
                except KeyboardInterrupt:
                    raise
                except socket.error as msg:
                    sys.stderr.write('ERROR: {}\n'.format(msg))
                finally:
                    ser_to_net.socket = None
                    sys.stderr.write('Disconnected\n')
                    client_socket.close()
        except KeyboardInterrupt:
            pass        
        sys.stderr.write('\n--- exit ---\n')
        serial_worker.stop()

    def close(self):
        if self.output:
            self.output.close()
        if self.serial and self.serial.is_open:
            self.serial.close()

def parse_arguments():
    aparser = argparse.ArgumentParser(description="Time Integration Board settings")
    group = aparser.add_argument_group('serial port')
    group.add_argument('--device', '-d', dest='device', type=str, default='/dev/ttyACM0', help='serial port device (default: /dev/ttyAMA0)')
    group.add_argument('--baudrate', '-b', dest='baudrate', type=int, default=115200, help='serial port baud rate, (default: 115200)')
    group.add_argument('--timeout', '-t', dest='timeout', type=float, default=0.3, help='serial port timeout in seconds, (default: 0.3)')
    group.add_argument('--echo', '-e', dest='echo', action='store_true', default=True, help='echo responses from TIC, (default: echo)')
    group.add_argument('--no-echo', '-ne', dest='echo', action='store_false', help='echo responses from TIC, (default: echo)')
    aparser.add_argument('--startup', '-s', dest='startup', type=str, help='startup commands send to TIC on start')
    aparser.add_argument('--command', '-c', dest='commands', type=str, action='append', nargs="+", default=[], help='command to be executed on TIC')
    aparser.add_argument('--output', '-o', dest='output', type=str, help='output filename')
    aparser.add_argument('--mqtt', '-m', dest='mqtt', action='store_true', default=False, help='publishes data to MQTT (default: False)')
    aparser.add_argument('--mqtttopic', '-mt', dest='mqtt_topic', type=str, default='rarach/timelab/tsib/<mqtt_device>/value', help='mqtt topic under which measured values will be published (default: rarach/timelab/tsib/<mqtt_device>/value)')
    aparser.add_argument('--mqttdevice', '-md', dest='mqtt_device', type=str, default='cmx', help='unique identifier of the device (default: cmx)')
    egroup = aparser.add_mutually_exclusive_group()
    egroup.add_argument('--read', '-r', dest='read', action='store_true', default=False, help='opens the serial port and read infinitely; All startup and -c commands are executed before.')
    egroup.add_argument('--localport', '-p', dest='localport', type=int, help='starts serial port forwarder on port; All startup and -c commands are executed before.')
    return aparser.parse_args()

def main():
    pass

if __name__ == "__main__":
    pargs = parse_arguments()    
    print(pargs)
    tsib = TSIB(serialdevice=pargs.device, baudrate=pargs.baudrate, 
                startupfile=pargs.startup, echo=pargs.echo, output=pargs.output)
    for c in pargs.commands:
        for cc in c:
            tsib.send_command(cc, echo=pargs.echo, response=pargs.echo)
    if pargs.read:
        tsib.read_infinite()
    if pargs.localport:
        pargs.mqtt_topic = pargs.mqtt_topic.replace('<mqtt_device>', pargs.mqtt_device)
        tsib.start_net_forwarder(pargs.localport, pargs.mqtt, pargs.mqtt_topic, pargs.mqtt_device)
    tsib.close()

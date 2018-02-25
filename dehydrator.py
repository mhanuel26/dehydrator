from ws_connection import ClientClosedError
from ws_server import WebSocketServer, WebSocketClient
from machine import UART
import select
import urequests
import ujson as json
from uio import StringIO
import time


class uartStream:
    def __init__(self, uart):
        self.uart = uart
        self.poll = select.poll()
        self.poll.register(self.uart, select.POLLIN)

    def read(self, n):
        count = n
        buf = b''
        while True:
            if count == 0:
                return buf
            events = self.poll.poll(100)
            if not events:
                return b'\x00'*n
            for file in events:
                if file[0] == self.uart:
                    buf += self.uart.read(1)
                    count -= 1

    def readUntil(self, ch):
        res = b''
        while True:
            rx_ch = self.read(1)
            if ord(rx_ch) == 0:
                return StringIO('null')
            else:
                if ord(rx_ch) == ord(ch):
                    return StringIO(res)
                res += rx_ch

    def write(self, data):
        self.uart.write(data)
        pass

class ControllerBridge:
    def __init__(self, serialStream):
        self.serialStream = serialStream
        self.conn = None
        self.config = {}

    def clean_conn_obj(self):
        self.conn = None
        return

    def set_conn_obj(self, conn):
        self.conn = conn
        return

    def set_config_val(self, key, value):
        self.config[key] = value
        return

    def flush(self):
        self.config = {}
        return

    def send_over_ws(self, data):
        if self.conn is None:
            return
        self.conn.write(data)

    def send_command(self):
        cmd_json = json.dumps(self.config)
        self.serialStream.write(cmd_json)
        self.serialStream.write('\n')
        return

    def process(self, data):
        try:
            parsed = data.split('\n')
            for item in parsed:
                if len(item) > 0:
                    cmd = json.loads(item)
                else:
                    return
                if cmd:
                    try:
                        if "direct-cmd" in cmd:
                            dircmd = cmd["direct-cmd"]
                            for command in dircmd:
                                print("command <%s>,  value: %s" % (command, dircmd[command]))
                                self.set_config_val(command, dircmd[command])
                            self.send_command()
                            self.flush()
                            continue
                    except KeyError:
                        print("KeyError exception: TCP CMD = %s" % repr(cmd))
                        return
                    except Exception as e:
                        print("process exception %s" % e)
                        return
        except ValueError:
            print("ValueError Exception")
            return

class DehydratorClient(WebSocketClient):
    def __init__(self, conn, bridge):
        super().__init__(conn)
        self.cntrlBridge = bridge

    def process(self):
        try:
            msg = self.connection.read()
            if not msg:
                return
            msg = msg.decode("utf-8")
            print("DehydratorClient read: %s" % msg)
            self.cntrlBridge.process(msg)
        except ClientClosedError:
            self.connection.close()


class DehydratorServer(WebSocketServer):
    def __init__(self, bridge):
        super().__init__(1)
        self.cntrlBridge = bridge

    def _make_client(self, conn):
        try:
            self.cntrlBridge.set_conn_obj(conn)
        except Exception as e:
            print("DehydratorServer exception %s" % e)
        return DehydratorClient(conn, self.cntrlBridge)

    def _cleanup_conn(self):
        self.cntrlBridge.clean_conn_obj()
        return

def process_arduino(serialStream, cntrlBridge):
    ''' process incoming data from Arduino '''
    valid = False
    try:
        rcv_json = json.load(serialStream.readUntil(ch = '\n'))
    except ValueError:
        return
    if rcv_json:
        try:
            try:
                if "sht15" in rcv_json:
                    if "error" in rcv_json["sht15"][0]:
                        if rcv_json["sht15"][0]["error"] == "noerr":
                            valid = True                           
            except:  
                pass
            if "reported" in rcv_json:
                valid = True
            if valid:
                data_json = json.dumps(rcv_json)
                print(data_json)
                cntrlBridge.send_over_ws(data_json)
        except KeyError:
            print("KeyError exception: rcv_json = %s" % repr(rcv_json))
            return

def run():
    print('Dehydrator v1.11 App Running')
    ''' Serial Link Setup '''
    uart = UART(0, 9600)
    uart.init(9600, bits=8, parity=None, stop=1)
    serialStream = uartStream(uart)
    cntrlBridge = ControllerBridge(serialStream)
    ''' Dehydrator Websocket Server Setup '''
    server = DehydratorServer(cntrlBridge)
    server.start()
    ''' start endless loop '''
    try:
        while True:
            server.process_all()
            process_arduino(serialStream, cntrlBridge)
    except KeyboardInterrupt:
        print("Dehydrator App Exit - Keyboard Interrupt")
        pass
    server.stop()
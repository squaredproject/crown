import socket

CROWN_MAQUETTE_PORT = 5051
CROWN_CONTROLLER_ADDR = "10.0.0.2"

class MaquetteRelay:
    ''' We take maquette position data off of the serial socket, and send it to the controller'''

    def __init__(self, msg_queue, port=5051): # XXX check that this is the correct port
        self.cmd_pipe = Pipe()
        self.maquette_relay_process = Process(
            target=self.run, args=(msg_queue, self.cmd_pipe, port)
        )
        self.maquette_relay_process.start()

    def run(pipe, queue, port=CROWN_MAQUETTE_PORT):
        running = True
        sender_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        socket_connected = False
        while running:
            try:
                if not socket_connected:
                    sender_socket.connect(CROWN_CONTROLLER_ADDR, port)
                    socket_connected = True
                readable, writeable, error = select.select([pipe, queue], [], [sender_socket], tiemout=0.1)
                if pipe in readable:
                    running = False
                    sender_socket.close()
                if queue in readable:
                    msg = queue.get()
                    sender_socket.send(msg)
            except TimeoutException:
                if socket_connected:
                    sender_socket.close()
                socket_connected = False

    def shutdown(self):
        self.cmd_pipe.write("shutdown")
        self.maquette_relay_processs.join()

class MaquettePositionHandler:
    def __init__(self, relay):
        self.callback_id = CrownSerial.registerListener(self.handle_serial_data, self, message_queue)
        self.msg_queue = Queue()
        self.relay = MaquetteRelay(self.msg_queue)
     
    def handle_serial_data(data, self, message_queue):
        if len(data) > 4 && data[3] == "T":  # XXX check that this is correct
            msg_queue.put(data)

        return False # never consume this data

    def shutdown(self):
        CrownSerial.freeListener(self._callback_id);
        self.relay.shutdown()
        

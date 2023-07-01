import logging
import socket
from multiprocessing import Process, Queue, Pipe
from multiprocessing.connection import wait
import time
import select

import CrownSerial
serial = CrownSerial

use_hostnames = True

CROWN_MAQUETTE_PORT = 5051
# CROWN_CONTROLLER_ADDR = "10.0.0.2"
CROWN_CONTROLLER_ADDR = "127.0.0.1"
CROWN_CONTROLLER_NAME = "hydraulics.local"

class MaquettePositionHandler:
    ''' Runs on the Maquette.
        On the callback to the serial data handler, put relevant messages in the
        relay queue, where they will get picked up by the MaquetteRelay and
        transmitted to the Controller '''

    def __init__(self):
        self.msg_queue = Queue()
        self.callback_id = CrownSerial.registerListener(self.handle_serial_data, (self, self.msg_queue), False)
        self.relay = MaquetteRelay(self.msg_queue)
     
    def handle_serial_data(self, msg_queue, data):
        if len(data) > 4 and data[3] == "T":  # XXX check that this is correct
            msg_queue.put(data)

        return False # never consume this data

    def shutdown(self):
        CrownSerial.freeListener(self.callback_id);
        self.relay.shutdown()


class MaquetteRelay:
    ''' Runs on the Maquette.
        Take position data from a queue and send it to the Controller '''

    def __init__(self, input_queue, port=CROWN_MAQUETTE_PORT):
        self.cmd_pipe, recv_pipe  = Pipe()
        self.maquette_relay_process = Process(
            target=self.run, args=(input_queue, recv_pipe, port,)
        )
        self.maquette_relay_process.start()

    def run(self, input_queue, pipe, port):
        running = True
        sender_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        socket_connected = False
        while running:
            try:
                if not socket_connected:
                    if use_hostnames:
                        sender_socket.connect((socket.gethostbyname(CROWN_CONTROLLER_NAME), port))
                    else:
                        sender_socket.connect((CROWN_CONTROLLER_ADDR, port))
                    socket_connected = True
                    print("Socket Connected!!")
                ready = wait([pipe, input_queue._reader], 0.1)
                if pipe in ready:
                    pipe_data = pipe.recv()
                    if pipe_data == "shutdown":
                        logging.info("Shutting down Maquette Relay")
                        running = False
                        sender_socket.close()
                if input_queue._reader in ready:
                    msg = input_queue.get()
                    print(f"Maquette relay - send message {msg}")
                    sender_socket.send(msg.encode("UTF-8"))
            except OSError as e:
                # print(f"Socket exception : {e}")
                if socket_connected:
                    sender_socket.close()
                sender_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                socket_connected = False

    def shutdown(self):
        self.cmd_pipe.send("shutdown")
        self.maquette_relay_process.join()


class MaquettePositionReceiver:
    ''' Runs on the Controller. 
        Listen for incoming position packets from the Maquette. Send them 
        to the sculpture (via serial to the mega), and possibly record them.
        XXX - If I knew whether the Recorder was active, I wouldn't bother
        constantly sending packets to it... FIXME
    '''
    def __init__(self, recording_queue, serial_out, port=CROWN_MAQUETTE_PORT):
        self.msg_pipe, recv_pipe = Pipe()
        self.maquette_position_gatherer = Process(
            target=self.run, name="Maquette Receiver", args=(recording_queue, serial_out.writeQueue, port, recv_pipe,)
        )
        logging.info("Maquette Receiver: Initialize")
        self.maquette_position_gatherer.start()

    def run(self, recording_queue, serial_queue, port, pipe):
        logger  = logging.getLogger("Relay")
        logfile = "/var/log/crown/relay.log"
        handler = logging.handlers.RotatingFileHandler(filename=logfile, maxBytes=100000, backupCount=2)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
        PACKET_LEN = 180  # This is more than I am likely to be sending at a time
        print("Maquette receiver creating listener")
        listener_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener_socket.bind(("", port))
        listener_socket.listen()
 
        logger.info("Maquette Receiver: Bound, now listening...")
        print("Maquette receiver bound")
        running = True
        frame = []
        accumulated_data = ""
        mode = "mode_manual"  # Should get a message from the switch... let's see if that always comes in.
        while running:
            readable, writeable, error = select.select(
                [listener_socket, pipe], [], [listener_socket], 0.1
            )
            if pipe in readable:  # Signal shutdown
                logger.info("Maquette Receiver: Received signal on pipe, shutting down receiver")
                running = False
                break

            if listener_socket in readable:  # Connection ready
                (client_socket, address) = listener_socket.accept()
                logger.info(f"Maquette Receiver: Accepted client, address {address}")
                accumulated_data = ""
                while True:
                    readable, writeable, error = select.select(
                        [client_socket, pipe], [], [client_socket], 0.1
                    )
                    if pipe in readable:
                        cmd = pipe.recv()
                        if cmd == "shutdown":
                            logger.info("Maquette Receiver: received disconnect signal")
                            running = False
                            break
                        elif cmd in ["mode_maquette", "mode_conductor", "mode_manual"]:
                            mode = cmd
                            logger.info(f"Changing mode to {cmd}")
                    if client_socket in error:
                        logger.info("Maquette Receiver: error on receiver")
                        break

                    if client_socket in readable:
                        data = client_socket.recv(PACKET_LEN)
                        if len(data) > 0:
                            logger.debug(f"Read data {data}")
                            try:
                                accumulated_data += data.decode("UTF-8")
                                while True:
                                    packet_start = accumulated_data.find("<")
                                    if packet_start > 0:
                                        accumulated_data = accumulated_data[packet_start:]
                                    elif packet_start < 0:
                                        break
                                    packet_end = accumulated_data.find(">")
                                    if packet_end > 0:
                                        packet = accumulated_data[packet_start:packet_end+1]
                                        if mode == "mode_maquette":
                                            if (serial_queue.qsize() > 30):
                                                logger.info(f"Maquette Receiver: Too much data on enet for serial queue, queue size {serial_queue.qsize()}, dropping enet packet {packet}")
                                            else:
                                                logger.debug(f"Maquette Receiver: Placing packet {packet} in serial queue")
                                                serial_queue.put(packet)
                                            # if recording_queue is not None:
                                            #     print("Sending data to recorder!")
                                            #   recording_queue.put(packet)
                                        else:
                                            pass  # throw the data away if we're in the wrong mode
                                        accumulated_data = accumulated_data[packet_end+1:]
                                    else:
                                        logger.debug(f"Could not find packet end in {accumulated_data}")
                                        break
                                else:
                                    break
                            except Exception as e:
                                logger.info(f"Error {e} ")
                    # XXX check the timeout for polling the switch status

                client_socket.close()
        print("Shutdown receiver")
        listener_socket.close()
        pipe.close()

    def switch_state_callback(response, pipe):
        pipe.send(response)

    def shutdown(self):
        self.msg_pipe.send("shutdown")
        self.maquette_position_gatherer.join()


if __name__ == "__main__":
    from Recording import CrownRecorder
    #  Data path: from queue to relay to known socket to receiver to recorder
    recorder_queue = Queue()
    input_queue = Queue()
    print("Create Recorder")
    my_recorder = CrownRecorder(recorder_queue)
    print("Create position receiver")
    my_receiver = MaquettePositionReceiver(recorder_queue)
    print("Create relay...")
    my_relay = MaquetteRelay(input_queue)

    for i in range(0, 10):
        print("Input message")
        input_queue.put(f"<{i}0T1111>")
        time.sleep(0.05)

    time.sleep(1)
    my_relay.shutdown()
    my_receiver.shutdown()
    my_recorder.shutdown() 

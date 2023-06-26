from collections.abc import Iterable
import logging
from logging.handlers import RotatingFileHandler
import time
import json
from sys import platform
import threading
import traceback

import multiprocessing as mp
from multiprocessing import Queue

try:
   mp.set_start_method('spawn', force=True)
   print("spawned")
except RuntimeError:
   pass

from flask import Flask, request, abort, make_response, jsonify
from flask_cors import CORS


from MaquetteRelay import MaquettePositionReceiver
from Recording import CrownRecorder

test = False
if test:
    import CrownSerialMock

    serial = CrownSerialMock
else:
    import CrownSerial

    serial = CrownSerial


app = Flask(
    "crown", static_url_path="/"
)  # , static_folder="/home/pi/crown/Hydraulics/controller/rpi/static")
CORS(app)


def serve_forever(httpPort=5000):
    print(f"CROWN webserver serving on port {httpPort}")
    app.run("0.0.0.0", port=httpPort, threaded=True)


# XXX - note that you can select on Queues. select.select() works just fine...
# change the code to use select!!

# An Async request consists of both an initial request call, and a filter that makes
# sure we return the correct response back to the requestor. For the moment, the
# implementation of the various requests/filters is bound up in the infrastructure
# definition - the usage is sufficiently limited that this makes sense.
class AsyncRequest:
    def __init__(self, command, tower_id=-1, joint_id=-1, args=[""], to_maquette=False):
        self.joint_id = joint_id
        self.tower_id = tower_id
        self.response = None
        self.response_filter = ResponseFilter(command, to_maquette=to_maquette)
        self.to_maquette = to_maquette
        self.command = command
        self.args = args

    def make_request(self):
        logger.debug(
            f"Sending message to sculpture, command is {self.command}, to_maquette is {self.to_maquette}"
        )
        send_sculpture_message(
            self.command,
            tower_id=self.tower_id,
            joint_id=self.joint_id,
            args=self.args,
            to_maquette=self.to_maquette,
        )

    def filter(self, message):
        filtered_response = self.response_filter.filter(
            message, [self.tower_id, self.joint_id]
        )
        if filtered_response != None:
            self.response = filtered_response
        return filtered_response

    def have_response(self):
        return self.response is not None

    def set_response(self, response):
        self.response = response


class AsyncRequester:
    """Handles aggregating multiple requests. (Since we're using a CAN bus under the
    covers, the data packets received are very small. We deal with this by making a
    lot of requests for small amounts of data)"""

    def __init__(self, request_list, timeout=1.0):
        if not isinstance(request_list, Iterable):
            request_list = [request_list]
        self.request_list = request_list
        self.timeout = timeout
        self.data_ready = False

        self.listener = serial.registerListener(AsyncRequester._async_callback, [self])

    def run(self):
        """Make all requests, wait for responses. Timeout if responses have not been
        received by the specified timeout"""
        logger.debug("AsyncRequest - run")
        try:
            endTime = time.time() + self.timeout
            self._request()
            while not self.data_ready and time.time() < endTime:
                time.sleep(0.15)
        except Exception as e:
            logger.error(f"Exception in AsyncRequester! {e}")
            traceback.print_exc()

        logger.debug("Async request - finished")
        serial.freeListener(self.listener)

        logger.debug("free listener finished")

        if self.data_ready:
            logger.debug("Async request - have response")
            return True
        else:
            logger.info("Timeout - No data returned, aborting\n")
            return False

    def _request(self):
        """Re-request anything that we haven't gotten a response for"""
        for request in self.request_list:
            if not request.have_response():
                request.make_request()

    @staticmethod
    def _async_callback(
        myself, message
    ):  # callback... can't call through class so I fake it...
        logger.info(f"Callback, response {message}")
        try:
            found_requestor = False
            for request in myself.request_list:
                filtered_response = request.filter(message)
                if filtered_response:
                    request.set_response(filtered_response[:])  # copying
                    found_requestor = True
                    break
            if not found_requestor:
                logger.error(f"Could not find requestor for message {message}")
            have_all_requests = True
            for request in myself.request_list:
                if not request.have_response():
                    have_all_requests = False
                    break
            myself.data_ready = have_all_requests
            # if there are no responses left, trigger wakeup of main thread.   # XXX - use select
        except Exception as e:
            myself.data_ready = False
            logger.warning("Exception on async serial callback! {e}")
            traceback.print_exc()

        return False


gTowerRange = [0, 1, 2, 3]
gJointRange = [0, 1, 2]


""" Description of Serial protocol...
    The Raspberry Pi communicates serially with the ArduinoMega, which
    in turn communicates with the hydraulics and the maquette. Commands to 
    the ArduinoMega take the following form:
    Hydraulics command:  <tower_id> + COMMAND + (arguments, if any)
    Maquette command:  "m" + COMMAND + (arguments, if any)
                       "m" + <tower_id> + COMMAND + (arguments, if any)
"""

# Sculpture
GENERAL_STATUS_REQUEST = "s"
INPUT_MODE_REQUEST = "i"
JOINT_STATUS_REQUEST = "j"  # XXX not currently exposed in web api
JOINT_LIMITS_REQUEST = "l"
TOWER_POSITION_REQUEST = "T"
VALVE_DRIVE_REQUEST = "v"
EXTENDED_STATUS_REQUEST = "e"  # XXX - not currently exposed in web api
PID_VALUES_REQUEST = "p"
HOME_COMMAND = "H"
SET_HOME_SPEED_COMMAND = "h"
SET_TARGETS_COMMAND = "t"
SET_LIMITS_COMMAND = "L"
SET_MIN_COMMAND = "m"
SET_MAX_COMMAND = "M"
SET_CENTER_COMMAND = "C"
SET_I_VALUE_COMMAND = "I"
SET_P_VALUE_COMMAND = "P"
NEUTER_COMMAND = "n"  # Removes immediate input to joint - sets target to current value.
SET_RUN_STATE_COMMAND = "r"  # Allow a tower to move... or not
SET_HOME_STATE_COMMAND = "w"
SET_JOINT_ENABLE_COMMAND = "E"  # software joint enable/disable
SET_CANONICAL_TARGETS_COMMAND = "f"
SET_CANONICAL_JOINT_TARGETS_COMMAND = "F"

# Maquette
MAQUETTE_CALIBRATION_REQUEST = "c"
MAQUETTE_STATUS_REQUEST = "s"
SET_MAQUETTE_MODE_COMMAND = "M"
SET_MAQUETTE_TOWER_CENTER_COMMAND = "C"
SET_MAQUETTE_TOWER_ENABLE_COMMAND = "E"
SET_MAQUETTE_JOINT_ENABLE_COMMAND = "e"
SET_MAQUETTE_TOWER_DECALIBRATE_COMMAND = "X"


class ResponseFilter:
    """ResponseFilter
    Used to check whether a response on the serial bus is the response
    to a previous request. For reasons I am uncertain of, I am not using
    an incrementing serial number. XXX CSW
    XXX - This is a terrible terrible protocol, or perhaps just a terrible implementation.
    I should not have to put the information for where the request starts in
    the filter infrastructure.
    """

    def __init__(self, request_type, to_maquette=False):
        self.request_type = request_type
        self.maquette_request = to_maquette
        self.command_idx = 1
        self.tower_idx = 2
        self.joint_idx = 3
        self.filter_joint = False
        if self.maquette_request:
            self.start_idx = 3
            # NB - maquette commands all start with 'm'
            self.command_idx += 1
            self.tower_idx += 1
            self.joint_idx += 1
        else:
            if self.request_type in [JOINT_LIMITS_REQUEST, JOINT_STATUS_REQUEST]:
                self.start_idx = 4
                self.filter_joint = True
            elif self.request_type in [
                GENERAL_STATUS_REQUEST,
                TOWER_POSITION_REQUEST,
                PID_VALUES_REQUEST,
                VALVE_DRIVE_REQUEST,
                EXTENDED_STATUS_REQUEST,
            ]:
                self.start_idx = 3
                self.filter_joint = False
            else:
                raise RuntimeError(f"Unknown request {request_type}, aborting")

    def filter(self, message, args):
        """If this message is a response to the request, return message stripped of header
        Otherwise, return None
        """
        ret = self.validate_response_header(message, args[0])
        try:
            if (not ret) or (
                self.filter_joint and (args[1] != int(message[self.joint_idx]))
            ):
                return None
            else:
                return message[self.start_idx :]
        except ValueError:
            pass

        return None

    # XXX - I don't check for overruns on the response header. FIXME
    def validate_response_header(self, message, tower_id):
        if message[0] != "!":
            print("Message doesn't have response bang!")
            return False

        if self.maquette_request:
            if message[1] != "m":
                print("Looking for maquette message, but response doesn't start with m")
                return False

        if message[self.command_idx] != self.request_type:
            print("Message command type wrong")
            return False

        try:
            # XXX - what is not a tower? idx -1 or none? be consistent
            if tower_id not in [-1, None] and int(message[self.tower_idx]) != tower_id:
                print("Message tower id doesn't match")
                return False
        except ValueError:
            print("Value error in validate response header!")
            return False

        return True


def send_sculpture_message(
    command, tower_id=-1, joint_id=-1, args=[""], to_maquette=False
):
    """send_sculpture_message
    Send a message to the arduino that talks to the sculpture driver and
    the maquette hardware.
    Both tower_id and joint_id are ints.
    XXX - There is an open question whether the numbering is zero based or one-based.
    I believe I'm using both
    """
    if joint_id is None or joint_id < 0:
        joint_local = ""
    else:
        joint_local = str(joint_id)

    if tower_id is None or tower_id < 0:
        tower_local = ""
    else:
        tower_local = str(tower_id)

    if not isinstance(args, Iterable):
        args = [args]

    if not to_maquette:
        logger.debug(f"Writing <{tower_local}{joint_local}{command}{','.join(map(str,args))}>")
        serial.write(f"<{tower_local}{joint_local}{command}{','.join(map(str,args))}>")
    else:
        # NB - I can't change the tower serial protocol for legacy reasons. But I can change the maquette protocol
        # maquette protocol is <m{command}{tower}:{joint}:{command_separated_args}>
        if joint_local:
            joint_local = ":" + joint_local
        args_local = ",".join(map(str, args))
        if args_local:
            args_local = ":" + args_local
        logger.debug(f" Writing <m{command}{tower_local}{joint_local}{args_local}>")
        serial.write(f"<m{command}{tower_local}{joint_local}{args_local}>")


@app.route("/", methods=["GET"])
def show_index():
    return app.send_static_file("control_panel.html")

@app.route("/crown/sculpture", methods=["GET"])
def crown_get_sculpture_state():
    return _getSculptureState(range(1, 5))


@app.route("/crown/sculpture/towers/<int:tower_idx>", methods=["GET"])
def crown_get_tower_state(tower_idx):
    if tower_idx not in gTowerRange:
        return make_response("Invalid tower", 404)
    return _getSculptureState(tower_idx)


def _getSculptureState(towers):
    """Get the state of one or more towers
    Tower state is defined as the current joint position, the joint limits for each joint
     and the general state. It does not include PID values or current drive
     Returns list of
     {"towerId":towerId, "joints":[{"id":xx, "position":xx,
                                    "center":xx, "rightLimit":xx, leftLimit":xx
                                    "homed":true|false, "enabled":true|false} ...],
                         "running":true|false,
                         "error":true:false}"""

    if not isinstance(towers, Iterable):
        towers = [towers]
    
    # First, let's set up the object that's going to collate the results
    result_list = {}
   
    for tower_id in []: #towers:
        if tower_id not in gTowerRange:
            continue
        calls = []
        calls.append(AsyncRequest(TOWER_POSITION_REQUEST, tower_id=tower_id))
        calls.append(AsyncRequest(JOINT_LIMITS_REQUEST, tower_id=tower_id, joint_id=0))
        calls.append(AsyncRequest(JOINT_LIMITS_REQUEST, tower_id=tower_id, joint_id=1))
        calls.append(AsyncRequest(JOINT_LIMITS_REQUEST, tower_id=tower_id, joint_id=2))
        calls.append(AsyncRequest(GENERAL_STATUS_REQUEST, tower_id=tower_id))

        requester = AsyncRequester(calls)
        results = requester.run()
        if results:
            joint_list = []
            for joint_id in range(0, 3):
                joint_list.append(
                     {
                        "id": joint_id,
                        "position": [],
                        "max": 0,
                        "min": 0,
                        "center": 0,
                        "enabled": True,
                        "homed": True,
                    }
                )
            tower_obj = {
                "tower": tower_id,
                "joints": joint_list,
                "running": True,
                "error": False,
            }
            logger.debug(f"adding tower_id {tower_id} to resultList")
            result_list[tower_id] = tower_obj

            # Now, go through all the responses, and collate them correctly
            for call in calls:
                # the format of the individual responses is json string data
                result_obj = json.loads(call.response)
                tower_id = call.tower_id
                if call.command == TOWER_POSITION_REQUEST:
                    # print("resultlist towerId is {}".format(result_list[tower_id]))
                    # print("joints 0 is {}".format(result_list[tower_id]['joints'][0]))
                    result_list[tower_id]["joints"][0]["position"] = result_obj[0]
                    result_list[tower_id]["joints"][1]["position"] = result_obj[1]
                    result_list[tower_id]["joints"][2]["position"] = result_obj[2]
                if call.command == GENERAL_STATUS_REQUEST:
                    result_list[tower_id]["running"] = result_obj["running"]
                    result_list[tower_id]["error"] = result_obj["error"]
                    result_list[tower_id]["joints"][0]["enabled"] = result_obj["enabled"][0]
                    result_list[tower_id]["joints"][0]["homed"] = result_obj["homed"][0]
                    result_list[tower_id]["joints"][1]["enabled"] = result_obj["enabled"][1]
                    result_list[tower_id]["joints"][1]["homed"] = result_obj["homed"][1]
                    result_list[tower_id]["joints"][2]["enabled"] = result_obj["enabled"][2]
                    result_list[tower_id]["joints"][2]["homed"] = result_obj["homed"][2]
                if call.command == JOINT_LIMITS_REQUEST:
                    joint_idx = (
                        call.joint_id - 1
                    )  # joints 0 based. because we hate life XXX I don't have to propagate that
                    result_list[tower_id]["joints"][joint_idx]["center"] = result_obj[
                        "center"
                    ]
                    result_list[tower_id]["joints"][joint_idx]["min"] = result_obj["min"]
                    result_list[tower_id]["joints"][joint_idx]["max"] = result_obj["max"]
    
    general_call = AsyncRequest(INPUT_MODE_REQUEST, to_maquette=True)
    general_results = AsyncRequester([general_call]).run()
    if general_results:
        logger.debug("Results are {general_results}")
        result_obj = json.loads(general_call.response)
        result_list["general"] = result_obj

    if result_list:
        # Spit out JSON
        return jsonify(json.dumps(result_list))
    else:
        return make_response("No data from towers", 500)


def _simple_web_async_request(
    command, tower_id=-1, joint_id=-1, args=[""], to_maquette=False
):
    if tower_id != -1 and tower_id not in gTowerRange:
        return make_response("Invalid tower", 404)
    if joint_id != -1 and joint_id not in gJointRange:
        return make_response("Invalid joint", 404)
    try:
        call = AsyncRequest(
            command,
            tower_id=tower_id,
            joint_id=joint_id,
            args=args,
            to_maquette=to_maquette,
        )
        requester = AsyncRequester(call)
        results = requester.run()
        if results:
            return jsonify(call.response)
        else:
            return make_response("No response from tower", 500)
    except Exception as e:
        print(f"Exception in simple_web_async_request! {e}\n")
        traceback.print_exc()
    return make_response("Internal error", 500)


@app.route("/crown/sculpture/towers/<int:tower_idx>/position", methods=["GET", "PUT"])
def crown_get_tower_position(tower_idx):
    if method == "GET":
        """Get current position of all joints (as much as we can tell)"""
        return _simple_web_async_request(TOWER_POSITION_REQUEST, tower_id=tower_idx)
    else:
        """Set the target values for the hydraulics. Fire and forget."""
        # XXX - I should also be able to get these values, even if I have to store them on the pi
        if not _validate_tower(tower_idx):
            return make_response("Must have valid tower id", 400)
        elif set("j1", "j2", "j3") not in set(request.values):
            return make_response("Must contain joint value parameters j1, j2, j3", 400)
        # Convert -1.0 to 1.0 to -128 to 127, because the protocol doesn't do floats.
        j_pos = []
        for i in range(1, 3):
            j_pos[i] = float(request.values["j" + i])
            j_pos[i] = j_pos[i] * 128
            j_pos[i] = min(j_pos[i], 127)
            j_pos[i] = max(j_pos[i], -128)

        send_sculpture_message(
            SET_CANONICAL_TARGETS_COMMAND,
            tower_id=tower_idx,
            joint_id=1,
            args=int(j_pos[1]),
        )
        send_sculpture_message(
            SET_CANONICAL_TARGETS_COMMAND,
            tower_id=tower_idx,
            joint_id=2,
            args=int(j_pos[2]),
        )
        send_sculpture_message(
            SET_CANONICAL_TARGETS__COMMAND,
            tower_id=tower_idx,
            joint_id=3,
            args=int(j_pos[3]),
        )
        return make_response("Success", 200)


@app.route("/crown/sculpture/towers/<int:tower_id>/PID", methods=["GET", "PUT"])
def crown_get_set_tower_pid(tower_id):
    """Get current PI values for all joints, or set PI values for selected joints
    Returns {"p":[xx,xx,xx],"i":[xx,xx,xx]}"""
    if request.method == "GET":
        return _simple_web_async_request(PID_VALUES_REQUEST, tower_id=tower_id)
    else:  # method "PUT"
        if not _validate_tower(tower_id):
            return make_response("Must have valid tower id", 400)
        if "joint1_P" in request.values:
            send_sculpture_message(
                SET_P_VALUE_COMMAND,
                tower_id=tower_id,
                joint_id=1,
                args=request.values["joint1_P"],
            )
        if "joint2_P" in request.values:
            send_sculpture_message(
                SET_P_VALUE_COMMAND,
                tower_id=tower_id,
                joint_id=2,
                args=request.values["joint2_P"],
            )
        if "joint3_P" in request.values:
            send_sculpture_message(
                SET_P_VALUE_COMMAND,
                tower_id=tower_id,
                joint_id=3,
                args=request.values["joint3_P"],
            )
        if "joint1_I" in request.values:
            send_sculpture_message(
                SET_I_VALUE_COMMAND,
                tower_id=tower_id,
                joint_id=1,
                args=request.values["joint1_I"],
            )
        if "joint2_I" in request.values:
            send_sculpture_message(
                SET_I_VALUE_COMMAND,
                tower_id=tower_id,
                joint_id=2,
                args=request.values["joint2_I"],
            )
        if "joint3_I" in request.values:
            send_sculpture_message(
                SET_I_VALUE_COMMAND,
                tower_id=tower_id,
                joint_id=3,
                args=request.values["joint3_I"],
            )
        return make_response("Succcess", 200)


@app.route("/crown/sculpture/towers/<int:tower_id>/drive", methods=["GET"])
def crown_get_tower_drive(tower_id):
    """Get current drive values for all joints
    Returns [xx,xx,xx]"""
    return _simple_web_async_request(VALVE_DRIVE_REQUEST, tower_id=tower_id)


@app.route("/crown/sculpture/towers/<int:tower_id>/limits", methods=["GET", "PUT"])
def crown_get_set_joint_limits(tower_id):
    """Get limits (left, center, right) for all joints.
    Returns [[left,center,right],[left,center,right],[left,center,right]]"""
    if request.method == "GET":
        return _simple_web_async_request(TOWER_LIMITS_REQUEST, tower_id=tower_id)
    else:
        if "j1_min" in request.values:
            send_sculpture_message(
                SET_MIN_COMMAND,
                tower_id=tower_id,
                joint_id=1,
                args=[request.values["j1_min"]],
            )
        if "j1_max" in request.values:
            send_sculpture_message(
                SET_MAX_COMMAND,
                tower_id=tower_id,
                joint_id=1,
                args=[request.values["j1_max"]],
            )
        if "j2_min" in request.values:
            send_sculpture_message(
                SET_MIN_COMMAND,
                tower_id=tower_id,
                joint_id=2,
                args=[request.values["j2_min"]],
            )
        if "j2_max" in request.values:
            send_sculpture_message(
                SET_MAX_COMMAND,
                tower_id=tower_id,
                joint_id=2,
                args=[request.values["j2_max"]],
            )
        if "j3_min" in request.values:
            send_sculpture_message(
                SET_MIN_COMMAND,
                tower_id=tower_id,
                joint_id=3,
                args=[request.values["j3_min"]],
            )
        if "j3_max" in request.values:
            send_sculpture_message(
                SET_MAX_COMMAND,
                tower_id=tower_id,
                joint_id=3,
                args=[request.values["j3_max"]],
            )

    return make_response("Success", 200)
    # commandSetLimits(towerId, jointId, -250, 300);  # XXX - seems like this was set this way.... FIXME?
    # XXX - shouldn't we be able to restore joint limits, like if the arduino reboots?


@app.route(
    "/crown/sculpture/towers/<int:tower_id>/joints/<int:joint_id>/home", methods=["PUT"]
)
def crown_home(tower_id, joint_id):
    send_sculpture_message(HOME_COMMAND, tower_id=tower_id, joint_id=joint_id)
    return make_response("Success", 200)


@app.route(
    "/crown/sculpture/towers/<int:tower_id>/joints/<int:joint_id>/home_speed",
    methods=["PUT"],
)  # XXX Get? I don't think I care at this point
def crown_set_home_speed(tower_id, joint_id):
    if "speed" in request.values:
        send_sculpture_message(
            SET_HOME_SPEED_COMMAND,
            tower_id=tower_id,
            joint_id=joint_id,
            args=[int(request.values["speed"])],
        )
        return make_response("Success", 200)
    else:
        return make_response("Must have speed parameter", 400)


@app.route("/crown/sculpture/towers/<int:tower_id>/force_homed", methods=["PUT"])
def crown_force_home(tower_id):
    print("setting homed")
    send_sculpture_message(
        SET_HOME_STATE_COMMAND, tower_id=tower_id, joint_id=1, args=[1]
    )
    send_sculpture_message(
        SET_HOME_STATE_COMMAND, tower_id=tower_id, joint_id=2, args=[1]
    )
    send_sculpture_message(
        SET_HOME_STATE_COMMAND, tower_id=tower_id, joint_id=3, args=[1]
    )
    print("end homed")
    return make_response("Success", 200)


@app.route("/crown/sculpture/towers/<int:tower_idx>/running", methods=["PUT"])
def crown_set_run_state(tower_idx):
    args = []
    centered = None
    if "running" in request.values:
        onOff = (
            1
            if request.values["running"] in [True, "true", 1, "on", "running"]
            else 0
        )
        print(f"Running is {onOff}")
        args.append(onOff)
        if onOff:
            # homed = request.values.get("homed", False)
            # args.append(homed)
            # centered = request.values.get("centered", False)
            # args.append(centered)
            # print(f"Set sculpture running with home and center: homed = {homed}, centered = {centered}")
            pass
        send_sculpture_message(SET_RUN_STATE_COMMAND, tower_id=tower_idx, args=args)
    return make_response("Success", 200)


@app.route(
    "/crown/sculpture/towers/<int:tower_id>/joints/<int:joint_id>", methods=["PUT"]
)
def crown_set_joint_state(tower_id, joint_id):
    print("Attemting to set joint state")
    if "enable" in request.values:
        onOff = 1 if request.values["enable"] in [1, "on", "true"] else 0
    send_sculpture_message(
        SET_JOINT_ENABLE_COMMAND, tower_id=tower_id, joint_id=joint_id, args=[onOff]
    )
    return make_response("Success", 200)


@app.route(
    "/crown/sculpture/towers/<int:tower_id>/joints/<joint_id>/neuter", methods=["PUT"]
)
def commandNeuterValve(tower_id, joint_id):
    send_sculpture_message(NEUTER_COMMAND, tower_id=tower_id, joint_id=joint_id)
    return make_response("Success", 200)


@app.route("/crown/sculpture/towers/<int:tower_id>/center", methods=["PUT"])
def crown_set_center(tower_id):
    """Causes the current values of the tower joints to be set as the
    center values. This really *should* return the current values, and be
    considered at the low level as an AsyncRequest. XXX FIXME"""
    send_sculpture_message(SET_CENTER_COMMAND, tower_id=tower_id, joint_id=1)
    send_sculpture_message(SET_CENTER_COMMAND, tower_id=tower_id, joint_id=2)
    send_sculpture_message(SET_CENTER_COMMAND, tower_id=tower_id, joint_id=3)
    return make_response("Success", 200)


@app.route("/crown/sculpture/towers/<int:tower_id>/targets", methods=["PUT"])
def crown_set_targets(tower_id):
    """Set the target values for the hydraulics. Fire and forget."""
    # XXX - I should also be able to get these values, even if I have to store them on the pi
    if not _validate_tower(tower_id):
        return make_response("Must have valid tower id", 400)
    elif set("j1", "j2", "j3") not in set(request.values):
        return make_response("Must contain joint value parameters j1, j2, j3", 400)
    # XXX - the values here are strings, but the protocol expects ints... FIXME
    send_sculpture_message(
        SET_TARGETS_COMMAND, tower_id=tower_id, joint_id=1, args=request.values["j1"]
    )
    send_sculpture_message(
        SET_TARGETS_COMMAND, tower_id=tower_id, joint_id=2, args=request.values["j2"]
    )
    send_sculpture_message(
        SET_TARGETS_COMMAND, tower_id=tower_id, joint_id=3, args=request.values["j3"]
    )
    return make_response("Success", 200)


@app.route(
    "/crown/sculpture/towers/<int:tower_id>/joints/<int:joint_id>/position",
    methods=["PUT"],
)
def crown_set_canonical_targets(tower_id, joint_id):
    """Set the target values for the hydraulics. Fire and forget."""
    # XXX - I should also be able to get these values, even if I have to store them on the pi
    if not _validate_tower(tower_id):
        print("validate tower id fails")
        return make_response("Must have valid tower id", 400)
    if "pos" not in request.values:
        return make_response("Must contain position", 500)
    j_pos = float(request.values["pos"])
    j_pos = (j_pos * 128) + 128
    j_pos = min(255, j_pos)
    j_pos = max(0, j_pos)

    send_sculpture_message(
        SET_CANONICAL_JOINT_TARGETS_COMMAND,
        tower_id=tower_id,
        joint_id=joint_id,
        args=int(j_pos),
    )
    return make_response("Success", 200)


def _validate_tower(tower_id):
    return tower_id > 0 and tower_id < 5


# XXX What are saved parameters here?
@app.route("/crown/savedParameters", methods=["GET"])
def crown_get_parameters():
    pass


@app.route("/crown/maquette", methods=["GET", "PUT"])
def crown_get_maquette_state():
    if request.method == "GET":
        """Get the status of the maquette. Includes mode, position, limits"""
        return _simple_web_async_request(MAQUETTE_STATUS_REQUEST, to_maquette=True)
    else:
        """Only valid POST currently is to set the mode"""
        mode = request.values.get("mode")
        if mode:  # in request.values:
            # mode = request.values["mode"]
            modeInt = -1
            if mode == "POSE":
                modeInt = 2
            elif mode == "OFF":
                modeInt = 0
            elif mode == "IMMEDIATE":
                modeInt = 1
            elif mdoe == "PLAYBACK":
                modeInt = 3
            if modeInt >= 0:
                print(f"Setting mode to {modeInt}")
                return _simple_web_async_request(
                    SET_MAQUETTE_MODE_COMMAND, args=[modeInt], to_maquette=True
                )
            else:
                return make_response("Invalid mode", 400)
        else:
            return make_response("Must specify mode", 400)


@app.route("/crown/maquette/calibration", methods=["GET"])
def requestMaquetteCalibration():
    return _simple_web_async_request(MAQUETTE_CALIBRATION_REQUEST, to_maquette=True)


@app.route("/crown/maquette/towers/<int:tower_id>/calibrate", methods=["PUT"])
def maquette_calibrate(tower_id):
    print("calibrate tower called")
    send_sculpture_message(
        SET_MAQUETTE_TOWER_CENTER_COMMAND, tower_id=tower_id, to_maquette=True
    )
    return make_response("Success", 200)


@app.route("/crown/maquette/towers/<int:tower_id>", methods=["PUT"])
def maquette_set_tower(tower_id):
    print("set tower is called")
    if "enable" in request.values:
        print("enable/disable")
        tf = request.values.get("enable")
        tf = 1 if tf in ["True", "true"] else 0
        print(f"setting tower enable {tf} on tower {tower_id}")
        return _simple_web_async_request(
            SET_MAQUETTE_TOWER_ENABLE_COMMAND,
            tower_id=tower_id,
            args=[tf],
            to_maquette=True,
        )
    elif "calibrate" in request.values:
        print("calibrate")
        return _simple_web_async_request(
            SET_MAQUETTE_TOWER_CENTER_COMMAND, tower_id=tower_id, to_maquette=True
        )
    elif "decalibrate" in request.values:
        print("decalibrate")
        return _simple_web_async_request(
            SET_MAQUETTE_TOWER_DECALIBRATE_COMMAND, tower_id=tower_id, to_maquette=True
        )
    return make_response("Invalid Parameter", 400)


@app.route(
    "/crown/maquette/towers/<int:tower_id>/joints/<int:joint_id>", methods=["PUT"]
)
def maquette_set_joint(tower_id, joint_id):
    print("set joint is called")
    if "enable" in request.values:
        tf = request.values.get("enable")
        tf = 1 if tf in ["True", "true"] else 0
        send_sculpture_message(
            SET_MAQUETTE_JOINT_ENABLE_COMMAND,
            args=[tf],
            tower_id=tower_id,
            joint_id=joint_id,
            to_maquette=True,
        )
        return make_response("Success", 200)
    return make_response("Invalid Parameter", 400)


# @app.route("/crown/playback", methods=["GET"])
# def crown_get_playback_state():
#    response = getPlaybackState()


# @app.route("/crown/record", methods=["GET"])
# def crown_get_maquette_state():
#    response = getRecordingState()


# @app.route("/crown/playlists", methods=["GET"])
# def crown_get_playlists():
#    response = getPlaylists()


# @app.route("/crown/clips", methods=["GET"])
# def crown_get_clips():
#    response = getPlaylists()


# XXX - what are the saved parameters?
@app.route("/crown/parameters", methods=["PUT"])
def crown_set_saved_parameters_():
    pass


@app.route("/crown/sculpture/towers/<int:tower_id>/debug", methods=["PUT"])
def crown_set_tower_debug(tower_id):
    pass


@app.route("/crown/sculpture/towers/<int:tower_id>/playback", methods=["PUT"])
def crown_set_playback(tower_id):
    pass


@app.route("/crown/sculpture/towers/<int:tower_id>recording", methods=["PUT"])
def crown_set_recording(tower_id):
    pass


@app.route("/crown/sculpture/towers/<int:tower_id>/playlists", methods=["PUT"])
def crown_set_playlists():
    pass


# class HomingStatusHandler():
#     homingStatus = []
#
#    def init():
#        for i in range(0,4):
#            homingStatus.append(None)
#        serial.registerListener(HomingStatusHandler.homingCallback, None)  # XXX - note that this adds a dependency on initing crown serial before creating this object!
#
#    def setHomingStatus(homingMessage):
#        towerId = homingMessage[2]
#        try:
#            homingObject = json.loads(homingMessage[4:])
#            HomwingStatusHandlr.homingStatus[towerId] = homingObject
#        except ValueError:
#            logger.error("malformed json in homing message")
#
#    def getHomingStatus(towerId):
#        if (towerId not in gTowerRange):
#            return None
#        return HomingStatusHandler.homingStatus[towerId]
#
#    def homingCallback(message, args):
#        if (validateRequestHeader(message, None, HOME_COMMAND)) :
#            HomingStatusHandler.setHomingStatus(message)
#            return True
#        else:
#            return False


if platform == "linux" or platform == "linux2":
    logfile = '/var/log/crown/crown_1.log'
    # logfile = "crown.log"
elif platform == "darwin":
    logfile = "crown.log"

global logger

if __name__ == "__main__":
    global logger
    logger  = logging.getLogger("Crown")
    handler = logging.handlers.RotatingFileHandler(filename=logfile, maxBytes=100000)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    logger.setLevel(logging.DEBUG)
    logger.info("Starting Crown Controller")
    # logging.basicConfig(filename=logfile, level=logging.DEBUG)

    serial.init()

    recorder_queue = Queue()
    recorder = CrownRecorder(recorder_queue)
    maquette_receiver = MaquettePositionReceiver(recorder_queue, serial)

    # homingHandler = HomingStatusHandler()

    time.sleep(1)  # why am I doing this?

    try:
        serve_forever(5050)
    except KeyboardInterrupt:
        pass

    serial.shutdown()

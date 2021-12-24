import time
import atexit
import logging
import monotonic
import threading
from .state import *
from queue import Queue
from .Agent import Agent
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega



class Connection:
    """
    The connection handles all interaction with the network and the MAVLink master device.
    This implementation has been inspired by the following source:
        * Project: Dronekit
        * Repository: dronekit
        * URL: https://github.com/dronekit/dronekit-python/blob/c7143e0ba94d414416fd36679fd3306ce31a7e69/dronekit/mavlink.py#L114
    """
    def __init__(self, port: str, 
                 baud: int, 
                 source_system: int=255, 
                 source_component: int=0, 
                 cmd_timeout: float=1.0,
                 log: bool=False, 
                 debug: bool=False) -> None:

        self.logger = self.__init_logger('connection', log=log, debug=debug)

        # Create a new mavlink connection
        self.master = mavutil.mavlink_connection(port, 
                                                 baud=baud, 
                                                 source_system=source_system, 
                                                 source_component=source_component,
                                                 autoreconnect=True)

        # Ensure that a connection has been successfully established
        # Integrate a 2 second timeout
        resp = self.master.wait_heartbeat(timeout=2)

        if resp is None:
            raise TimeoutError('The system was unable to establish a connection with the specified device within the timeout period')

        # Class variables
        self.cmd_timeout = cmd_timeout
        self.connected = True
        self.devices = {}
        self.message_listeners = {}
        self.message_senders = {}
        self.outgoing_msgs = Queue()
        self.ack_msg_flag = False

        # Register the exit callback
        atexit.register(self.disconnect)

        # Threads
        self.heartbeat_t = threading.Thread(target=self.__heartbeat)
        self.heartbeat_t.daemon = True

        self.incoming_msg_t = threading.Thread(target=self.__incoming_msg_handler)
        self.incoming_msg_t.daemon = True

        self.outgoing_msg_t = threading.Thread(target=self.__send_msg_handler)
        self.outgoing_msg_t.daemon = True


        """
        Messaage Listeners
        """

        @self.on_message(['HEARTBEAT'])
        def listener(self, msg) -> None:
            """
            Callback function used to handle incoming messages
            """
            # Make sure that the message isn't from a GCS
            if msg.get_type() == mavutil.mavlink.MAV_TYPE_GCS:
                return
            
            # Get the system ID and component ID
            sys_id = msg.get_srcSystem()
            comp_id = msg.get_srcComponent()

            # Create a new device assigned the respective sysid:compid pair
            device = Agent(sys_id, comp_id)

            # If the device hasn't been seen before, save it
            if sys_id not in self.devices:
                self.devices[sys_id] = device
            else:
                # The connection has been restored
                if self.devices[sys_id].timeout:
                    self.logger.info(f'Connection to device {sys_id}:{comp_id} has been restored')
                
            # Update the last heartbeat variable
            self.devices[sys_id].last_heartbeat = monotonic.monotonic()
            
            self.devices[sys_id].timeout = False

            return


        @self.on_message(['HEARTBEAT'])
        def listener(self, msg) -> None:
            """
            Handle general device information contained within a heartbeat
            """
            # Ignore messages sent by a GCS
            if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                return

            sys_id = msg.get_srcSystem()

            # Let the heartbeat implementation handle this
            if not sys_id in self.devices:
                return

            self.devices[sys_id].armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            self.devices[sys_id].system_status = msg.system_status
            self.devices[sys_id].vehicle_type = msg.type

            # Update the last heartbeat
            self.devices[sys_id].last_heartbeat = monotonic.monotonic()

            try:
                # NOTE: We assume that ArduPilot will be used
                self.devices[sys_id].flight_mode = mavutil.mode_mapping_bynumber(msg.type)[msg.custom_mode]
            except Exception as e:
                # We received an invalid message
                pass

            return
        

        @self.on_message(['GLOBAL_POSITION_INT'])
        def listener(self, msg) -> None:
            """
            Handle the a GPS position message
            """
            sys_id = msg.get_srcSystem()

            # Let the heartbeat implementation handle this
            if not sys_id in self.devices:
                return

            # Update the device velocity
            if self.devices[sys_id].velocity is None:
                v = Velocity(msg.vx / 100, msg.vy / 100, msg.vz / 100)
                self.devices[sys_id].velocity = v
            else:
                self.devices[sys_id].velocity.vx = msg.vx / 100
                self.devices[sys_id].velocity.vy = msg.vy / 100
                self.devices[sys_id].velocity.vz = msg.vz / 100

            # Update the device location
            if self.devices[sys_id].location is None:
                loc = Location(msg.lat / 1.0e7, msg.lon / 1.0e7, msg.alt / 1000)
                self.devices[sys_id].location = loc
            else:
                self.devices[sys_id].location.latitude = msg.lat / 1.0e7
                self.devices[sys_id].location.longitude = msg.lon / 1.0e7
                self.devices[sys_id].location.altitude = msg.alt / 1000

            return

        
        @self.on_message(['ATTITUDE'])
        def listener(self, msg) -> None:
            """
            Handle an agent attitude message
            """
            sys_id = msg.get_srcSystem()

            # Let the heartbeat implementation handle this
            if not sys_id in self.devices:
                return

            # Update the respective devices attitude
            if self.devices[sys_id].attitude is None:
                att = Attitude(msg.pitch, msg.yaw, msg.roll, msg.pitchspeed, msg.yawspeed, msg.rollspeed)
                self.devices[sys_id].attitude = att
            else:
                self.devices[sys_id].attitude.pitch = msg.pitch
                self.devices[sys_id].attitude.roll = msg.roll
                self.devices[sys_id].attitude.yaw = msg.yaw
                self.devices[sys_id].attitude.pitch_speed = msg.pitchspeed
                self.devices[sys_id].attitude.roll_speed = msg.rollspeed
                self.devices[sys_id].attitude.yaw_speed = msg.yawspeed
            
            return


        @self.on_message(['SYS_STATUS'])
        def listener(self, msg) -> None:
            """
            Handle the system status message containing battery state
            """
            sys_id = msg.get_srcSystem()

            # Let the heartbeat implementation handle this
            if not sys_id in self.devices:
                return

            # Update the battery information
            if self.devices[sys_id].battery is None:
                batt = Battery(msg.voltage_battery, msg.current_battery, msg.battery_remaining)
                self.devices[sys_id].battery = batt
            else:
                self.devices[sys_id].battery.voltage = msg.voltage_battery
                self.devices[sys_id].battery.current = msg.current_battery
                self.devices[sys_id].battery.level = msg.battery_remaining

            return


        @self.on_message(['GPS_RAW_INT'])
        def listener(self, msg) -> None:
            """
            Handle the GPS status information
            """
            sys_id = msg.get_srcSystem()

            # Let the heartbeat implementation handle this
            if not sys_id in self.devices:
                return

            # Read the GPS status information
            if self.devices[sys_id].gps_info is None:
                info = GPSInfo(msg.eph, msg.epv, msg.fix_type, msg.satellites_visible)
                self.devices[sys_id].gps_info = info
            else:
                self.devices[sys_id].gps_info.eph = msg.eph
                self.devices[sys_id].gps_info.epv = msg.epv
                self.devices[sys_id].gps_info.fix_type = msg.fix_type
                self.devices[sys_id].gps_info.satellites_visible = msg.satellites_visible
            
            return


        @self.on_message(['EKF_STATUS_REPORT'])
        def listener(self, msg) -> None:
            """
            Handle an EKF status message
            """
            sys_id = msg.get_srcSystem()

            # Let the heartbeat implementation handle this
            if not sys_id in self.devices:
                return

            # Read the EKF Status information
            if self.devices[sys_id].ekf is None:
                ekf = EKFStatus(msg.velocity_variance, 
                                msg.pos_horiz_variance, 
                                msg.pos_vert_variance, 
                                msg.compass_variance, 
                                msg.terrain_alt_variance,
                                (msg.flags & ardupilotmega.EKF_POS_HORIZ_ABS) > 0,
                                (msg.flags & ardupilotmega.EKF_CONST_POS_MODE) > 0,
                                (msg.flags & ardupilotmega.EKF_PRED_POS_HORIZ_ABS) > 0)
                self.devices[sys_id].ekf = ekf
            else:
                # Read variance properties
                self.devices[sys_id].ekf.velocity_variance = msg.velocity_variance
                self.devices[sys_id].ekf.pos_horiz_variance = msg.pos_horiz_variance
                self.devices[sys_id].ekf.pos_vert_variance = msg.pos_vert_variance
                self.devices[sys_id].ekf.compass_variance = msg.compass_variance
                self.devices[sys_id].ekf.terrain_alt_variance = msg.terrain_alt_variance

                # Read flags
                self.devices[sys_id].ekf.pos_horiz_abs = (msg.flags & ardupilotmega.EKF_POS_HORIZ_ABS) > 0
                self.devices[sys_id].ekf.const_pos_mode = (msg.flags & ardupilotmega.EKF_CONST_POS_MODE) > 0
                self.devices[sys_id].ekf.pred_pos_horiz_abs = (msg.flags & ardupilotmega.EKF_PRED_POS_HORIZ_ABS) > 0

            return


        """
        Arming commands
        """

        @self.send_message(['arm'])
        def sender(self, sys_id, comp_id, require_ack=False) -> None:
            """
            Arm an agent
            """
            self.__send_arming_msg(1, sys_id, comp_id, require_ack)

            return

        
        @self.send_message(['disarm'])
        def sender(self, sys_id, comp_id, require_ack=False) -> None:
            """
            Disarm an agent
            """
            self.__send_arming_msg(0, sys_id, comp_id, require_ack)

            return

        
        """
        Pre-flight calibration commands
        """

        @self.send_message(['accelcal'])
        def sender(self, sys_id, comp_id, require_ack=False) -> None:
            """
            Perform a full accelerometer calibration on the selected agent
            """
            self.__send_preflight_calibration_msg(1, sys_id, comp_id, require_ack)

            return 


        @self.send_message(['accelcalsimple'])
        def sender(self, sys_id, comp_id, require_ack=False) -> None:
            """
            Perform a simple accelerometer calibration on the selected agent
            """
            self.__send_preflight_calibration_msg(4, sys_id, comp_id, require_ack)

            return


        @self.send_message(['ahrstrim'])
        def sender(self, sys_id, comp_id, require_ack=False) -> None:
            """
            Instruct and agent to recalibrate its ahrs parameters
            """
            self.__send_preflight_calibration_msg(2, sys_id, comp_id, require_ack)

            return


        """
        Flight Modes
        """

        @self.send_message(['stabilize'])
        def sender(self, sys_id, comp_id, require_ack=False) -> None:
            """
            Set an agent to STABILIZE mode
            """
            self.__send_flight_mode_msg('STABILIZE', sys_id, comp_id, require_ack)

            return


        @self.send_message(['acro'])
        def sender(self, sys_id, comp_id, require_ack=False) -> None:
            """
            Set an agent to ACRO mode
            """
            self.__send_flight_mode_msg('ACRO', sys_id, comp_id, require_ack)

            return

        
        @self.send_message(['althold'])
        def sender(self, sys_id, comp_id, require_ack=False) -> None:
            """
            Set an agent to ALT_HOLD mode
            """
            self.__send_flight_mode_msg('ALT_HOLD', sys_id, comp_id, require_ack)

            return


        @self.send_message(['auto'])
        def sender(self, sys_id, comp_id, require_ack=False) -> None:
            """
            Set an agent to AUTO mode
            """
            self.__send_flight_mode_msg('AUTO', sys_id, comp_id, require_ack)

            return

        
        @self.send_message(['loiter'])
        def sender(self, sys_id, comp_id, require_ack=False) -> None:
            """
            Set an agent to LOITER mode
            """
            self.__send_flight_mode_msg('LOITER', sys_id, comp_id, require_ack)

            return


        @self.send_message(['rtl'])
        def sender(self, sys_id, comp_id, require_ack=False) -> None:
            """
            Set an agent to RTL mode
            """
            self.__send_flight_mode_msg('RTL', sys_id, comp_id, require_ack)

            return


        @self.send_message(['land'])
        def sender(self, sys_id, comp_id, require_ack=False) -> None:
            """
            Set an agent to LAND mode
            """
            self.__send_flight_mode_msg('LAND', sys_id, comp_id, require_ack)

            return


        @self.send_message(['throw'])
        def sender(self, sys_id, comp_id, require_ack=False) -> None:
            """
            Set an agent to THROW mode
            """
            self.__send_flight_mode_msg('THROW', sys_id, comp_id, require_ack)

            return


        @self.send_message(['systemid'])
        def sender(self, sys_id, comp_id, require_ack=False) -> None:
            """
            Set an agent to SYSTEM ID mode
            """
            self.__send_flight_mode_msg('SYSTEMID', sys_id, comp_id, require_ack)

            return


        @self.send_message(['guided'])
        def sender(self, sys_id, comp_id, require_ack=False) -> None:
            """
            Set an agent to GUIDED mode
            """
            self.__send_flight_mode_msg('GUIDED', sys_id, comp_id, require_ack)

            return

        
        """
        HRL commands
        """

        @self.send_message(['startros'])
        def sender(self, sys_id, comp_id, require_ack=False) -> None:
            """
            Start ROS on the respective agent
            """
            self.__send_hrl_msg(0, sys_id, comp_id, require_ack)

            return


        @self.send_message(['stopros'])
        def sender(self, sys_id, comp_id, require_ack=False) -> None:
            """
            Stop ROS on the respective agent
            """
            self.__send_hrl_msg(1, sys_id, comp_id, require_ack)

            return

        
        @self.send_message(['startpath'])
        def sender(self, sys_id, comp_id, require_ack=False) -> None:
            """
            Start path execution on the respective agent
            """
            self.__send_hrl_msg(2, sys_id, comp_id, require_ack)

            return

        
        @self.send_message(['stoppath'])
        def sender(self, sys_id, comp_id, require_ack=False) -> None:
            """
            Stop path execution on the respective agent
            """
            self.__send_hrl_msg(3, sys_id, comp_id, require_ack)

            return


    def __init_logger(self, name, debug: bool=False, log: bool=False) -> logging.Logger:
        """
        Initialize the logger with the desired debug levels
        """
        logging.basicConfig()

        # Set the desired debug level
        if debug or (debug and log):
            logger = logging.getLogger(name)
            logger.setLevel(logging.DEBUG)
            return logger
        elif log:
            logger = logging.getLogger(name)
            logger.setLevel(logging.INFO)
            return logger
        else:
            return logging.getLogger(name)


    def on_message(self, msg):
        """
        Decorator used to create a listener for a mavlink message
        """
        def decorator(fn):
            if isinstance(msg, list):
                for m in msg:
                    self.add_message_listener(m, fn)
            else:
                self.add_message_listener(msg, fn)

        return decorator


    def add_message_listener(self, msg, fn) -> None:
        """
        Add a new function to the dictionary of message listeners
        """
        if msg not in self.message_listeners:
            self.message_listeners[msg] = []
        
        if fn not in self.message_listeners[msg]:
            self.message_listeners[msg].append(fn)

        return

    
    def send_message(self, msg):
        """
        Decorator used to create a listener for a mavlink message
        """
        def decorator(fn):
            if isinstance(msg, list):
                for m in msg:
                    self.add_message_sender(m, fn)
            else:
                self.add_message_sender(msg, fn)

        return decorator


    def add_message_sender(self, msg, fn) -> None:
        """
        Add a new function to the dictionary of message listeners
        """
        if msg not in self.message_senders:
            self.message_senders[msg] = []
        
        if fn not in self.message_senders[msg]:
            self.message_senders[msg].append(fn)

        return


    def __heartbeat(self) -> None:
        """
        Function used to sent a heartbeat to the network indicating that the GCS
        is still operating
        """
        while self.connected:
            self.master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

            # Send a heartbeat every 2 seconds
            time.sleep(2)

        return


    def __incoming_msg_handler(self) -> None:
        """
        Handle incoming messages and distribute them to their respective handlers
        """
        while self.connected:
            # Skip the read if the sending thread is attempting
            # to acknowledge reception
            if self.ack_msg_flag:
                continue

            # Update the timeout flag for each device
            for sys_id in self.devices:
                if self.devices[sys_id].last_heartbeat is not None:
                    self.devices[sys_id].timeout = (monotonic.monotonic() - self.devices[sys_id].last_heartbeat) >= self.devices[sys_id].timeout_period

            # Read a new message
            try:
                msg = self.master.recv_msg()
            except mavutil.mavlink.MAVError as e:
                self.logger.debug('An error occurred on MAVLink message reception')
                msg = None
            except Exception:
                # Log any other unexpected exception
                self.logger.exception('Exception while receiving message: ', exc_info=True)
                msg = None
            
            if not msg:
                continue

            # Apply the respective message handler(s)
            if msg.get_type() in self.message_listeners:
                for fn in self.message_listeners[msg.get_type()]:
                    try:
                        fn(self, msg)
                    except Exception:
                        self.logger.exception(f'Exception in message handler for {msg.get_type()}', exc_info=True)

        return

    
    def __send_msg_handler(self) -> None:
        """
        Handle sending messages to the agents on the network
        """
        while self.connected:
            if self.outgoing_msgs.qsize() > 0:

                # Get the next message
                (msg, ack) = self.outgoing_msgs.get(timeout=1)

                # Send the message if there is a message sender for it
                if msg.get_type() in self.message_senders:
                    for fn in self.message_senders[msg.get_type()]:
                        try:
                            fn(self, msg.sys_id, msg.comp_id, ack)
                        except Exception:
                            self.logger.exception(f'Exception in message sender for {msg.get_type()}', exc_info=True)

        return

    
    def __send_arming_msg(self, msg, sys_id, comp_id, require_ack=False) -> None:
        """
        Helper method used to send an arming command (arm or disarm)
        """

        if require_ack:
            ack = False
            
            while not ack:
                self.master.mav.command_long_send(sys_id, comp_id,
                                                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                                    msg, 0, 0, 0, 0, 0, 0)

                if self.__ack_sys_cmd(timeout=self.cmd_timeout):
                    ack = True
                    self.logger.debug(f'The system has acknowledged reception of the arming command: {msg}')
                else:
                    self.logger.exception('The system was unable to confirm reception of the arming command: {msg}. Re-attempting message send.')
        else:
            self.master.mav.command_long_send(sys_id, comp_id,
                                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                                msg, 0, 0, 0, 0, 0, 0)

            if self.__ack_sys_cmd(timeout=self.cmd_timeout):
                self.logger.debug('The system has acknowledged reception of the arming command: {msg}')
            else:
                self.logger.exception('The system was unable to confirm reception of the arming command: {msg}')

        return


    def __send_preflight_calibration_msg(self, msg, sys_id, comp_id, require_ack=False) -> None:
        """
        Helper method used to send a pre-flight calibration message
        """
        if require_ack:
            ack = False
            
            while not ack:
                self.master.mav.command_long_send(sys_id, comp_id,
                                                    mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                                    0, 0, 0, 0, msg, 0, 0)

                if self.__ack_sys_cmd(timeout=self.cmd_timeout):
                    ack = True
                    self.logger.debug(f'The system has acknowledged reception of the pre-flight calibration command: {msg}')
                else:
                    self.logger.exception('The system was unable to confirm reception of the pre-flight calibration command: {msg}. Re-attempting message send.')
        else:
            self.master.mav.command_long_send(sys_id, comp_id,
                                                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                                0, 0, 0, 0, msg, 0, 0)

            if self.__ack_sys_cmd(timeout=self.cmd_timeout):
                self.logger.debug('The system has acknowledged reception of the pre-flight calibration command: {msg}')
            else:
                self.logger.exception('The system was unable to confirm reception of the pre-flight calibration command: {msg}')

        return
        

    def __send_flight_mode_msg(self, msg, sys_id, comp_id, require_ack=False) -> None:
        """
        Helper method used to set a flight mode on a particular agent
        """
        mode_id = self.master.mode_mapping()[msg]

        self.master.target_system = sys_id
        self.master.target_component = comp_id

        if require_ack:
            ack = False
            
            while not ack:
                self.master.set_mode(mode_id)

                if self.__ack_sys_cmd(timeout=self.cmd_timeout):
                    ack = True
                    self.logger.debug(f'The system has acknowledged reception of the {msg} command')
                else:
                    self.logger.exception(f'The system was unable to confirm reception of the {msg} command. Re-attempting message send.')
        else:
            self.master.set_mode(mode_id)

            if self.__ack_sys_cmd(timeout=self.cmd_timeout):
                self.logger.debug(f'The system has acknowledged reception of the {msg} command')
            else:
                self.logger.exception(f'The system was unable to confirm reception of the {msg} command')

        return


    def __send_hrl_msg(self, msg, sys_id, comp_id, require_ack=False) -> None:
        """
        Helper method used to send a desired HRL message
        """
        self.master.target_system = sys_id
        self.master.target_component = comp_id

        if require_ack:
            ack = False
            
            while not ack:
                self.master.mav.named_value_int_send('hrl-state-arg', msg)

                if self.__ack_sys_cmd(timeout=self.cmd_timeout):
                    ack = True
                    self.logger.debug(f'The system has acknowledged reception of the HRL {msg} command')
                else:
                    self.logger.exception(f'The system was unable to confirm reception of the HRL {msg} command. Re-attempting message send.')
        else:
            self.master.mav.named_value_int_send('hrl-state-arg', msg)

            if self.__ack_sys_cmd(timeout=self.cmd_timeout):
                self.logger.debug(f'The system has acknowledged reception of the HRL {msg} command')
            else:
                self.logger.exception(f'The system was unable to confirm reception of the HRL {msg} command')

        return


    def __ack_sys_cmd(self, timeout=1.0) -> bool:
        """
        Helper method used to ensure that the flight mode command was acknowledged 
        in the provided timeout period
        """
        # Skip mavlink message reads in the consumer thread
        self.ack_msg_flag = True
        
        # Flag indicating whether the message was acknowledged
        ack_success = False

        # Start acknowledgement timer
        start_t = time.time()

        while time.time() - start_t < timeout:
            # Read a new message
            try:
                ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=False)
                ack_msg = ack_msg.to_dict()
                
                if ack_msg['mavpackettype'] == 'COMMAND_ACK':
                    ack_success = True
                    self.logger.debug(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
                    break
            except mavutil.mavlink.MAVError as e:
                self.logger.debug('An error occurred on MAVLink message reception')
            except AttributeError:
                # Catch errors with converting the message to a dict
                pass
            except Exception:
                # Log any other unexpected exception
                self.logger.exception('Exception while receiving message: ', exc_info=False)

        # Continue reading status messages
        self.ack_msg_flag = False
        
        return ack_success

    
    def start_connection(self) -> None:
        """
        Helper method used to start non-mavlink related connection processes
        """
        self.__start_t()

        return

    
    def __start_t(self) -> None:
        """
        Start all threads available to the connection for message reception, 
        message sending, and heartbeat handling
        """
        self.heartbeat_t.start()
        self.incoming_msg_t.start()
        self.outgoing_msg_t.start()

        return

    
    def __stop_t(self) -> None:
        """
        Join all threads
        """
        if self.heartbeat_t is not None:
            self.heartbeat_t.join()

        if self.incoming_msg_t is not None:
            self.incoming_msg_t.join()

        if self.outgoing_msg_t is not None:
            self.outgoing_msg_t.join()

        return

    
    def disconnect(self) -> None:
        """
        Close the connection and disconnect all threads
        """
        self.connected = False
        self.__stop_t()
        self.devices.clear()

        if self.master is not None:
            self.master.close()

        return
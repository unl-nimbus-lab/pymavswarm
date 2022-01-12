import math
import time
import atexit
import logging
import monotonic
import threading
from .msg import *
from .state import *
from enum import Enum
from queue import Queue
from .Agent import Agent
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega



class Connection:
    """
    The connection handles all interaction with the network and the MAVLink master device.
    Input Params:
        port             [str]   : The port over which a connection should be established
        baud             [int]   : The baudrate that a connection should be established with
        source_system    [int]   : The system ID of the connection
        source_component [int]   : The component ID of the connection
        msg_timeout      [float] : The period of time that pymavswarm should re-attempt a message send if 
                                   a message isn't acknowledged by an agent
        ack_timeout      [float] : The period of time that pymavswarm should check for an acknowledgement bit
                                   from the agent that it sent a message to
        log              [bool]  : Boolean indicating whether the system should log the outputs to the terminal screen
        debug            [bool]  : Boolean indicating whether to run pymavswarm in debug mode
    """
    def __init__(self, port: str, 
                 baud: int, 
                 source_system: int=255, 
                 source_component: int=0,
                 agent_timeout: float=30.0,
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

            # Create a new tuple key
            device_tuple = (sys_id, comp_id)

            # Create a new device assigned the respective sysid:compid pair
            device = Agent(sys_id, comp_id, timeout_period=agent_timeout)

            # If the device hasn't been seen before, save it
            if device_tuple not in self.devices:
                self.devices[device_tuple] = device
            else:
                # The connection has been restored
                if self.devices[device_tuple].timeout:
                    self.logger.info(f'Connection to device {sys_id}:{comp_id} has been restored')
                
            # Update the last heartbeat variable
            self.devices[device_tuple].last_heartbeat = monotonic.monotonic()
            
            self.devices[device_tuple].timeout = False

            return


        @self.on_message(['HEARTBEAT'])
        def listener(self, msg) -> None:
            """
            Handle general device information contained within a heartbeat
            """
            # Ignore messages sent by a GCS
            if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                return

            # Get the system ID and component ID
            sys_id = msg.get_srcSystem()
            comp_id = msg.get_srcComponent()

            # Create a new tuple key
            device_tuple = (sys_id, comp_id)

            # Let the heartbeat implementation handle this
            if not device_tuple in self.devices:
                return

            self.devices[device_tuple].armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            self.devices[device_tuple].system_status = msg.system_status
            self.devices[device_tuple].vehicle_type = msg.type

            # Update the last heartbeat
            self.devices[device_tuple].last_heartbeat = monotonic.monotonic()

            try:
                # NOTE: We assume that ArduPilot will be used
                self.devices[device_tuple].flight_mode = mavutil.mode_mapping_bynumber(msg.type)[msg.custom_mode]
            except Exception as e:
                # We received an invalid message
                pass

            return
        

        @self.on_message(['GLOBAL_POSITION_INT'])
        def listener(self, msg) -> None:
            """
            Handle the a GPS position message
            """
            # Get the system ID and component ID
            sys_id = msg.get_srcSystem()
            comp_id = msg.get_srcComponent()

            # Create a new tuple key
            device_tuple = (sys_id, comp_id)

            # Let the heartbeat implementation handle this
            if not device_tuple in self.devices:
                return

            # Update the device velocity
            if self.devices[device_tuple].velocity is None:
                v = Velocity(msg.vx / 100, msg.vy / 100, msg.vz / 100)
                self.devices[device_tuple].velocity = v
            else:
                self.devices[device_tuple].velocity.vx = msg.vx / 100
                self.devices[device_tuple].velocity.vy = msg.vy / 100
                self.devices[device_tuple].velocity.vz = msg.vz / 100

            # Update the device location
            if self.devices[device_tuple].location is None:
                loc = Location(msg.lat / 1.0e7, msg.lon / 1.0e7, msg.relative_alt / 1000)
                self.devices[device_tuple].location = loc
            else:
                self.devices[device_tuple].location.latitude = msg.lat / 1.0e7
                self.devices[device_tuple].location.longitude = msg.lon / 1.0e7
                self.devices[device_tuple].location.altitude = msg.relative_alt / 1000

            return

        
        @self.on_message(['ATTITUDE'])
        def listener(self, msg) -> None:
            """
            Handle an agent attitude message
            """
            # Get the system ID and component ID
            sys_id = msg.get_srcSystem()
            comp_id = msg.get_srcComponent()

            # Create a new tuple key
            device_tuple = (sys_id, comp_id)

            # Let the heartbeat implementation handle this
            if not device_tuple in self.devices:
                return

            # Update the respective devices attitude
            if self.devices[device_tuple].attitude is None:
                att = Attitude(msg.pitch, msg.yaw, msg.roll, msg.pitchspeed, msg.yawspeed, msg.rollspeed)
                self.devices[device_tuple].attitude = att
            else:
                self.devices[device_tuple].attitude.pitch = msg.pitch
                self.devices[device_tuple].attitude.roll = msg.roll
                self.devices[device_tuple].attitude.yaw = msg.yaw
                self.devices[device_tuple].attitude.pitch_speed = msg.pitchspeed
                self.devices[device_tuple].attitude.roll_speed = msg.rollspeed
                self.devices[device_tuple].attitude.yaw_speed = msg.yawspeed
            
            return


        @self.on_message(['SYS_STATUS'])
        def listener(self, msg) -> None:
            """
            Handle the system status message containing battery state
            """
            # Get the system ID and component ID
            sys_id = msg.get_srcSystem()
            comp_id = msg.get_srcComponent()

            # Create a new tuple key
            device_tuple = (sys_id, comp_id)

            # Let the heartbeat implementation handle this
            if not device_tuple in self.devices:
                return

            # Update the battery information
            if self.devices[device_tuple].battery is None:
                batt = Battery(msg.voltage_battery, msg.current_battery, msg.battery_remaining)
                self.devices[device_tuple].battery = batt
            else:
                self.devices[device_tuple].battery.voltage = msg.voltage_battery
                self.devices[device_tuple].battery.current = msg.current_battery
                self.devices[device_tuple].battery.level = msg.battery_remaining

            return


        @self.on_message(['GPS_RAW_INT'])
        def listener(self, msg) -> None:
            """
            Handle the GPS status information
            """
            # Get the system ID and component ID
            sys_id = msg.get_srcSystem()
            comp_id = msg.get_srcComponent()

            # Create a new tuple key
            device_tuple = (sys_id, comp_id)

            # Let the heartbeat implementation handle this
            if not device_tuple in self.devices:
                return

            # Read the GPS status information
            if self.devices[device_tuple].gps_info is None:
                info = GPSInfo(msg.eph, msg.epv, msg.fix_type, msg.satellites_visible)
                self.devices[device_tuple].gps_info = info
            else:
                self.devices[device_tuple].gps_info.eph = msg.eph
                self.devices[device_tuple].gps_info.epv = msg.epv
                self.devices[device_tuple].gps_info.fix_type = msg.fix_type
                self.devices[device_tuple].gps_info.satellites_visible = msg.satellites_visible
            
            return


        @self.on_message(['EKF_STATUS_REPORT'])
        def listener(self, msg) -> None:
            """
            Handle an EKF status message
            """
            # Get the system ID and component ID
            sys_id = msg.get_srcSystem()
            comp_id = msg.get_srcComponent()

            # Create a new tuple key
            device_tuple = (sys_id, comp_id)

            # Let the heartbeat implementation handle this
            if not device_tuple in self.devices:
                return

            # Read the EKF Status information
            if self.devices[device_tuple].ekf is None:
                ekf = EKFStatus(msg.velocity_variance, 
                                msg.pos_horiz_variance, 
                                msg.pos_vert_variance, 
                                msg.compass_variance, 
                                msg.terrain_alt_variance,
                                (msg.flags & ardupilotmega.EKF_POS_HORIZ_ABS) > 0,
                                (msg.flags & ardupilotmega.EKF_CONST_POS_MODE) > 0,
                                (msg.flags & ardupilotmega.EKF_PRED_POS_HORIZ_ABS) > 0)
                self.devices[device_tuple].ekf = ekf
            else:
                # Read variance properties
                self.devices[device_tuple].ekf.velocity_variance = msg.velocity_variance
                self.devices[device_tuple].ekf.pos_horiz_variance = msg.pos_horiz_variance
                self.devices[device_tuple].ekf.pos_vert_variance = msg.pos_vert_variance
                self.devices[device_tuple].ekf.compass_variance = msg.compass_variance
                self.devices[device_tuple].ekf.terrain_alt_variance = msg.terrain_alt_variance

                # Read flags
                self.devices[device_tuple].ekf.pos_horiz_abs = (msg.flags & ardupilotmega.EKF_POS_HORIZ_ABS) > 0
                self.devices[device_tuple].ekf.const_pos_mode = (msg.flags & ardupilotmega.EKF_CONST_POS_MODE) > 0
                self.devices[device_tuple].ekf.pred_pos_horiz_abs = (msg.flags & ardupilotmega.EKF_PRED_POS_HORIZ_ABS) > 0

            return


        @self.send_message(['arm'])
        def sender(self, msg: SystemCommandMsg, fn_id: int=0) -> None:
            """
            Arm an agent
            """
            self.master.mav.command_long_send(msg.target_id, msg.target_comp,
                                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                              0,
                                              1, 0, 0, 0, 0, 0, 0)
            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the arm command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the arm command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack

        
        @self.send_message(['disarm'])
        def sender(self, msg: SystemCommandMsg, fn_id: int=0) -> None:
            """
            Disarm an agent
            """
            self.master.mav.command_long_send(msg.target_id, msg.target_comp,
                                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                              0,
                                              0, 0, 0, 0, 0, 0, 0)
            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the disarm command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the disarm command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack

        
        @self.send_message(['kill'])
        def sender(self, msg: SystemCommandMsg, fn_id: int=0) -> None:
            """
            Force disarm an agent
            """
            self.master.mav.command_long_send(msg.target_id, msg.target_comp,
                                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                              0,
                                              0, 21196, 0, 0, 0, 0, 0)
            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the kill command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the kill command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['reboot'])
        def sender(self, msg: SystemCommandMsg, fn_id: int=0) -> None:
            """
            Reboot an agent
            """
            self.master.mav.command_long_send(msg.target_id, msg.target_comp,
                                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                              0,
                                              1, 0, 0, 0, 0, 0, 0)
            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the reboot command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the reboot command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['shutdown'])
        def sender(self, msg: SystemCommandMsg, fn_id: int=0) -> None:
            """
            Shutdown an agent
            """
            self.master.mav.command_long_send(msg.target_id, msg.target_comp,
                                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                              0,
                                              2, 0, 0, 0, 0, 0, 0)
            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the shutdown command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the shutdown command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['accelcal'])
        def sender(self, msg: PreflightCalibrationMsg, fn_id: int=0) -> None:
            """
            Perform a full accelerometer calibration on the selected agent
            """
            self.master.mav.command_long_send(msg.target_id, msg.target_comp,
                                              mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 
                                              0,
                                              0, 0, 0, 0, 1, 0, 0)
            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the accelerometer calibration command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the accelerometer calibration command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['accelcalsimple'])
        def sender(self, msg: PreflightCalibrationMsg, fn_id: int=0) -> None:
            """
            Perform a simple accelerometer calibration on the selected agent
            """
            self.master.mav.command_long_send(msg.target_id, msg.target_comp,
                                              mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 
                                              0,
                                              0, 0, 0, 0, 4, 0, 0)
            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the simple accelerometer calibration command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the simple accelerometer calibration command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['ahrstrim'])
        def sender(self, msg: PreflightCalibrationMsg, fn_id: int=0) -> None:
            """
            Perform an AHRS trim on the selected agent
            """
            self.master.mav.command_long_send(msg.target_id, msg.target_comp,
                                              mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 
                                              0,
                                              0, 0, 0, 0, 2, 0, 0)
            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the AHRS trim command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the AHRS trim command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['gyrocal'])
        def sender(self, msg: PreflightCalibrationMsg, fn_id: int=0) -> None:
            """
            Perform a gyroscope calibration on the selected agent
            """
            self.master.mav.command_long_send(msg.target_id, msg.target_comp,
                                              mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 
                                              0,
                                              1, 0, 0, 0, 0, 0, 0)
            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the gyroscope calibration command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the gyroscope calibration command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['magnetometercal'])
        def sender(self, msg: PreflightCalibrationMsg, fn_id: int=0) -> None:
            """
            Perform a magnetometer calibration on the selected agent
            """
            self.master.mav.command_long_send(msg.target_id, msg.target_comp,
                                              mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 
                                              0,
                                              0, 1, 0, 0, 0, 0, 0)
            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the magnetometer calibration command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the magnetometer calibration command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['groundpressurecal'])
        def sender(self, msg: PreflightCalibrationMsg, fn_id: int=0) -> None:
            """
            Perform a ground pressure calibration on the selected agent
            """
            self.master.mav.command_long_send(msg.target_id, msg.target_comp,
                                              mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 
                                              0,
                                              0, 0, 3, 0, 0, 0, 0)
            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the ground pressue calibration command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the ground pressue calibration command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['airspeedcal'])
        def sender(self, msg: PreflightCalibrationMsg, fn_id: int=0) -> None:
            """
            Perform airspeed calibration on the selected agent
            """
            self.master.mav.command_long_send(msg.target_id, msg.target_comp,
                                              mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 
                                              0,
                                              0, 0, 0, 0, 0, 2, 0)
            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the airspeed calibration command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the airspeed calibration command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['barotempcal'])
        def sender(self, msg: PreflightCalibrationMsg, fn_id: int=0) -> None:
            """
            Perform a barometer temperature calibration on the selected agent
            """
            self.master.mav.command_long_send(msg.target_id, msg.target_comp,
                                              mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 
                                              0,
                                              0, 0, 0, 0, 0, 0, 3)
            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the barometer temperature calibration command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the barometer temperature calibration command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['stabilize'])
        def sender(self, msg: FlightModeMsg, fn_id: int=0) -> None:
            """
            Set an agent to STABILIZE mode
            """
            # Reset target
            self.master.target_system = msg.target_id
            self.master.target_component = msg.target_comp

            # Send flight mode
            self.master.set_mode(self.master.mode_mapping()['STABILIZE'])

            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the flight mode STABILIZE command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the flight mode STABILIZE command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['acro'])
        def sender(self, msg: FlightModeMsg, fn_id: int=0) -> None:
            """
            Set an agent to ACRO mode
            """
            # Reset target
            self.master.target_system = msg.target_id
            self.master.target_component = msg.target_comp

            # Send flight mode
            self.master.set_mode(self.master.mode_mapping()['ACRO'])

            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the flight mode ACRO command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the flight mode ACRO command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack

        
        @self.send_message(['althold'])
        def sender(self, msg: FlightModeMsg, fn_id: int=0) -> None:
            """
            Set an agent to ALT_HOLD mode
            """
            # Reset target
            self.master.target_system = msg.target_id
            self.master.target_component = msg.target_comp

            # Send flight mode
            self.master.set_mode(self.master.mode_mapping()['ALT_HOLD'])

            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the flight mode ALT_HOLD command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the flight mode ALT_HOLD command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['auto'])
        def sender(self, msg: FlightModeMsg, fn_id: int=0) -> None:
            """
            Set an agent to AUTO mode
            """
            # Reset target
            self.master.target_system = msg.target_id
            self.master.target_component = msg.target_comp

            # Send flight mode
            self.master.set_mode(self.master.mode_mapping()['AUTO'])

            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the flight mode AUTO command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the flight mode AUTO command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack

        
        @self.send_message(['loiter'])
        def sender(self, msg: FlightModeMsg, fn_id: int=0) -> None:
            """
            Set an agent to LOITER mode
            """
            # Reset target
            self.master.target_system = msg.target_id
            self.master.target_component = msg.target_comp

            # Send flight mode
            self.master.set_mode(self.master.mode_mapping()['LOITER'])

            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the flight mode LOITER command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the flight mode LOITER command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['rtl'])
        def sender(self, msg: FlightModeMsg, fn_id: int=0) -> None:
            """
            Set an agent to RTL mode
            """
            # Reset target
            self.master.target_system = msg.target_id
            self.master.target_component = msg.target_comp

            # Send flight mode
            self.master.set_mode(self.master.mode_mapping()['RTL'])

            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the flight mode RTL command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the flight mode RTL command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['land'])
        def sender(self, msg: FlightModeMsg, fn_id: int=0) -> None:
            """
            Set an agent to LAND mode
            """
            # Reset target
            self.master.target_system = msg.target_id
            self.master.target_component = msg.target_comp

            # Send flight mode
            self.master.set_mode(self.master.mode_mapping()['LAND'])

            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the flight mode LAND command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the flight mode LAND command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['throw'])
        def sender(self, msg: FlightModeMsg, fn_id: int=0) -> None:
            """
            Set an agent to THROW mode
            """
            # Reset target
            self.master.target_system = msg.target_id
            self.master.target_component = msg.target_comp

            # Send flight mode
            self.master.set_mode(self.master.mode_mapping()['THROW'])

            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the flight mode THROW command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the flight mode THROW command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['systemid'])
        def sender(self, msg: FlightModeMsg, fn_id: int=0) -> None:
            """
            Set an agent to SYSTEM ID mode
            """
            # Reset target
            self.master.target_system = msg.target_id
            self.master.target_component = msg.target_comp

            # Send flight mode
            self.master.set_mode(self.master.mode_mapping()['SYSTEMID'])

            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the flight mode SYSTEMID command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the flight mode SYSTEMID command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['guided'])
        def sender(self, msg: FlightModeMsg, fn_id: int=0) -> None:
            """
            Set an agent to GUIDED mode
            """
            # Reset target
            self.master.target_system = msg.target_id
            self.master.target_component = msg.target_comp

            # Send flight mode
            self.master.set_mode(self.master.mode_mapping()['GUIDED'])

            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the flight mode GUIDED command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the flight mode GUIDED command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack

        
        @self.send_message(['startpath'])
        def sender(self, msg: HRLMsg, fn_id: int=0) -> None:
            """
            Start path execution on the respective agent
            """
            # Reset target
            self.master.target_system = msg.target_id
            self.master.target_component = msg.target_comp

            # Send flight mode
            self.master.mav.named_value_int_send(int(time.time()), str.encode('hrl-state-arg'), 0)

            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the start flight path HRL command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the start flight path HRL command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack

        
        @self.send_message(['stoppath'])
        def sender(self, msg: HRLMsg, fn_id: int=0) -> None:
            """
            Stop path execution on the respective agent
            """
            # Reset target
            self.master.target_system = msg.target_id
            self.master.target_component = msg.target_comp

            # Send flight mode
            self.master.mav.named_value_int_send(int(time.time()), str.encode('hrl-state-arg'), 1)

            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the stop flight path HRL command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the stop flight path HRL command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['airspeed'])
        def sender(self, msg: FlightSpeedMsg, fn_id: int=0) -> None:
            """
            Set a new airspeed on an agent
            """
            self.master.mav.command_long_send(msg.target_id, msg.target_comp,
                                              mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 
                                              0,
                                              0, msg.speed, -1, 0, 0, 0, 0)

            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the air speed command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the air speed command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['groundspeed'])
        def sender(self, msg: FlightSpeedMsg, fn_id: int=0) -> None:
            """
            Set a new airspeed on an agent
            """
            self.master.mav.command_long_send(msg.target_id, msg.target_comp,
                                              mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 
                                              0,
                                              1, msg.speed, -1, 0, 0, 0, 0)

            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the ground speed command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the ground speed command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['climbspeed'])
        def sender(self, msg: FlightSpeedMsg, fn_id: int=0) -> None:
            """
            Set a new airspeed on an agent
            """
            self.master.mav.command_long_send(msg.target_id, msg.target_comp,
                                              mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 
                                              0,
                                              2, msg.speed, -1, 0, 0, 0, 0)

            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the climb speed command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the climb speed command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['descentspeed'])
        def sender(self, msg: FlightSpeedMsg, fn_id: int=0) -> None:
            """
            Set a new airspeed on an agent
            """
            self.master.mav.command_long_send(msg.target_id, msg.target_comp,
                                              mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 
                                              0,
                                              3, msg.speed, -1, 0, 0, 0, 0)

            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the descent speed command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the descent speed command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack

        
        @self.send_message(['simpletakeoff'])
        def sender(self, msg: TakeoffMsg, fn_id: int=0) -> None:
            """
            Perform a simple takeoff command (just takeoff to a set altitude)
            Note that acknowledgement of this command does not indicate that the 
            altitude was reached, but rather that the system will attempt to reach 
            the specified altitude
            """
            if msg.altitude < 0 or math.isinf(msg.altitude) or math.isnan(msg.altitude):
                self.logger.exception(f'An invalid takeoff altitude was provided ({msg.altitude}). Please send a valid takeoff altitude')
                return

            self.master.mav.command_long_send(msg.target_id, msg.target_comp,
                                              mavutil.mavlink.MAV_CMD_NAV_LAND, 
                                              0,
                                              0, 0, 0, 0, 0, 0, msg.altitude)

            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the simple takeoff command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the simple takeoff command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['takeoff'])
        def sender(self, msg: TakeoffMsg, fn_id: int=0) -> None:
            """
            Perform a takeoff command (use lat, lon, and alt)
            Note that acknowledgement of this command does not indicate that the 
            altitude was reached, but rather that the system will attempt to reach 
            the specified altitude
            """
            if msg.altitude < 0 or math.isinf(msg.altitude) or math.isnan(msg.altitude):
                self.logger.exception(f'An invalid takeoff altitude was provided ({msg.altitude}). Please send a valid takeoff altitude')
                return

            self.master.mav.command_long_send(msg.target_id, msg.target_comp,
                                              mavutil.mavlink.MAV_CMD_NAV_LAND, 
                                              0,
                                              0, 0, 0, 0, msg.lat, msg.lon, msg.altitude)

            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the takeoff command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the takeoff command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['simplewaypoint'])
        def sender(self, msg: WaypointMsg, fn_id: int=0) -> None:
            """
            Perform a simple waypoint command (just lat, lon, and alt)
            Note that acknowledgement of this command does not indicate that the 
            waypoint was reached, but rather that the system will attempt to reach 
            the specified waypoint
            """
            if msg.altitude < 0 or math.isinf(msg.altitude) or math.isnan(msg.altitude):
                self.logger.exception(f'An invalid takeoff altitude was provided ({msg.altitude}). Please send a valid waypoint altitude')
                return

            self.master.mav.mission_item_send(msg.target_id, msg.target_comp, 
                                              0, 
                                              mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                              mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 
                                              0, 0, 0, 0, 0, 0, 
                                              msg.lat, msg.lon, msg.alt)

            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the simple waypoint command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the simple waypoint command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


        @self.send_message(['waypoint'])
        def sender(self, msg: WaypointMsg, fn_id: int=0) -> bool:
            """
            Perform a waypoint navigation command
            Note that acknowledgement of this command does not indicate that the 
            waypoint was reached, but rather that the system will attempt to reach 
            the specified waypoint
            """
            if msg.altitude < 0 or math.isinf(msg.altitude) or math.isnan(msg.altitude):
                self.logger.exception(f'An invalid takeoff altitude was provided ({msg.altitude}). Please send a valid waypoint altitude')
                return

            self.master.mav.mission_item_send(msg.target_id, msg.target_comp, 
                                              0, 
                                              mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                              mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 
                                              0, 0, 
                                              msg.hold, msg.accept_radius, msg.pass_radius, msg.yaw, msg.lat, msg.lon, msg.alt)

            ack = False

            if self.__ack_sys_cmd(timeout=msg.ack_timeout):
                ack = True
            else:
                if msg.retry:
                    if self.__retry_msg_send(msg, self.message_senders[msg.get_type()][fn_id]):
                        ack = True
                    
            if ack:
                self.logger.info(f'Successfully acknowledged reception of the waypoint command sent to Agent ({msg.target_id}, {msg.target_comp})')    
            else:
                self.logger.error(f'Failed to acknowledge reception of the waypoint command sent to Agent ({msg.target_id}, {msg.target_comp})')

            return ack


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
        This implementation has been inspired by the following source:
            * Project: Dronekit
            * Repository: dronekit
            * URL: https://github.com/dronekit/dronekit-python
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
        This implementation has been inspired by the following source:
            * Project: Dronekit
            * Repository: dronekit
            * URL: https://github.com/dronekit/dronekit-python
        """
        if msg not in self.message_listeners:
            self.message_listeners[msg] = []
        
        if fn not in self.message_listeners[msg]:
            self.message_listeners[msg].append(fn)

        return

    
    def send_message(self, msg):
        """
        Decorator used to create a sender for a mavlink message
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
        Add a new function to the dictionary of message senders
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
            for key in self.devices:
                if self.devices[key].last_heartbeat is not None:
                    self.devices[key].timeout = (monotonic.monotonic() - self.devices[key].last_heartbeat) >= self.devices[key].timeout

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
                msg = self.outgoing_msgs.get(timeout=1)

                # Send the message if there is a message sender for it
                if msg.get_type() in self.message_senders:
                    for fn_id, fn in enumerate(self.message_senders[msg.get_type()]):
                        try:
                            fn(self, msg, fn_id=fn_id)
                        except Exception:
                            self.logger.exception(f'Exception in message sender for {msg.get_type()}', exc_info=True)

        return


    def __retry_msg_send(self, msg, fn) -> bool:
        """
        Retry a message send until the an acknowledgement is received or a timeout occurs
        """
        ack = False
        start_time = time.time()
        
        # Don't let the message come back here and create an infinite loop
        msg.retry = False

        while time.time() - start_time >= msg.msg_timeout:
            # Reattempt the message send
            if fn(self, msg):
                ack = True
                break

        return ack


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
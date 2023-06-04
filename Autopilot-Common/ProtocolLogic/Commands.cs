using System;
using System.Net;
using System.Reflection;
using System.Text;

namespace AutopilotCommon
{
    public class Commands
    {
        private long startTime;
        //private const byte systemID = 1;
        //private const byte componentID = 1;
        private static MAVLink.MavlinkParse parser = new MAVLink.MavlinkParse();
        private Data data;
        private Ap ap;
        private ParameterHandler parameters;
        private Action<string> Log;
        public Commands(Data data, Ap ap, Action<string> Log, ParameterHandler parameters)
        {
            this.Log = Log;
            startTime = DateTime.UtcNow.Ticks;
            this.data = data;
            this.ap = ap;
            this.parameters = parameters;
        }
        public void ConnectEvent(ClientObject client)
        {
            client.requestedRates[MAVLink.MAVLINK_MSG_ID.HEARTBEAT] = 1f;
            Log("Client connected");
        }

        public void DisconnectEvent(ClientObject client)
        {
            Log("Client disconnected");
        }

        //Commands
        [ReceiveCommand(MAVLink.MAV_CMD.DO_SET_SERVO)]
        public void RequestSetServo(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            AckCommand(client, command, MAVLink.MAV_CMD_ACK.OK);
            Log($"REQUEST SET SERVO: {command.param1}, PWM {command.param2}");
#pragma warning disable CS0219 // Variable is assigned but its value is never used
            //MAVLink.mavlink_servo_output_raw_t servo = new MAVLink.mavlink_servo_output_raw_t();
#pragma warning restore CS0219 // Variable is assigned but its value is never used
            //client.SendMessage(command);
        }

        [ReceiveCommand(MAVLink.MAV_CMD.PREFLIGHT_CALIBRATION)]
        public void PreflightCalibration(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            Log($"PREFLIGHT_CALIBRATE");
            AckCommand(client, command, MAVLink.MAV_CMD_ACK.OK);
        }

        [ReceiveCommand(MAVLink.MAV_CMD.REQUEST_PROTOCOL_VERSION)]
        public void RequestProtocolVersion(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            AckCommand(client, command, MAVLink.MAV_CMD_ACK.OK);
            MAVLink.mavlink_protocol_version_t version = new MAVLink.mavlink_protocol_version_t(){
                min_version = 1,
                version = 2,
                max_version = 2,
            };
            client.SendMessage(version);
            Log($"REQUEST_PROTOCOL_VERSION, VERSION = {version.version}");
        }

        [ReceiveCommand(MAVLink.MAV_CMD.SET_MESSAGE_INTERVAL)]
        public void MessageInterval(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            AckCommand(client, command, MAVLink.MAV_CMD_ACK.OK);
            Log($"SET_MESSAGE_INTERVAL: {(MAVLink.MAVLINK_MSG_ID)command.param1} = {command.param2}");
            client.requestedRates[(MAVLink.MAVLINK_MSG_ID)command.param1] = command.param2 * 1;
        }

        //a "one shot" version of MessageInterwal
        [ReceiveCommand(MAVLink.MAV_CMD.REQUEST_MESSAGE)]
        //TODO: Call the approriate callback
        public void RequestMessage(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            AckCommand(client, command, MAVLink.MAV_CMD_ACK.OK);
            Log($"REQUEST_MESSAGE: {(MAVLink.MAVLINK_MSG_ID)command.param1} = {command.param2}");
            //client.requestedRates[(MAVLink.MAVLINK_MSG_ID)command.param1] = command.param2 * 1;
        }
 /*
       FLIGHTMODES

       0 = MANUAL
       1 = CIRCLE
       2 = STABILIZE
       3 = TRAINING
       4 = ACRO
       5 = FBWA
       6 = FBWB
       7 = CRUISE
       8 = AUTOTUNE
       10 = AUTO
       11 = RTL
       12 = LOITER
       14 = AVOID_ADSB
       15 = GUIDED
       17 = QSTABILIZE
       18 = QHOVER
       19 = QLOITER
       20 = QLAND
       21 = QRTL
       22 = QAUTOTUNE
       23 = QACRO
       24 = THERMAL
        */
        [ReceiveCommand(MAVLink.MAV_CMD.REQUEST_AUTOPILOT_CAPABILITIES)]
        public void RequestAutopilot(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            byte[] emptyByte = new byte[8];
            Log("REQUEST AUTOPILOT");
            AckCommand(client, command, MAVLink.MAV_CMD_ACK.OK);
            MAVLink.mavlink_autopilot_version_t autopilot2 = new MAVLink.mavlink_autopilot_version_t(){
                capabilities = 
                              (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.MAVLINK2
                            | (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.MISSION_FLOAT
                            | (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.PARAM_FLOAT
                            //| (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.MISSION_INT
                            //| (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.COMMAND_INT
                            //| (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.PARAM_UNION
                            | (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.FLIGHT_INFORMATION
                            | (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.FLIGHT_TERMINATION
                            | (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.SET_ACTUATOR_TARGET
                            | (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.FLIGHT_TERMINATION,
                uid = 13546,
                flight_sw_version = 1,
                middleware_sw_version = 1,
                os_sw_version = 1,
                board_version = 1,
                vendor_id = 0,
                product_id = 0,
                flight_custom_version = emptyByte,
                middleware_custom_version = emptyByte,
                os_custom_version = emptyByte,
                uid2 = emptyByte
            };

            //MAVLink.mavlink_autopilot_version_t autopilot = new MAVLink.mavlink_autopilot_version_t();
            //autopilot.capabilities |= (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.COMMAND_INT;
            //autopilot.capabilities |= (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.FLIGHT_INFORMATION;
            //autopilot.capabilities |= (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.FLIGHT_TERMINATION;
            //autopilot.capabilities |= (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.MAVLINK2;
            ////autopilot.capabilities |= (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.MISSION_FLOAT;
            //autopilot.capabilities |= (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.PARAM_FLOAT;
            //autopilot.capabilities |= (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.SET_ACTUATOR_TARGET;
            //autopilot.board_version = 1;
            //autopilot.flight_sw_version = 1;
            //autopilot.os_sw_version = 1;
            //byte[] emptyByte = new byte[8];
            //autopilot.flight_custom_version = emptyByte;
            //autopilot.middleware_custom_version = emptyByte;
            //autopilot.os_custom_version = emptyByte;
            //autopilot.product_id = 0;
            //autopilot.vendor_id = 0;
            //autopilot.uid = 13546;
            client.SendMessage(autopilot2);
        }

        [ReceiveCommand(MAVLink.MAV_CMD.COMPONENT_ARM_DISARM)]
        public void RequestArmDisarm(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            AckCommand(client, command, MAVLink.MAV_CMD_ACK.OK);
            Log($"REQUEST ARM DISARM: STATE {command.param1}, {command.param2}");
            if((int)ap.state==3)
            ap.state = MAVLink.MAV_STATE.ACTIVE;
            if((int)ap.state==4)
            ap.state = MAVLink.MAV_STATE.STANDBY;
            client.SendMessage(command);
        }
        public void AckCommand(ClientObject client, MAVLink.mavlink_command_long_t command, MAVLink.MAV_CMD_ACK ackType)
        {
            MAVLink.mavlink_command_ack_t ack = new MAVLink.mavlink_command_ack_t(){
                command = command.command,
                result = (byte)ackType,
                target_system = command.target_system,
                target_component = command.target_component,
            };
        }

    }
}
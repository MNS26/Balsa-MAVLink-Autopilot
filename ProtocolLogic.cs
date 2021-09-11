using System;
using component_information;
using System.Collections.Generic;
using DarkLog;
using System.Security.Cryptography;

namespace Autopilot
{
    public class ProtocolLogic
    {


        private long startTime;
        private const byte systemID = 1;
        private const byte componentID = 1;
        private static MAVLink.MavlinkParse parser = new MAVLink.MavlinkParse();
        private DataStore data;
        private ModLog log;

        public ProtocolLogic(DataStore data, ModLog log)
        {
            startTime = DateTime.UtcNow.Ticks;
            this.data = data;
            this.log = log;
        }

        public void ConnectEvent(ClientObject client)
        {
            client.requestedRates[MAVLink.MAVLINK_MSG_ID.HEARTBEAT] = 2f;
            log.Log("Client connected");
        }

        public void DisconnectEvent(ClientObject client)
        {
            log.Log("Client disconnected");
        }

        public void ReceiveSetRate(ClientObject client, MAVLink.MAVLinkMessage rawMessage)
        {
            //client.requestedRates[message.messageType] = message.rate;
        }

        public void SendHeartbeat(ClientObject client)
        {
            MAVLink.mavlink_heartbeat_t message = new MAVLink.mavlink_heartbeat_t();
            message.custom_mode = 0;
            message.type = (byte)MAVLink.MAV_TYPE.FIXED_WING;
            message.autopilot = (byte)MAVLink.MAV_AUTOPILOT.ARDUPILOTMEGA;
            message.base_mode = (byte)MAVLink.MAV_MODE.AUTO_ARMED;
            message.system_status = (byte)MAVLink.MAV_STATE.ACTIVE;
            message.mavlink_version = (byte)MAVLink.MAVLINK_VERSION;
            client.SendMessage(message);
        }

        public void SendAttitude(ClientObject client)
        {
            MAVLink.mavlink_attitude_t message = new MAVLink.mavlink_attitude_t();
            message.pitch = data.pitch;
            message.roll = data.roll;
            message.yaw = data.yaw;
            message.pitchspeed = 0;
            message.rollspeed = 0;
            message.yawspeed = 0;
            message.time_boot_ms = GetUptime();
            client.SendMessage(message);
        }

        public void SendPosition(ClientObject client)
        {
            MAVLink.mavlink_global_position_int_t message = new MAVLink.mavlink_global_position_int_t();
            message.lat = data.latitude;
            message.lon = data.longitude;
            message.alt = data.altitude;
            message.relative_alt = data.altitude;
            message.hdg = 0;
            message.vx = 0;
            message.vy = 0;
            message.vz = 0;
            message.time_boot_ms = GetUptime();
            client.SendMessage(message);
        }
/*
        public  void ParamRequestList(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            Console.WriteLine("PARAM_REQUEST_LIST");
            MAVLink.mavlink_param_request_list_t message = (MAVLink.mavlink_param_request_list_t)messageRaw.data;
            for (int i = 0; i < parameters.Count; i++)
            {
                Parameter p = parameters[i];
                MAVLink.mavlink_param_value_t sendMessage = new MAVLink.mavlink_param_value_t();
                sendMessage.param_id = p.GetIDBytes();
                sendMessage.param_type = (byte)p.type;
                sendMessage.param_value = p.value;
                sendMessage.param_count = (ushort)parameters.Count;
                sendMessage.param_index = (ushort)i;
                client.SendMessage(sendMessage);
            }
        }
*/
        public void RequestDataStream(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            //TODO: Implement ALL of these.
            MAVLink.mavlink_request_data_stream_t message = (MAVLink.mavlink_request_data_stream_t)messageRaw.data;
            Console.WriteLine($"REQUEST_DATA_STREAM {message.req_stream_id} = {message.req_message_rate}");
            switch ((MAVLink.MAV_DATA_STREAM)message.req_stream_id)
            {
                case MAVLink.MAV_DATA_STREAM.ALL:
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.RAW_IMU] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.GPS_RAW_INT] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.GPS_STATUS] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_SCALED] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_RAW] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.SERVO_OUTPUT_RAW] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.LOCAL_POSITION_NED] = 1f / message.req_message_rate;
                    break;
                case MAVLink.MAV_DATA_STREAM.RAW_SENSORS:
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.RAW_IMU] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.GPS_RAW_INT] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.GPS_STATUS] = 1f / message.req_message_rate;
                    break;
                case MAVLink.MAV_DATA_STREAM.EXTENDED_STATUS:
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.GPS_STATUS] = 1f / message.req_message_rate;
                    //Can't find CONTROL_STATUS or AUX_STATUS
                    break;
                case MAVLink.MAV_DATA_STREAM.RC_CHANNELS:
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_SCALED] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_RAW] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.SERVO_OUTPUT_RAW] = 1f / message.req_message_rate;
                    break;
                case MAVLink.MAV_DATA_STREAM.RAW_CONTROLLER:
                    //Can't find these messages
                    break;
                case MAVLink.MAV_DATA_STREAM.POSITION:
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.LOCAL_POSITION_NED] = 1f / message.req_message_rate;
                    //Can't find GLOBAL_POSITION
                    break;
                case MAVLink.MAV_DATA_STREAM.EXTRA1:
                    break;
                case MAVLink.MAV_DATA_STREAM.EXTRA2:
                    break;
                case MAVLink.MAV_DATA_STREAM.EXTRA3:
                    break;
            }
        }

        public void SendRawIMU(ClientObject client)
        {
            MAVLink.mavlink_raw_imu_t message = new MAVLink.mavlink_raw_imu_t();
            DateTime epoch = new DateTime(1970, 1, 1, 0, 0, 0, 0, DateTimeKind.Utc);
            TimeSpan diff = DateTime.UtcNow - epoch;
            message.time_usec = (ulong)(diff.TotalMilliseconds * 1000);
            message.xacc = 0;
            message.yacc = 0;
            message.zacc = 0;
            message.xgyro = 0;
            message.ygyro = 0;
            message.zgyro = 0;
            message.xmag = 0;
            message.ymag = 0;
            message.zmag = 0;
            message.id = 0;
            message.temperature = 6000;
            client.SendMessage(message);
        }

        public void RequestAutopilot(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            Console.WriteLine("REQUEST AUTOPILOT");
            AckCommand(client, command, MAVLink.MAV_CMD_ACK.OK);
            MAVLink.mavlink_autopilot_version_t autopilot = new MAVLink.mavlink_autopilot_version_t();
            autopilot.capabilities = autopilot.capabilities | (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.MISSION_FLOAT;
            autopilot.capabilities = autopilot.capabilities | (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.PARAM_FLOAT;
            autopilot.capabilities = autopilot.capabilities | (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.COMMAND_INT;
            autopilot.board_version = 1;
            autopilot.flight_sw_version = 1;
            autopilot.os_sw_version = 1;
            byte[] emptyByte = new byte[8];
            autopilot.flight_custom_version = emptyByte;
            autopilot.middleware_custom_version = emptyByte;
            autopilot.os_custom_version = emptyByte;
            autopilot.product_id = 0;
            autopilot.vendor_id = 0;
            autopilot.uid = 1;
            client.SendMessage(autopilot);
        }

        public void SendGPSStatus(ClientObject client)
        {
            MAVLink.mavlink_gps_status_t message = new MAVLink.mavlink_gps_status_t();
            message.satellites_visible = 10;
            byte[] prn = new byte[20];
            byte[] used = new byte[20];
            byte[] ele = new byte[20];
            byte[] azi = new byte[20];
            byte[] snr = new byte[20];
            for (int i = 0; i < 10; i++)
            {
                prn[i] = (byte)(i + 1);
                if (i <= 8)
                {
                    used[i] = 1;
                }
                ele[i] = (byte)(45 + i * 2);
                azi[i] = (byte)(i * 20);
                snr[i] = (byte)(30 + i);
            }
            message.satellite_prn = prn;
            message.satellite_used = used;
            message.satellite_elevation = ele;
            message.satellite_azimuth = azi;
            message.satellite_snr = snr;
            client.SendMessage(message);
        }

        public void AckCommand(ClientObject client, MAVLink.mavlink_command_long_t command, MAVLink.MAV_CMD_ACK ackType)
        {
            MAVLink.mavlink_command_ack_t ack = new MAVLink.mavlink_command_ack_t();
            ack.command = command.command;
            ack.result = (byte)ackType;
            ack.target_system = command.target_system;
            ack.target_component = command.target_component;
        }

        private uint GetUptime()
        {
            long timeMS = (DateTime.UtcNow.Ticks - startTime) / TimeSpan.TicksPerMillisecond;
            return (uint)timeMS;
        }
    }
}

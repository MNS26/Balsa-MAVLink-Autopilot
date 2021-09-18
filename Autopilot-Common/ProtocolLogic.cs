using System;
using System.Text;
using System.Collections.Generic;
using System.Linq;

namespace AutopilotCommon
{
    public class ProtocolLogic
    {

        private long startTime;
        private const byte systemID = 1;
        private const byte componentID = 1;
        private static MAVLink.MavlinkParse parser = new MAVLink.MavlinkParse();
        private DataStore data;
        private ParameterHandler parameters;
        private Action<string> Log;

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

        public ProtocolLogic(DataStore data, Action<string> Log, ParameterHandler parameters)
        {
            this.Log = Log;
            startTime = DateTime.UtcNow.Ticks;
            this.data = data;
            this.parameters = parameters;
        }

        public void ConnectEvent(ClientObject client)
        {
            client.requestedRates[MAVLink.MAVLINK_MSG_ID.HEARTBEAT] = 0.25f;
            Log("Client connected");
        }

        public void DisconnectEvent(ClientObject client)
        {
            Log("Client disconnected");
        }

        public void ReceiveSetRate(ClientObject client, MAVLink.MAVLinkMessage rawMessage)
        {
            //client.requestedRates[message.messageType] = message.rate;
        }

        public void ParamRequestList(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            Log("PARAM_REQUEST_LIST");
            MAVLink.mavlink_param_request_list_t message = (MAVLink.mavlink_param_request_list_t)messageRaw.data;
            int index = 0;
            int count = parameters.GetCount();
            foreach (Parameter p in parameters)
            {
                MAVLink.mavlink_param_value_t sendMessage = new MAVLink.mavlink_param_value_t();
                sendMessage.param_id = p.GetIDBytes();
                sendMessage.param_type = (byte)p.type;
                sendMessage.param_value = p.value;
                sendMessage.param_count = (ushort)count;
                sendMessage.param_index = (ushort)index++;
                client.SendMessage(sendMessage);
            }
        }

        public void RequestDataStream(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            //TODO: Implement ALL of these.
            MAVLink.mavlink_request_data_stream_t message = (MAVLink.mavlink_request_data_stream_t)messageRaw.data;
            Log($"REQUEST_DATA_STREAM  TYPE:{(MAVLink.MAV_DATA_STREAM)message.req_stream_id}  =  {message.req_message_rate}");
            switch ((MAVLink.MAV_DATA_STREAM)message.req_stream_id)
            {
                case MAVLink.MAV_DATA_STREAM.ALL:
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.ATTITUDE] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.RAW_IMU] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.GPS_RAW_INT] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.GPS_STATUS] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.RADIO_STATUS] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_SCALED] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_RAW] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.SERVO_OUTPUT_RAW] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.LOCAL_POSITION_NED] = 1f / message.req_message_rate;
                    //client.requestedRates[MAVLink.MAVLINK_MSG_ID.RAW_RPM] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.AUTOPILOT_VERSION] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.VFR_HUD] = 1f / message.req_message_rate;
                    break;
                case MAVLink.MAV_DATA_STREAM.RAW_SENSORS:
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.ATTITUDE] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.RAW_IMU] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.GPS_RAW_INT] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.GPS_STATUS] = 1f / message.req_message_rate;
                    break;
                case MAVLink.MAV_DATA_STREAM.EXTENDED_STATUS:
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.LOCAL_POSITION_NED] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.GPS_STATUS] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.RADIO_STATUS] = 1f / message.req_message_rate;
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
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.VFR_HUD] = 1f / message.req_message_rate;
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

        public void SystemTime(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            MAVLink.mavlink_system_time_t message = (MAVLink.mavlink_system_time_t)messageRaw.data;
            Log($"SYSTEM_TIME {message.time_unix_usec}");
        }

        public void RequestProtocolVersion(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            AckCommand(client, command, MAVLink.MAV_CMD_ACK.OK);
            MAVLink.mavlink_protocol_version_t version = new MAVLink.mavlink_protocol_version_t();
            version.min_version = 1;
            version.max_version = 3;
            version.version = 2;
            client.SendMessage(version);
            Log($"REQUEST_PROTOCOL_VERSION, VERSION = {version.version}");
        }

        public void MessageInterval(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            AckCommand(client, command, MAVLink.MAV_CMD_ACK.OK);
            Log($"SET_MESSAGE_INTERVAL: {(MAVLink.MAVLINK_MSG_ID)command.param1} = {command.param2}");
            client.requestedRates[(MAVLink.MAVLINK_MSG_ID)command.param1] = command.param1 * 1000000;
        }

        public void RequestAutopilot(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            Log("REQUEST AUTOPILOT");
            AckCommand(client, command, MAVLink.MAV_CMD_ACK.OK);
            MAVLink.mavlink_autopilot_version_t autopilot = new MAVLink.mavlink_autopilot_version_t();
            autopilot.capabilities |= (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.MISSION_FLOAT;
            autopilot.capabilities |= (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.PARAM_FLOAT;
            autopilot.capabilities |= (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.COMMAND_INT;
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

        public void SendHeartbeat(ClientObject client)
        {
            MAVLink.mavlink_heartbeat_t message = new MAVLink.mavlink_heartbeat_t();
            message.custom_mode = 0;
            message.type = (byte)MAVLink.MAV_TYPE.FIXED_WING;
            message.autopilot = (byte)MAVLink.MAV_AUTOPILOT.ARDUPILOTMEGA;
            message.base_mode = (byte)MAVLink.MAV_MODE.MANUAL_DISARMED;
            message.system_status = (byte)MAVLink.MAV_STATE.ACTIVE;
            message.mavlink_version = (byte)MAVLink.MAVLINK_VERSION;
            client.SendMessage(message);


            uint sensors = (uint)(MAVLink.MAV_SYS_STATUS_SENSOR._3D_GYRO | MAVLink.MAV_SYS_STATUS_SENSOR._3D_ACCEL | MAVLink.MAV_SYS_STATUS_SENSOR._3D_MAG | MAVLink.MAV_SYS_STATUS_SENSOR.ABSOLUTE_PRESSURE | MAVLink.MAV_SYS_STATUS_SENSOR.BATTERY | MAVLink.MAV_SYS_STATUS_SENSOR.GPS);
            MAVLink.mavlink_sys_status_t sysStatus = new MAVLink.mavlink_sys_status_t();
            sysStatus.onboard_control_sensors_present = sensors;
            sysStatus.onboard_control_sensors_enabled = sensors;
            sysStatus.onboard_control_sensors_health = sensors;
            //1%
            sysStatus.load = 50;
            sysStatus.voltage_battery = 11000;
            sysStatus.current_battery = 1000;
            client.SendMessage(sysStatus);
        }

        public void SendAttitude(ClientObject client)
        {
            MAVLink.mavlink_attitude_t message = new MAVLink.mavlink_attitude_t();
            message.pitch = data.radpitch;
            message.roll = data.radroll;
            message.yaw = data.radyaw;
            message.pitchspeed = 0;
            message.rollspeed = 0;
            message.yawspeed = 0;
            message.time_boot_ms = GetUptime();
            client.SendMessage(message);
        }

        public void SendGPSGlobalPosition(ClientObject client)
        {
            MAVLink.mavlink_global_position_int_t message = new MAVLink.mavlink_global_position_int_t();
            message.lat = data.latitude;
            message.lon = data.longitude;
            message.alt = (int)data.altitude;
            message.relative_alt = 0;//(int)data.altitude;
            message.hdg = (ushort)data.heading;
            message.vx = 0;
            message.vy = 0;
            message.vz = 0;
            message.time_boot_ms = GetUptime();
            client.SendMessage(message);
        }

        public void SendVFRHud(ClientObject client)
        {
            MAVLink.mavlink_vfr_hud_t message = new MAVLink.mavlink_vfr_hud_t();
            message.airspeed = data.iaspeed;
            message.climb = data.cr;
            message.alt = data.altitude;
            message.heading = (short)data.heading;
            //TODO
            message.groundspeed = 0;
            message.throttle = 0;
        }

        public void SendGPSRaw(ClientObject client)
        {
            MAVLink.mavlink_gps_raw_int_t message = new MAVLink.mavlink_gps_raw_int_t();
            message.lat = data.latitude;
            message.lon = data.longitude;
            message.alt = (int)data.altitude;
            message.eph = 300;
            message.epv = 500;
            message.vel = 1000;
            message.cog = 25000;
            message.satellites_visible = 10;
            message.alt_ellipsoid = (int)data.altitude;
            message.h_acc = 10;
            message.v_acc = 10;
            message.vel_acc = 10;
            message.hdg_acc = 10;
            message.yaw = (ushort)data.yaw;
            client.SendMessage(message);
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

        public void SendRawIMU(ClientObject client)
        {
            MAVLink.mavlink_raw_imu_t message = new MAVLink.mavlink_raw_imu_t();
            DateTime epoch = new DateTime(1970, 1, 1, 0, 0, 0, 0, DateTimeKind.Utc);
            TimeSpan diff = DateTime.UtcNow - epoch;
            message.time_usec = (ulong)(diff.TotalMilliseconds * 1000);
            message.xacc = (short)data.accx;
            message.yacc = (short)data.accy;
            message.zacc = (short)data.accz;
            message.xgyro = (short)data.gyrox;
            message.ygyro = (short)data.gyroy;
            message.zgyro = (short)data.gyroz;
            message.xmag = (short)data.magx;
            message.ymag = (short)data.magy;
            message.zmag = (short)data.magz;
            message.id = 0;
            message.temperature = 6000;
            client.SendMessage(message);
        }

        public void SendRPM(ClientObject client)
        {
            MAVLink.mavlink_raw_rpm_t message = new MAVLink.mavlink_raw_rpm_t();
            //message.rpm1 = data.avrrpm;
            client.SendMessage(message);
        }


        public void SendRadioStatus(ClientObject client)
        {
            MAVLink.mavlink_radio_status_t message = new MAVLink.mavlink_radio_status_t();
            message.rssi = (byte)data.rssi;
            message.remrssi = 200;
            message.txbuf = 99;
            message.rxerrors = 0;
            message.@fixed = 0;
            client.SendMessage(message);
        }

        public void SendRadioChannelsRaw(ClientObject client)
        {
            MAVLink.mavlink_rc_channels_t message = new MAVLink.mavlink_rc_channels_t();
            message.rssi = 200;
            message.chan1_raw = (ushort)data.ch1;
            message.chan2_raw = (ushort)data.ch2;
            message.chan3_raw = (ushort)data.ch3;
            message.chan4_raw = (ushort)data.ch4;
            message.chan5_raw = (ushort)data.ch5;
            message.chan6_raw = (ushort)data.ch6;
            message.chan7_raw = (ushort)data.ch7;
            message.chan8_raw = (ushort)data.ch8;
            client.SendMessage(message);
        }

        public void SendRadioChannelsScaled(ClientObject client)
        {
            MAVLink.mavlink_rc_channels_scaled_t message = new MAVLink.mavlink_rc_channels_scaled_t();
            message.rssi = 200;
            message.chan1_scaled = 5000;
            message.chan2_scaled = 5000;
            message.chan3_scaled = 5000;
            message.chan4_scaled = 5000;
            message.chan5_scaled = 5000;
            message.chan6_scaled = 5000;
            message.chan7_scaled = 5000;
            message.chan8_scaled = 5000;
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

        public void SetParameter(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            MAVLink.mavlink_param_set_t message = (MAVLink.mavlink_param_set_t)messageRaw.data;
            string param_id = Encoding.UTF8.GetString(message.param_id);
            parameters.SetParameter(param_id, message.param_value, (MAVLink.MAV_PARAM_TYPE)message.param_type);
            Parameter p = parameters.GetParameter(param_id);
            MAVLink.mavlink_param_value_t sendMessage = new MAVLink.mavlink_param_value_t
            {
                param_id = p.GetIDBytes(),
                param_type = (byte)p.type,
                param_value = p.value,
                param_index = (ushort)parameters.IndexOf(p),
                param_count = (ushort)parameters.GetCount()
            };
            client.SendMessage(sendMessage);
        }

        public uint GetUptime()
        {
            long timeMS = (DateTime.UtcNow.Ticks - startTime) / TimeSpan.TicksPerMillisecond;
            return (uint)timeMS;
        }
    }
}

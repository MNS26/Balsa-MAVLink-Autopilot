﻿using System;
using System.Net;
using System.Reflection;
using System.Text;

namespace AutopilotCommon
{
    public class ProtocolLogic
    {
        private long startTime;
        private const byte systemID = 1;
        private const byte componentID = 1;
        private static MAVLink.MavlinkParse parser = new MAVLink.MavlinkParse();
        private Data data;
        private Ap ap;
        private ParameterHandler parameters;
        private Action<string> Log;
        public ProtocolLogic(Data data, Ap ap, Action<string> Log, ParameterHandler parameters)
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
        //[ReceiveCommand(MAVLink.MAV_CMD.DO_SET_SERVO)]
        public void RequestSetServo(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            AckCommand(client, command, MAVLink.MAV_CMD_ACK.OK);
            Log($"REQUEST SET SERVO: {command.param1}, PWM {command.param2}");
#pragma warning disable CS0219 // Variable is assigned but its value is never used
            //MAVLink.mavlink_servo_output_raw_t servo = new MAVLink.mavlink_servo_output_raw_t();
#pragma warning restore CS0219 // Variable is assigned but its value is never used
            //client.SendMessage(command);
        }

        //[ReceiveCommand(MAVLink.MAV_CMD.PREFLIGHT_CALIBRATION)]
        public void PreflightCalibration(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            Log($"PREFLIGHT_CALIBRATE");
            AckCommand(client, command, MAVLink.MAV_CMD_ACK.OK);
        }

        //[ReceiveCommand(MAVLink.MAV_CMD.REQUEST_PROTOCOL_VERSION)]
        public void RequestProtocolVersion(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            AckCommand(client, command, MAVLink.MAV_CMD_ACK.OK);
            MAVLink.mavlink_protocol_version_t version = new MAVLink.mavlink_protocol_version_t();
            version.min_version = 1;
            version.version = 2;
            version.max_version = 2;
            client.SendMessage(version);
            Log($"REQUEST_PROTOCOL_VERSION, VERSION = {version.version}");
        }

        //[ReceiveCommand(MAVLink.MAV_CMD.SET_MESSAGE_INTERVAL)]
        public void MessageInterval(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            AckCommand(client, command, MAVLink.MAV_CMD_ACK.OK);
            Log($"SET_MESSAGE_INTERVAL: {(MAVLink.MAVLINK_MSG_ID)command.param1} = {command.param2}");
            client.requestedRates[(MAVLink.MAVLINK_MSG_ID)command.param1] = command.param2 * 1;
        }

        //a "one shot" version of MessageInterwal
        //[ReceiveCommand(MAVLink.MAV_CMD.REQUEST_MESSAGE)]
        //TODO: Call the approriate callback
        public void RequestMessage(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            AckCommand(client, command, MAVLink.MAV_CMD_ACK.OK);
            Log($"REQUEST_MESSAGE: {(MAVLink.MAVLINK_MSG_ID)command.param1} = {command.param2}");
            client.requestedRates[(MAVLink.MAVLINK_MSG_ID)command.param1] = command.param2 * 1;
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

        //[ReceiveCommand(MAVLink.MAV_CMD.REQUEST_AUTOPILOT_CAPABILITIES)]
        public void RequestAutopilot(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            Log("REQUEST AUTOPILOT");
            AckCommand(client, command, MAVLink.MAV_CMD_ACK.OK);
            MAVLink.mavlink_autopilot_version_t autopilot = new MAVLink.mavlink_autopilot_version_t();
            autopilot.capabilities |= (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.COMMAND_INT;
            autopilot.capabilities |= (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.FLIGHT_INFORMATION;
            autopilot.capabilities |= (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.FLIGHT_TERMINATION;
            //autopilot.capabilities |= (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.MAVLINK2;
            //autopilot.capabilities |= (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.MISSION_FLOAT;
            autopilot.capabilities |= (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.PARAM_FLOAT;
            autopilot.capabilities |= (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.SET_ACTUATOR_TARGET;
            autopilot.board_version = 0;
            autopilot.flight_sw_version = 0;
            autopilot.os_sw_version = 0;
            byte[] emptyByte = new byte[8];
            autopilot.flight_custom_version = emptyByte;
            autopilot.middleware_custom_version = emptyByte;
            autopilot.os_custom_version = emptyByte;
            autopilot.product_id = 0;
            autopilot.vendor_id = 0;
            autopilot.uid = 13546;
            client.SendMessage(autopilot);
        }

        //[ReceiveCommand(MAVLink.MAV_CMD.COMPONENT_ARM_DISARM)]
        public void RequestArmDisarm(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            AckCommand(client, command, MAVLink.MAV_CMD_ACK.OK);
            Log($"REQUEST ARM DISARM: STATE {command.param1}, {command.param2}");
        }

        //Messages
        //[ReceiveMessage(MAVLink.MAVLINK_MSG_ID.HEARTBEAT)]
        public void Heartbeat(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            MAVLink.mavlink_heartbeat_t message = (MAVLink.mavlink_heartbeat_t)messageRaw.data;
        }

        //[ReceiveMessage(MAVLink.MAVLINK_MSG_ID.SYSTEM_TIME)]
        public void SystemTime(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            MAVLink.mavlink_system_time_t message = (MAVLink.mavlink_system_time_t)messageRaw.data;
            Log($"SYSTEM_TIME {message.time_unix_usec}");
        }

        //[ReceiveMessage(MAVLink.MAVLINK_MSG_ID.PARAM_REQUEST_LIST)]
        public void ParamRequestList(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            Log($"PARAM_REQUEST_LIST");
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

        //[ReceiveMessage(MAVLink.MAVLINK_MSG_ID.MISSION_REQUEST_LIST)]
        public void SendMissionList(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            Log($"MISSION_REQUEST_LIST: {messageRaw}");
            //MAVLink.mavlink_mission_request_list_t missionList = (MAVLink.mavlink_mission_request_list_t)messageRaw.data;
            MAVLink.mavlink_mission_count_t missionCount = new MAVLink.mavlink_mission_count_t();
            missionCount.target_system = systemID;
            missionCount.target_component = componentID;
            missionCount.count = 0;
            missionCount.mission_type = (byte)MAVLink.MAV_MISSION_TYPE.ALL;
            client.SendMessage(missionCount);
        }

        //[ReceiveMessage(MAVLink.MAVLINK_MSG_ID.MISSION_ACK)]
        public void SendMissionAck(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            Log("MISSION_ACK");
            MAVLink.mavlink_mission_ack_t message = (MAVLink.mavlink_mission_ack_t)messageRaw.data;
            message.target_system = systemID;
            message.target_component = componentID;
            message.type = 0;
            message.mission_type = (byte)MAVLink.MAV_MISSION_TYPE.ALL;
            client.SendMessage(message);
        }

        //[ReceiveMessage(MAVLink.MAVLINK_MSG_ID.LOG_REQUEST_LIST)]
        public void RequestLogList(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            //we currently cant generate logs so we dont have a use for start and end count for logs
            //MAVLink.mavlink_log_request_list_t message = new MAVLink.mavlink_log_request_list_t();
            MAVLink.mavlink_log_entry_t message = new MAVLink.mavlink_log_entry_t{
            id = 0,
            num_logs = 0,
            last_log_num = 0,
            time_utc = 0,
            size = 0
            };
            client.SendMessage(message);
        }

        //[ReceiveMessage(MAVLink.MAVLINK_MSG_ID.PARAM_SET)]
        public void SetParameter(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            MAVLink.mavlink_param_set_t message = (MAVLink.mavlink_param_set_t)messageRaw.data;
            string param_id = Encoding.UTF8.GetString(message.param_id).Replace("\0", String.Empty);
            parameters.SetParameter(param_id, message.param_value, (MAVLink.MAV_PARAM_TYPE)message.param_type);
            Parameter p = parameters.GetParameter(param_id);
            Log($"SET PARAMETER: {p.id} = {p.value}");
            //Reply to parameter verifying we received it
            MAVLink.mavlink_param_value_t sendMessage = new MAVLink.mavlink_param_value_t
            {
                param_id = p.GetIDBytes(),
                param_type = (byte)p.type,
                param_value = p.value,
                param_index = (ushort)p.index,
                param_count = (ushort)parameters.GetCount()
            };
            client.SendMessage(sendMessage);
        }

        //[ReceiveMessage(MAVLink.MAVLINK_MSG_ID.REQUEST_DATA_STREAM)]
        public void RequestDataStream(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            //TODO: Implement ALL of these.
            MAVLink.mavlink_request_data_stream_t message = (MAVLink.mavlink_request_data_stream_t)messageRaw.data;
            MAVLink.MAV_DATA_STREAM mdsType = (MAVLink.MAV_DATA_STREAM)message.req_stream_id;
            Log($"REQUEST_DATA_STREAM  TYPE:{mdsType}  =  {message.req_message_rate}");
            foreach (MethodInfo mi in typeof(ProtocolLogic).GetMethods())
            {
                SendMessage sm = mi.GetCustomAttribute<SendMessage>();
                SendCategory category = mi.GetCustomAttribute<SendCategory>();
                if (sm != null && category != null && (category.type == mdsType || mdsType == MAVLink.MAV_DATA_STREAM.ALL))
                {
                    client.requestedRates[sm.id] = 1f / message.req_message_rate;
                }
            }
        }

        //[ReceiveMessage(MAVLink.MAVLINK_MSG_ID.STATUSTEXT)]
        public void Statustext(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            MAVLink.mavlink_statustext_t status = (MAVLink.mavlink_statustext_t)messageRaw.data;
            MAVLink.mavlink_statustext_t sendStatus = new MAVLink.mavlink_statustext_t
            {
                severity = status.severity,
                text = status.text,
                id = status.id,
                chunk_seq = status.chunk_seq
            };
            client.SendMessage(sendStatus);
        }

        [SendMessage(MAVLink.MAVLINK_MSG_ID.HEARTBEAT)]
        public void SendHeartbeat(ClientObject client)
        {
            MAVLink.mavlink_heartbeat_t message = new MAVLink.mavlink_heartbeat_t();
            message.custom_mode = 0;
            message.type = (byte)MAVLink.MAV_TYPE.FIXED_WING;
            message.autopilot = (byte)MAVLink.MAV_AUTOPILOT.ARDUPILOTMEGA;
            //message.autopilot = (byte)MAVLink.MAV_AUTOPILOT.GENERIC;
            //message.autopilot = (byte)MAVLink.MAV_AUTOPILOT.GENERIC_MISSION_FULL;
            //message.autopilot = (byte)MAVLink.MAV_AUTOPILOT.GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY;
            message.base_mode = (byte)MAVLink.MAV_MODE.MANUAL_ARMED;
            message.system_status = (byte)MAVLink.MAV_STATE.ACTIVE;
            message.mavlink_version = (byte)MAVLink.MAVLINK_VERSION;
            client.SendMessage(message);


            uint sensors = (uint)(  MAVLink.MAV_SYS_STATUS_SENSOR._3D_GYRO |
                                    MAVLink.MAV_SYS_STATUS_SENSOR._3D_ACCEL |
                                    MAVLink.MAV_SYS_STATUS_SENSOR._3D_MAG |
                                    MAVLink.MAV_SYS_STATUS_SENSOR.ABSOLUTE_PRESSURE |
                                    MAVLink.MAV_SYS_STATUS_SENSOR.BATTERY |
                                    MAVLink.MAV_SYS_STATUS_SENSOR.GPS |
                                    MAVLink.MAV_SYS_STATUS_SENSOR.RC_RECEIVER
                                );
            MAVLink.mavlink_sys_status_t sysStatus = new MAVLink.mavlink_sys_status_t();
            sysStatus.onboard_control_sensors_present = sensors;
            sysStatus.onboard_control_sensors_enabled = sensors;
            sysStatus.onboard_control_sensors_health = sensors;
            //1%
            sysStatus.load = data.load;
            sysStatus.voltage_battery = 11000;
            sysStatus.current_battery = 1000;
            client.SendMessage(sysStatus);
        }

        //[SendMessage(MAVLink.MAVLINK_MSG_ID.ATTITUDE)]
        //[SendCategory(MAVLink.MAV_DATA_STREAM.RAW_SENSORS)]
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

        //[SendMessage(MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT)]
        //[SendCategory(MAVLink.MAV_DATA_STREAM.POSITION)]
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

        //[SendMessage(MAVLink.MAVLINK_MSG_ID.VFR_HUD)]
        //[SendCategory(MAVLink.MAV_DATA_STREAM.EXTRA1)]
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

        //[SendMessage(MAVLink.MAVLINK_MSG_ID.GPS_RAW_INT)]
        //[SendCategory(MAVLink.MAV_DATA_STREAM.POSITION)]
        public void SendGPSRaw(ClientObject client)
        {
            MAVLink.mavlink_gps_raw_int_t message = new MAVLink.mavlink_gps_raw_int_t();
            message.lat = data.latitude;
            message.lon = data.longitude;
            message.alt = (int)data.altitude;
            message.eph = 30;
            message.epv = 50;
            message.vel = 1000;
            message.cog = 25000;
            message.satellites_visible = 15;
            message.alt_ellipsoid = (int)data.altitude;
            message.h_acc = 10;
            message.v_acc = 10;
            message.vel_acc = 10;
            message.hdg_acc = 10;
            message.yaw = (ushort)data.yaw;
            client.SendMessage(message);
        }

        //[SendMessage(MAVLink.MAVLINK_MSG_ID.GPS_STATUS)]
        //[SendCategory(MAVLink.MAV_DATA_STREAM.EXTENDED_STATUS)]
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

        //[SendMessage(MAVLink.MAVLINK_MSG_ID.RAW_IMU)]
        //[SendCategory(MAVLink.MAV_DATA_STREAM.RAW_SENSORS)]
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

        //[SendMessage(MAVLink.MAVLINK_MSG_ID.RAW_RPM)]
        //[SendCategory(MAVLink.MAV_DATA_STREAM.RAW_SENSORS)]
        public void SendRPM(ClientObject client)
        {
            MAVLink.mavlink_raw_rpm_t message = new MAVLink.mavlink_raw_rpm_t();
            message.frequency = data.avrrpm;
            client.SendMessage(message);
        }

        //[SendMessage(MAVLink.MAVLINK_MSG_ID.RADIO_STATUS)]
        //[SendCategory(MAVLink.MAV_DATA_STREAM.EXTENDED_STATUS)]
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

        //[SendMessage(MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_SCALED)]
        //[SendCategory(MAVLink.MAV_DATA_STREAM.RC_CHANNELS)]
        public void SendRadioChannelsScaled(ClientObject client)
        {
            MAVLink.mavlink_rc_channels_scaled_t message = new MAVLink.mavlink_rc_channels_scaled_t();
            message.rssi = (byte)data.rssi;
            message.chan1_scaled = ((short)(data.ChannelsRC[0]*1000));
            message.chan2_scaled = ((short)(data.ChannelsRC[1]*1000));
            message.chan3_scaled = ((short)(data.ChannelsRC[2]*1000));
            message.chan4_scaled = ((short)(data.ChannelsRC[3]*1000));
            message.chan5_scaled = ((short)(data.ChannelsRC[4]*1000));
            message.chan6_scaled = ((short)(data.ChannelsRC[5]*1000));
            message.chan7_scaled = ((short)(data.ChannelsRC[6]*1000));
            message.chan8_scaled = ((short)(data.ChannelsRC[7]*1000));
            client.SendMessage(message);
        }

        //[SendMessage(MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_RAW)]
        //[SendCategory(MAVLink.MAV_DATA_STREAM.RC_CHANNELS)]
        public void SendRadioChannelsRaw(ClientObject client)
        {
            MAVLink.mavlink_rc_channels_t message = new MAVLink.mavlink_rc_channels_t();
            message.chan1_raw = (ushort)(1500 + data.ChannelsRC[0] * 500);
            message.chan2_raw = (ushort)(1500 + data.ChannelsRC[1] * 500);
            message.chan3_raw = (ushort)(1500 + data.ChannelsRC[2] * 500);
            message.chan4_raw = (ushort)(1500 + data.ChannelsRC[3] * 500);
            message.chan5_raw = (ushort)(1500 + data.ChannelsRC[4] * 500);
            message.chan6_raw = (ushort)(1500 + data.ChannelsRC[5] * 500);
            message.chan7_raw = (ushort)(1500 + data.ChannelsRC[6] * 500);
            message.chan8_raw = (ushort)(1500 + data.ChannelsRC[7] * 500);
            message.chan9_raw = (ushort)(1500 + data.ChannelsRC[8] * 500);
            message.chan10_raw = (ushort)(1500 + data.ChannelsRC[9] * 500);
            message.chan11_raw = (ushort)(1500 + data.ChannelsRC[10] * 500);
            message.chan12_raw = (ushort)(1500 + data.ChannelsRC[11] * 500);
            message.chan13_raw = (ushort)(1500 + data.ChannelsRC[12] * 500);
            message.chan14_raw = (ushort)(1500 + data.ChannelsRC[13] * 500);
            message.chan15_raw = (ushort)(1500 + data.ChannelsRC[14] * 500);
            message.chan16_raw = (ushort)(1500 + data.ChannelsRC[15] * 500);
            message.chan17_raw = (ushort)(1500 + data.ChannelsRC[16] * 500);
            message.chan18_raw = (ushort)(1500 + data.ChannelsRC[17] * 500);
            message.chancount = 32;
            client.SendMessage(message);
        }

        //[SendMessage(MAVLink.MAVLINK_MSG_ID.SERVO_OUTPUT_RAW)]
        //[SendCategory(MAVLink.MAV_DATA_STREAM.RC_CHANNELS)]
        public void ServoRaw(ClientObject client)
        {
            MAVLink.mavlink_servo_output_raw_t ServoRaw = new MAVLink.mavlink_servo_output_raw_t{
                time_usec = GetUptime(),
                servo1_raw = (ushort)(1500 + data.ChannelsServo[0] * 500),
                servo2_raw = (ushort)(1500 + data.ChannelsServo[1] * 500),
                servo3_raw = (ushort)(1500 + data.ChannelsServo[2] * 500),
                servo4_raw = (ushort)(1500 + data.ChannelsServo[3] * 500),
                servo5_raw = (ushort)(1500 + data.ChannelsServo[4] * 500),
                servo6_raw = (ushort)(1500 + data.ChannelsServo[5] * 500),
                servo7_raw = (ushort)(1500 + data.ChannelsServo[6] * 500),
                servo8_raw = (ushort)(1500 + data.ChannelsServo[7] * 500),
                servo9_raw = (ushort)(1500 + data.ChannelsServo[8] * 500),
                servo10_raw = (ushort)(1500 + data.ChannelsServo[9] * 500),
                servo11_raw = (ushort)(1500 + data.ChannelsServo[10] * 500),
                servo12_raw = (ushort)(1500 + data.ChannelsServo[11] * 500),
                servo13_raw = (ushort)(1500 + data.ChannelsServo[12] * 500),
                servo14_raw = (ushort)(1500 + data.ChannelsServo[13] * 500),
                servo15_raw = (ushort)(1500 + data.ChannelsServo[14] * 500),
                servo16_raw = (ushort)(1500 + data.ChannelsServo[15] * 500),
            };
            client.SendMessage(ServoRaw);
        }
        //[SendMessage(MAVLink.MAVLINK_MSG_ID.ACTUATOR_OUTPUT_STATUS)]
        //[SendCategory(MAVLink.MAV_DATA_STREAM.RC_CHANNELS)]
        public void ActuatorStatus(ClientObject client)
        {
            MAVLink.mavlink_actuator_output_status_t actuator = new MAVLink.mavlink_actuator_output_status_t{
                time_usec = GetUptime(),
                active = 32,
                actuator = data.ChannelsServo
            };
            client.SendMessage(actuator);
        }

        public void AckCommand(ClientObject client, MAVLink.mavlink_command_long_t command, MAVLink.MAV_CMD_ACK ackType)
        {
            MAVLink.mavlink_command_ack_t ack = new MAVLink.mavlink_command_ack_t();
            ack.command = command.command;
            ack.result = (byte)ackType;
            ack.target_system = command.target_system;
            ack.target_component = command.target_component;
        }

        public uint GetUptime()
        {
            long timeMS = (DateTime.UtcNow.Ticks - startTime) / TimeSpan.TicksPerMillisecond;
            return (uint)timeMS;
        }
    }
}

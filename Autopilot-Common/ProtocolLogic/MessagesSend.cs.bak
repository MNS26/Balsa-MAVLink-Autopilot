using System;
using System.Reflection;
using System.Text;

namespace AutopilotCommon
{
    public class MessagesSend
    {
        private long startTime;
        private const byte systemID = 1;
        private const byte componentID = 1;
        private static MAVLink.MavlinkParse parser = new MAVLink.MavlinkParse();
        private Data data;
        private Ap ap;
        private ParameterHandler parameters;
        private Action<string> Log;
        public MessagesSend(Data data, Ap ap, Action<string> Log, ParameterHandler parameters)
        {
            this.Log = Log;
            startTime = DateTime.UtcNow.Ticks;
            this.data = data;
            this.ap = ap;
            this.parameters = parameters;
        }
        [SendMessage(MAVLink.MAVLINK_MSG_ID.HEARTBEAT)]
        public void SendHeartbeat(ClientObject client)
        {
            MAVLink.mavlink_heartbeat_t message = new MAVLink.mavlink_heartbeat_t(){
                custom_mode = (uint)0,
                type = (byte)MAVLink.MAV_TYPE.FIXED_WING,
                autopilot = (byte)MAVLink.MAV_AUTOPILOT.ARDUPILOTMEGA,
                            //|(byte)MAVLink.MAV_AUTOPILOT.GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY,
                            //|(byte)MAVLink.MAV_AUTOPILOT.GENERIC_MISSION_FULL,
                base_mode = (byte)ap.mode_flag,
                system_status = (byte)ap.state,
                mavlink_version = (byte)MAVLink.MAVLINK_VERSION,
            };
            client.SendMessage(message);


            uint sensors = (uint)(  MAVLink.MAV_SYS_STATUS_SENSOR._3D_GYRO |
                                    MAVLink.MAV_SYS_STATUS_SENSOR._3D_ACCEL |
                                    MAVLink.MAV_SYS_STATUS_SENSOR._3D_MAG |
                                    MAVLink.MAV_SYS_STATUS_SENSOR.ABSOLUTE_PRESSURE |
                                    MAVLink.MAV_SYS_STATUS_SENSOR.BATTERY |
                                    MAVLink.MAV_SYS_STATUS_SENSOR.GPS |
                                    MAVLink.MAV_SYS_STATUS_SENSOR.RC_RECEIVER
                                );
            MAVLink.mavlink_sys_status_t sysStatus = new MAVLink.mavlink_sys_status_t(){
                onboard_control_sensors_present = sensors,
                onboard_control_sensors_enabled = sensors,
                onboard_control_sensors_health = sensors,
                //1%
                load = 0,
                voltage_battery = 11000,
                current_battery = 1000,
            };
            client.SendMessage(sysStatus);
        }

        [SendMessage(MAVLink.MAVLINK_MSG_ID.ATTITUDE)]
        [SendCategory(MAVLink.MAV_DATA_STREAM.RAW_SENSORS)]
        public void SendAttitude(ClientObject client)
        {
            MAVLink.mavlink_attitude_t message = new MAVLink.mavlink_attitude_t()
            {
                pitch = data.radpitch,
                roll = data.radroll,
                yaw = data.radyaw,
                pitchspeed = 0,
                rollspeed = 0,
                yawspeed = 0,
                time_boot_ms = GetUptime()
            };
            client.SendMessage(message);
        }

        //[SendMessage(MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT)]
        //[SendCategory(MAVLink.MAV_DATA_STREAM.POSITION)]
        public void SendGPSGlobalPosition(ClientObject client)
        {
            MAVLink.mavlink_global_position_int_t message = new MAVLink.mavlink_global_position_int_t()
            {
            lat = data.latitude,
            lon = data.longitude,
            alt = (int)data.altitude,
            relative_alt = 0,//(int)data.altitude,
            hdg = (ushort)data.heading,
            vx = 0,
            vy = 0,
            vz = 0,
            time_boot_ms = GetUptime(),
            };
            client.SendMessage(message);
        }

        [SendMessage(MAVLink.MAVLINK_MSG_ID.VFR_HUD)]
        [SendCategory(MAVLink.MAV_DATA_STREAM.EXTRA1)]
        public void SendVFRHud(ClientObject client)
        {
            MAVLink.mavlink_vfr_hud_t message = new MAVLink.mavlink_vfr_hud_t(){
            airspeed = data.iaspeed,
            climb = data.cr,
            alt = data.altitude,
            heading = (short)data.heading,
            //TODO
            groundspeed = 0,
            throttle = 0,
            };
            client.SendMessage(message);
        }

        [SendMessage(MAVLink.MAVLINK_MSG_ID.GPS_RAW_INT)]
        [SendCategory(MAVLink.MAV_DATA_STREAM.POSITION)]
        public void SendGPSRaw(ClientObject client)
        {
            MAVLink.mavlink_gps_raw_int_t message = new MAVLink.mavlink_gps_raw_int_t()
            {
                lat = data.latitude,
                lon = data.longitude,
                alt = (int)data.altitude,
                eph = 30,
                epv = 50,
                vel = 1000,
                cog = 25000,
                satellites_visible = 15,
                alt_ellipsoid = (int)data.altitude,
                h_acc = 10,
                v_acc = 10,
                vel_acc = 10,
                hdg_acc = 10,
                yaw = (ushort)data.yaw
            };
            client.SendMessage(message);
        }

        [SendMessage(MAVLink.MAVLINK_MSG_ID.GPS_STATUS)]
        [SendCategory(MAVLink.MAV_DATA_STREAM.EXTENDED_STATUS)]
        public void SendGPSStatus(ClientObject client)
        {
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
            };
            MAVLink.mavlink_gps_status_t message = new MAVLink.mavlink_gps_status_t()
            {
                satellites_visible = 10,
                satellite_prn = prn,
                satellite_used = used,
                satellite_elevation = ele,
                satellite_azimuth = azi,
                satellite_snr = snr
            };
            client.SendMessage(message);
        }

        [SendMessage(MAVLink.MAVLINK_MSG_ID.RAW_IMU)]
        [SendCategory(MAVLink.MAV_DATA_STREAM.RAW_SENSORS)]
        public void SendRawIMU(ClientObject client)
        {
            DateTime epoch = new DateTime(1970, 1, 1, 0, 0, 0, 0, DateTimeKind.Utc);
            TimeSpan diff = DateTime.UtcNow - epoch;
            MAVLink.mavlink_raw_imu_t message = new MAVLink.mavlink_raw_imu_t()
            {
                time_usec = (ulong)(diff.TotalMilliseconds * 1000),
                xacc = (short)data.accx,
                yacc = (short)data.accy,
                zacc = (short)data.accz,
                xgyro = (short)data.gyrox,
                ygyro = (short)data.gyroy,
                zgyro = (short)data.gyroz,
                xmag = (short)data.magx,
                ymag = (short)data.magy,
                zmag = (short)data.magz,
                id = 0,
                temperature = 6000
            };
            client.SendMessage(message);
        }

        [SendMessage(MAVLink.MAVLINK_MSG_ID.RAW_RPM)]
        [SendCategory(MAVLink.MAV_DATA_STREAM.RAW_SENSORS)]
        public void SendRPM(ClientObject client)
        {
            MAVLink.mavlink_raw_rpm_t message = new MAVLink.mavlink_raw_rpm_t()
            {
                frequency = data.avrrpm
            };
            client.SendMessage(message);
        }

        [SendMessage(MAVLink.MAVLINK_MSG_ID.RADIO_STATUS)]
        [SendCategory(MAVLink.MAV_DATA_STREAM.EXTENDED_STATUS)]
        public void SendRadioStatus(ClientObject client)
        {
            MAVLink.mavlink_radio_status_t message = new MAVLink.mavlink_radio_status_t()
            {
                rssi = (byte)data.rssi,
                remrssi = 200,
                txbuf = 99,
                rxerrors = 0,
                @fixed = 0
            };
            client.SendMessage(message);
        }

        [SendMessage(MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_SCALED)]
        [SendCategory(MAVLink.MAV_DATA_STREAM.RC_CHANNELS)]
        public void SendRadioChannelsScaled(ClientObject client)
        {
            MAVLink.mavlink_rc_channels_scaled_t message = new MAVLink.mavlink_rc_channels_scaled_t()
            {
                rssi = (byte)data.rssi,
                chan1_scaled = (short)(data.ChannelsRC[0] * 1000),
                chan2_scaled = (short)(data.ChannelsRC[1] * 1000),
                chan3_scaled = (short)(data.ChannelsRC[2] * 1000),
                chan4_scaled = (short)(data.ChannelsRC[3] * 1000),
                chan5_scaled = (short)(data.ChannelsRC[4] * 1000),
                chan6_scaled = (short)(data.ChannelsRC[5] * 1000),
                chan7_scaled = (short)(data.ChannelsRC[6] * 1000),
                chan8_scaled = (short)(data.ChannelsRC[7] * 1000)
            };
            client.SendMessage(message);
        }

        [SendMessage(MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_RAW)]
        [SendCategory(MAVLink.MAV_DATA_STREAM.RC_CHANNELS)]
        public void SendRadioChannelsRaw(ClientObject client)
        {
            MAVLink.mavlink_rc_channels_t message = new MAVLink.mavlink_rc_channels_t()
            {
                chan1_raw = (ushort)(1500 + data.ChannelsRC[0] * 500),
                chan2_raw = (ushort)(1500 + data.ChannelsRC[1] * 500),
                chan3_raw = (ushort)(1500 + data.ChannelsRC[2] * 500),
                chan4_raw = (ushort)(1500 + data.ChannelsRC[3] * 500),
                chan5_raw = (ushort)(1500 + data.ChannelsRC[4] * 500),
                chan6_raw = (ushort)(1500 + data.ChannelsRC[5] * 500),
                chan7_raw = (ushort)(1500 + data.ChannelsRC[6] * 500),
                chan8_raw = (ushort)(1500 + data.ChannelsRC[7] * 500),
                chan9_raw = (ushort)(1500 + data.ChannelsRC[8] * 500),
                chan10_raw = (ushort)(1500 + data.ChannelsRC[9] * 500),
                chan11_raw = (ushort)(1500 + data.ChannelsRC[10] * 500),
                chan12_raw = (ushort)(1500 + data.ChannelsRC[11] * 500),
                chan13_raw = (ushort)(1500 + data.ChannelsRC[12] * 500),
                chan14_raw = (ushort)(1500 + data.ChannelsRC[13] * 500),
                chan15_raw = (ushort)(1500 + data.ChannelsRC[14] * 500),
                chan16_raw = (ushort)(1500 + data.ChannelsRC[15] * 500),
                chan17_raw = (ushort)(1500 + data.ChannelsRC[16] * 500),
                chan18_raw = (ushort)(1500 + data.ChannelsRC[17] * 500),
                chancount = 32
            };
            client.SendMessage(message);
        }

        [SendMessage(MAVLink.MAVLINK_MSG_ID.SERVO_OUTPUT_RAW)]
        [SendCategory(MAVLink.MAV_DATA_STREAM.RC_CHANNELS)]
        public void ServoRaw(ClientObject client)
        {
            MAVLink.mavlink_servo_output_raw_t ServoRaw = new MAVLink.mavlink_servo_output_raw_t()
            {
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
        [SendMessage(MAVLink.MAVLINK_MSG_ID.ACTUATOR_OUTPUT_STATUS)]
        [SendCategory(MAVLink.MAV_DATA_STREAM.RC_CHANNELS)]
        public void ActuatorStatus(ClientObject client)
        {
            MAVLink.mavlink_actuator_output_status_t actuator = new MAVLink.mavlink_actuator_output_status_t()
            {
                time_usec = GetUptime(),
                active = 32,
                actuator = data.ChannelsServo
            };
            client.SendMessage(actuator);
        }
        public uint GetUptime()
        {
            long timeMS = (DateTime.UtcNow.Ticks - startTime) / TimeSpan.TicksPerMillisecond;
            return (uint)timeMS;
        }
    }
}
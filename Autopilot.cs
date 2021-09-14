using FSControl;
using Modules;
using UnityEngine;

namespace Autopilot
{
    public class Autopilot //: MonoBehaviour
    {

        DataStore data;
        ProtocolLogic protocol;
        NetworkHandler handler;

        public void Start()
        {
            Log("Start!");
            data = new DataStore();
            protocol = new ProtocolLogic(data);

            handler = new NetworkHandler();
            handler.RegisterConnect(protocol.ConnectEvent);
            handler.RegisterReceive(MAVLink.MAVLINK_MSG_ID.PARAM_REQUEST_LIST, protocol.ParamRequestList);
            handler.RegisterReceive(MAVLink.MAVLINK_MSG_ID.REQUEST_DATA_STREAM, protocol.RequestDataStream);
            handler.RegisterReceive(MAVLink.MAVLINK_MSG_ID.SYSTEM_TIME, protocol.SystemTime);
            handler.RegisterReceiveCommand(MAVLink.MAV_CMD.REQUEST_AUTOPILOT_CAPABILITIES, protocol.RequestAutopilot);
            //handler.RegisterReceiveCommand(MAVLink.MAV_C)
            handler.RegisterSend(MAVLink.MAVLINK_MSG_ID.VFR_HUD, protocol.SendVFRHud);
            handler.RegisterSend(MAVLink.MAVLINK_MSG_ID.RPM, protocol.SendRPM);
            handler.RegisterSend(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, protocol.SendHeartbeat);
            handler.RegisterSend(MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT, protocol.SendGPSGlobalPosition);
            handler.RegisterSend(MAVLink.MAVLINK_MSG_ID.GPS_RAW_INT, protocol.SendGPSRaw);
            handler.RegisterSend(MAVLink.MAVLINK_MSG_ID.ATTITUDE, protocol.SendAttitude);
            handler.RegisterSend(MAVLink.MAVLINK_MSG_ID.RAW_IMU, protocol.SendRawIMU);
            handler.RegisterSend(MAVLink.MAVLINK_MSG_ID.GPS_STATUS, protocol.SendGPSStatus);
            handler.RegisterSend(MAVLink.MAVLINK_MSG_ID.RADIO_STATUS, protocol.SendRadioStatus);
            handler.RegisterSend(MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_SCALED, protocol.SendRadioChannelsScaled);
            handler.RegisterSend(MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_RAW, protocol.SendRadioChannelsRaw);
            handler.RegisterDisconnect(protocol.DisconnectEvent);
            handler.StartServer(Autopilot.Log);

            //DontDestroyOnLoad(this);
        }

        //running this as fast as possible
        public void Update()
        {
            if (!GameLogic.inGame || !GameLogic.SceneryLoaded || GameLogic.LocalPlayerVehicle == null || !GameLogic.LocalPlayerVehicle.InitComplete)
            {
                data.radpitch = 0;
                data.radroll = 0;
                data.radyaw = 0;
                data.pitch = 0;
                data.roll = 0;
                data.yaw = 0;
                data.cr = 0;
                data.dth = 0;
                data.accx = 0;
                data.accy = 0;
                data.accz = 0;
                data.rssi = 0;
                data.avrrpm = 0;
                data.latitude = 0;
                data.longitude = 0;
                data.altitude = 0;
                data.heading = 0;
                data.iaspeed = 0;
                data.name = "";
                return;
            }
            Vehicle v = GameLogic.LocalPlayerVehicle;

            var props = v.GetModules<Propeller>();
            if (props.Count != 0)
            {
                data.avrrpm = 0;
                foreach (var p in props)
                {
                    data.avrrpm += p.GetRotSpeed();
                }
                data.avrrpm /= props.Count;
                data.avrrpm *= 1.66667f;
            }

            data.radpitch = FSControlUtil.GetVehiclePitch(v);
            data.pitch = data.radpitch * Mathf.Rad2Deg;
            data.radroll = FSControlUtil.GetVehicleRoll(v);
            data.roll = data.radroll * Mathf.Rad2Deg;
            data.radyaw = FSControlUtil.GetVehicleYaw(v);
            data.yaw = data.radyaw * Mathf.Rad2Deg;

            //Metres -> mm
            data.altitude = v.Physics.Altitude * 1000f;
            data.iaspeed = v.Physics.Speed;
            data.heading = v.Physics.HeadingDegs;
            data.cr = v.Physics.VerticalSpeed;

            data.accx = v.Physics.Acceleration.x;
            data.accy = v.Physics.Acceleration.y;
            data.accz = v.Physics.Acceleration.z;
            data.name = v.name;

            //Balsa is YUp
            //Mavlink is degE7, 1° = 111 km 1E7/111000 = ~90
            data.latitude = (int)(FloatingOrigin.GetAbsoluteWPos(v.transform.position).z * 90.09f);
            data.longitude = (int)(FloatingOrigin.GetAbsoluteWPos(v.transform.position).x * 90.09f);

            //controller stuff
            data.rssi = map(v.SignalStrength.SignalDegradation, 0, 1, 255, 0);


        }

        public void FixedUpdate()
        {
        }

        private float map(float value, float fromLow, float fromHigh, float toLow, float toHigh)
        {
            return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
        }

        //It's nice to identify in the log where things came from
        public static void Log(string text)
        {
            Debug.Log($"{Time.realtimeSinceStartup} [Autopilot] {text}");
        }
    }
}

using DarkLog;
using FSControl;
using UnityEngine;

namespace Autopilot
{
    public class NetworkTestMain : MonoBehaviour
    {

        ModLog log;
        DataStore data;
        ProtocolLogic protocol;
        NetworkHandler handler;

        public void Start()
        {
            log = new ModLog("NetworkTest");
            log.Log("Start!");
            data = new DataStore();
            protocol = new ProtocolLogic(data, log);

            handler = new NetworkHandler();
            handler.RegisterConnect(protocol.ConnectEvent);
            handler.RegisterReceive(MAVLink.MAVLINK_MSG_ID.PARAM_REQUEST_LIST, protocol.ParamRequestList);
            handler.RegisterReceive(MAVLink.MAVLINK_MSG_ID.REQUEST_DATA_STREAM, protocol.RequestDataStream);
            handler.RegisterReceive(MAVLink.MAVLINK_MSG_ID.SYSTEM_TIME, protocol.SystemTime);
            handler.RegisterSend(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, protocol.SendHeartbeat);
            handler.RegisterSend(MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT, protocol.SendPosition);
            handler.RegisterSend(MAVLink.MAVLINK_MSG_ID.ATTITUDE, protocol.SendAttitude);
            handler.RegisterSend(MAVLink.MAVLINK_MSG_ID.RAW_IMU, protocol.SendRawIMU);
            handler.RegisterSend(MAVLink.MAVLINK_MSG_ID.GPS_STATUS, protocol.SendGPSStatus);
            handler.RegisterDisconnect(protocol.DisconnectEvent);
            handler.StartServer(log.Log);
            DontDestroyOnLoad(this);
        }

        public void FixedUpdate()
        {
            if (!GameLogic.inGame || !GameLogic.SceneryLoaded || GameLogic.LocalPlayerVehicle == null || !GameLogic.LocalPlayerVehicle.InitComplete)
            {
                data.pitch = 0;
                data.roll = 0;
                data.yaw = 0;
                data.latitude = 0;
                data.longitude = 0;
                data.altitude = 0;
                data.heading = 0;
                return;
            }
            Vehicle v = GameLogic.LocalPlayerVehicle;
            data.pitch = FSControlUtil.GetVehiclePitch(v) * Mathf.Rad2Deg;
            data.roll = FSControlUtil.GetVehicleRoll(v) * Mathf.Rad2Deg;
            data.yaw = FSControlUtil.GetVehicleYaw(v) * Mathf.Rad2Deg;
            //Metres -> mm
            data.altitude = v.Physics.Altitude * 1000f;

            data.heading = v.Physics.HeadingDegs;

            //Balsa is YUp
            //Mavlink is degE7, 1° = 111 km 1E7/111000 = ~90
            data.latitude = (int)(FloatingOrigin.GetAbsoluteWPos(v.transform.position).x / 90d);
            data.longitude = (int)(FloatingOrigin.GetAbsoluteWPos(v.transform.position).z / 90d);
        }
    }
}

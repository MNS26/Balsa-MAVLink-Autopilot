using System;
using DarkLog;
using UnityEngine;
using FSControl;
using IO;
using component_information;
using System.Collections.Generic;
using Autopilot;
using System.Runtime.InteropServices;

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
            //hander.RegisterReceive(MAVLink.MAVLINK_MSG_ID.SET_RATE, protocol.ReceiveSetRate);
            handler.RegisterSend(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, protocol.SendHeartbeat);
            //handler.RegisterReceive(MAVLink.MAVLINK_MSG_ID.PARAM_REQUEST_LIST, protocol.ParamRequestList);
            handler.RegisterReceive(MAVLink.MAVLINK_MSG_ID.REQUEST_DATA_STREAM, protocol.RequestDataStream);
            handler.RegisterSend(MAVLink.MAVLINK_MSG_ID.ATTITUDE, protocol.SendAttitude);
            handler.RegisterReceiveCommand(MAVLink.MAV_CMD.REQUEST_AUTOPILOT_CAPABILITIES, protocol.RequestAutopilot);
            handler.RegisterSend(MAVLink.MAVLINK_MSG_ID.RAW_IMU, protocol.SendRawIMU);
            handler.RegisterSend(MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT, protocol.SendPosition);
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
                return;
            }
            Vehicle v = GameLogic.LocalPlayerVehicle;
            data.pitch = FSControlUtil.GetVehiclePitch(v) * Mathf.Rad2Deg;
            data.roll = FSControlUtil.GetVehicleRoll(v) * Mathf.Rad2Deg;
            data.yaw = v.Physics.HeadingDegs;
            //Balsa is YUp
            //Mavlink is degE7, 1° = 111 km 1E7/111000 = ~90
            data.latitude = (int)(FloatingOrigin.GetAbsoluteWPos(v.transform.position).x / 90d);
            data.longitude = (int)(FloatingOrigin.GetAbsoluteWPos(v.transform.position).z / 90d);
            //Metres -> mm
            data.altitude = (int)(FloatingOrigin.GetAbsoluteWPos(v.transform.position).y * 1000d);
        }
    }
}

using FSControl;
using Modules;
using UnityEngine;
using AutopilotCommon;

namespace Autopilot
{
    public class Autopilot : MonoBehaviour
    {

        DataStore data;
        ProtocolLogic protocol;
        NetworkHandler handler;

        public void Start()
        {
            Log("Start!");
            data = new DataStore();
            protocol = new ProtocolLogic(data, Log);
            handler = new NetworkHandler(protocol, Log);
            handler.StartServer();
            DontDestroyOnLoad(this);
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

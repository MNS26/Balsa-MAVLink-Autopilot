using AutopilotCommon;
using FSControl;
using Modules;
using UnityEngine;
using System.IO;
namespace Autopilot
{
    public class Autopilot : MonoBehaviour
    {
        DataStore data;
        ApStore ap;
        ProtocolLogic protocol;
        NetworkHandler handler;

        public void Start()
        {
            GameObject.DontDestroyOnLoad(this);
            Log("Start!");
            data = new DataStore();
            ap = new ApStore();
            ParameterHandler parameters;
            parameters = new ParameterHandler(PathUtil.Resolve(".") + "/Addons/Autopilot/", "Parameters.dat", Log);
            protocol = new ProtocolLogic(data, ap, Log, parameters);
            handler = new NetworkHandler(protocol, Log);
            handler.StartServer();
            //If you want to stick around
            GameEvents.Vehicles.OnVehicleSpawned.AddListener(VehicleSpawned);
        }


        private void VehicleSpawned(Vehicle vehicle)
        {
            Autopilot.Log("OVS Main");
            if (vehicle == GameLogic.LocalPlayerVehicle)
            {
                if (!vehicle.gameObject.TryGetComponent(out AutopilotComponent _))
                {
                    Log("Adding autopilot controller to " + vehicle.name);
                    AutopilotComponent ac = vehicle.gameObject.AddComponent(typeof(AutopilotComponent)) as AutopilotComponent;
                    ac.OnVehicleSpawn(vehicle);

                }
                else
                {
                    Log("Autopilot controller already exists for " + vehicle.name);
                }
            }
        }

        //running this as fast as possible
        public void Update()
        {
            if (!GameLogic.inGame || !GameLogic.SceneryLoaded || GameLogic.LocalPlayerVehicle == null || !GameLogic.LocalPlayerVehicle.InitComplete)
            {
                data.ch1 = data.ch2 = data.ch3 = data.ch4 = data.ch5 = data.ch6 = data.ch7 = data.ch8 = 1500;
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
                ap.mode = MAVLink.MAV_MODE_FLAG.SAFETY_ARMED;
                data.avrrpm = 0;
                data.latitude = 0;
                data.longitude = 0;
                data.altitude = 0;
                data.heading = 0;
                data.iaspeed = 0;
                data.name = "";

                data.magx = Vector3.Dot(new Vector3(0, 0, 1), new Vector3(0, 0, 1)) * 500;
                data.magy = Vector3.Dot(new Vector3(0, 0, 0), new Vector3(0, 0, 1)) * 500;
                data.magz = Vector3.Dot(new Vector3(0, 0, 0), new Vector3(0, 0, 1)) * 500;
                data.currentGVelx = 0;
                data.currentGVely = 0;
                data.currentGVelz = 0;
                data.lastGVelx = 0;
                data.lastGVely = 0;
                data.lastGVelz = 0;
                data.gyrox = 0;
                data.gyroy = 0;
                data.gyroz = 0;
                data.currentAVelx = 0;
                data.currentAVely = 0;
                data.currentAVelz = 0;
                data.lastAVelx = 0;
                data.lastAVely = 0;
                data.lastAVelz = 0;
                data.accx = 0;
                data.accy = 0;
                data.accz = 0;
                return;
            }
            Vehicle v = GameLogic.LocalPlayerVehicle;

            data.radpitch = FSControlUtil.GetVehiclePitch(v); data.pitch = data.radpitch * Mathf.Rad2Deg;
            data.radroll = FSControlUtil.GetVehicleRoll(v); data.roll = data.radroll * Mathf.Rad2Deg;
            data.radyaw = FSControlUtil.GetVehicleYaw(v); data.yaw = data.radyaw * Mathf.Rad2Deg;


            //Metres -> mm
            data.altitude = v.Physics.Altitude * 1000f;
            data.iaspeed = v.Physics.Speed;
            data.heading = v.Physics.HeadingDegs;
            data.cr = v.Physics.VerticalSpeed;
            data.name = v.transform.name;

            //IMU
            data.magx = Vector3.Dot(v.transform.forward, new Vector3(-1, 0, 0)) * 500; //good
            data.magy = Vector3.Dot(v.transform.forward, new Vector3(0, -1, 0)) * 500; //good
            data.magz = Vector3.Dot(v.transform.forward, new Vector3(0, 0, 1)) * 500; //good

            data.currentGVelx = v.Physics.AngularVelocity.x * 1000;
            data.currentGVely = v.Physics.AngularVelocity.y * 1000;
            data.currentGVelz = v.Physics.AngularVelocity.z * 1000;
            data.gyrox = (data.currentGVelx - data.lastGVelx) / Time.deltaTime;
            data.gyroy = (data.currentGVely - data.lastGVely) / Time.deltaTime;
            data.gyroz = (data.currentGVelz - data.lastGVelz) / Time.deltaTime;
            data.lastGVelx = data.currentGVelx;
            data.lastGVely = data.currentGVely;
            data.lastGVelz = data.currentGVelz;

            data.currentAVelx = v.Physics.Velocity.x * 1000;
            data.currentAVely = v.Physics.Velocity.y * 1000;
            data.currentAVelz = v.Physics.Velocity.z * 1000;
            data.accx = (data.currentAVelx - data.lastAVelx) / Time.deltaTime;
            data.accy = (data.currentAVely - data.lastAVely) / Time.deltaTime;
            data.accz = (data.currentAVelz - data.lastAVelz) / Time.deltaTime;
            data.lastAVelx = data.currentAVelx;
            data.lastAVely = data.currentAVely;
            data.lastAVelz = data.currentAVelz;



            //Balsa is YUp
            //Mavlink is degE7, 1° = 111 km 1E7/111000 = ~90
            data.latitude = (int)(FloatingOrigin.GetAbsoluteWPos(v.transform.position).z * 90.09f);
            data.longitude = (int)(FloatingOrigin.GetAbsoluteWPos(v.transform.position).x * 90.09f);



            //controller stuff
            data.rssi = map(v.SignalStrength.SignalDegradation, 0, 1, 255, 0);

            data.ch1 = InputSettings.Axis_Roll.GetAxis();
            data.ch2 = InputSettings.Axis_Pitch.GetAxis();
            data.ch3 = InputSettings.Axis_Throttle.GetAxis();
            data.ch4 = InputSettings.Axis_Yaw.GetAxis();
            data.ch5 = InputSettings.Axis_A.GetAxis();
            data.ch6 = InputSettings.Axis_B.GetAxis();
            data.ch7 = InputSettings.Axis_C.GetAxis();
            data.ch8 = InputSettings.Axis_D.GetAxis();
        }

        public void FixedUpdate()
        {
            if (!GameLogic.inGame || !GameLogic.SceneryLoaded || GameLogic.LocalPlayerVehicle == null || !GameLogic.LocalPlayerVehicle.InitComplete)
            {
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
            var engine = v.GetModules<Engine>();
            if (engine.Count != 0 && engine[0].running)
            {
                ap.mode = MAVLink.MAV_MODE_FLAG.SAFETY_ARMED;
            }
            else
            {
                ap.mode = MAVLink.MAV_MODE_FLAG.MANUAL_INPUT_ENABLED;
            }

        }

        public static float map(float value, float fromLow, float fromHigh, float toLow, float toHigh)
        {
            return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
        }

        //It's nice to identify in the log where things came from
        public static void Log(string text)
        {
            // Unity didn't like this
            // Debug.Log($"{Time.realtimeSinceStartup} [Autopilot] {text}");
            Debug.Log($"[Autopilot] {text}");
        }
    }
}

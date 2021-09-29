using AutopilotCommon;
using FSControl;
using UnityEngine;

namespace Autopilot
{
    public class AutopilotComponent : MonoBehaviour, IVehicleComponent
    {
        //
        DataStore data = Autopilot.data;
        ApStore ap = Autopilot.ap;

        Vehicle vehicle;
        public FBWModule fbwModule;

        public PID pitchPid;
        public PID rollPid;
        public PID speedPid;

        public PID verticalSpeedPid;
        public PID altitudePid;
        public PID headingPid;

        float altitude = 100;
        float heading = 210;
        float speed = 80;

        private double GetVehiclePitch()
        {
            return FSControlUtil.GetVehiclePitch(vehicle) * Mathf.Rad2Deg;
        }
        private double GetVehicleYaw()
        {
            return FSControlUtil.GetVehicleYaw(vehicle) * Mathf.Rad2Deg;
        }
        private double GetVehicleRoll()
        {
            return FSControlUtil.GetVehicleRoll(vehicle) * Mathf.Rad2Deg;
        }

        private double GetRoll()
        {
            if (-1 <= data.ch8 && data.ch8 < -0.6) //mode 1
            { return (FSControlUtil.GetVehicleRoll(vehicle) * Mathf.Rad2Deg)+Autopilot.map(data.ch1, -1, 1, -20, 20); }
            if (-0.6 < data.ch8 && data.ch8 < -0.3) //mode 2
            { return Autopilot.map(data.ch1, -1, 1, -60, 60); }
            if (-0.3 < data.ch8 && data.ch8 <= 0.00) //mode 3
            { return (FSControlUtil.GetVehicleRoll(vehicle) * Mathf.Rad2Deg) + Autopilot.map(data.ch1, -1, 1, -25, 25); }
            if (0 < data.ch8 && data.ch8 <= 0.3) //mode 4
            { return (FSControlUtil.GetVehicleRoll(vehicle) * Mathf.Rad2Deg) + Autopilot.map(data.ch1, -1, 1, -90, 90); }
            if (0.3 < data.ch8 && data.ch8 <= 0.6) //mode 5
            { return 0.00; }
            if (0.6 < data.ch8 && data.ch8 <= 1) //mode 6
            { return 0.00; }
            else
                return 0.00;
        }
        private double GetPitch()
        {
            if (-1 <= data.ch8 && data.ch8 < -0.6) //mode 1
            { return (FSControlUtil.GetVehiclePitch(vehicle) * Mathf.Rad2Deg)+Autopilot.map(data.ch2, -1, 1, -20, 20); }
            if (-0.6 < data.ch8 && data.ch8 < -0.3) //mode 2
            { return Autopilot.map(data.ch2, -1, 1, -60, 60); }
            if (-0.3 < data.ch8 && data.ch8 <= 0.00) //mode 3
            { return Autopilot.map(data.ch2, -1, 1, -25, 25) + (verticalSpeedPid.Output());}
            if (0 < data.ch8 && data.ch8 < 0.3) //mode 4
            { return (FSControlUtil.GetVehiclePitch(vehicle) * Mathf.Rad2Deg) + Autopilot.map(data.ch2, -1, 1, -90, 90); }
            if (3 < data.ch8 && data.ch8 < 0.6) //mode 5
            { return 0.00; }
            if (6 < data.ch8 && data.ch8 >= 1) //mode 6
            { return 0.00; }
            else
                return 0.00;
        }
        private double GetYaw() //not used
        {
            if (-1 >= data.ch8 && data.ch8 < -0.6) //mode 1
            { return (FSControlUtil.GetVehicleYaw(vehicle) * Mathf.Rad2Deg)+Autopilot.map(data.ch4, -1, 1, -20, 20); }
            if (-0.6 >= data.ch8 && data.ch8 < -0.3) //mode 2
            { return (FSControlUtil.GetVehicleYaw(vehicle) * Mathf.Rad2Deg) + Autopilot.map(data.ch4, -1, 1, -60, 60); }
            if (-0.3 >= data.ch8 && data.ch8 <= 0.00) //Hmode 3
            { return (FSControlUtil.GetVehicleYaw(vehicle) * Mathf.Rad2Deg) + Autopilot.map(data.ch4, -1, 1, -25, 25); }
            else
                return (FSControlUtil.GetVehicleYaw(vehicle) * Mathf.Rad2Deg) + Autopilot.map(data.ch4, -1, 1, -30, 30);


        }
        private double GetThrottle()
        {
            return Autopilot.map(data.ch3, -1, 1, 0, 100);
        }





        private double GetHeadingError()
        {
            //Autopilot.Log(vehicle.Physics.HeadingDegs.ToString());
            double error = heading - vehicle.Physics.HeadingDegs;
            if (error > 180)
            {
                error -= 360;
            }
            if (error < -180)
            {
                error += 360;
            }
            return error;
        }

        public void OnVehicleSpawn(Vehicle vehicle)
        {
            Autopilot.Log("OVS Component");
            this.vehicle = vehicle;
            fbwModule = new FBWModule(APUpdate);
            vehicle.Autotrim.host.RegisterFBWModule(fbwModule);

            //Vertical
            //Clamp VS to -10m/s to 10m/s. Kp = 5m/s per 10m. 0.5.
            //altitudePid = new PID(0.5, 0, 0, -3, 3, () => { return altitude; }, () => { return Time.time; }, (double v) => { vs = (float)v; });
            altitudePid = new PID()
            {
                kP = 0.5,
                kI = 0,
                kD = 0,
                rangeMin = -1,
                rangeMax = 1,
                input = () => { return vehicle.Physics.Altitude; },
                setpoint = () => { return altitude; },
                clockSource = () => { return Time.time; },
            };

            //Clamp pitch to -30 to 30 degrees. Kp = 30 degrees for 10m/s. 3.
            //verticalSpeedPid = new PID(3, 0.1, 0, -30, 30, () => { return vehicle.Physics.VerticalSpeed; }, () => { return vs; }, () => { return Time.time; }, (double v) => { pitch = (float)v; });
            verticalSpeedPid = new PID()
            {
                kP = 3,
                kI = 0.2,
                kD = 0,
                rangeMin = -30,
                rangeMax = 30,
                input = () => { return vehicle.Physics.VerticalSpeed; },
                setpoint = () => 0.00,
                clockSource = () => { return Time.time; },
            };

            //Clamp control from -1 to 1. Kp = 50 degrees error = full deflection, 0.02.
            pitchPid = new PID()
            {
                kP = 0.02,
                kI = 0.005,
                kD = 0.005,
                rangeMin = -1,
                rangeMax = 1,
                input = GetVehiclePitch,
                setpoint = GetPitch,//verticalSpeedPid.Output,
                clockSource = () => { return Time.time; },
                outputCallback = (double output) => { fbwModule.pitch = (float)output; },
            };

            //Horizontal
            //Clamp to -30 to 30 degrees, Kp = 20 degrees error = full deflection, 0.05.
            headingPid = new PID()
            {
                kP = 0.05,
                kI = 0.01,
                kD = 0.01,
                rangeMin = -30,
                rangeMax = 30,
                input = GetHeadingError,
                setpoint = () => { return 0; },
                clockSource = () => { return Time.time; },
            };

            //Clamp control from -1 to 1. Kp = 50 degrees error = full deflection, 0.02. Clamp yaw to 0.5 * roll.
            rollPid = new PID()
            {

                kP = 0.02,
                kI = 0.001,
                kD = 0,
                rangeMin = -1,
                rangeMax = 1,
                input = GetVehicleRoll,
                setpoint = GetRoll,//headingPid.Output,
                clockSource = () => { return Time.time; },
                outputCallback = (double output) => { fbwModule.roll = (float)output; fbwModule.yaw = (float)output / 2f; },
            };

            //Autothrottle
            speedPid = new PID()
            {
                kP = 0.02,
                kI = 0.01,
                kD = 0,
                rangeMin = 0,
                rangeMax = 1,
                input = () => { return vehicle.Physics.Speed * 3.6f; },
                setpoint = GetThrottle,//() => { return speed; },
                clockSource = () => { return Time.time; },
                outputCallback = (double output) => { fbwModule.throttle = (float)output; },
            };
        }

        public void APUpdate()
        {
            if (vehicle != null && vehicle.Autotrim != null && vehicle.Autotrim.enabled)
            {
                vehicle.Autotrim.DisableAT();
            }
            //Autopilot.Log($"{data.ch8}");
            /*
            fbwModule.pitchEnabled = false;
            fbwModule.rollEnabled = false;
            fbwModule.yawEnabled = false;
            fbwModule.throttleEnabled = false;
            */
            //Autopilot.Log($"{GetPitch()}");
            //Vertical
            altitudePid.FixedUpdate();
            //Autopilot.Log($"Alt error: {altitudePid.error}, output: {altitudePid.outputValue}");
            verticalSpeedPid.FixedUpdate();
            //Autopilot.Log($"VS error: {verticalSpeedPid.error}, output: {verticalSpeedPid.outputValue}");
            pitchPid.FixedUpdate();
            //Autopilot.Log($"Pitch error: {pitchPid.error}, output: {pitchPid.outputValue}");
            fbwModule.pitchEnabled = true;

            headingPid.FixedUpdate();
            //Autopilot.Log($"Heading error: {headingPid.error}, output: {headingPid.outputValue}");

            rollPid.FixedUpdate();
            //Autopilot.Log($"Roll error: {rollPid.error}, output: {rollPid.outputValue}");
            fbwModule.rollEnabled = true;
            fbwModule.yawEnabled = true;

            speedPid.FixedUpdate();
            //Autopilot.Log($"Speed error: {speedPid.error}, output: {speedPid.outputValue}");
            fbwModule.throttleEnabled = false;
        }
    }
}
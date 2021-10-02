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
        ParameterHandler parameters = Autopilot.parameters;

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

        public void Start()
        {
            parameters.ParameterEvent += ParameterWatcher;
        }

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
            { return (FSControlUtil.GetVehicleRoll(vehicle) * Mathf.Rad2Deg) + (data.ch1 * 20); }
            if (-0.6 < data.ch8 && data.ch8 < -0.3) //mode 2
            { return (data.ch1 * 60); }
            if (-0.3 < data.ch8 && data.ch8 < 0) //mode 3
            { return (FSControlUtil.GetVehicleRoll(vehicle) * Mathf.Rad2Deg) + (data.ch1 * 25); }
            if (0 < data.ch8 && data.ch8 < 0.3) //mode 4
            { return (FSControlUtil.GetVehicleRoll(vehicle) * Mathf.Rad2Deg) + (data.ch1 * 90); }
            if (0.3 < data.ch8 && data.ch8 < 0.6) //mode 5
            { return (FSControlUtil.GetVehicleRoll(vehicle) * Mathf.Rad2Deg); }
            if (0.6 < data.ch8 && data.ch8 <= 1) //mode 6
            { return 0.00; }
            else
                return (FSControlUtil.GetVehicleRoll(vehicle) * Mathf.Rad2Deg);
        }
        private double GetPitch()
        {
            if (-1 <= data.ch8 && data.ch8 < -0.6) //mode 1
            { return (FSControlUtil.GetVehiclePitch(vehicle) * Mathf.Rad2Deg) + (data.ch2 * 20); }
            if (-0.6 < data.ch8 && data.ch8 < -0.3) //mode 2
            { return (data.ch2 * 60); }
            if (-0.3 < data.ch8 && data.ch8 < 0) //mode 3
            { return (data.ch2 * 25) + (verticalSpeedPid.Output()); }
            if (0 < data.ch8 && data.ch8 < 0.3) //mode 4
            { return (FSControlUtil.GetVehiclePitch(vehicle) * Mathf.Rad2Deg) + (data.ch2 * 90); }
            if (0.3 < data.ch8 && data.ch8 < 0.6) //mode 5
            { return (FSControlUtil.GetVehiclePitch(vehicle) * Mathf.Rad2Deg); }
            if (0.6 < data.ch8 && data.ch8 <= 1) //mode 6
            { return 0.00; }
            else
                return (FSControlUtil.GetVehiclePitch(vehicle) * Mathf.Rad2Deg);
        }
        private double GetYaw() //not used
        {
            if (-1 <= data.ch8 && data.ch8 < -0.6) //mode 1
            { return (FSControlUtil.GetVehicleYaw(vehicle) * Mathf.Rad2Deg) + (data.ch4 * 25); }
            if (-0.6 < data.ch8 && data.ch8 < -0.3) //mode 2
            { return (FSControlUtil.GetVehicleYaw(vehicle) * Mathf.Rad2Deg) + (data.ch4 * 60); }
            if (-0.3 < data.ch8 && data.ch8 < 0) //Hmode 3
            { return (FSControlUtil.GetVehicleYaw(vehicle) * Mathf.Rad2Deg) + (data.ch4 * 25); }
            if (0 < data.ch8 && data.ch8 < 0.03) //Hmode 4
            { return (FSControlUtil.GetVehicleYaw(vehicle) * Mathf.Rad2Deg) + (data.ch4 * 25); }
            if (0.3 < data.ch8 && data.ch8 < 0.06) //Hmode 5
            { return (FSControlUtil.GetVehicleYaw(vehicle) * Mathf.Rad2Deg) + (data.ch4 * 25); }
            if (0.6 < data.ch8 && data.ch8 <= 1) //Hmode 6
            { return (FSControlUtil.GetVehicleYaw(vehicle) * Mathf.Rad2Deg) + (data.ch4 * 25); }
            else
                return (FSControlUtil.GetVehicleYaw(vehicle) * Mathf.Rad2Deg);


        }
        private double GetThrottle()
        {
            if (-1 <= data.ch8 && data.ch8 < -0.6) //mode 1
            { return (data.ch3 * 100); }
            if (-0.6 < data.ch8 && data.ch8 < -0.3) //mode 2
            { return (data.ch3 * 100); }
            if (-0.3 < data.ch8 && data.ch8 < 0) //mode 3
            { return (data.ch3 * 100); }
            if (0 < data.ch8 && data.ch8 < 0.3) //mode 4
            { return (data.ch3 * 100); }
            if (0.3 < data.ch8 && data.ch8 < 0.6) //mode 5
            { return (data.ch3 * 100); }
            if (0.6 < data.ch8 && data.ch8 <= 1) //mode 6
            { return (data.ch3 * 100); }
            else
                return (data.ch3 * 100);

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
                rangeMin = -parameters.GetParameter("LIM_PITCH_MIN").value / 100,
                rangeMax = parameters.GetParameter("LIM_PITCH_MAX").value / 100,
                input = () => { return vehicle.Physics.VerticalSpeed; },
                setpoint = () => 0.00,
                clockSource = () => { return Time.time; },
            };

            //Clamp control from -1 to 1. Kp = 50 degrees error = full deflection, 0.02.
            pitchPid = new PID()
            {
                kP = parameters.GetParameter("PTCH_RATE_P").value,
                kI = parameters.GetParameter("PTCH_RATE_I").value,
                kD = parameters.GetParameter("PTCH_RATE_D").value,
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
                rangeMin = -parameters.GetParameter("LIM_ROLL_CD").value / 100,
                rangeMax = parameters.GetParameter("LIM_ROLL_CD").value / 100,
                input = GetHeadingError,
                setpoint = () => { return 0; },
                clockSource = () => { return Time.time; },
            };

            //Clamp control from -1 to 1. Kp = 50 degrees error = full deflection, 0.02. Clamp yaw to 0.5 * roll.
            rollPid = new PID()
            {

                kP = parameters.GetParameter("RLL_RATE_P").value,
                kI = parameters.GetParameter("RLL_RATE_I").value,
                kD = parameters.GetParameter("RLL_RATE_D").value,
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

        private void ParameterWatcher(Parameter p)
        {
            switch (p.id)
            {
                case "PTCH_RATE_P":
                    pitchPid.kP = p.value;
                    break;
                case "PTCH_RATE_I":
                    pitchPid.kI = p.value;
                    break;
                case "PTCH_RATE_D":
                    pitchPid.kD = p.value;
                    break;
                case "RLL_RATE_P":
                    rollPid.kP = p.value;
                    break;
                case "RLL_RATE_I":
                    rollPid.kI = p.value;
                    break;
                case "RLL_RATE_D":
                    rollPid.kD = p.value;
                    break;
                case "LIM_PITCH_MIN":
                    verticalSpeedPid.rangeMin = p.value / 100;
                    break;
                case "LIM_PITCH_MAX":
                    verticalSpeedPid.rangeMax = p.value / 100;
                    break;
                case "LIM_ROLL_CD":
                    headingPid.rangeMin = -p.value / 100;
                    headingPid.rangeMax = p.value / 100;
                    break;
            }
        }
    }
}
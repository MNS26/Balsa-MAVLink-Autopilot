using AutopilotCommon;
using FSControl;
using UnityEngine;
using System;

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
#pragma warning disable CS0414 // Variable is assigned but its value is never used
        float speed = 80;
#pragma warning disable CS0414 // Variable is assigned but its value is never used
        int mode;
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
            switch (mode)
            {
                case -6:
                    return (FSControlUtil.GetVehicleRoll(vehicle) * Mathf.Rad2Deg) + (data.channels[0] * 25);
                case -3:
                    return (data.channels[0] * 25);
                case -1:
                    return (FSControlUtil.GetVehicleRoll(vehicle) * Mathf.Rad2Deg) + (data.channels[0] * 25);
                case 1:
                    return (FSControlUtil.GetVehicleRoll(vehicle) * Mathf.Rad2Deg) + (data.channels[0] * 25);
                case 3:
                    return (FSControlUtil.GetVehicleRoll(vehicle) * Mathf.Rad2Deg);
                case 6:
                    return 0.00;
                default:
                    return (FSControlUtil.GetVehicleRoll(vehicle) * Mathf.Rad2Deg);

            }
        }
        private double GetPitch()
        {
            switch (mode)
            {
                case -6:
                    return (FSControlUtil.GetVehiclePitch(vehicle) * Mathf.Rad2Deg) + (data.channels[1] * 45);
                case -3:
                    return (data.channels[1] * 45);
                case -1:
                    return (data.channels[1] * 45) + (verticalSpeedPid.Output());
                case 1:
                    return (FSControlUtil.GetVehiclePitch(vehicle) * Mathf.Rad2Deg) + (data.channels[1] * 45);
                case 3:
                    return (FSControlUtil.GetVehiclePitch(vehicle) * Mathf.Rad2Deg);
                case 6:
                    return 0.00;
                default:
                    return (FSControlUtil.GetVehiclePitch(vehicle) * Mathf.Rad2Deg);
            }
        }
        private double GetYaw() //not used
        {
            switch (mode)
            {
                case -6:
                    return (FSControlUtil.GetVehicleYaw(vehicle) * Mathf.Rad2Deg) + (data.channels[3] * 45);
                case -3:
                    return (FSControlUtil.GetVehicleYaw(vehicle) * Mathf.Rad2Deg) + (data.channels[3] * 45);
                case -1:
                    return (FSControlUtil.GetVehicleYaw(vehicle) * Mathf.Rad2Deg) + (data.channels[3] * 45);
                case 1:
                    return (FSControlUtil.GetVehicleYaw(vehicle) * Mathf.Rad2Deg) + (data.channels[3] * 45);
                case 3:
                    return (FSControlUtil.GetVehicleYaw(vehicle) * Mathf.Rad2Deg) + (data.channels[3] * 45);
                case 6:
                    return (FSControlUtil.GetVehicleYaw(vehicle) * Mathf.Rad2Deg) + (data.channels[3] * 45);
                default:
                    return (FSControlUtil.GetVehicleYaw(vehicle) * Mathf.Rad2Deg);
            }
        }
        private double GetThrottle()
        {
            switch (mode)
            {
                case -6:
                    return (data.channels[2] * 100);
                case -3:
                    return (data.channels[2] * 100);
                case -1:
                    return (data.channels[2] * 100);
                case 1:
                    return (data.channels[2] * 100);
                case 3:
                    return (data.channels[2] * 100);
                case 6:
                    return (data.channels[2] * 100);
                default:
                    return (data.channels[2] * 100);
            }
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
                kP = 0.5,
                kI = 0.1,
                kD = 0.1,
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
                input = () => {return FSControlUtil.GetVehiclePitch(vehicle) * Mathf.Rad2Deg;},//GetVehiclePitch,
                setpoint = GetPitch,//verticalSpeedPid.Output,
                clockSource = () => { return Time.time; },
                outputCallback = (double output) => {fbwModule.pitch = (float)output;}

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
                input =  () => {return FSControlUtil.GetVehicleRoll(vehicle) * Mathf.Rad2Deg;},//GetVehicleRoll,
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
            mode = (int)(data.channels[7] * 6.125);

            /*
            fbwModule.pitchEnabled = false;
            fbwModule.rollEnabled = false;
            fbwModule.yawEnabled = false;
            fbwModule.throttleEnabled = false;
            */
            //Vertical
            altitudePid.FixedUpdate();
            verticalSpeedPid.FixedUpdate();
            pitchPid.FixedUpdate();
            fbwModule.pitchEnabled = false;

            headingPid.FixedUpdate();

            rollPid.FixedUpdate();
            fbwModule.rollEnabled = false;
            fbwModule.yawEnabled = false;

            speedPid.FixedUpdate();
            //Autopilot.Log($"roll: {rollPid.outputValue} pitch: {pitchPid.outputValue}");
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
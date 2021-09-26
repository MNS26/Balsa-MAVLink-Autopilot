using System;
using UnityEngine;
using FSControl;
using AutopilotCommon;

namespace Autopilot
{
    public class AutopilotComponent : MonoBehaviour, IVehicleComponent
    {
        //
        DataStore data;
        ApStore ap;

        Vehicle vehicle;
        FBWModule fbwModule;
        //Vertical
        PID pitchPid;
        PID verticalSpeedPid;
        PID altitudePid;

        float altitude = 100;

        //Horizontal
        PID rollPid;
        PID headingPid;
        float heading = 210;

        //Speed
        PID speedPid;

        //km/h
        float speed = 80;

        private double GetPitch()
        {
            return FSControlUtil.GetVehiclePitch(vehicle) * Mathf.Rad2Deg;
        }
        private double GetRoll()
        {
            return FSControlUtil.GetVehicleRoll(vehicle) * Mathf.Rad2Deg;
        }

        private double GetAcroRoll()
        {
            return (data.ch1 * 10) + FSControlUtil.GetVehicleRoll(vehicle) * Mathf.Rad2Deg; ;
        }
        private double GetAcroPitch()
        {
            return (data.ch2*10) + FSControlUtil.GetVehiclePitch(vehicle) * Mathf.Rad2Deg; ;
        }
        private double GetAcroYaw()
        {
            return (data.ch4*10) + FSControlUtil.GetVehicleYaw(vehicle) * Mathf.Rad2Deg; ;
        }
        private double GetThrottle()
        {
            return Autopilot.map(data.ch3,-1,1,0,100);
        }





        private double GetHeadingError()
        {
            Autopilot.Log(vehicle.Physics.HeadingDegs.ToString());
            double error = heading - vehicle.Physics.HeadingDegs;
            if (error > 180)
            {
                error = error - 360;
            }
            if (error < -180)
            {
                error = error + 360;
            }
            return error;
        }

        public void OnVehicleSpawn(Vehicle vehicle)
        {
            Autopilot.Log("OVS Component");
            data = new DataStore();
            ap = new ApStore();
            this.vehicle = vehicle;
            fbwModule = new FBWModule(APUpdate);
            vehicle.Autotrim.host.RegisterFBWModule(fbwModule);

            //Vertical
            //Clamp VS to -10m/s to 10m/s. Kp = 5m/s per 10m. 0.5.
            //altitudePid = new PID(0.5, 0, 0, -3, 3, }, () => { return altitude; }, () => { return Time.time; }, (double v) => { vs = (float)v; });
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
                kI = 0.1,
                kD = 0,
                rangeMin = -30,
                rangeMax = 30,
                input = () => { return vehicle.Physics.Altitude; },
                setpoint = altitudePid.Output,
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
                input = GetPitch,
                setpoint = verticalSpeedPid.Output,
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

            double temp = GetRoll();
            //Clamp control from -1 to 1. Kp = 50 degrees error = full deflection, 0.02. Clamp yaw to 0.5 * roll.
            rollPid = new PID()
            {
                kP = 0.02,
                kI = 0.001,
                kD = 0,
                rangeMin = -1,
                rangeMax = 1,
                input = GetRoll,
                setpoint = headingPid.Output,
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
                setpoint = () => { return speed; },
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
            /*
            apModule.pitchEnabled = false;
            apModule.rollEnabled = false;
            apModule.yawEnabled = false;
            apModule.throttleEnabled = false;
            */
            //Vertical
            altitudePid.FixedUpdate();
            //Autopilot.Log($"Alt error: {altitudePid.error}, output: {altitudePid.outputValue}");
            verticalSpeedPid.FixedUpdate();
            //Autopilot.Log($"VS error: {verticalSpeedPid.error}, output: {verticalSpeedPid.outputValue}");
            pitchPid.FixedUpdate();
            Autopilot.Log($"Pitch error: {pitchPid.error}, output: {pitchPid.outputValue}");
            fbwModule.pitchEnabled = true;

            headingPid.FixedUpdate();
            Autopilot.Log($"Heading error: {headingPid.error}, output: {headingPid.outputValue}");
            
            rollPid.FixedUpdate();
            Autopilot.Log($"Roll error: {rollPid.error}, output: {rollPid.outputValue}");
            fbwModule.rollEnabled = true;
            fbwModule.yawEnabled = true;

            speedPid.FixedUpdate();
            Autopilot.Log($"Speed error: {speedPid.error}, output: {speedPid.outputValue}");
            fbwModule.throttleEnabled = true;
        }
    }
}
using FSControl;
using UnityEngine;

namespace Autopilot
{
    public class AutopilotComponent : MonoBehaviour, IVehicleComponent
    {
        Vehicle vehicle;
        FBWModule apModule;
        PID pitchPid;
        PID verticalSpeedPid;
        PID altitudePid;
        float pitch = 0;
        float vs = 0;
        float altitude = 100;
        bool vsEnabled = true;
        bool altitudeEnabled = true;
        //Horizontal
        PID rollPid;
        PID headingPid;
        float roll = 15;
        float heading = 210;
        bool headingEnabled = true;
        //Speed
        PID speedPid;
        //km/h
        float speed = 80;
        bool speedEnabled = true;

        bool toggle = false;

        private double GetPitch()
        {
            return FSControlUtil.GetVehiclePitch(vehicle) * Mathf.Rad2Deg;
        }

        private double GetRoll()
        {
            return FSControlUtil.GetVehicleRoll(vehicle) * Mathf.Rad2Deg;
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
            this.vehicle = vehicle;
            apModule = new FBWModule(APUpdate);

            vehicle.Autotrim.host.RegisterFBWModule(apModule);

            //Vertical
            //Clamp control from -1 to 1. Kp = 50 degrees error = full deflection, 0.02.
            pitchPid = new PID(0.02, 0.005, 0.005, -1, 1, GetPitch, () => { return pitch; },
                                                                    () => { return Time.time; },
                                                                    (double v) => { apModule.pitch = (float)v; });

            //Clamp pitch to -30 to 30 degrees. Kp = 30 degrees for 10m/s. 3.
            verticalSpeedPid = new PID(3, 0.1, 0, -30, 30, () => { return vehicle.Physics.VerticalSpeed; },
                                                           () => { return vs; },
                                                           () => { return Time.time; },
                                                           (double v) => { pitch = (float)v; });

            //Clamp VS to -10m/s to 10m/s. Kp = 5m/s per 10m. 0.5.
            altitudePid = new PID(0.5, 0, 0, -3, 3, () => { return vehicle.Physics.Altitude; },
                                                    () => { return altitude; },
                                                    () => { return Time.time; },
                                                    (double v) => { vs = (float)v; });

            //Horizontal
            //Clamp control from -1 to 1. Kp = 50 degrees error = full deflection, 0.02. Clamp yaw to 0.5 * roll.
            rollPid = new PID(0.02, 0.001, 0, -1, 1, GetRoll, () => { return roll; },
                                                              () => { return Time.time; },
                                                              (double v) => { apModule.roll = (float)v; apModule.yaw = (float)v / 2f; });

            //Clamp to -30 to 30 degrees, Kp = 20 degrees error = full deflection, 0.05.
            headingPid = new PID(0.05, 0.01, 0, -30, 30, GetHeadingError, () => { return 0; },
                                                                          () => { return Time.time; },
                                                                          (double v) => { roll = (float)v; });

            //Speed
            //Clamp from 0 to 1, Kp = 5m/s full deflection, 0.2.
            speedPid = new PID(0.2, 0.01, 0, 0, 1, () => { return vehicle.Physics.Speed * 3.6f; },
                                                   () => { return speed; },
                                                   () => { return Time.time; },
                                                   (double v) => { apModule.throttle = (float)v; });
        }

        public void APUpdate()
        {
            if (vehicle != null && vehicle.Autotrim != null && vehicle.Autotrim.enabled)
            {
                vehicle.Autotrim.DisableAT();
            }
            apModule.pitchEnabled = false;
            apModule.rollEnabled = false;
            apModule.yawEnabled = false;
            apModule.throttleEnabled = false;
            //Vertical
            if (altitudeEnabled)
            {
                altitudePid.FixedUpdate();
                Autopilot.Log($"Alt error: {altitudePid.error}, output: {altitudePid.outputValue}");
            }
            if (vsEnabled)
            {
                verticalSpeedPid.FixedUpdate();
                pitchPid.FixedUpdate();
                Autopilot.Log($"VS error: {verticalSpeedPid.error}, output: {verticalSpeedPid.outputValue}");
                Autopilot.Log($"Pitch error: {pitchPid.error}, output: {pitchPid.outputValue}");
                apModule.pitchEnabled = true;
            }
            if (headingEnabled)
            {
                //headingPid.FixedUpdate();
                rollPid.FixedUpdate();
                //Autopilot.Log($"Roll error: {rollPid.error}, output: {rollPid.outputValue}");
                //Autopilot.Log($"Heading error: {headingPid.error}, output: {headingPid.outputValue}");
                apModule.rollEnabled = true;
                apModule.yawEnabled = true;
            }
            if (speedEnabled)
            {
                speedPid.FixedUpdate();
                //Autopilot.Log($"Speed error: {speedPid.error}, output: {speedPid.outputValue}");
                apModule.throttleEnabled = true;
            }
        }
    }
}
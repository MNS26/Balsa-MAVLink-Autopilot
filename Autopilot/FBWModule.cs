using FSControl;
using System;

namespace Autopilot
{
    public class FBWModule : IFlyByWire
    {
        public float pitch;
        public bool pitchEnabled;
        public float roll;
        public bool rollEnabled;
        public float yaw;
        public bool yawEnabled;
        public float throttle;
        public bool throttleEnabled;
        public Action UpdateEvent;

        public FBWModule(Action UpdateEvent)
        {
            this.UpdateEvent = UpdateEvent;
        }

        public void OnRegistered(FBWHostBase host)
        {
            Autopilot.Log("OPCS Register");
        }

        public void OnUnregistered(FBWHostBase host)
        {
            Autopilot.Log("OPCS Unregister");
        }

        public void OnProcessCtrlState(ref FSInputState data, Vehicle vehicle)
        {
            UpdateEvent();
            if (pitchEnabled)
            {
                data.pitch = pitch;
            }
            if (rollEnabled)
            {
                data.roll = roll;
            }
            if (yawEnabled)
            {
                data.yaw = yaw;
            }
            if (throttleEnabled)
            {
                data.throttle = throttle;
            }
        }
    }
}
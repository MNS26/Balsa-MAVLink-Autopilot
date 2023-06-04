using System;
namespace AutopilotCommon
{
    public class ReceiveCommand : Attribute
    {
        public MAVLink.MAV_CMD id;

        public ReceiveCommand(MAVLink.MAV_CMD id)
        {
            this.id = id;
        }
    }
}

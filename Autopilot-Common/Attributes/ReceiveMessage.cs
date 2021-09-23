using System;
namespace AutopilotCommon
{
    public class ReceiveMessage : Attribute
    {
        public MAVLink.MAVLINK_MSG_ID id;

        public ReceiveMessage(MAVLink.MAVLINK_MSG_ID id)
        {
            this.id = id;
        }
    }
}

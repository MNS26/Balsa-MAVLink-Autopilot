using System;
namespace AutopilotCommon
{
    public class SendMessage : Attribute
    {
        public MAVLink.MAVLINK_MSG_ID id;

        public SendMessage(MAVLink.MAVLINK_MSG_ID id)
        {
            this.id = id;
        }
    }
}

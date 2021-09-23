using System;
namespace AutopilotCommon
{
    public class SendCategory : Attribute
    {
        public MAVLink.MAV_DATA_STREAM type;

        public SendCategory(MAVLink.MAV_DATA_STREAM type)
        {
            this.type = type;
        }
    }
}

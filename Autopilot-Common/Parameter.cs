using System.Text;

namespace AutopilotCommon
{
    public class Parameter
    {
        public readonly int index;
        public string id;
        public float value;
        public MAVLink.MAV_PARAM_TYPE type;

        public Parameter(int index, string id, float value, MAVLink.MAV_PARAM_TYPE type)
        {
            this.id = id;
            this.value = value;
            this.type = type;
        }

        public byte[] GetIDBytes()
        {
            byte[] retVal = new byte[16];
            Encoding.UTF8.GetBytes(id).CopyTo(retVal, 0);
            return retVal;
        }
    }
}

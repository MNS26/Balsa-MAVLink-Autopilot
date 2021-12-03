using System;
namespace AutopilotConsole
{
    public class Message
    {
        public ushort[] channelsRaw = new ushort[14];
        public float[] channels = new float[14];
    }
}

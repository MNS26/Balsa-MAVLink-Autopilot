using System;
using System.Collections.Generic;
namespace AutopilotConsole
{
    public static class Decoder
    {
        public static Queue<Message> messages = new Queue<Message>();
        private static RingBuffer buffer = new RingBuffer();
        private static bool readingHeader = true;
        private static byte[] u32 = new byte[32];
        public static void Decode(byte[] bytes)
        {
            buffer.Write(bytes);

            u32[0] = 0x20;
            u32[1] = 0x40;
            while (buffer.Available > 1)
            {
                //Find header
                while (readingHeader && buffer.Available > 1)
                {
                    if (buffer.ReadByte() == u32[0] && buffer.ReadByte() == u32[1])
                    {
                        readingHeader = false;
                    }
                }

                //Read message
                if (!readingHeader && buffer.Available < 32)
                {
                    return;
                }


                buffer.Read(u32, 2, 30);
                ushort checksumCompute = 0xFFFF;
                for (int i = 0; i < 30; i++)
                {
                    checksumCompute -= u32[i];
                }
                ushort messageChecksum = BitConverter.ToUInt16(u32, 30);

                if (checksumCompute == messageChecksum)
                {
                    Message m = new Message();
                    for (int i = 0; i < 14; i++)
                    {
                        m.channelsRaw[i] = BitConverter.ToUInt16(u32, 2 + (i * 2));
                        m.channels[i] = -1f + (m.channelsRaw[i] - 500) / 1000f;
                    }
                    messages.Enqueue(m);
                }

                readingHeader = true;
            }
        }
    }
}

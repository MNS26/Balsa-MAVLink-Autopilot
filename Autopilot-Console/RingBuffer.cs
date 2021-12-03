using System;
using System.IO;
using System.Collections.Generic;
namespace AutopilotConsole
{
    public class RingBuffer
    {
        public byte[] buffer = new byte[128];
        int readPos;
        int writePos;

        public int Available
        {
            get
            {
                return (writePos - readPos) % buffer.Length;
            }
        }

        public void Write(byte[] bytes)
        {
            int firstWrite = buffer.Length - writePos;
            if (firstWrite > bytes.Length)
            {
                firstWrite = bytes.Length;
            }
            int secondWrite = bytes.Length - firstWrite;
            Array.Copy(bytes, 0, buffer, writePos, firstWrite);
            writePos += firstWrite;
            if (writePos == buffer.Length)
            {
                writePos = 0;
            }
            if (secondWrite > 0)
            {
                Array.Copy(bytes, 0, buffer, writePos, secondWrite);
                writePos = secondWrite;
            }
        }

        public void Read(byte[] dest, int offset, int length)
        {
            int firstRead = buffer.Length - readPos;
            if (firstRead > length)
            {
                firstRead = length;
            }
            int secondRead = length - firstRead;
            Array.Copy(buffer, readPos, dest, offset, firstRead);
            readPos += firstRead;
            if (readPos == buffer.Length)
            {
                readPos = 0;
            }
            if (secondRead > 0)
            {
                Array.Copy(buffer, readPos, dest, firstRead + offset, secondRead);
            }
        }

        public byte ReadByte()
        {
            byte returnValue = buffer[readPos];
            readPos++;
            if (readPos == buffer.Length)
            {
                readPos = 0;
            }
            return returnValue;
        }
    }
}

using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Threading;

namespace AutopilotCommon
{
    public class ClientObject
    {
        private NetworkHandler networkHandler;
        private AutoResetEvent sendEvent;
        private Action<ClientObject, MAVLink.MAVLINK_MSG_ID> requestMessage;
        public Action<string> Log;

        public TcpClient client;
        //Rates are stored in hertz
        public ConcurrentDictionary<MAVLink.MAVLINK_MSG_ID, float> requestedRates = new ConcurrentDictionary<MAVLink.MAVLINK_MSG_ID, float>();
        //Send time stored in ticks
        public ConcurrentDictionary<MAVLink.MAVLINK_MSG_ID, long> nextSendTime = new ConcurrentDictionary<MAVLink.MAVLINK_MSG_ID, long>();
        public ConcurrentQueue<byte[]> outgoingMessages = new ConcurrentQueue<byte[]>();

        public byte[] buffer = new byte[280];
        public bool readingHeader = true;
        public int readPos = 0;
        public int bytesLeft = 8;

        //Mavlink
        private Dictionary<Type, MAVLink.MAVLINK_MSG_ID> mavTypes;
        private MAVLink.MavlinkParse parser = new MAVLink.MavlinkParse();
        private int sendSequence = 0;

        public ClientObject(NetworkHandler networkHandler, AutoResetEvent sendEvent, Action<string> Log, Action<ClientObject, MAVLink.MAVLINK_MSG_ID> requestMessage)
        {
            this.networkHandler = networkHandler;
            this.sendEvent = sendEvent;
            this.Log = Log;
            this.requestMessage = requestMessage;
        }

        public void Update()
        {
            long currentTime = DateTime.UtcNow.Ticks;
            foreach (KeyValuePair<MAVLink.MAVLINK_MSG_ID, float> kvp in requestedRates)
            {
                bool sendType = false;
                if (nextSendTime.TryGetValue(kvp.Key, out long nextTime))
                {
                    if (currentTime > nextTime)
                    {
                        sendType = true;
                        nextSendTime[kvp.Key] = currentTime + (long)(kvp.Value * TimeSpan.TicksPerSecond);
                    }
                }
                else
                {
                    sendType = true;
                    nextSendTime[kvp.Key] = currentTime + (long)(kvp.Value * TimeSpan.TicksPerSecond);
                }
                if (sendType)
                {
                    requestMessage(this, kvp.Key);
                }
            }
        }

        public void SendMessage(object message)
        {
            //Log($"TX: {message}");
            SendMessageBytes(parser.GenerateMAVLinkPacket20(GetTypeMapping(message), message, false, 1, 1, sendSequence++));
        }

        public void SendMessageBytes(byte[] message)
        {
            outgoingMessages.Enqueue(message);
            sendEvent.Set();
        }

        private MAVLink.MAVLINK_MSG_ID GetTypeMapping(object message)
        {
            if (mavTypes == null)
            {
                mavTypes = new Dictionary<Type, MAVLink.MAVLINK_MSG_ID>();
                foreach (MAVLink.message_info mi in MAVLink.MAVLINK_MESSAGE_INFOS)
                {
                    mavTypes.Add(mi.type, (MAVLink.MAVLINK_MSG_ID)mi.msgid);
                }
            }
            return mavTypes[message.GetType()];
        }
    }
}

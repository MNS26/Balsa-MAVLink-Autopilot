﻿using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Threading;

namespace AutopilotCommon
{
    public class NetworkHandler
    {
        private Action<string> Log;
        private Commands commands;
        private MessagesReceived messagesReceived;
        private MessagesSend messagesSend;

        //Mavlink
        private MAVLink.MavlinkParse parser = new MAVLink.MavlinkParse();

        //Network components
        private ConcurrentBag<ClientObject> clients = new ConcurrentBag<ClientObject>();
        private Thread sendThread;
        private Thread receiveThread;
        private AutoResetEvent sendEvent = new AutoResetEvent(false);
        private TcpListener listener;

        //Callbacks
        private Action<ClientObject> connectCallback;
        private Dictionary<MAVLink.MAVLINK_MSG_ID, Action<ClientObject, MAVLink.MAVLinkMessage>> receiveCallbacks = new Dictionary<MAVLink.MAVLINK_MSG_ID, Action<ClientObject, MAVLink.MAVLinkMessage>>();
        private Dictionary<MAVLink.MAV_CMD, Action<ClientObject, MAVLink.mavlink_command_long_t>> receiveCommandCallbacks = new Dictionary<MAVLink.MAV_CMD, Action<ClientObject, MAVLink.mavlink_command_long_t>>();
        private Dictionary<MAVLink.MAVLINK_MSG_ID, Action<ClientObject>> sendCallbacks = new Dictionary<MAVLink.MAVLINK_MSG_ID, Action<ClientObject>>();
        private Action<ClientObject> disconnectCallback;

        //Unprocessed callbacks
        private Action<ClientObject, MAVLink.MAVLinkMessage> unprocessedMessageCallback;
        private Action<ClientObject, MAVLink.mavlink_command_long_t> unprocessedCommandCallback;

        //Generator
        private Dictionary<Type, MAVLink.MAVLINK_MSG_ID> typeMapping = new Dictionary<Type, MAVLink.MAVLINK_MSG_ID>();

        private bool running = true;
        public NetworkHandler(Commands commands, MessagesReceived messagesReceived, MessagesSend messagesSend, Action<string> Log)
        {
            this.commands = commands;
            this.messagesReceived = messagesReceived;
            this.messagesSend = messagesSend;
            this.Log = Log;
            RegisterConnect(commands.ConnectEvent);
            RegisterConnect(messagesReceived.ConnectEvent);
            RegisterDisconnect(commands.DisconnectEvent);
            RegisterDisconnect(messagesReceived.DisconnectEvent);
            AutoRegister();
        }

        private void AutoRegister()
        {
            Type TCommands = typeof(Commands);
            Type TMessagesReceived = typeof(MessagesReceived);
            Type TMessagesSend = typeof(MessagesSend);
            MethodInfo[] MethodInfoCommands = TCommands.GetMethods();
            MethodInfo[] MethodInfoMessageReceive = TMessagesReceived.GetMethods();
            MethodInfo[] MethodInfoMessageSend = TMessagesSend.GetMethods();
            foreach (MethodInfo mi in MethodInfoMessageReceive)
            {
                foreach (Attribute att in mi.GetCustomAttributes())
                {
                    if (att is ReceiveMessage)
                    {
                        ReceiveMessage rm = (ReceiveMessage)att;
                        Action<ClientObject, MAVLink.MAVLinkMessage> callMethod = (Action<ClientObject, MAVLink.MAVLinkMessage>)mi.CreateDelegate(typeof(Action<ClientObject, MAVLink.MAVLinkMessage>), messagesReceived);
                        RegisterReceive(rm.id, callMethod);
                    }
                }
            }
            foreach (MethodInfo mi in MethodInfoMessageSend)
            {
                foreach (Attribute att in mi.GetCustomAttributes())
                {
                    if (att is SendMessage)
                    {
                        SendMessage sm = (SendMessage)att;
                        Action<ClientObject> callMethod = (Action<ClientObject>)mi.CreateDelegate(typeof(Action<ClientObject>), messagesSend);
                        RegisterSend(sm.id, callMethod);
                    }
                }
            }
            foreach (MethodInfo mi in MethodInfoCommands)
            {
                foreach (Attribute att in mi.GetCustomAttributes())
                {
                    if (att is ReceiveCommand)
                    {
                        ReceiveCommand rc = (ReceiveCommand)att;
                        Action<ClientObject, MAVLink.mavlink_command_long_t> callMethod = (Action<ClientObject, MAVLink.mavlink_command_long_t>)mi.CreateDelegate(typeof(Action<ClientObject, MAVLink.mavlink_command_long_t>), commands);
                        RegisterReceiveCommand(rc.id, callMethod);
                    }
                }
            }
        }
//start server thread
        public void StartServer()
        {
            listener = new TcpListener(new IPEndPoint(IPAddress.Any, 5760));
            listener.Start();
            listener.BeginAcceptTcpClient(HandleConnect, null);
            Start();
        }

        private void Start()
        {
            receiveThread = new Thread(new ThreadStart(ReceiveMain));
            receiveThread.Name = "Receive Thread";
            receiveThread.Start();
            sendThread = new Thread(new ThreadStart(SendMain));
            sendThread.Name = "Receive Thread";
            sendThread.Start();
        }

        public void Stop()
        {
            running = false;
            if (listener != null)
            {
                listener.Stop();
            }
            sendThread.Join();
            receiveThread.Join();
        }

        public void RegisterConnect(Action<ClientObject> callback)
        {
            connectCallback = callback;
        }

        public void RegisterDisconnect(Action<ClientObject> callback)
        {
            disconnectCallback = callback;
        }

        public void RegisterUnprocessedMessage(Action<ClientObject, MAVLink.MAVLinkMessage> callback)
        {
            unprocessedMessageCallback = callback;
        }

        public void RegisterUnprocessedCommand(Action<ClientObject, MAVLink.mavlink_command_long_t> callback)
        {
            unprocessedCommandCallback = callback;
        }

        public void RegisterReceive(MAVLink.MAVLINK_MSG_ID messageType, Action<ClientObject, MAVLink.MAVLinkMessage> callback)
        {
            receiveCallbacks[messageType] = callback;
        }

        public void RegisterReceiveCommand(MAVLink.MAV_CMD commandType, Action<ClientObject, MAVLink.mavlink_command_long_t> callback)
        {
            receiveCommandCallbacks[commandType] = callback;
        }

        public void RegisterSend(MAVLink.MAVLINK_MSG_ID messageType, Action<ClientObject> callback)
        {
            sendCallbacks[messageType] = callback;
        }


        private void RequestMessage(ClientObject client, MAVLink.MAVLINK_MSG_ID messageType)
        {
            if (sendCallbacks.ContainsKey(messageType))
            {
                sendCallbacks[messageType](client);
            }
        }

        private void HandleConnect(IAsyncResult ar)
        {
            try
            {
                ClientObject client = new ClientObject(this, sendEvent, Log, RequestMessage);
                client.client = listener.EndAcceptTcpClient(ar);
                client.requestedRates[MAVLink.MAVLINK_MSG_ID.HEARTBEAT] = 1f;
                clients.Add(client);
                if (connectCallback != null)
                {
                    connectCallback(client);
                }
            }
            catch (ObjectDisposedException ode)
            {
                if (running)
                {
                    Log($"Failed to accept client: {ode.Message}");
                }
            }
            catch (SocketException se)
            {
                if (running)
                {
                    Log($"Failed to accept client: {se.Message}");
                }
            }
            if (running)
            {
                listener.BeginAcceptTcpClient(HandleConnect, null);
            }
        }

        private void HandleDisconnect(ClientObject client)
        {
            lock (client)
            {
                if (client.client != null)
                {
                    client.client = null;
                    if (disconnectCallback != null)
                    {
                        disconnectCallback(client);
                    }
                }
            }
        }

        private void ReceiveMain()
        {
            while (running)
            {
                foreach (ClientObject client in clients)
                {
                    NetworkStream ns;
                    try
                    {
                        ns = client.client.GetStream();
                    }
                    catch
                    {
                        HandleDisconnect(client);
                        RebuildClients();
                        continue;
                    }
                    if (ns.DataAvailable)
                    {
                        int bytesRead = 0;
                        try
                        {
                            bytesRead = ns.Read(client.buffer, client.readPos, client.bytesLeft);
                        }
                        catch
                        {
                            HandleDisconnect(client);
                            RebuildClients();
                            continue;
                        }
                        client.readPos += bytesRead;
                        client.bytesLeft -= bytesRead;
                        if (bytesRead == 0)
                        {
                            HandleDisconnect(client);
                            RebuildClients();
                        }
                        if (client.bytesLeft == 0)
                        {
                            if (client.readingHeader)
                            {
                                //Process 0 byte messages
                                client.bytesLeft = client.buffer[1];
                                if (client.buffer[0] != 0xFE && client.buffer[0] != 0xFD)
                                {
                                    throw new IndexOutOfRangeException("Mavlink messages start with 0xFE or 0xFD");
                                }
                                //This is a mavlink2 message, we need an extra 4 bytes for the header
                                if (client.buffer[0] == 0xFD)
                                {
                                    client.bytesLeft += 4;
                                    //Check for signature in mavlink 2
                                    if ((client.buffer[2] & MAVLink.MAVLINK_IFLAG_SIGNED) == MAVLink.MAVLINK_IFLAG_SIGNED)
                                    {
                                        client.bytesLeft += 13;
                                    }
                                }

                                if (client.bytesLeft == 0)
                                {
                                    MAVLink.MAVLinkMessage message = new MAVLink.MAVLinkMessage(client.buffer);
                                    ProcessMessage(client, message);
                                    client.readingHeader = true;
                                    client.readPos = 0;
                                    client.bytesLeft = 8;
                                }
                                else
                                {
                                    client.readingHeader = false;
                                }
                            }
                            else
                            {
                                //Process messages with payloads
                                MAVLink.MAVLinkMessage message = new MAVLink.MAVLinkMessage(client.buffer);
                                ProcessMessage(client, message);
                                client.readingHeader = true;
                                client.readPos = 0;
                                client.bytesLeft = 8;
                            }
                        }
                    }
                }
                Thread.Sleep(1);
            }
        }

        private void ProcessMessage(ClientObject client, MAVLink.MAVLinkMessage message)
        {
            bool processed = false;
            //Log($"RX: {message.data} {message.msgid}");
            if (message.msgid == (uint)MAVLink.MAVLINK_MSG_ID.COMMAND_LONG)
            {
                processed = true;
                MAVLink.mavlink_command_long_t command = (MAVLink.mavlink_command_long_t)message.data;
                if (receiveCommandCallbacks.ContainsKey((MAVLink.MAV_CMD)command.command))
                {
                    receiveCommandCallbacks[(MAVLink.MAV_CMD)command.command](client, command);
                }
                else
                {
                    if (unprocessedCommandCallback != null)
                    {
                        unprocessedCommandCallback(client, command);
                    }
                }
            }
            if (!processed)
            {
                if (receiveCallbacks.ContainsKey((MAVLink.MAVLINK_MSG_ID)message.msgid))
                {
                    processed = true;
                    receiveCallbacks[(MAVLink.MAVLINK_MSG_ID)message.msgid](client, message);
                }
            }
            if (!processed)
            {
                if (unprocessedMessageCallback != null)
                {
                    unprocessedMessageCallback(client, message);
                }
            }
        }

        private void RebuildClients()
        {
            ConcurrentBag<ClientObject> newClients = new ConcurrentBag<ClientObject>();
            foreach (ClientObject client in clients)
            {
                if (client.client != null)
                {
                    newClients.Add(client);
                }
            }
            clients = newClients;
        }

        private void SendMain()
        {
            while (running)
            {
                sendEvent.WaitOne(50);
                foreach (ClientObject client in clients)
                {
                    if (!client.client.Connected)
                    {
                        HandleDisconnect(client);
                        RebuildClients();
                    }
                    else
                    {
                        while (client.outgoingMessages.TryDequeue(out byte[] sendMessage))
                        {
                            try
                            {
                                client.client.GetStream().Write(sendMessage, 0, sendMessage.Length);
                            }
                            catch
                            {
                                HandleDisconnect(client);
                                RebuildClients();
                            }
                        }
                    }
                    client.Update();
                }
            }
        }
    }
}
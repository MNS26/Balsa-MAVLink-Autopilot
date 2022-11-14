using System;
using System.Threading;
using UnityEngine;
using HID;
using AutopilotCommon;



namespace AutopilotConsole
{
    class MainClass
    {
        public static void Main(string[] args)
        {
            Thread.CurrentThread.Name = "Main";
            DataStore data;
            ApStore ap;
            ParameterHandler parameters;
            ProtocolLogic protocol;
            NetworkHandler handler;
            if(args.Length > 0){
                if(args[0] == "HID"){
                    Interface HIDinterface;
                    HIDinterface = new Interface(Console.WriteLine);
                    HIDinterface.StartServer();
                }
            }
            Console.WriteLine("Start!");
            data = new DataStore();
            ap = new ApStore();           
            data.mode=0;         
            parameters = new ParameterHandler("", "Parameters.txt", nothing);
            protocol = new ProtocolLogic(data, ap, nothing, parameters);
            handler = new NetworkHandler(protocol, nothing);
            //handler.RegisterUnprocessedCommand(UnprocessedCommand);
            //handler.RegisterUnprocessedMessage(UnprocessedMessage);
            handler.StartServer();
            bool running = true;
            int count = 0;
            long last_loop = 0;
            while (running)
            {
                while((DateTime.UtcNow.Ticks - last_loop)/TimeSpan.TicksPerMillisecond >100)
                {
                    last_loop = DateTime.UtcNow.Ticks;
                    try{
                    count++;
                    }
                    catch
                    {
                        count = 0;
                    }
                }
                data.heading = count % 360;
                data.radyaw = (float)((count % 360 / 360d) * 2 * Math.PI);
                data.magx = Vector3.Dot(new Vector3(0, 0, 1), new Vector3(0, 0, 1)) * 500;
                data.magy = Vector3.Dot(new Vector3(0, 0, 1), new Vector3(1, 0, 0)) * 500;
                data.magz = Vector3.Dot(new Vector3(0, 0, 1), new Vector3(0, 1, 0)) * 500;
                data.accx = 0;
                data.accy = 1000;
                data.accz = 0;
                data.gyrox = 1;
                data.gyroy = 2;
                data.gyroz = 3;
            }
        }
        public static void nothing(string text)
        {}

        static void UnprocessedMessage(ClientObject client, MAVLink.MAVLinkMessage message)
        {
            Console.WriteLine($"Unprocessed message: {message.msgid} , {message.data}");
        }
        static void UnprocessedCommand(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            Console.WriteLine($"Unprocessed command: {command.command} , {command.confirmation}");
        }
    }
}

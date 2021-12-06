using AutopilotCommon;
using System;
using UnityEngine;

namespace AutopilotConsole
{
    class MainClass
    {
        public static void Main(string[] args)
        {
            DataStore data;
            ApStore ap;
            ParameterHandler parameters;
            ProtocolLogic protocol;
            NetworkHandler handler;
            Console.WriteLine("Start!");
            data = new DataStore();
            ap = new ApStore();

            parameters = new ParameterHandler("/../../../Autopilot/bin/debug/", "Parameters.txt", Console.WriteLine);

            protocol = new ProtocolLogic(data, ap, Console.WriteLine, parameters);
            handler = new NetworkHandler(protocol, Console.WriteLine);
            handler.RegisterUnprocessedCommand(UnprocessedCommand);
            handler.RegisterUnprocessedMessage(UnprocessedMessage);
            handler.StartServer();
            bool running = true;
            int count = 0;
            while (running)
            {
                count++;
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

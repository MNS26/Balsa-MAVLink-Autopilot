using System;
using System.Threading;
using AutopilotCommon;
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
            parameters = new ParameterHandler("../../../Autopilot/bin/Debug/", "Parameters.dat", "Defaults.dat");

            protocol = new ProtocolLogic(data, ap,Console.WriteLine, parameters);
            handler = new NetworkHandler(protocol, Console.WriteLine);
            handler.StartServer();
            bool running = true;
            int count = 0;
            while (running)
            {
                count++;
                Thread.Sleep(1000);
                data.heading = count % 360;
                data.radyaw = (float)((count % 360 / 360d) * 2 * Math.PI);
                data.magx = Vector3.Dot(new Vector3(0, 0, 1), new Vector3(0, 0, 1)) * 500;

                data.magy = Vector3.Dot(new Vector3(0, 0, 1), new Vector3(1, 0, 0)) * 500;
                data.magz = Vector3.Dot(new Vector3(0, 0, 1), new Vector3(0, 1, 0)) * 500;
            }
        }
    }
}

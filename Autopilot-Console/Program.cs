using AutopilotCommon;
using System;
using System.IO;
using System.Reflection;
using UnityEngine;
using System.Threading;

namespace AutopilotConsole
{
    class MainClass
    {
        static string CurrentPath = Directory.GetCurrentDirectory();
        public static void Main(string[] args)
        {

            Thread.CurrentThread.Name = "Main";
            DataStore data;
            ApStore ap;
            ParameterHandler parameters;
            ProtocolLogic protocol;
            NetworkHandler handler;
            Console.WriteLine("Start!");
            data = new DataStore();
            ap = new ApStore();
            bool useTelemetry = true;

            parameters = new ParameterHandler(CurrentPath+"/../../../../Autopilot/bin//Debug/net472/", "Parameters.txt", Console.WriteLine);

            protocol = new ProtocolLogic(data, ap, Console.WriteLine, parameters);
            handler = new NetworkHandler(protocol, Console.WriteLine);
            handler.RegisterUnprocessedCommand(UnprocessedCommand);
            handler.RegisterUnprocessedMessage(UnprocessedMessage);
            handler.StartServer();
            bool running = true;
            int count = 0;
            InitReflections();
            while (running)
            {
                try{
                count++;
                }
                catch
                {
                    count = 0;
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
                Thread.Sleep(100);
            }
        }

        public static void InitReflections()
		{
			object[] sensorArray = null;
            Type sensorType = null;
            Type sensorEnumType = null;
            foreach (Assembly ass in AppDomain.CurrentDomain.GetAssemblies())
            {
                if (ass.GetName().Name == "WirelessRXMain")
                {
					object instance = ass.GetType("WirelessRXMain.WirelessRXMain").GetProperty("Instance").GetGetMethod().Invoke(null, null);
                    sensorArray = ass.GetType("WirelessRXMain.WirelessRXMain")?.GetProperty("Sensors")?.GetGetMethod()?.Invoke(instance, null) as object[];
                }
                if (ass.GetName().Name == "WirelessRXLib")
                {
                    sensorType = ass.GetType("WirelessRXLib.Sensor");
                    sensorEnumType = ass.GetType("WirelessRXLib.SensorType");
                }
            }
            if (sensorType == null || sensorEnumType == null || sensorArray == null)
            {
                //useTelemetry = false;
                return;
            }
			object sensorCell = sensorEnumType.GetField("CELL").GetValue(null);
            //This compile warning is incorrect, we actually want to pass the method, not call it and pass its value.
            //Warning goes away if cast to delegate
            object newSensor = Activator.CreateInstance(sensorType, new object[] { sensorCell, (Delegate)GetVoltage });
            sensorArray[2] = newSensor;        
		}
        private static int GetVoltage()
        {
            return 390;
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

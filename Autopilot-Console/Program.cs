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
            Data data;
            Ap ap;
            ParameterHandler parameters;
            NetworkHandler handler;

            Commands commands;
            Messages messages;

            Console.WriteLine("Start!");
            data = new Data();
            ap = new Ap();
            parameters  = new ParameterHandler("", "Parameters.txt", nothing);

            commands    = new Commands(data, ap, Console.WriteLine, parameters);
            messages    = new Messages(data, ap, Console.WriteLine, parameters);

            handler     = new NetworkHandler(commands, messages, Console.WriteLine);

            handler.RegisterUnprocessedCommand(UnprocessedCommand);
            handler.RegisterUnprocessedMessage(UnprocessedMessage);
            
            //hid.StartServer();
            handler.StartServer();
            
            bool running = true;
            int count = 0;
            long last_loop = 0;
            for( var index = 0; index<data.ChannelsServo.Length;index++)
            {
                data.ChannelsServo[index] = ushort.MaxValue;
            }
            while (running)
            {
                //for( var index = 0; index<hid.HIDinput.Length;index++)
                //{
                //    if(index<data.ChannelsRC.Length)//in case we somehow have more than 32 in total from HIDInput
                //    data.ChannelsRC[index] = map<double>(hid.HIDinput[index], 0, 2047, -1, 1);
                //}
                while((DateTime.UtcNow.Ticks - last_loop) / TimeSpan.TicksPerMillisecond >= 100)
                {
                    last_loop = DateTime.UtcNow.Ticks;
                    //for( var index = 0; index<hid.HIDinput.Length;index++)
                    //{
                    //    Console.Write(data.ChannelsRC[index]+ " ");
                    //}
                    //Console.WriteLine();

                    //try{
                    //count++;
                    //}
                    //catch
                    //{
                    //    count = 0;
                    //}

                    data.heading = count++ % 360;
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
        }

        //keep this so we can replace the Console.WriteLine with this
        public static void nothing(string text)
        {}

        static void UnprocessedMessage(ClientObject client, MAVLink.MAVLinkMessage message)
        {
            Console.WriteLine($"Unprocessed message: #{message.msgid} ({message.data})");
        }
        static void UnprocessedCommand(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            Console.WriteLine($"Unprocessed command: {command.command} ({command.confirmation})");
        }
        private static T map<T>(T value, T fromLow, T fromHigh, T toLow, T toHigh) //where T : IComparable<T>
        {
            return (T)(((dynamic)value - (dynamic)fromLow) * ((dynamic)toHigh - (dynamic)toLow) / ((dynamic)fromHigh - (dynamic)fromLow) + (dynamic)toLow);
        }
    }
}

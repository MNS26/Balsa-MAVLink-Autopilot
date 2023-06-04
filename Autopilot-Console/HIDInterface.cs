using System;
using System.Diagnostics;
using System.Linq;
using System.Text.RegularExpressions;
using System.Threading;
using HidSharp;
using HidSharp.Experimental;
using HidSharp.Reports;
using HidSharp.Reports.Encodings;
namespace HID
{
    public class Interface
    {
        private Action<string> Log;
        private Thread HIDThread;
        private int AxisCount;
        private int ButtonCount;
        public double[] HIDinput{
            private set;
            get; 
            } = new double[32];
        //Report Reports;
        public Interface(Action<string> Log)
        {
            this.Log = Log;
        }
        private void WriteDeviceItemInputParserResult(HidSharp.Reports.Input.DeviceItemInputParser parser,int Buttons,int Axis,bool AxisFirst)
        {
            if (parser.HasChanged)
            {
                int valueCount = parser.ValueCount;
                for (int valueIndex = 0; valueIndex < valueCount; valueIndex++)
                {
                    var dataValue = parser.GetValue(valueIndex);
                    if(AxisFirst != true)
                    {
                        if(valueIndex<Buttons)
                        {
                            HIDinput[valueIndex+Axis]= dataValue.GetPhysicalValue()*2047; // Buttons
                        }
                        else if(valueIndex-Buttons<Axis)
                        {
                            HIDinput[valueIndex-Buttons] = Convert.ToSingle(dataValue.GetPhysicalValue()); // Axis
                        }
                    }
                    else
                    {
                        if(valueIndex<Axis)
                        {
                            HIDinput[valueIndex]= Convert.ToSingle(dataValue.GetPhysicalValue()); // Axit
                        }
                        else if(valueIndex-Axis<Buttons)
                        {
                            HIDinput[valueIndex+Axis] = dataValue.GetPhysicalValue()*2047; // Buttons
                        }
                    }
                    Log(string.Format("  {0}: {1}",
                                    (Usage)dataValue.Usages.FirstOrDefault(), dataValue.GetPhysicalValue()));
                }
            }
        }
        public void StartServer()
        {
            Start();
        }
        private void Start()
        {
            HIDThread = new Thread(new ThreadStart(HIDMain));
            HIDThread.Name = "HID Thread";
            HIDThread.Start();
        }
        private void HIDMain()
        {
            var list = DeviceList.Local;
            list.Changed += (sender, e) => Log("Device list changed.");

            //all devices
            var allDeviceList = list.GetAllDevices().ToArray();
            Log("Full device list:");
            foreach (HidDevice dev in allDeviceList)
            {       
                int start = dev.ToString().IndexOf(".")+1;
                int end = dev.ToString().Length - start;
                Log(dev.ToString().Remove(start, end) + " " + dev.GetFriendlyName() + " @" + dev.DevicePath);
            }
            Log("");
            //list
            var stopwatch = Stopwatch.StartNew();
            var hidDeviceList = list.GetHidDevices().ToArray();
            Log($"Complete device list (took {stopwatch.ElapsedMilliseconds} ms to get {hidDeviceList.Length} devices):");
            foreach (HidDevice dev in hidDeviceList)
            {
                if(dev.GetManufacturer() != "OpenTX")
                break;
                //if(dev.GetFileSystemName() == "/dev/hidraw1")
                {
                Log("");
                Log(dev.DevicePath);
                //Console.WriteLine(string.Join(",", dev.GetDevicePathHierarchy())); // TODO
                Log($"{dev.GetFriendlyName()} {dev.GetProductName()} {dev.GetManufacturer()} {dev.GetFileSystemName()}");
                Log(dev.ToString());

                try
                {
                    Log($"Max Lengths: Input {dev.GetMaxInputReportLength()}, Output {dev.GetMaxOutputReportLength()}, Feature {dev.GetMaxFeatureReportLength()}");
                }
                catch (UnauthorizedAccessException e)
                {
                    Log(e.ToString());
                    Log("");
                    continue;
                }
                try
                {
                   //int ButtonCount;
                   //int AxisCount;
                    bool isButtons = false;
                    byte[] rawReportDescriptor = dev.GetRawReportDescriptor();
                    Log("Report Descriptor:");
                    Log($"  {string.Join(" ", rawReportDescriptor.Select(d => d.ToString("X2")))} ({rawReportDescriptor.Length} bytes)");
                    Regex re = new Regex(@"\d+");
                    int indent = 0;
                    //loop through report descriptor and list them
                    foreach (var element in EncodedItem.DecodeItems(rawReportDescriptor, 0, rawReportDescriptor.Length))
                    {
                        if (element.ItemType == ItemType.Main && element.TagForMain == MainItemTag.EndCollection) { indent -= 2; }

                        Log($"  {new string(' ', indent)}{element}");
                        
                        if((int)element.TagForGlobal == 0)//if usage page
                        {
                            Match m = re.Match($"{element}");
                            if(m.Value == "9") //if usage page is buttons
                            {
                                isButtons = true;
                            }
                            else if(m.Value == "1")//if usage page is Axis
                            {
                                isButtons = false;
                            }
                        }
                        if((int)element.TagForGlobal == 9)//if report count
                        {
                            Match m = re.Match($"{element}");
                            if(isButtons)
                            {
                                ButtonCount = int.Parse(m.Value);
                            }
                            else
                            {
                                AxisCount = int.Parse(m.Value);
                            }
                        }
                        if (element.ItemType == ItemType.Main && element.TagForMain == MainItemTag.Collection) { indent += 2; }
                    }
                    Log($"Axis count: {AxisCount} Button count: {ButtonCount}  Buttons first: {isButtons}");
                    var reportDescriptor = dev.GetReportDescriptor();

                    // Lengths should match.
                    Debug.Assert(dev.GetMaxInputReportLength() == reportDescriptor.MaxInputReportLength);
                    Debug.Assert(dev.GetMaxOutputReportLength() == reportDescriptor.MaxOutputReportLength);
                    Debug.Assert(dev.GetMaxFeatureReportLength() == reportDescriptor.MaxFeatureReportLength);

                    foreach (var deviceItem in reportDescriptor.DeviceItems)
                    {
                        foreach (var usage in deviceItem.Usages.GetAllValues())
                        {
                            Log(string.Format("Usage: {0:X4} {1}", usage, (Usage)usage));
                        }
                        foreach (var report in deviceItem.Reports)
                        {
                            Log(string.Format("{0}: ReportID={1}, Length={2}, Items={3}",
                                                report.ReportType, report.ReportID, report.Length, report.DataItems.Count));
                            /*foreach (var dataItem in report.DataItems)
                            {
                                Log(string.Format("  {0} Elements x {1} Bits, Units: {2}, Expected Usage Type: {3}, Flags: {4}, Usages: {5}",
                                    dataItem.ElementCount, dataItem.ElementBits, dataItem.Unit.System, dataItem.ExpectedUsageType, dataItem.Flags,
                                    string.Join(", ", dataItem.Usages.GetAllValues().Select(usage => usage.ToString("X4") + " " + ((Usage)usage).ToString()))));
                            }*/
                        }
                        Log("Opening device ");
                        HidStream hidStream;
                        if (dev.TryOpen(out hidStream))
                        {
                            Log("Opened device.");
                            hidStream.ReadTimeout = Timeout.Infinite;
                            using (hidStream)
                            {
                                var inputReportBuffer = new byte[dev.GetMaxInputReportLength()];
                                var inputReceiver = reportDescriptor.CreateHidDeviceInputReceiver();
                                var inputParser = deviceItem.CreateDeviceItemInputParser();
                                inputReceiver.Start(hidStream);
                                int startTime = Environment.TickCount;
                                while (true)
                                {
                                    //if (inputReceiver.WaitHandle.WaitOne(1000))
                                    {
                                        if (!inputReceiver.IsRunning) { break; } // Disconnected?
                                        Report report;
                                        while (inputReceiver.TryRead(inputReportBuffer, 0, out report))
                                        {
                                            // Parse the report if possible.
                                            // This will return false if (for example) the report applies to a different DeviceItem.
                                            if (inputParser.TryParseReport(inputReportBuffer, 0, report))
                                            {
                                                WriteDeviceItemInputParserResult(inputParser,ButtonCount,AxisCount,isButtons);
                                            }
                                        }
                                    }
                                    //uint elapsedTime = (uint)(Environment.TickCount - startTime);
                                    //if (elapsedTime >= 20000) { break; } // Stay open for 20 seconds.
                                }
                            }
                            Log("Closed device.");
                        }
                        else
                        {
                            Log("Failed to open device.");
                        }
                        Log("");
                    }
                    Log("");
                }
                catch (Exception e)
                {
                    Log(e.ToString());
                }
                }
            }
        }
    }
}

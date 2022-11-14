#define SHOW_CHANGES_ONLY

using System;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using HidSharp;
using HidSharp.Experimental;
using HidSharp.Reports;
using HidSharp.Reports.Encodings;
using HidSharp.Utility;
using Newtonsoft.Json.Converters;

namespace HID
{
    public class Interface
    {
        private Action<string> Log;
        private Thread HIDThread;
#pragma warning disable CS0169
        Report Reports;
#pragma warning disable CS0169 

        public Interface(Action<string> Log)
        {
            this.Log = Log;
        }

        void WriteDeviceItemInputParserResult(HidSharp.Reports.Input.DeviceItemInputParser parser)
        {
#if SHOW_CHANGES_ONLY
            while (parser.HasChanged)
            {
                int changedIndex = parser.GetNextChangedIndex();
                var previousDataValue = parser.GetPreviousValue(changedIndex);
                var dataValue = parser.GetValue(changedIndex);
                Log(string.Format("  {0}: {1} -> {2}",
                                  (Usage)dataValue.Usages.FirstOrDefault(), previousDataValue.GetPhysicalValue(), dataValue.GetPhysicalValue()));
            }
#else
            if (parser.HasChanged)
            {
                int valueCount = parser.ValueCount;

                for (int valueIndex = 0; valueIndex < valueCount; valueIndex++)
                {
                    var dataValue = parser.GetValue(valueIndex);
                    Log(string.Format("  {0}: {1}",
                                      (Usage)dataValue.Usages.FirstOrDefault(), dataValue.GetPhysicalValue()));

                }

                Console.WriteLine("");
            }
#endif
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
            var allDeviceList = list.GetAllDevices().ToArray();
            Log("All device list:");
            foreach (Device dev in allDeviceList)
            {
                Log(dev.ToString() + " @" + dev.DevicePath);
            }
            //Bluetooth
            var bleDeviceList = list.GetBleDevices().ToArray();
            Log("BLE device list:");
            foreach (BleDevice dev in bleDeviceList)
            {
                Log(dev.ToString() + "@" + dev.DevicePath);
                foreach (var service in dev.GetServices())
                {
                    Log(string.Format("\tService: {0}", service.Uuid));
                    foreach (var characteristic in service.GetCharacteristics())
                    {
                        Log(string.Format("\t\tCharacteristic: {0} (Properties: {1})", characteristic.Uuid, characteristic.Properties));
                        foreach (var descriptor in characteristic.GetDescriptors())
                        {
                            Log(string.Format("\t\t\tDescriptor: {0}", descriptor.Uuid));
                        }
                    }
                    if (service.Uuid == new BleUuid("63dc0001-fa35-4205-b09f-0fc6072ec515"))
                    {
                        try
                        {
                            using (var svc = dev.Open(service))
                            {
                                Log("Opened!");
                                BleCharacteristic rx = null;
                                foreach (var ch in service.GetCharacteristics())
                                {
                                    Log(string.Format("{0} = {1}", ch.Uuid, ch.IsReadable ? string.Join(" ", svc.ReadCharacteristic(ch)) : "N/A"));
                                    foreach (var d in ch.GetDescriptors())
                                    {
                                        Log(string.Format("\t{0} = {1}", d.Uuid, string.Join(" ", svc.ReadDescriptor(d))));
                                    }
                                    if (BleCccd.Notification != svc.ReadCccd(ch))
                                    {
                                        svc.WriteCccd(ch, BleCccd.Notification);
                                    }
                                    if (ch.Uuid == new BleUuid("63dc0002-fa35-4205-b09f-0fc6072ec515")) { rx = ch; }
                                }
                                Action beginReadEvent = null;
                                AsyncCallback endReadEvent = null;
                                beginReadEvent = () =>
                                    {
                                        svc.BeginReadEvent(endReadEvent, null);
                                    };
                                endReadEvent = ar =>
                                    {
                                        BleEvent @event;
                                        try
                                        {
                                            @event = svc.EndReadEvent(ar);
                                        }
                                        catch (ObjectDisposedException)
                                        {
                                            Log("closed");
                                            return;
                                        }
                                        catch (TimeoutException)
                                        {
                                            Log("timed out");
                                            @event = default(BleEvent);
                                        }
                                        if (@event.Value != null)
                                        {
                                            Log(string.Format("{0} -> {1}", @event.Characteristic, string.Join(" ", @event.Value.Select(x => x.ToString()))));
                                            if (rx != null)
                                            {
                                                Log("writing");
                                                svc.WriteCharacteristicWithoutResponse(rx, new[] { (byte)0xdd, (byte)1, (byte)'A' });
                                            }
                                        }
                                        beginReadEvent();
                                    };
                                beginReadEvent();
                                Thread.Sleep(30000);
                            }
                        }
                        catch (Exception e)
                        {
                            Log(e.ToString());
                        }
                    }
                }
            }

            //list
            var stopwatch = Stopwatch.StartNew();
            var hidDeviceList = list.GetHidDevices().ToArray();
            Log($"Complete device list (took {stopwatch.ElapsedMilliseconds} ms to get {hidDeviceList.Length} devices):");
            foreach (HidDevice dev in hidDeviceList)
            {
                if(dev.GetManufacturer() == "OpenTX")
                {
                Log("");
                Log(dev.DevicePath);
                //Console.WriteLine(string.Join(",", dev.GetDevicePathHierarchy())); // TODO
                Log(dev.ToString());

                try
                {
                    Log(string.Format("Max Lengths: Input {0}, Output {1}, Feature {2}",
                        dev.GetMaxInputReportLength(),
                        dev.GetMaxOutputReportLength(),
                        dev.GetMaxFeatureReportLength()));
                }
                catch (UnauthorizedAccessException e)
                {
                    Log(e.ToString());
                    Log("");
                    continue;
                }

                /*try
                {
                    Console.WriteLine($"Serial Ports: {string.Join(",", dev.GetSerialPorts())}");
                }
                catch
                {
                    Console.WriteLine("Serial Ports: Unknown on this platform.");
                }*/
                try
                {
                    var rawReportDescriptor = dev.GetRawReportDescriptor();
                    Log("Report Descriptor:");
                    Log($"  {string.Join(" ", rawReportDescriptor.Select(d => d.ToString("X2")))} ({rawReportDescriptor.Length} bytes)");

                    int indent = 0;
                    foreach (var element in EncodedItem.DecodeItems(rawReportDescriptor, 0, rawReportDescriptor.Length))
                    {
                        if (element.ItemType == ItemType.Main && element.TagForMain == MainItemTag.EndCollection) { indent -= 2; }

                        Log($"  {new string(' ', indent)}{element}");

                        if (element.ItemType == ItemType.Main && element.TagForMain == MainItemTag.Collection) { indent += 2; }
                    }

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
                            Console.WriteLine("Opened device.");
                            hidStream.ReadTimeout = Timeout.Infinite;
                            using (hidStream)
                            {
                                var inputReportBuffer = new byte[dev.GetMaxInputReportLength()];
                                var inputReceiver = reportDescriptor.CreateHidDeviceInputReceiver();
                                var inputParser = deviceItem.CreateDeviceItemInputParser();
                                IAsyncResult ar = null;
                                //int startTime = Environment.TickCount;
                                while (true)
                                {
                                    if (ar == null)
                                    {
                                        ar = hidStream.BeginRead(inputReportBuffer, 0, inputReportBuffer.Length, null, null);
                                    }
                                    if (ar != null)
                                    {
                                        if (ar.IsCompleted)
                                        {

                                            int byteCount = hidStream.EndRead(ar);
                                            ar = null;
                                            if (byteCount > 0)
                                            {
                                                string hexOfBytes = string.Join(" ", inputReportBuffer.Take(byteCount).Select(b => b.ToString("X2")));
                                                Log($"{hexOfBytes}");
                                            }
                                        }
                                        else
                                        {
                                            ar.AsyncWaitHandle.WaitOne(1000);
                                        }
                                    }
                                    //uint elapsedTime = (uint)(Environment.TickCount - startTime);
                                    //if (elapsedTime >= 20000) { break; } // Stay open for 20 seconds.
                                }
                                //11111111 00000111
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
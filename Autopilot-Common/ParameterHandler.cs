using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Reflection;

namespace AutopilotCommon
{
    public class ParameterHandler : IEnumerable<Parameter>
    {
        public event Action<Parameter> ParameterEvent;
        private bool saveEnabled = false;
        private string saveFile;
        private int freeID = 0;
        private List<Parameter> parameters = new List<Parameter>();
        private Dictionary<string, Parameter> index = new Dictionary<string, Parameter>();
        private Action<string> Log;

        public ParameterHandler(string dir, string saveFile, Action<string> Log)
        {
            this.Log = Log;
            this.saveFile = dir + saveFile;
            Console.WriteLine(dir+saveFile);
            Load();
        }

        public void Load()
        {
            lock (parameters)
            {
                Assembly ass = Assembly.GetExecutingAssembly();
                int defaultTotal = 0;
                int savedTotal = 0;
                using (Stream s = ass.GetManifestResourceStream("AutopilotCommon.Defaults.txt"))
                {
                    defaultTotal = LoadStream(s);
                }
                //DON'T LOAD IN DEBUG MODE, EDIT DEFAULTS INSTEAD
//#if !DEBUG
                if (File.Exists(saveFile))
                {
                    using (FileStream fs = new FileStream(saveFile, FileMode.Open))
                    {
                        savedTotal = LoadStream(fs);
                    }
                }
//#endif
                saveEnabled = true;
                //Save new/missing parameters.
                if (defaultTotal != savedTotal)
                {
                    Save(saveFile);
                }
            }
        }

        private int LoadStream(Stream s)
        {
            int loadTotal = 0;
            using (StreamReader sr = new StreamReader(s))
            {

                string currentLine = null;
                while ((currentLine = sr.ReadLine()) != null)
                {
                    loadTotal++;
                    int indexOfColon = currentLine.IndexOf(":", StringComparison.InvariantCulture);
                    int indexOfEquals = currentLine.IndexOf("=", StringComparison.InvariantCulture);
                    string typePart = currentLine.Substring(0, indexOfColon);
                    string idPart = currentLine.Substring(indexOfColon + 1, indexOfEquals - indexOfColon - 1);
                    string valuePart = currentLine.Substring(indexOfEquals + 1);
                    float valueFloat = float.Parse(valuePart);
                    MAVLink.MAV_PARAM_TYPE paramType = (MAVLink.MAV_PARAM_TYPE)Enum.Parse(typeof(MAVLink.MAV_PARAM_TYPE), typePart);
                    SetParameter(idPart, valueFloat, paramType);
                }
            }
            return loadTotal;
        }

        public void Save(string file)
        {
            lock (parameters)
            {
                using (StreamWriter sw = new StreamWriter(file))
                {
                    foreach (Parameter p in parameters)
                    {
                        sw.WriteLine($"{p.type}:{p.id}={p.value}");
                    }
                }
            }
        }

        public Parameter GetParameter(string id)
        {
            if (index.ContainsKey(id))
            {
                return index[id];
            }
            return null;
        }

        public int SetParameter(string id, float value, MAVLink.MAV_PARAM_TYPE type)
        {
            lock (parameters)
            {
                Parameter p = GetParameter(id);
                if (p != null)
                {
                    p.value = value;
                    p.type = type;
                }
                else
                {
                    p = new Parameter(freeID++, id, value, type);
                    //Log($"{p.id},{p.value},{p.type}");
                    parameters.Add(p);
                }
                index[id] = p;
                if (saveEnabled)
                {
                    Save(saveFile);
                }
                ParameterEvent?.Invoke(p);
                return parameters.IndexOf(p);
            }
        }

        public int GetCount()
        {
            return freeID;
        }

        public IEnumerator<Parameter> GetEnumerator()
        {
            return parameters.GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }
    }
}
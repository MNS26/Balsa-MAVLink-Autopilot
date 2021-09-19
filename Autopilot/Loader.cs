using System;
using UnityEngine;
using BalsaCore;

namespace Autopilot
{
    [BalsaAddon]
    public class Loader
    {
        private static bool loaded = false;
        private static GameObject go;
        private static MonoBehaviour mod;
        private static MonoBehaviour cam;

        [BalsaAddonFinalize]
        public static void BalsaInit()
        {

            if (!loaded)
            {
                loaded = true;
                go = new GameObject();
            }
            mod = go.AddComponent<Autopilot>();

        }

        [BalsaAddonInit(invokeTime = AddonInvokeTime.Flight)]
        public static void BalsaInitFlight()
        {
            //if(!loaded)
            //{
            //    loaded = true;
            //    go = new GameObject();
            //}
            //cam = go.AddComponent<FPVCam>();
        }

        [BalsaAddonFinalize(invokeTime = AddonInvokeTime.Flight)]
        public static void BalsaFinalizeFlight()
        {
            
        }
        //Game exit
        [BalsaAddonFinalize]
        public static void BalsaFinalize()
        {
            go.DestroyGameObject();   
        }
    }
}

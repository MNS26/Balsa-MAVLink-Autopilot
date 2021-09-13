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

        //main menu
        [BalsaAddonInit(invokeTime = AddonInvokeTime.MainMenu)]
        public static void BalsaInit()
        {
            if (!loaded)
            {
                loaded = true;
                go = new GameObject();
                mod = go.AddComponent<Autopilot>();
;
            }
        }

        //Game exit
        [BalsaAddonFinalize]
        public static void BalsaFinalize()
        {
            mod = go.AddComponent<Autopilot>();
        }
    }
}

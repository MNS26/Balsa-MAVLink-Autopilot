using BalsaCore;
using UnityEngine;

namespace KitBash_Autopilot
{
    [BalsaAddon]
    public class Loader
    {
        private static bool loaded = false;
        private static GameObject go;
        private static MonoBehaviour mod;
        [BalsaAddonInit(invokeTime = AddonInvokeTime.MainMenu)]
        public static void BalsaInit()
        {
            if (!loaded)
            {
                loaded = true;
                go = new GameObject();
            }
            mod = go.AddComponent<KitBash_Autopilot>();
        }

        [BalsaAddonInit(invokeTime = AddonInvokeTime.Flight)]
        public static void BalsaInitFlight()
        {
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

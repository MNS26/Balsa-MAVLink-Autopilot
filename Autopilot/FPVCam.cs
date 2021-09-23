using Modules;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Autopilot
{
    public class FPVCam : MonoBehaviour
    {
        private GameObject camera;
        public void Start()
        {            
        }

        public void Update()
        {
            if (!GameLogic.inGame || !GameLogic.SceneryLoaded || GameLogic.LocalPlayerVehicle == null || !GameLogic.LocalPlayerVehicle.InitComplete)
            {
                return;
            }


        }

        public void FuxedUpdate()
        {

        }
    }
}

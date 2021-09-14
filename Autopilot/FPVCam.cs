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
        public GameObject CameraPrefab;
        private GameObject camera;
        public void Start()
        {
            DontDestroyOnLoad(this);
            camera = (GameObject)GameObject.Instantiate(CameraPrefab);
            camera.transform.SetParent(transform);

            
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

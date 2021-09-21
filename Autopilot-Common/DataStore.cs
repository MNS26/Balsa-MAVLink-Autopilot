namespace AutopilotCommon
{
    public class DataStore
    {
        //plane
        public float pitch;
        public float roll;
        public float yaw;
        public float radpitch;
        public float radroll;
        public float radyaw;
        public float cr;
        public float iaspeed;
        public float rpm;
        public float avrrpm;
        public float battery;
        public float heading;
        public float altitude;
        public float dth;
        public int latitude;
        public int longitude;
        public string name;
        //IMU
        public float gyrox;
        public float gyroy;
        public float gyroz;
        public float accx;
        public float accy;
        public float accz;
        public float magx;
        public float magy;
        public float magz;
        //gyro velocity
        public float currentGVelx;
        public float lastGVelx;
        public float currentGVely;
        public float lastGVely;
        public float currentGVelz;
        public float lastGVelz;
        //accel velocity
        public float currentAVelx;
        public float lastAVelx;
        public float currentAVely;
        public float lastAVely;
        public float currentAVelz;
        public float lastAVelz;
        //controller
        public float rssi;
        public float ch1;  // roll
        public float ch2;  // pitch
        public float ch3;  // throttle
        public float ch4;  // yaw
        public float ch5;  // 
        public float ch6;  // 
        public float ch7;  // 
        public float ch8;  // 
        public float ch9;  // 
        public float ch10; // 
        public float ch11; // 
        public float ch12; // 
        public float ch13; // 
        public float ch14; // 
        public float ch15; // 
        public float ch16; // 

    }
public class ApStore
    {
        public short sysState;
        public short armed;
    }
}

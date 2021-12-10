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
        public float[] channels = new float[16]; //all game channels
        public ushort[] RCchannels = new ushort[16]; //all RC channels
        public float rssi;

        //serial stuff
        public bool serial = false;
    }
    public class ApStore
    {
        public MAVLink.MAV_MODE_FLAG mode;
        public MAVLink.MAV_STATE state;
        public float P;
        public float I;
        public float D;
    }
}

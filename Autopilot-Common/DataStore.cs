namespace AutopilotCommon
{
    public class DataStore
    {
        //plane
        public float pitch,roll, yaw;
        public float radpitch, radroll, radyaw;
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
        public ushort load;
        public string name;
        //IMU
        public float gyrox, gyroy, gyroz;
        public float accx, accy, accz;
        public float magx, magy, magz;
        //gyro velocity
        public float currentGVelx, lastGVelx;
        public float currentGVely, lastGVely;
        public float currentGVelz, lastGVelz;
        //accel velocity
        public float currentAVelx, lastAVelx;
        public float currentAVely, lastAVely;
        public float currentAVelz, lastAVelz;
        //controller
        public float[] channels = new float[16];
        public ushort[] APchannels = new ushort[16];
        public float rssi;

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

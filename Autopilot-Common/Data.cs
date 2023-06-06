namespace AutopilotCommon
{
    public class Data
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
        public int mode_flag;
        public byte mode;
        public int state;

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

        /****************/
        /*  CONTROLLER  */
        /****************/

        
        public double[] ChannelsRC = new double[32];

        public float[] ChannelsServo = new float[32];
        public float rssi;

    }
    public class Ap
    {
        public MAVLink.MAV_MODE_FLAG mode_flag = 0;
        public MAVLink.MAV_MODE mode = MAVLink.MAV_MODE.PREFLIGHT;
        public MAVLink.MAV_STATE state = MAVLink.MAV_STATE.STANDBY;
        public float P;
        public float I;
        public float D;
    }
    public enum FLIGHTMODES
    {
        MANUAL,
        CIRCLE,
        STABILIZE,
        TRAINING,
        ACRO,
        FBWA,
        FBWB,
        CRUISE,
        AUTOTUNE,
        RESERVED_9,
        AUTO,
        RTL,
        LOITER,
        RESERVED_13,
        AVOID_ADSB,
        GUIDED,
        RESERVED_16,
        QSTABILIZE,
        QHOVER,
        QLOITER,
        QLAND,
        QRTL,
        QAUTOTUNE,
        QACRO,
        THERMAL
    }
}

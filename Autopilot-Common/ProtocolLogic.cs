using System;
using System.Collections.Generic;

namespace AutopilotCommon
{
    public class ProtocolLogic
    {
        private static List<Parameter> parameters = new List<Parameter>();

        private long startTime;
        private const byte systemID = 1;
        private const byte componentID = 1;
        private static MAVLink.MavlinkParse parser = new MAVLink.MavlinkParse();
        private DataStore data;
        private Action<string> Log;

        /*
       FLIGHTMODES

       0 = MANUAL
       1 = CIRCLE
       2 = STABILIZE
       3 = TRAINING
       4 = ACRO
       5 = FBWA
       6 = FBWB
       7 = CRUISE
       8 = AUTOTUNE
       10 = AUTO
       11 = RTL
       12 = LOITER
       14 = AVOID_ADSB
       15 = GUIDED
       17 = QSTABILIZE
       18 = QHOVER
       19 = QLOITER
       20 = QLAND
       21 = QRTL
       22 = QAUTOTUNE
       23 = QACRO
       24 = THERMAL
        */

        public ProtocolLogic(DataStore data, Action<string> Log)
        {
            this.Log = Log;
            startTime = DateTime.UtcNow.Ticks;
            this.data = data;
            parameters.Add(new Parameter("RCMAP_ROLL", 1f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RCMAP_PITCH", 2f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RCMAP_YAW", 4f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RCMAP_THROTTLE", 3f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GLIDE_SLOPE_MIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GLIDE_SLOPE_THR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("STICK_MIXING", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TKOFF_THR_MINSPD", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TKOFF_THR_MINACC", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TKOFF_THR_DELAY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TKOFF_TDRAG_ELEV", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TKOFF_TDRAG_SPD1", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TKOFF_ROTATE_SPD", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TKOFF_THR_SLEW", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TKOFF_PLIM_SEC", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TKOFF_FLAP_PCNT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LEVEL_ROLL_LIMIT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("USE_REV_THRUST", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ALT_OFFSET", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("WP_RADIUS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("WP_MAX_RADIUS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("WP_LOITER_RAD", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RTL_RADIUS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("STALL_PREVENTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARSPD_FBW_MIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARSPD_FBW_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FBWB_ELEV_REV", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TERRAIN_FOLLOW", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TERRAIN_LOOKAHD", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FBWB_CLIMB_RATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("THR_MIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("THR_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TKOFF_THR_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("THR_SLEWRATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FLAP_SLEWRATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("THR_SUPP_MAN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("THR_PASS_STAB", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("THR_FAILSAFE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("THR_FS_VALUE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TRIM_THROTTLE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("THROTTLE_NUDGE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FS_SHORT_ACTN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FS_SHORT_TIMEOUT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FS_LONG_ACTN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FS_LONG_TIMEOUT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FS_GCS_ENABL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FLTMODE_CH:", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FLTMODE1", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FLTMODE2", 5f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FLTMODE3", 7f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FLTMODE4", 15f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FLTMODE5", 11f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FLTMODE6", 21f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INITIAL_MODE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LIM_ROLL_CD", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LIM_PITCH_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LIM_PITCH_MIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ACRO_ROLL_RATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ACRO_PITCH_RATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ACRO_LOCKING", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GROUND_STEER_ALT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GROUND_STEER_DPS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TRIM_AUTO", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("MIXING_GAIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RUDDER_ONLY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("MIXING_OFFSET", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("DSPOILR_RUD_RATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("SYS_NUM_RESETS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LOG_BITMASK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TRIM_ARSPD_CM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("SCALING_SPEED", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("MIN_GNDSPD_CM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TRIM_PITCH_CD", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ALT_HOLD_RTL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ALT_HOLD_FBWCM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FLAP_1_PERCNT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FLAP_1_SPEED", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FLAP_2_PERCNT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FLAP_2_SPEED", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("OVERRIDE_CHAN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RTL_AUTOLAND", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CRASH_ACC_THRESH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CRASH_DETECT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RNGFND_LANDING", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("SYSID_ENFORCE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RUDD_DT_GAIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("MANUAL_RCMASK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("HOME_RESET_ALT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FLIGHT_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TKOFF_ACCEL_CNT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("DSPOILER_CROW_W1", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("DSPOILER_CROW_W2", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TKOFF_TIMEOUT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("DSPOILER_OPTS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("DSPOILER_AILMTCH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FWD_BAT_VOLT_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FWD_BAT_VOLT_MIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FWD_BAT_IDX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FS_EKF_THRESH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RTL_CLIMB_MIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("MAN_EXPO_ROLL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("MAN_EXPO_PITCH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("MAN_EXPO_RUDDER", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ONESHOT_MASK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ADSB_TYPE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ADSB_LIST_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ADSB_LIST_RADIUS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ADSB_ICAO_ID", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ADSB_EMIT_TYPE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ADSB_LEN_WIDTH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ADSB_OFFSET_LAT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ADSB_OFFSET_LON", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ADSB_RF_SELECT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ADSB_SQUAWK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ADSB_RF_CAPABLE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ADSB_LIST_ALT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ADSB_ICAO_SPECL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ADSB_LOG", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AFS_ENABLE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AFS_MAN_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AFS_HB_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AFS_WP_COMMS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AFS_WP_GPS_LOSS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AFS_TERMINATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AFS_TERM_ACTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AFS_TERM_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AFS_AMSL_LIMIT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AFS_AMSL_ERR_GPS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AFS_QNH_PRESSURE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AFS_MAX_GPS_LOSS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AFS_MAX_COM_LOSS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AFS_GEOFENCE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AFS_RC", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AFS_RC_MAN_ONLY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AFS_DUAL_LOSS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AFS_RC_FAIL_TIME", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AHRS_GPS_GAIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AHRS_GPS_USE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AHRS_YAW_P", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AHRS_RP_P", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AHRS_WIND_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AHRS_WIND_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AHRS_TRIM_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AHRS_TRIM_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AHRS_ORIENTATION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AHRS_COMP_BETA", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AHRS_GPS_MINSATS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AHRS_EKF_TYPE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AHRS_CUSTOM_ROLL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AHRS_CUSTOM_PIT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AHRS_CUSTOM_YAW", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARMING_REQUIRE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARMING_ACCTHRESH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARMING_RUDDER", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARMING_MIS_ITEMS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARMING_CHECK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARSPD_TYPE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARSPD_DEVID", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARSPD_USE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARSPD_OFFSET", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARSPD_RATIO", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARSPD_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARSPD_AUTOCAL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARSPD_TUBE_ORDER", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARSPD_SKIP_CAL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARSPD_PSI_RANGE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARSPD_BUS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARSPD_PRIMARY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARSPD_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARSPD_WIND_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ARSPD_WIND_WARN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AVD_ENABLE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AVD_F_ACTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AVD_W_ACTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AVD_F_RCVRY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AVD_OBS_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AVD_W_TIME", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AVD_F_TIME", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AVD_W_DIST_XY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AVD_F_DIST_XY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AVD_W_DIST_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AVD_F_DIST_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("AVD_F_ALT_MIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_MONITOR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_CAPACITY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_WATT_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_SERIAL_NUM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_LOW_TIMER", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_FS_VOLTSRC", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_LOW_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_LOW_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_CRT_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_CRT_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_FS_LOW_ACT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_FS_CRT_ACT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_ARM_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_ARM_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_VOLT_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_CURR_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_VOLT_MULT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_AMP_PERVLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_AMP_OFFSET", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_I2C_BUS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_I2C_ADDR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_MONITOR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_CAPACITY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_WATT_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_SERIAL_NUM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_LOW_TIMER", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_FS_VOLTSRC", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_LOW_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_LOW_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_CRT_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_CRT_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_FS_LOW_ACT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_FS_CRT_ACT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_ARM_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_ARM_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_VOLT_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_CURR_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_VOLT_MULT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_AMP_PERVLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_AMP_OFFSET", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_I2C_BUS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_I2C_ADDR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_MONITOR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_CAPACITY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_WATT_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_SERIAL_NUM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_LOW_TIMER", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_FS_VOLTSRC", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_LOW_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_LOW_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_CRT_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_CRT_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_FS_LOW_ACT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_FS_CRT_ACT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_ARM_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_ARM_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_VOLT_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_CURR_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_VOLT_MULT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_AMP_PERVLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_AMP_OFFSET", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_I2C_BUS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT3_I2C_ADDR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_MONITOR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_CAPACITY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_WATT_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_SERIAL_NUM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_LOW_TIMER", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_FS_VOLTSRC", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_LOW_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_LOW_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_CRT_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_CRT_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_FS_LOW_ACT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_FS_CRT_ACT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_ARM_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_ARM_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_VOLT_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_CURR_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_VOLT_MULT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_AMP_PERVLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_AMP_OFFSET", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_I2C_BUS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT4_I2C_ADDR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_MONITOR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_CAPACITY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_WATT_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_SERIAL_NUM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_LOW_TIMER", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_FS_VOLTSRC", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_LOW_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_LOW_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_CRT_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_CRT_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_FS_LOW_ACT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_FS_CRT_ACT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_ARM_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_ARM_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_VOLT_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_CURR_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_VOLT_MULT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_AMP_PERVLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_AMP_OFFSET", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_I2C_BUS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_I2C_ADDR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_MONITOR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_CAPACITY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_WATT_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_SERIAL_NUM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_LOW_TIMER", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_FS_VOLTSRC", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_LOW_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_LOW_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_CRT_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_CRT_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_FS_LOW_ACT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_FS_CRT_ACT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_ARM_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_ARM_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_VOLT_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_CURR_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_VOLT_MULT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_AMP_PERVLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_AMP_OFFSET", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_I2C_BUS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT5_I2C_ADDR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_MONITOR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_CAPACITY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_WATT_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_SERIAL_NUM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_LOW_TIMER", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_FS_VOLTSRC", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_LOW_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_LOW_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_CRT_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_CRT_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_FS_LOW_ACT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_FS_CRT_ACT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_ARM_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_ARM_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_VOLT_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_CURR_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_VOLT_MULT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_AMP_PERVLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_AMP_OFFSET", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_I2C_BUS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT6_I2C_ADDR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT7_MONITOR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT7_CAPACITY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT7_WATT_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT7_SERIAL_NUM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT7_LOW_TIMER", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT7_FS_VOLTSRC", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT7_LOW_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT7_LOW_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT7_CRT_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT7_CRT_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT7_FS_LOW_ACT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT7_FS_CRT_ACT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT7_ARM_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT7_ARM_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT2_VOLT_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT72_CURR_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT7_VOLT_MULT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT7_AMP_PERVLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT7_AMP_OFFSET", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT7_I2C_BUS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT7_I2C_ADDR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_MONITOR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_CAPACITY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_WATT_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_SERIAL_NUM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_LOW_TIMER", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_FS_VOLTSRC", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_LOW_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_LOW_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_CRT_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_CRT_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_FS_LOW_ACT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_FS_CRT_ACT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_ARM_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_ARM_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_VOLT_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_CURR_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_VOLT_MULT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_AMP_PERVLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_AMP_OFFSET", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_I2C_BUS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT8_I2C_ADDR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_MONITOR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_CAPACITY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_WATT_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_SERIAL_NUM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_LOW_TIMER", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_FS_VOLTSRC", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_LOW_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_LOW_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_CRT_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_CRT_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_FS_LOW_ACT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_FS_CRT_ACT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_ARM_VOLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_ARM_MAH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_VOLT_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_CURR_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_VOLT_MULT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_AMP_PERVLT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_AMP_OFFSET", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_I2C_BUS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BATT9_I2C_ADDR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_SER1_RTSCTS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_SER2_RTSCTS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_SER3_RTSCTS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_SER4_RTSCTS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_SER5_RTSCTS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_SAFETYENABLE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_SBUS_OUT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_SERIAL_NUM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_SAFETY_MASK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_IMU_TARGTEMP", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_TYPE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_IO_ENABLE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_SAFETYOPTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_VBUS_MIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_VSERVO_MIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_SD_SLOWDOWN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_PWM_VOLT_SEL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_BOOT_DELAY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_IMUHEAT_P", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_IMUHEAT_I", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_IMUHEAT_IMAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_ALT_CONFIG", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_RADIO_TYPE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_RADIO_PROT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_RADIO_DEBUG", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_RADIO_DISCRC", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_RADIO_SIGCH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_RADIO_PPSCH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_RADIO_TELEM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_RADIO_TXPOW", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_RADIO_FCCTST", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_RADIO_STKMD", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_RADIO_TESTCH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_RADIO_TSIGCH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_RADIO_TPPSCH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_RADIO_TXMAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_RADIO_BZOFS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_RADIO_ABTIME", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_RADIO_ABLVL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_RTC_TYPES", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BRD_RTC_TZ_MIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BTN_ENABLE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BTN_PIN1", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BTN_PIN2", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BTN_PIN3", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BTN_PIN4", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BTN_REPORT_SEND", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BTN_OPTIONS1", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BTN_OPTIONS2", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BTN_OPTIONS3", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BTN_OPTIONS4", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BTN_FUNC1", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BTN_FUNC2", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BTN_FUNC3", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("BTN_FUNC4", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CAM_TRIGG_TYPE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CAM_DURATION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CAM_SERVO_ON", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CAM_SERVO_OFF", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CAM_TRIGG_DIST", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CAM_RELAY_ON", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CAM_MIN_INTERVAL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CAM_MAX_ROLL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CAM_FEEDBACK_PIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CAM_FEEDBACK_POL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CAM_AUTO_ONLY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CAM_TYPE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CAM_RC_TYPE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CAM_RC_FEATURES", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CAM_RC_BT_DELAY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CAM_RC_BTN_DELAY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CAM_RC_MDE_DELAY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CAM_RC_CONTROL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("MNT_RC_IN_ROLL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("MNT_RC_IN_PAN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("MNT_RC_IN_TILT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CHUTE_ENABLED", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CHUTE_TYPE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CHUTE_SERVO_ON", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CHUTE_SERVO_OFF", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CHUTE_ALT_MIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CHUTE_DELAY_MS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CHUTE_CRT_SINK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("CHUTE_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_OFS_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_OFS_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_OFS_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_DEC", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_LEARN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_USE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_AUTODEC", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_MOTCT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_MOT_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_MOT_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_MOT_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_ORIENT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_EXTERNAL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_OFS2_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_OFS2_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_OFS2_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_MOT2_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_MOT2_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_MOT2_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_OFS3_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_OFS3_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_OFS3_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_MOT3_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_MOT3_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_MOT3_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_DEV_ID", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_DEV_ID2", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_DEV_ID3", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_USE2", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_ORIENT2", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_EXTERN2", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_USE3", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_ORIENT3", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_EXTERN3", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_DIA_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_DIA_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_DIA_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_ODI_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_ODI_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_ODI_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_DIA2_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_DIA2_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_DIA2_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_ODI2_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_ODI2_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_ODI2_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_DIA3_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_DIA3_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_DIA3_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_ODI3_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_ODI3_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_ODI3_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_CAL_FIT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_OFFS_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_TYPEMASK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_FLTR_RNG", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_AUTO_ROT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_PRIO1_ID", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_PRIO2_ID", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_PRIO3_ID", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_ENABLE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_SCALE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_SCALE2", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_SCALE3", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_DEV_ID4", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_DEV_ID5", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_DEV_ID6", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_DEV_ID7", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_DEV_ID8", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_CUS_ROLL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_CUS_PIT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_CUS_YAW", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_PMOT_EN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_PMOT_EXP", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_PMOT1_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_PMOT1_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_PMOT1_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_PMOT2_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_PMOT2_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_PMOT2_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_PMOT3_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_PMOT3_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_PMOT3_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_PMOT4_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_PMOT4_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("COMPASS_PMOT4_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EAHRS_TYPE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EAHRS_RATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EFI_TYPE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EFI_COEF1", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EFI_COEF2", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_ENABLE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_GPS_TYPE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_VELNE_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_VELD_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_VEL_I_GATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_POSNE_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_POS_I_GATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_GLITCH_RAD", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_ALT_SOURCE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_ALT_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_HGT_I_GATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_HGT_DELAY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_MAG_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_MAG_CAL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_MAG_I_GATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_EAS_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_EAS_I_GATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_RNG_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_RNG_I_GATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_MAX_FLOW", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_FLOW_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_FLOW_I_GATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_FLOW_DELAY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_GYRO_P_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_ACC_P_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_GBIAS_P_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_GSCL_P_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_ABIAS_P_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_WIND_P_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_WIND_PSCALE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_GPS_CHECK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_IMU_MASK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_CHECK_SCALE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_NOAID_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_YAW_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_YAW_I_GATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_TAU_OUTPUT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_MAGE_P_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_MAGB_P_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_RNG_USE_HGT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_TERR_GRAD", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_BCN_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_BCN_I_GTE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_BCN_DELAY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_RNG_USE_SPD", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_MAG_MASK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_OGN_HGT_MASK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_FLOW_USE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_MAG_EF_LIM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_HRT_FILT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_GSF_RUN_MASK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_GSF_USE_MASK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK2_GSF_RST_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_ENABLE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_VELNE_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_VELD_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_VEL_I_GATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_POSNE_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_POS_I_GATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_GLITCH_RAD", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_ALT_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_HGT_I_GATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_HGT_DELAY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_MAG_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_MAG_CAL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_MAG_I_GATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_EAS_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_EAS_I_GATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_RNG_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_RNG_I_GATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_MAX_FLOW", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_FLOW_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_FLOW_I_GATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_FLOW_DELAY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_GYRO_P_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_ACC_P_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_GBIAS_P_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_ABIAS_P_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_WIND_P_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_WIND_PSCALE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_GPS_CHECK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_IMU_MASK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_CHECK_SCALE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_NOAID_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_BETA_MASK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_YAW_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_YAW_I_GATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_TAU_OUTPUT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_MAGE_P_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_MAGB_P_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_RNG_USE_HGT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_TERR_GRAD", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_BCN_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_BCN_I_GTE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_BCN_DELAY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_RNG_USE_SPD", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_ACC_BIAS_LIM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_MAG_MASK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_OGN_HGT_MASK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_VIS_VERR_MIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_VIS_VERR_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_WENC_VERR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_FLOW_USE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_HRT_FILT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_MAG_EF_LIM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_GSF_RUN_MASK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_GSF_USE_MASK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_GSF_RST_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_ERR_THRESH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_AFFINITY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_DRAG_BCOEF_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_DRAG_BCOEF_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_DRAG_M_NSE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_DRAG_MCOEF", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_OGNM_TEST_SF", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_GND_EFF_DZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_PRIMARY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_SRC1_POSXY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_SRC1_VELXY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_SRC1_POSZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_SRC1_VELZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_SRC1_YAW", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_SRC2_POSXY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_SRC2_VELXY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_SRC2_POSZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_SRC2_VELZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_SRC2_YAW", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_SRC3_POSXY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_SRC3_VELXY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_SRC3_POSZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_SRC3_VELZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_SRC3_YAW", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("EK3_SRC_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FENCE_ENABLE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FENCE_TYPE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FENCE_ACTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FENCE_ALT_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FENCE_RADIUS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FENCE_MARGIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FENCE_TOTAL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FENCE_ALT_MIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FENCE_RET_RALLY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FENCE_RET_ALT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FENCE_AUTOENABLE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FFT_ENABLE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FFT_MINHZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FFT_MAXHZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FFT_SAMPLE_MODE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FFT_WINDOW_SIZE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FFT_WINDOW_OLAP", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FFT_FREQ_HOVER", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FFT_THR_REF", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FFT_SNR_REF", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FFT_ATT_REF", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FFT_BW_HOVER", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FFT_HMNC_FIT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FFT_HMNC_PEAK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FLOW_TYPE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FLOW_FXSCALER", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FLOW_FYSCALER", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FLOW_ORIENT_YAW", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FLOW_POS_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FLOW_POS_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FLOW_POS_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("FLOW_ADDR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GEN_TYPE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_TYPE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_TYPE2", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_NAVFILTER", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_AUTO_SWITCH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_MIN_DGPS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_SBAS_MODE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_MIN_ELEV", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_SBP_LOGMASK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_RAW_DATA", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_GNSS_MODE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_SAVE_CFG", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_GNSS_MODE2", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_AUTO_CONFIG", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_RATE_MS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_RATE_MS2", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_POS1_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_POS1_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_POS1_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_POS2_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_POS2_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_POS2_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_DELAY_MS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_DELAY_MS2", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_BLEND_MASK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_BLEND_TC", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_DRV_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_COM_PORT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_COM_PORT2", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_PRIMARY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_CAN_NODEID1", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_CAN_NODEID2", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS1_CAN_OVRIDE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS2_CAN_OVRIDE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_MB1_TYPE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_MB1_OFS_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_MB1_OFS_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_MB1_OFS_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_MB2_TYPE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_MB2_OFS_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_MB2_OFS_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GPS_MB2_OFS_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GRIP_ENABLE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GRIP_TYPE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GRIP_GRAB", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GRIP_RELEASE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GRIP_NEUTRAL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GRIP_REGRAB", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GRIP_UAVCAN_ID", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GUIDED_P", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GUIDED_I", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GUIDED_D", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GUIDED_FF", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GUIDED_IMAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GUIDED_FLTT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GUIDED_FLTE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GUIDED_FLTD", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("GUIDED_SMAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ICE_ENABLE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ICE_START_CHAN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ICE_STARTER_TIME", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ICE_START_DELAY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ICE_RPM_THRESH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ICE_PWM_IGN_ON", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ICE_PWM_IGN_OFF", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ICE_PWM_STRT_ON", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ICE_PWM_STRT_OFF", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ICE_RPM_CHAN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ICE_START_PCT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ICE_IDLE_PCT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ICE_IDLE_RPM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ICE_IDLE_DB", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ICE_IDLE_SLEW", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ICE_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("ICE_STARTCHN_MIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_GYROFFS_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_GYROFFS_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_GYROFFS_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_GYR2OFFS_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_GYR2OFFS_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_GYR2OFFS_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_GYR3OFFS_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_GYR3OFFS_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_GYR3OFFS_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACCSCAL_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACCSCAL_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACCSCAL_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACCOFFS_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACCOFFS_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACCOFFS_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACC2SCAL_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACC2SCAL_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACC2SCAL_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACC2OFFS_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACC2OFFS_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACC2OFFS_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACC3SCAL_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACC3SCAL_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACC3SCAL_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACC3OFFS_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACC3OFFS_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACC3OFFS_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_GYRO_FILTER", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACCEL_FILTER", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_USE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_USE2", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_USE3", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_STILL_THRESH", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_GYR_CAL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_TRIM_OPTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACC_BODYFIX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_POS1_X", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_POS1_Y", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_POS1_Z", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_GYR_ID", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACC_ID", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_FAST_SAMPLE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ENABLE_MASK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_GYRO_RATE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_ACC1_CALTEMP", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_GYR1_CALTEMP", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_TCAL_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_HNTCH_ENABLE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_HNTCH_FREQ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_HNTCH_BW", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_HNTCH_ATT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_HNTCH_HMNCS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_HNTCH_REF", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_HNTCH_MODE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_HNTCH_OPTS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_LOG_BAT_CNT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_LOG_BAT_MASK", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_LOG_BAT_OPT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_LOG_BAT_LGIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_LOG_BAT_LGCT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_NOTCH_ENABLE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_NOTCH_ATT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_NOTCH_FREQ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("INS_NOTCH_BW", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_SLOPE_RCALC", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_ABORT_DEG", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_PITCH_CD", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_FLARE_ALT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_FLARE_SEC", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_PF_ALT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_PF_SEC", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_PF_ARSPD", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_THR_SLEW", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_DISARMDELAY", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_THEN_NEUTRL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_ABORT_THR", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_FLAP_PERCNT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_TYPE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_DS_V_FWD", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_DS_SLOPE_A", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_DS_SLOPE_B", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_DS_APP_EXT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_DS_V_DWN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_DS_SLEW_SPD", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_DS_ELEV_PWM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_DS_ARSP_MAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_DS_ARSP_MIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_DS_L1", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_DS_L1_I", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_DS_YAW_LIM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_DS_L1_TCON", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_DS_P", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_DS_I", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_DS_D", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_DS_IMAX", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_DS_ABORTALT", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("LAND_DS_AIL_SCL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("MIS_TOTAL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("MIS_RESTART", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("MIS_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RALLY_TOTAL", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RALLY_LIMIT_KM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RALLY_INCL_HOME", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC_OVERRIDE_TIME", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC_PROTOCOLS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC0_MIN", 1000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC0_DZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC0_MAX", 2000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC0_TRIM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC0_REVERSED", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC0_OPTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC1_MIN", 1000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC1_DZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC1_MAX", 2000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC1_TRIM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC1_REVERSED", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC1_OPTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC2_MIN", 1000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC2_DZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC2_MAX", 2000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC2_TRIM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC2_REVERSED", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC2_OPTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC3_MIN", 1000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC3_DZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC3_MAX", 2000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC3_TRIM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC3_REVERSED", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC3_OPTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC4_MIN", 1000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC4_DZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC4_MAX", 2000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC4_TRIM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC4_REVERSED", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC4_OPTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC5_MIN", 1000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC5_DZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC5_MAX", 2000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC5_TRIM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC5_REVERSED", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC5_OPTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC6_MIN", 1000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC6_DZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC6_MAX", 2000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC6_TRIM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC6_REVERSED", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC6_OPTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC7_MIN", 1000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC7_DZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC7_MAX", 2000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC7_TRIM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC7_REVERSED", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC7_OPTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC8_MIN", 1000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC8_DZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC8_MAX", 2000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC8_TRIM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC8_REVERSED", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC8_OPTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC9_MIN", 1000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC9_DZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC9_MAX", 2000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC9_TRIM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC9_REVERSED", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC9_OPTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC10_MIN", 1000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC10_DZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC10_MAX", 2000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC10_TRIM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC10_REVERSED", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC10_OPTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC11_MIN", 1000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC11_DZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC11_MAX", 2000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC11_TRIM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC11_REVERSED", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC11_OPTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC12_MIN", 1000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC12_DZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC12_MAX", 2000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC12_TRIM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC12_REVERSED", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC12_OPTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC13_MIN", 1000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC13_DZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC13_MAX", 2000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC13_TRIM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC13_REVERSED", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC13_OPTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC14_MIN", 1000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC14_DZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC14_MAX", 2000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC14_TRIM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC14_REVERSED", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC14_OPTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC15_MIN", 1000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC15_DZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC15_MAX", 2000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC15_TRIM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC15_REVERSED", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC15_OPTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC16_MIN", 1000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC16_DZ", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC16_MAX", 2000f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC16_TRIM", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC16_REVERSED", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("RC16_OPTION", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("STAT_FLTTIME", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TERRAIN_ENABLE", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TERRAIN_SPACING", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TERRAIN_OPTIONS", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
            parameters.Add(new Parameter("TERRAIN_MARGIN", 0f, MAVLink.MAV_PARAM_TYPE.UINT8));
        }

        public void ConnectEvent(ClientObject client)
        {
            client.requestedRates[MAVLink.MAVLINK_MSG_ID.HEARTBEAT] = 0.25f;
            Log("Client connected");
        }

        public void DisconnectEvent(ClientObject client)
        {
            Log("Client disconnected");
        }

        public void ReceiveSetRate(ClientObject client, MAVLink.MAVLinkMessage rawMessage)
        {
            //client.requestedRates[message.messageType] = message.rate;
        }

        public void ParamRequestList(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            Log("PARAM_REQUEST_LIST");
            MAVLink.mavlink_param_request_list_t message = (MAVLink.mavlink_param_request_list_t)messageRaw.data;
            for (int i = 0; i < parameters.Count; i++)
            {
                Parameter p = parameters[i];
                MAVLink.mavlink_param_value_t sendMessage = new MAVLink.mavlink_param_value_t();
                sendMessage.param_id = p.GetIDBytes();
                sendMessage.param_type = (byte)p.type;
                sendMessage.param_value = p.value;
                sendMessage.param_count = (ushort)parameters.Count;
                sendMessage.param_index = (ushort)i;
                client.SendMessage(sendMessage);
            }
        }

        public void RequestDataStream(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            //TODO: Implement ALL of these.
            MAVLink.mavlink_request_data_stream_t message = (MAVLink.mavlink_request_data_stream_t)messageRaw.data;
            Log($"REQUEST_DATA_STREAM  TYPE:{(MAVLink.MAV_DATA_STREAM)message.req_stream_id}  =  {message.req_message_rate}");
            switch ((MAVLink.MAV_DATA_STREAM)message.req_stream_id)
            {
                case MAVLink.MAV_DATA_STREAM.ALL:
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.ATTITUDE] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.RAW_IMU] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.GPS_RAW_INT] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.GPS_STATUS] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.RADIO_STATUS] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_SCALED] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_RAW] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.SERVO_OUTPUT_RAW] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.LOCAL_POSITION_NED] = 1f / message.req_message_rate;
                    //client.requestedRates[MAVLink.MAVLINK_MSG_ID.RAW_RPM] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.AUTOPILOT_VERSION] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.VFR_HUD] = 1f / message.req_message_rate;
                    break;
                case MAVLink.MAV_DATA_STREAM.RAW_SENSORS:
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.ATTITUDE] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.RAW_IMU] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.GPS_RAW_INT] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.GPS_STATUS] = 1f / message.req_message_rate;
                    break;
                case MAVLink.MAV_DATA_STREAM.EXTENDED_STATUS:
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.LOCAL_POSITION_NED] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.GPS_STATUS] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.RADIO_STATUS] = 1f / message.req_message_rate;
                    //Can't find CONTROL_STATUS or AUX_STATUS
                    break;
                case MAVLink.MAV_DATA_STREAM.RC_CHANNELS:
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_SCALED] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_RAW] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.SERVO_OUTPUT_RAW] = 1f / message.req_message_rate;
                    break;
                case MAVLink.MAV_DATA_STREAM.RAW_CONTROLLER:
                    //Can't find these messages
                    break;
                case MAVLink.MAV_DATA_STREAM.POSITION:
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.VFR_HUD] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT] = 1f / message.req_message_rate;
                    client.requestedRates[MAVLink.MAVLINK_MSG_ID.LOCAL_POSITION_NED] = 1f / message.req_message_rate;
                    //Can't find GLOBAL_POSITION
                    break;
                case MAVLink.MAV_DATA_STREAM.EXTRA1:
                    break;
                case MAVLink.MAV_DATA_STREAM.EXTRA2:
                    break;
                case MAVLink.MAV_DATA_STREAM.EXTRA3:
                    break;
            }
        }

        public void SystemTime(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            MAVLink.mavlink_system_time_t message = (MAVLink.mavlink_system_time_t)messageRaw.data;
            Log($"SYSTEM_TIME {message.time_unix_usec}");
        }

        public void RequestProtocolVersion(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            AckCommand(client, command, MAVLink.MAV_CMD_ACK.OK);
            MAVLink.mavlink_protocol_version_t version = new MAVLink.mavlink_protocol_version_t();
            version.min_version = 1;
            version.max_version = 3;
            version.version = 2;
            client.SendMessage(version);
            Log($"REQUEST_PROTOCOL_VERSION, FAILED, NO MAVLINK2");
        }

        public void MessageInterval(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            AckCommand(client, command, MAVLink.MAV_CMD_ACK.OK);
            Log($"SET_MESSAGE_INTERVAL: {(MAVLink.MAVLINK_MSG_ID)command.param1} = {command.param2}");
            client.requestedRates[(MAVLink.MAVLINK_MSG_ID)command.param1] = command.param1 * 1000000;
        }

        public void RequestAutopilot(ClientObject client, MAVLink.mavlink_command_long_t command)
        {
            Log("REQUEST AUTOPILOT");
            AckCommand(client, command, MAVLink.MAV_CMD_ACK.OK);
            MAVLink.mavlink_autopilot_version_t autopilot = new MAVLink.mavlink_autopilot_version_t();
            autopilot.capabilities |= (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.MISSION_FLOAT;
            autopilot.capabilities |= (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.PARAM_FLOAT;
            autopilot.capabilities |= (ulong)MAVLink.MAV_PROTOCOL_CAPABILITY.COMMAND_INT;
            autopilot.board_version = 1;
            autopilot.flight_sw_version = 1;
            autopilot.os_sw_version = 1;
            byte[] emptyByte = new byte[8];
            autopilot.flight_custom_version = emptyByte;
            autopilot.middleware_custom_version = emptyByte;
            autopilot.os_custom_version = emptyByte;
            autopilot.product_id = 0;
            autopilot.vendor_id = 0;
            autopilot.uid = 1;
            client.SendMessage(autopilot);
        }

        public void SendHeartbeat(ClientObject client)
        {
            MAVLink.mavlink_heartbeat_t message = new MAVLink.mavlink_heartbeat_t();
            message.custom_mode = 0;
            message.type = (byte)MAVLink.MAV_TYPE.FIXED_WING;
            message.autopilot = (byte)MAVLink.MAV_AUTOPILOT.ARDUPILOTMEGA;
            message.base_mode = (byte)MAVLink.MAV_MODE.MANUAL_DISARMED;
            message.system_status = (byte)MAVLink.MAV_STATE.ACTIVE;
            message.mavlink_version = (byte)MAVLink.MAVLINK_VERSION;
            client.SendMessage(message);


            uint sensors = (uint)(MAVLink.MAV_SYS_STATUS_SENSOR._3D_GYRO | MAVLink.MAV_SYS_STATUS_SENSOR._3D_ACCEL | MAVLink.MAV_SYS_STATUS_SENSOR._3D_MAG | MAVLink.MAV_SYS_STATUS_SENSOR.ABSOLUTE_PRESSURE | MAVLink.MAV_SYS_STATUS_SENSOR.BATTERY | MAVLink.MAV_SYS_STATUS_SENSOR.GPS);
            MAVLink.mavlink_sys_status_t sysStatus = new MAVLink.mavlink_sys_status_t();
            sysStatus.onboard_control_sensors_present = sensors;
            sysStatus.onboard_control_sensors_enabled = sensors;
            sysStatus.onboard_control_sensors_health = sensors;
            //1%
            sysStatus.load = 50;
            sysStatus.voltage_battery = 11000;
            sysStatus.current_battery = 1000;
            client.SendMessage(sysStatus);
        }

        public void SendAttitude(ClientObject client)
        {
            MAVLink.mavlink_attitude_t message = new MAVLink.mavlink_attitude_t();
            message.pitch = data.radpitch;
            message.roll = data.radroll;
            message.yaw = data.radyaw;
            message.pitchspeed = 0;
            message.rollspeed = 0;
            message.yawspeed = 0;
            message.time_boot_ms = GetUptime();
            client.SendMessage(message);
        }

        public void SendGPSGlobalPosition(ClientObject client)
        {
            MAVLink.mavlink_global_position_int_t message = new MAVLink.mavlink_global_position_int_t();
            message.lat = data.latitude;
            message.lon = data.longitude;
            message.alt = (int)data.altitude;
            message.relative_alt = 0;//(int)data.altitude;
            message.hdg = (ushort)data.heading;
            message.vx = 0;
            message.vy = 0;
            message.vz = 0;
            message.time_boot_ms = GetUptime();
            client.SendMessage(message);
        }

        public void SendVFRHud(ClientObject client)
        {
            MAVLink.mavlink_vfr_hud_t message = new MAVLink.mavlink_vfr_hud_t();
            message.airspeed = data.iaspeed;
            message.climb = data.cr;
            message.alt = data.altitude;
            message.heading = (short)data.heading;
            //TODO
            message.groundspeed = 0;
            message.throttle = 0;
        }

        public void SendGPSRaw(ClientObject client)
        {
            MAVLink.mavlink_gps_raw_int_t message = new MAVLink.mavlink_gps_raw_int_t();
            message.lat = data.latitude;
            message.lon = data.longitude;
            message.alt = (int)data.altitude;
            message.eph = 300;
            message.epv = 500;
            message.vel = 1000;
            message.cog = 25000;
            message.satellites_visible = 10;
            message.alt_ellipsoid = (int)data.altitude;
            message.h_acc = 10;
            message.v_acc = 10;
            message.vel_acc = 10;
            message.hdg_acc = 10;
            message.yaw = (ushort)data.yaw;
            client.SendMessage(message);
        }

        public void SendGPSStatus(ClientObject client)
        {
            MAVLink.mavlink_gps_status_t message = new MAVLink.mavlink_gps_status_t();
            message.satellites_visible = 10;
            byte[] prn = new byte[20];
            byte[] used = new byte[20];
            byte[] ele = new byte[20];
            byte[] azi = new byte[20];
            byte[] snr = new byte[20];
            for (int i = 0; i < 10; i++)
            {
                prn[i] = (byte)(i + 1);
                if (i <= 8)
                {
                    used[i] = 1;
                }
                ele[i] = (byte)(45 + i * 2);
                azi[i] = (byte)(i * 20);
                snr[i] = (byte)(30 + i);
            }
            message.satellite_prn = prn;
            message.satellite_used = used;
            message.satellite_elevation = ele;
            message.satellite_azimuth = azi;
            message.satellite_snr = snr;
            client.SendMessage(message);
        }

        public void SendRawIMU(ClientObject client)
        {
            MAVLink.mavlink_raw_imu_t message = new MAVLink.mavlink_raw_imu_t();
            DateTime epoch = new DateTime(1970, 1, 1, 0, 0, 0, 0, DateTimeKind.Utc);
            TimeSpan diff = DateTime.UtcNow - epoch;
            message.time_usec = (ulong)(diff.TotalMilliseconds * 1000);
            message.xacc = (short)data.accx;
            message.yacc = (short)data.accy;
            message.zacc = (short)data.accz;
            message.xgyro = (short)data.gyrox;
            message.ygyro = (short)data.gyroy;
            message.zgyro = (short)data.gyroz;
            message.xmag = (short)data.magx;
            message.ymag = (short)data.magy;
            message.zmag = (short)data.magz;
            message.id = 0;
            message.temperature = 6000;
            client.SendMessage(message);
        }

        public void SendRPM(ClientObject client)
        {
            MAVLink.mavlink_raw_rpm_t message = new MAVLink.mavlink_raw_rpm_t();
            //message.rpm1 = data.avrrpm;
            client.SendMessage(message);
        }


        public void SendRadioStatus(ClientObject client)
        {
            MAVLink.mavlink_radio_status_t message = new MAVLink.mavlink_radio_status_t();
            message.rssi = (byte)data.rssi;
            message.remrssi = 200;
            message.txbuf = 99;
            message.rxerrors = 0;
            message.@fixed = 0;
            client.SendMessage(message);
        }

        public void SendRadioChannelsRaw(ClientObject client)
        {
            MAVLink.mavlink_rc_channels_t message = new MAVLink.mavlink_rc_channels_t();
            message.rssi = 200;
            message.chan1_raw = (ushort)data.ch1;
            message.chan2_raw = (ushort)data.ch2;
            message.chan3_raw = (ushort)data.ch3;
            message.chan4_raw = (ushort)data.ch4;
            message.chan5_raw = (ushort)data.ch5;
            message.chan6_raw = (ushort)data.ch6;
            message.chan7_raw = (ushort)data.ch7;
            message.chan8_raw = (ushort)data.ch8;
            client.SendMessage(message);
        }

        public void SendRadioChannelsScaled(ClientObject client)
        {
            MAVLink.mavlink_rc_channels_scaled_t message = new MAVLink.mavlink_rc_channels_scaled_t();
            message.rssi = 200;
            message.chan1_scaled = 5000;
            message.chan2_scaled = 5000;
            message.chan3_scaled = 5000;
            message.chan4_scaled = 5000;
            message.chan5_scaled = 5000;
            message.chan6_scaled = 5000;
            message.chan7_scaled = 5000;
            message.chan8_scaled = 5000;
            client.SendMessage(message);
        }
    

    public void AckCommand(ClientObject client, MAVLink.mavlink_command_long_t command, MAVLink.MAV_CMD_ACK ackType)
        {
            MAVLink.mavlink_command_ack_t ack = new MAVLink.mavlink_command_ack_t();
            ack.command = command.command;
            ack.result = (byte)ackType;
            ack.target_system = command.target_system;
            ack.target_component = command.target_component;
        }

        public void SetParameter(ClientObject client, string id, float value, MAVLink.MAV_PARAM_TYPE type)
        {
            Parameter p = null;
            bool add = true;
            int sendID = -1;
            for (int i = 0; i < parameters.Count; i++)
            {

                if (parameters[i].id == id)
                {
                    p = parameters[i];
                    p.value = value;
                    p.type = type;
                    add = false;
                    sendID = i;
                }
            }
            if (add)
            {
                p = new Parameter(id, value, type);
                parameters.Add(p);
                sendID = parameters.Count - 1;
            }
            MAVLink.mavlink_param_value_t sendMessage = new MAVLink.mavlink_param_value_t();
            sendMessage.param_id = p.GetIDBytes();
            sendMessage.param_type = (byte)p.type;
            sendMessage.param_value = p.value;
            sendMessage.param_index = (ushort)sendID;
            sendMessage.param_count = (ushort)parameters.Count;
            client.SendMessage(sendMessage);
        }

        public uint GetUptime()
        {
            long timeMS = (DateTime.UtcNow.Ticks - startTime) / TimeSpan.TicksPerMillisecond;
            return (uint)timeMS;
        }
    }
}

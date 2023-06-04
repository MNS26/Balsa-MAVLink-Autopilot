using System;
using System.Net;
using System.Reflection;
using System.Text;
using UnityEngine.Networking.NetworkSystem;

namespace AutopilotCommon
{
    public class MessagesReceived
    {
        private long startTime;
        private const byte systemID = 1;
        private const byte componentID = 1;
        private static MAVLink.MavlinkParse parser = new MAVLink.MavlinkParse();
        private Data data;
        private Ap ap;
        private ParameterHandler parameters;
        private Action<string> Log;
        public MessagesReceived(Data data, Ap ap, Action<string> Log, ParameterHandler parameters)
        {
            this.Log = Log;
            startTime = DateTime.UtcNow.Ticks;
            this.data = data;
            this.ap = ap;
            this.parameters = parameters;
        }
        public void ConnectEvent(ClientObject client)
        {
            client.requestedRates[MAVLink.MAVLINK_MSG_ID.HEARTBEAT] = 1f;
            Log("Client connected");
        }

        public void DisconnectEvent(ClientObject client)
        {
            Log("Client disconnected");
        }

        [ReceiveMessage(MAVLink.MAVLINK_MSG_ID.HEARTBEAT)]
        public void Heartbeat(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            MAVLink.mavlink_heartbeat_t message = (MAVLink.mavlink_heartbeat_t)messageRaw.data;
        }

        [ReceiveMessage(MAVLink.MAVLINK_MSG_ID.SYSTEM_TIME)]
        public void SystemTime(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            MAVLink.mavlink_system_time_t message = (MAVLink.mavlink_system_time_t)messageRaw.data;
            Log($"SYSTEM_TIME {message.time_unix_usec}");
        }

        [ReceiveMessage(MAVLink.MAVLINK_MSG_ID.PARAM_REQUEST_READ)]
        public void ParamRequestRead(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            MAVLink.mavlink_param_request_read_t message = (MAVLink.mavlink_param_request_read_t)messageRaw.data;
            Log($"PARAM_REQUEST_READ:  Target system {message.target_system}  Target component {message.target_component}  param index {message.param_index}");
//            MAVLink.mavlink_param_request_read_t sendMessage = new MAVLink.mavlink_param_request_read_t(){
//                target_system = systemID,
//                target_component = componentID,
//                param_id = message.param_id,
//                param_index = (short)-1,
//            };
//            client.SendMessage(sendMessage);

                MAVLink.mavlink_param_value_t sendMessage = new MAVLink.mavlink_param_value_t(){
                    param_id = message.param_id,
                    param_type = (byte)0,
                    param_value = -1,
                    param_count = (ushort)0,
                    param_index = (ushort)0,
                };
                client.SendMessage(sendMessage);
        }

        [ReceiveMessage(MAVLink.MAVLINK_MSG_ID.PARAM_REQUEST_LIST)]
        public void ParamRequestList(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            Log($"PARAM_REQUEST_LIST");
            MAVLink.mavlink_param_request_list_t message = (MAVLink.mavlink_param_request_list_t)messageRaw.data;
            int index = 0;
            int count = parameters.GetCount();
            foreach (Parameter p in parameters)
            {

                MAVLink.mavlink_param_value_t sendMessage = new MAVLink.mavlink_param_value_t(){
                    param_id = p.GetIDBytes(),
                    param_type = (byte)p.type,
                    param_value = p.value,
                    param_count = (ushort)count,
                    param_index = (ushort)index++,
                };
                client.SendMessage(sendMessage);
            }
        }

        [ReceiveMessage(MAVLink.MAVLINK_MSG_ID.MISSION_REQUEST_LIST)]
        public void SendMissionList(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            Log($"MISSION_REQUEST_LIST: {messageRaw}");
            //MAVLink.mavlink_mission_request_list_t missionList = (MAVLink.mavlink_mission_request_list_t)messageRaw.data;
            MAVLink.mavlink_mission_count_t missionCount = new MAVLink.mavlink_mission_count_t(){
                target_system = systemID,
                target_component = componentID,
                count = 0,
                mission_type = (byte)MAVLink.MAV_MISSION_TYPE.ALL,
            };
            client.SendMessage(missionCount);
        }

        [ReceiveMessage(MAVLink.MAVLINK_MSG_ID.MISSION_REQUEST_INT)]
        public void SendMissionReq(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            Log($"MISSION_REQUEST: {messageRaw}");
            MAVLink.mavlink_mission_request_int_t message = new MAVLink.mavlink_mission_request_int_t()
            {
                target_system = systemID,
                target_component = componentID,
                seq = 0,
                mission_type = (byte)MAVLink.MAV_MISSION_TYPE.MISSION,
            };
            client.SendMessage(message);
        }

        [ReceiveMessage(MAVLink.MAVLINK_MSG_ID.MISSION_ACK)]
        public void SendMissionAck(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            Log("MISSION_ACK");
            MAVLink.mavlink_mission_ack_t message = (MAVLink.mavlink_mission_ack_t)messageRaw.data;
            message.target_system = systemID;
            message.target_component = componentID;
            message.type = 0;
            message.mission_type = (byte)MAVLink.MAV_MISSION_TYPE.ALL;
            client.SendMessage(message);
        }

        [ReceiveMessage(MAVLink.MAVLINK_MSG_ID.LOG_REQUEST_LIST)]
        public void RequestLogList(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            //we currently cant generate logs so we dont have a use for start and end count for logs
            //MAVLink.mavlink_log_request_list_t message = new MAVLink.mavlink_log_request_list_t();
            MAVLink.mavlink_log_entry_t message = new MAVLink.mavlink_log_entry_t()
            {
                id = 0,
                num_logs = 0,
                last_log_num = 0,
                time_utc = 0,
                size = 0
            };
            client.SendMessage(message);
        }

        [ReceiveMessage(MAVLink.MAVLINK_MSG_ID.PARAM_SET)]
        public void SetParameter(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {}

        [ReceiveMessage(MAVLink.MAVLINK_MSG_ID.REQUEST_DATA_STREAM)]
        public void RequestDataStream(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            //TODO: Implement ALL of these.
            MAVLink.mavlink_request_data_stream_t message = (MAVLink.mavlink_request_data_stream_t)messageRaw.data;
            MAVLink.MAV_DATA_STREAM mdsType = (MAVLink.MAV_DATA_STREAM)message.req_stream_id;
            //Log($"REQUEST_DATA_STREAM  TYPE:{mdsType}  =  {message.req_message_rate}");
            //Log($"REQUEST_DATA_STREAM  =  {message.req_message_rate}");
            foreach (MethodInfo mi in typeof(ProtocolLogic).GetMethods())
            {
                SendMessage sm = mi.GetCustomAttribute<SendMessage>();
                SendCategory category = mi.GetCustomAttribute<SendCategory>();
                if (sm != null && category != null && (category.type == mdsType || mdsType == MAVLink.MAV_DATA_STREAM.ALL))
                {
                    client.requestedRates[sm.id] = 1f / message.req_message_rate;
                }
            }
        }

        [ReceiveMessage(MAVLink.MAVLINK_MSG_ID.STATUSTEXT)]
        public void Statustext(ClientObject client, MAVLink.MAVLinkMessage messageRaw)
        {
            MAVLink.mavlink_statustext_t status = (MAVLink.mavlink_statustext_t)messageRaw.data;
            MAVLink.mavlink_statustext_t sendStatus = new MAVLink.mavlink_statustext_t()
            {
                severity = status.severity,
                text = status.text,
                id = status.id,
                chunk_seq = status.chunk_seq
            };
            client.SendMessage(sendStatus);
        }
    }
}
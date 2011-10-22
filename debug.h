#if 0
#define D(_x_) _x_
#define DEBUG_PRINT(_msg_) Serial.print(_msg_)
#define DEBUG_TRACE(_msg_) DEBUG_PRINT(_msg_ "\r\n")
#define DEBUG_TRACE_COND(cond, msg_true, msg_false) ((cond)? (1,DEBUG_TRACE(msg_true)) : (0,DEBUG_TRACE(msg_false))
#define DEBUG_ENABLE(_baud_) Serial.begin(_baud_)
#else
#define D(_x_)
#define DEBUG_PRINT(_msg_)
#define DEBUG_TRACE(_msg_)
#define DEBUG_TRACE_COND(cond, msg_true, msg_false) 
#define DEBUG_ENABLE(_baud_)
#endif

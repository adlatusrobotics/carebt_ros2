uint8 ACTION=0
uint8 SEQUENCE=1
uint8 PARALLEL=2
uint8 RATE_CONTROL=3
uint8 FALLBACK=4
uint8 ROOT=5
uint8 OTHER=255

uint8 PENDING=0
uint8 LIVE=1
uint8 COMPLETED=2
uint8 REMOVED=3

uint8 STATUS_IDLE=0
uint8 STATUS_RUNNING=1
uint8 STATUS_SUSPENDED=2
uint8 STATUS_SUCCESS=3
uint8 STATUS_FAILURE=4
uint8 STATUS_ABORTED=5
uint8 STATUS_FIXED=6
uint8 STATUS_UNKNOWN=255

uint64 call_id
uint64 instance_id
uint64 parent_call_id
uint32 child_index
string class_name
string module_name
uint8 node_kind
uint8 lifecycle
uint8 status
string contingency_message
carebt_msgs/ExecutionTraceParameter[] parameters
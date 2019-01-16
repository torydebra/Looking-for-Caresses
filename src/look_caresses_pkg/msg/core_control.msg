
# direction: downstream
# destination: biomimetic core
# description: control signals



#### HEADER

# standard header
std_msgs/Header header

# message flags
uint32 msg_flags

# set as many of these as required to force synchronisation
# between those streams (you must set identical flags on
# each stream)
uint32 FLAG_SYNC_PLATFORM=1
uint32 FLAG_SYNC_CORE=2



#### BRANCH

float32[8] priming_multiplicative
float32[8] priming_additive



#### ACTION

priority_peak priority_peak
float32[8] disinhibition
float32[8] priority_scale
float32[8] priority_offset



#### AFFECT

affect mood_drive_target
float32 mood_drive_gamma

affect emotion_drive_target
float32 emotion_drive_gamma

sleep sleep_drive_target
float32 sleep_drive_gamma



#### EXPRESS

float32 drive_vocal



#### BODY

# array of pushes; provide up to four pushes at once;
# members with zero pushvec have no effect
push[] body_push





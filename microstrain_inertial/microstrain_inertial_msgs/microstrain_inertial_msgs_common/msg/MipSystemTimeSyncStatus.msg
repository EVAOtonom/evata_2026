# Message definition for the MIP field https://files.microstrain.com/CV7+Online/external_content/dcp/Data/0xa0/data/0x02.htm

# Header containing common information
#   header.frame_id has no meaning in this message
microstrain_inertial_msgs/MipHeader header

# True if sync with the PPS signal is currently valid. False if PPS feature is disabled or a PPS signal is not detected.
bool time_sync

# Elapsed time in seconds since last PPS was received, with a maximum value of 255.
uint8 last_pps_rcvd
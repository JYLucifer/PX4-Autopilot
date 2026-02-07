uint64 timestamp        # [us] Time since system start (microseconds)

uint8 frame             # [@enum FRAME_TYPE] Protocol frame type identifier
uint8 role              # [@enum ROLE] Device role (e.g., anchor, tag, node)
uint32 id               # Unique device identifier ID
LinktrackData[8] nodes  # Array of node data (max 8 nodes)

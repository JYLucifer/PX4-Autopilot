uint64 timestamp        # [us] Time since system start (microseconds) [2](@ref)

# ================== Paging Information ==================
uint8 page              # Current page index (0-based)
uint8 total_pages       # Total number of pages in this transmission
uint8 total_count       # Total number of nodes across all pages
uint8 valid_count       # Number of valid nodes in current page [4](@ref)

# ================== Node Data Arrays (per node in current page) ==================
uint8[27] role          # [@enum NODE_ROLE] Role/type of each node
uint8[27] id            # Unique identifier for each node
float32[27] dis         # [m] [@invalid NaN] Distance measurement for each node (meters)
float32[27] fp_rssi     # [dBm] [@invalid NaN] Forward path RSSI (signal strength) for each node
float32[27] rx_rssi     # [dBm] [@invalid NaN] Receive signal strength indicator for each node
float32[27] angle       # [rad] [@invalid NaN] Angle measurement (AOA) for each node (radians) [3](@ref)

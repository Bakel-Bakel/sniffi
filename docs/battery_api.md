# Battery API (Dashboard Integration)

## Topics
- `/battery_status` (`sniffi_msgs/BatteryStatus`) — 1 Hz
- `/return_to_dock` (`std_msgs/Bool`) — emitted when SOC <= threshold
- `/estop` (`std_msgs/Bool`) — operator sets true to stop; false to clear
- `/heartbeat` (`std_msgs/Bool`) — dashboard publishes every 1s

## REST (if using a ROS–HTTP bridge)
- `GET /api/battery/status` → latest battery data
- `POST /api/estop { "estop": true }` → assert estop
- `POST /api/estop/clear` → clear estop
- `POST /api/heartbeat` → publish heartbeat

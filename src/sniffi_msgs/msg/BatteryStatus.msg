# Sniffi Battery Status
builtin_interfaces/Time stamp
float32 voltage          # Volts
float32 current          # Amps (+discharge, -charge)
float32 state_of_charge  # 0..100 (%)
float32 temperature      # Celsius (optional)
bool    external_power   # true if on dock / charging
bool    bms_ok           # true if BMS reports healthy

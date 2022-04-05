# CAN
CAN control in CARLA simulation

# Requirements
1. Ubuntu 18.04 or higher
2. VCAN 
  
    in command:
  ```bash
  sudo apt-get install can-utils
  ```
3. Carla
(https://carla.readthedocs.io/en/latest/start_quickstart/)

# Setup
1. activate vcan(in CAN folder)

```bash
sudo ./vcan.sh
```

2. activate carla server

in command:

```bash
cd /opt/carla-simulator
./CarlaUE4.sh
```

to enhance performance:
```bash
cd /opt/carla-simulator/PythonAPI/util
python3 config.py --no-rendering
```

3. creating traffic(in CAN folder)

```bash
python3 generate_traffic.py
```

4. 

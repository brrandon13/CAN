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

in carla-simulator folder


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


in CAN folder


3. creating traffic

```bash
python3 generate_traffic.py
```

4. execute 

```bash
python3 can_control_by_agent.py
```

in other terminal:

```bash
python3 can_control_module.py
```

5. to look for the CAN data:

```bash
candump vcan0 | cantools decode SantaFe.dbc


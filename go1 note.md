sudo nmap -PR -sn --script broadcast-dns-service-discovery 192.168.123.0/24 | cat

Nmap scan report for 192.168.123.10
Host is up (0.00021s latency).
MAC Address: 00:80:E1:00:00:00 (STMicroelectronics SRL)

Nmap scan report for 192.168.123.14
Host is up (0.00029s latency).
MAC Address: 48:B0:2D:6A:31:0A (Nvidia)

# used for deployment
Nmap scan report for 192.168.123.15
Host is up (0.00047s latency).
MAC Address: 48:B0:2D:76:2F:3D (Nvidia)

Nmap scan report for 192.168.123.161
Host is up (0.00038s latency).
MAC Address: E4:5F:01:B6:E3:21 (Raspberry Pi Trading)

# nuc
user unitree
pass 123

1. L2+A to lay down
2. L2+B to enter damping mode
3. start lcm_position
4. start deploy.py

ssh unitree@192.168.123.15

cd ~/unitree_go1_deploy/unitree_legged_sdk/build
./lcm_position


cd ~/unitree_go1_deploy/unitree_legged_sdk/build && ./lcm_position
conda deactivate && cd ~/unitree_go1_deploy/go1_deploy/go1_gym_deploy/scripts/ && python3 deploy.py

# if the joint has weird sound, restart this program

cd ~/unitree_go1_deploy/go1_deploy/go1_gym_deploy/scripts/
python3 deploy.py
# r2 once to calibrate
# r2 again to start
# r2 again to stop

sudo systemctl restart networking

# install wifi driver
https://github.com/lwfinger/rtl8852au
fix fallthrough compile error: https://github.com/lwfinger/rtl8852au/pull/83/commits/7ccdbffda216b3be4cb8d3df0c0c9cf5c1d29663
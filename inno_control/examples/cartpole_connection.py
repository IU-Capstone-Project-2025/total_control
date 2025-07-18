from inno_control.devices import CartPole
from inno_control.devices.port_scan import find_your_device

port = input('Type your device port (enter for scan)\n')

# sudo $(which python) script.py

if port:
    device = CartPole(port)    
else:
    device = CartPole(find_your_device())


print('connecting..')
device.connect(do_init_activity = True)
device.start_experimnet()
print('connected!')
s = device.get_joint_state()
print(f'Current state: {s}')

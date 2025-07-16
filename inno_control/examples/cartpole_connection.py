from inno_control.devices import CartPole
from inno_control.devices.port_scan import find_your_device

port = input('Type your device port (enter for scan)\n')



if port:
    device = CartPole(port)    
else:
    device = CartPole(find_your_device())


print('connecting..')
device.connect(do_init_activity = False)
print('connected!')

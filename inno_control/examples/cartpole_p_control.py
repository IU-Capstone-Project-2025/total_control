from inno_control.devices import CartPole
from inno_control.devices.port_scan import find_your_device

port = input('Type your device port (enter for scan)\n')


if port:
    device = CartPole(port)
else:
    device = CartPole(find_your_device())


device.connect(do_init_activity = True)
device.start_experimnet()
print('Press enter to start control')

max_force = 90
centre = 6251
input()


while True:
    try:
        res = device.get_joint_state()
        if res:
            print(res)
            res.split()
            coef = centre - int(res[2])
            force = (coef / abs(coef)) * 40 + 0.01 * coef
            if(abs(force) > max_force):
                force = max_force
            # print(force)
            device.set_joint_efforts(force)
    except (Exception) as e:
        print(e)
        continue

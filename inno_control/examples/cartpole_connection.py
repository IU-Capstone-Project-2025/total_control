from inno_control.devices import CartPole
from inno_control.devices.port_scan import find_your_device


# a = CartPole(find_your_device())
a = CartPole('/dev/cu.usbserial-0001')


print('connecting..')
a.connect(do_init_activity = True)
print('connected!')
print('starting..')
a.start_experimnet()
print('started!')

max_force = 90
centre = 6251
input()


while True:
    try:
        res = a.get_joint_state()
        if res:
            print(res)
            res.split()
            coef = centre - int(res[0])
            force = (coef / abs(coef)) * 40 + 0.01 * coef
            if(abs(force) > max_force):
                force = max_force
            # print(force)
            a.set_joint_efforts(force)
    except (Exception) as e:
        print(e)
        continue

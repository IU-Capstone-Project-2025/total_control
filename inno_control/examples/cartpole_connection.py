from inno_control.devices import CartPole

a = CartPole('/dev/cu.usbserial-110')
print('Начало положено...')
a.connect()
a.start_experimnet()

max_force = 90
centre = 6251
input()
while True:
    try:
        res = a.get_joint_state().split()
        print(res)
        coef = centre - int(res[0])
        force = (coef / abs(coef)) * 40 + 0.01 * coef
        if(abs(force)>max_force):
            force = max_force
        print(force)
        a.set_joint_efforts(force)
    except (Exception) as e:
        print(e)
        continue

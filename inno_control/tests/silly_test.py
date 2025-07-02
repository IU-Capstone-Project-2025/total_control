import pytest

import inno_control

def test_case_0():
    #тут мы проверили, что у нас вообще что-то подключается и работает
    assert 1==1

def test_case_1():
    silly = inno_control.CartPole('/dev/cu.usbserial-0001')
    silly.connect()
    assert silly.get_state() == "READY"

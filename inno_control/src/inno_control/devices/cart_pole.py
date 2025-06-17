from .devices import LabDevice

class CartPole(LabDevice):
    """
    Class for entire Cart-Pole work
    """

    def __init__(self, port: str, baudrate: int = 921600, timeout: float = 1.0):
        super().__init__(port, baudrate, timeout)

    

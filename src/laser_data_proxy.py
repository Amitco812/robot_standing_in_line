from abc import abstractmethod, ABCMeta
from six import add_metaclass


@add_metaclass(ABCMeta)
class LaserDataProxy:

    @abstractmethod
    def get_laser_data(self):
        raise NotImplementedError("This Function Is Not Implemented!")

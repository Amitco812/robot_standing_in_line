from abc import abstractmethod, ABCMeta
from six import add_metaclass


@add_metaclass(ABCMeta)
class WallDetector():

    @abstractmethod
    def detect_wall():
        raise NotImplementedError("This Function Is Not Implemented!")

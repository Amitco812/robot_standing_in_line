from abc import abstractmethod, ABCMeta
from six import add_metaclass


'''
Class LineTracker - A component which tracks a line
Fields:
*done_tracking - True iff We are done tracking (F.E we are first in line) 
*prev_positions - A log for points we went through
'''


@add_metaclass(ABCMeta)
class LineTracker:
    def __init__(self):
        self.done_tracking = False
        self.prev_positions = []

    '''
    @Pre: 
        None
    @Params: 
        None
    @Return Value:
        True iff the tracking has ended
    Description: 
        None
    '''

    def get_done_tracking(self):
        return self.done_tracking

    '''
    @Pre: 
        done_tracking = False
    @Params: 
        None
    @Return Value:
        boolean- should_move
        tuple - (x,y,yaw) of goal location and rotation
    Description: 
        The next goal to go to in line
    '''
    @abstractmethod
    def get_next_position_in_line(self):
        raise NotImplementedError("This Function Is Not Implemented!")

    '''
    @Pre: 
        None
    @Params: 
        None
    @Return Value:
        The previous positions (for debugging purposes etc ..)
    Description: 
        None
    '''

    def get_prev_positions(self):
        return self.prev_positions

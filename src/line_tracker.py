from abc import ABC, abstractmethod

'''
Class LineTracker - A component which tracks a line
Fields:
*done_tracking - True iff We are done tracking (F.E we are first in line) 
*prev_positions - A log for points we went through
'''
class LineTracker(ABC):
    def __init__(self):
        self.done_tracking = False
        self.prev_positions = []

    
    '''
    @Description: returns true iff the tracking has ended
    '''
    def done_tracking(self):
        return self.done_tracking

    
    '''
    @Description: Returns the next goal to go to in line
    @Pre: done_tracking = False
    '''
    @abstractmethod
    def get_next_position_in_line(self):
        raise NotImplementedError("This Function Is Not Implemented!")

    
    '''
    @Description: move to the location provided using a strategy of your choice (move_base or any other way)
    @Params: 
    *data - the data needed to perform the action. (position, yaw, etc ..)
    '''
    @abstractmethod
    def move(self,data):
        raise NotImplementedError("This Function Is Not Implemented!")

    '''
    @Description: return the previous positions (for debugging purposes etc ..)
    '''
    def get_prev_positions(self):
        return self.prev_positions
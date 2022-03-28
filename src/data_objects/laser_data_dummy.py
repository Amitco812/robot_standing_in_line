from laser_data_proxy import LaserDataProxy


class LaserMsgDummy:
    def __init__(self, ranges):
        self.ranges = ranges

    def __getitem__(self, key):
        return self.ranges[key]

    def __setitem__(self, key, newvalue):
        self.ranges[key] = newvalue


class LaserDataDummy(LaserDataProxy):
    def __init__(self, data=None):
        self.data = data

    def get_laser_data(self):
        return self.data

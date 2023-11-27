class RingClass:
    def __init__(self, size_max):
        self.size = size_max
        self.data = []
    class __Full:
        def append(self, x):
            self.data[self.cur] = x
            self.cur = (self.cur+1) % self.size
        def tolist(self):
            return self.data[self.cur:]+self.data[:self.cur]
    def append(self, x):
        self.data.append(x)
        if len(self.data) == self.size:
            self.cur = 0
            self.__class__ = self.__Full
    def tolist(self):
        return self.data
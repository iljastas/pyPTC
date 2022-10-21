


class Control_Base():
    def __init__(self, feedforward=None, lah_distance=0.0):
        self.lah = lah_distance
        self.feedforward = feedforward
    
    def lah_distance(self):
        ''' get the lookahead (lah) distance '''
        return self.lah
        

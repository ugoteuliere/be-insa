from tools import Affine, Constant, Piecewise

# Compute a desired trajectory for the center of pressure, piecewise affine linking the
# input step positions in the plane
class CoPDes(Piecewise):
    single_support_time = .5
    double_support_time = .1 
    # constructor
    #   - start: initial position of the CoP
    #   - steps: list of 2D vectors representing the successive positions of the feet,
    #   - end: final position of the CoP
    def __init__(self, start, steps, end):
        super().__init__()
        t=0
        self.segments.append(Affine(0,self.double_support_time,start,steps[0]))
        t=t+self.double_support_time
        for step in steps[1:]:
            self.segments.append(Constant(t,t+self.single_support_time,self(t)))
            t=t+self.single_support_time
            self.segments.append(Affine(t,t+self.double_support_time,self(t),step))
            t=t+self.double_support_time
        self.segments.append(Affine(t,t+self.double_support_time,self(t),end))
        t=t+self.double_support_time
        self.segments.append(Constant(t,t+10,self(t)))

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import numpy as np
    start = np.array([0.,0.])
    steps = [np.array([0, -.1]), np.array([0.4, .1]),
             np.array([.8, -.1]), np.array([1.2, .1]),
             np.array([1.6, -.1]), np.array([1.6, .1])]
    end = np.array([1.6,0.])
    cop_des = CoPDes(start, steps, end)
    times = 0.01 * np.arange(500)
    cop = np.array(list(map(cop_des, times)))
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlabel("second")
    ax.set_ylabel("meter")
    ax.plot(times, cop[:,0], label="x_cop")
    ax.plot(times, cop[:,1], label="y_cop")
    ax.legend()
    plt.show()

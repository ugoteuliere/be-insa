from tools import Affine, Constant, Piecewise
import numpy as np

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
        t = 0.0
        
        current_pos = np.array(start)
        
        # On nettoie la liste des steps pour ne garder que X et Y (les 2 premiers éléments)
        # Cela transforme des steps [x, y, theta] en [x, y]
        steps_2d = [np.array(s[:2]) for s in steps] 
        end_2d = np.array(end[:2]) # Sécurité pour end aussi
        # ----------------------

        # Premier transfert (Double Support)
        self.segments.append(Affine(t, t + self.double_support_time, current_pos, steps_2d[0]))
        t += self.double_support_time
        current_pos = steps_2d[0]

        # Boucle sur les pas suivants (avec la liste corrigée steps_2d)
        for step in steps_2d[1:]:
            # Single Support
            self.segments.append(Constant(t, t + self.single_support_time, current_pos))
            t += self.single_support_time
            
            # Double Support
            self.segments.append(Affine(t, t + self.double_support_time, current_pos, step))
            t += self.double_support_time
            
            current_pos = step

        # Dernier segment vers end
        self.segments.append(Affine(t, t + self.double_support_time, current_pos, end_2d))
        t += self.double_support_time
        current_pos = end_2d

        # Fin stable
        self.segments.append(Constant(t, t + 10, current_pos))

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

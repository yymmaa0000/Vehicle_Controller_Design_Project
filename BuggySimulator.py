# please do not change this file
import numpy as np
import matplotlib.pyplot as plt


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def wrap2pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def addGaussianNoise(a,u,sigma):
    # add Gaussian noise to a scalar a, with mean u and covariance sigma
    a += np.random.normal(u, sigma, 1)
    return a[0]


class vehicle:
    # parameters
    lr = 1.7
    lf = 1.1
    Ca = 15000.0
    Iz = 3344.0
    f = 0.01
    m = 2000.0
    g = 10

    def __init__(self, state):
        # initialize the vehicle
        self.state = state
        self.observation = vehicle.observation(
            state.xd, state.yd, state.phid, state.delta, state.X, state.Y, state.phi)

    class state:
        deltaMax = np.pi/6
        deltaMin = -np.pi/6
        xdMax = 100.0
        xdMin = 0.0
        ydMax = 10.0
        ydMin = -10.0

        def __init__(self, xd = 0.0, yd = 0.0, phid = 0.0, delta = .0, X = .0, Y = .0, phi = .0):
            self.xd = xd
            self.yd = yd
            self.phid = phid
            self.delta = clamp(delta, self.deltaMin, self.deltaMax)
            self.X = X
            self.Y = Y
            self.phi = wrap2pi(phi)
        def showState(self):
            print('xd\t', self.xd, 'yd\t', self.yd, 'phid\t', self.phid, 'delta\t', self.delta,
                  'X\t', self.X, 'Y\t', self.Y, 'phi\t', self.phi)

    class observation(state):
        def __init__(self, xd = 0.0, yd = 0.0, phid = 0.0, delta = .0, X = .0, Y = .0, phi = .0):
            vehicle.state.__init__(self, xd, yd, phid, delta, X, Y, phi)
            self.addNoise()

        def copyState(self, state):
            self.xd = state.xd
            self.yd = state.yd
            self.phid = state.phid
            self.delta = state.delta
            self.X = state.X
            self.Y = state.Y
            self.phi = state.phi

        def addNoise(self):
            self.xd = addGaussianNoise(self.xd, 0, 0.5)
            self.yd = addGaussianNoise(self.yd, 0, 0.5)
            self.phid = addGaussianNoise(self.phid, 0, 0.05)
            self.delta = clamp(addGaussianNoise(self.delta, 0, 0.05), self.deltaMin, self.deltaMax)
            self.X = addGaussianNoise(self.X, 0, 1)
            self.Y = addGaussianNoise(self.Y, 0, 1)
            self.phi = wrap2pi(addGaussianNoise(self.phi, 0, 0.5))

    class command:
        # F: N
        # deltad: rad/s
        deltadMax = np.pi/6.0
        deltadMin = -np.pi/6.0
        FMax = 10000.0
        Fmin = -10000.0

        def __init__(self, F_ = 0.0, deltad_ = 0.0):
            self.F = clamp(F_, self.Fmin, self.FMax)
            self.deltad = clamp(deltad_, self.deltadMin, self.deltadMax)

        def showCommand(self):
            print('F:\t', self.F, 'deltad:\t', self.deltad)

    def update(self, command):
        # time step
        dt = 0.05
        # update state
        Ff = np.sign(self.state.xd)*self.f*self.m*self.g
        Ftotal = command.F - Ff if np.abs(command.F)>=np.abs(Ff) else 0
        # print(Ftotal)
        ax = 1/self.m*Ftotal
        if np.abs(self.state.xd) <= 0.5:
            Fyf = 0.0
            Fyr = 0.0
        else:
            Fyf = 2.0*self.Ca*(self.state.delta-(self.state.yd+self.lf*self.state.phid)/self.state.xd)
            Fyr = 2.0*self.Ca*(-(self.state.yd-self.lr*self.state.phid)/self.state.xd)

        xdd = self.state.phid* self.state.yd + ax
        ydd = - self.state.phid* self.state.xd + 1.0/self.m*(Fyf*np.cos(self.state.delta) - Fyr)
        phidd = 1.0/self.Iz*(self.lf*Fyf - self.lr*Fyr)
        Xd = self.state.xd*np.cos(self.state.phi) - self.state.yd*np.sin(self.state.phi)
        Yd = self.state.xd*np.sin(self.state.phi) + self.state.yd*np.cos(self.state.phi)
        self.state.xd += xdd*dt
        self.state.yd += ydd*dt
        self.state.phid += phidd*dt

        self.state.delta += command.deltad*dt
        # self.state.delta = command.deltad


        self.state.X += Xd*dt
        self.state.Y += Yd*dt
        self.state.phi += self.state.phid*dt
        self.applyConstrain()
        # update observation
        self.observation.copyState(self.state)
        self.observation.addNoise()
        # self.state.showState()


    def applyConstrain(self):
        # phi should be between +- pi
        self.state.phi = wrap2pi(self.state.phi)
        # state constraint
        self.state.delta = clamp(self.state.delta,self.state.deltaMin,self.state.deltaMax)
        self.state.xd = clamp(self.state.xd, self.state.xdMin, self.state.xdMax)
        self.state.yd = clamp(self.state.yd, self.state.ydMin, self.state.ydMax)

    def showState(self):
        self.state.showState()


if __name__ == "__main__":
    a = vehicle(vehicle.state(Y = 0.0,xd = 1))
    n = 1000

    X = []
    Y = []
    delta = []
    xd = []
    yd = []
    phi = []
    phid = []
    for i in range(n):
        # if i% 1 ==0:
        X.append(a.state.X)
        Y.append(a.state.Y)
        delta.append(a.state.delta)
        xd.append(a.state.xd)
        yd.append(a.state.yd)
        phid.append(a.state.phid)
        phi.append(a.state.phi)
        if a.state.xd > 3:
            c = a.command(deltad_=np.sin(i/10), F_=-10000)
        else:
            c = a.command(deltad_=np.sin(i/10), F_=10000.0)
        a.update(command = c)

    plt.subplot(321)
    plt.title('position')
    plt.plot(X,Y,'r')

    plt.subplot(322)
    plt.title('delta')
    plt.plot(delta, 'r')

    plt.subplot(323)
    plt.title('xd')
    plt.plot(xd, 'r')

    plt.subplot(324)
    plt.title('yd')
    plt.plot(yd, 'r')

    plt.subplot(325)
    plt.title('phi')
    plt.plot(phi, 'r')

    plt.subplot(326)
    plt.title('phid')
    plt.plot(phid, 'r')

    plt.show()
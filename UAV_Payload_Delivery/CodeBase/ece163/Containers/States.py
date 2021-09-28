from ..Utilities import Rotations
from ..Utilities import MatrixMath
import math

testingAbs_tol = 1e-6

class vehicleState:
    def __init__(self, pn=0.0, pe=0.0, pd=0.0, u=0.0, v=0.0, w=0.0, yaw=0.0, pitch=0.0, roll=0.0, p=0.0, q=0.0, r=0.0, dcm=None):
        """
        Defines the vehicle states to define the vehicle current position and orientation. Positions are in NED
        coordinates, velocity is ground speed in body coordinates, we carry both the Euler angles and the DCM together,
        and the rotation rates are in the body frame.

        :param pn: vehicle inertial north position [m]
        :param pe: vehicle inertial east position [m]
        :param pd: vehicle inertial down position [m] (Altitude is -pd)
        :param u: vehicle ground speed in body frame x [m/s]
        :param v: vehicle ground speed in body frame y [m/s]
        :param w: vehicle ground speed in body frame z [m/s]
        :param yaw: yaw angle [rad]
        :param pitch: pitch angle [rad]
        :param roll: roll angle [rad]
        :param p: body roll rate about body-x axis [rad/s]
        :param q: body pitch rate about body-y axis [rad/s]
        :param r: body yaw rate about body-z axis [rad/s]
        """
        # positions
        self.pn = pn
        self.pe = pe
        self.pd = pd
        # velocities
        self.u = u
        self.v = v
        self.w = w
        # Euler Angles
        if dcm is None:
            self.yaw = yaw
            self.pitch = pitch
            self.roll = roll
            # Direction Cosine Matrix
            self.R = Rotations.euler2DCM(yaw, pitch, roll) # R transforms from inertial to body
                                                           # use transpose to go the other way
                                                           # Euler angles and R are redundant
        else:
            self.R = dcm
            self.yaw, self.pitch, self.roll = Rotations.dcm2Euler(self.R)

        # body rates
        self.p = p
        self.q = q
        self.r = r
        # Airspeed and Flight Angles (assume wind is zero)
        self.Va = math.hypot(self.u, self.v, self.w)    # Airspeed
        self.alpha = math.atan2(self.w, self.u)         # angle of attack
        if math.isclose(self.Va, 0.0):                  # Sideslip Angle, no airspeed
            self.beta = 0.0
        else:
            self.beta = math.asin(self.v/self.Va)       # Sideslip Angle, normal definition
        pdotned = MatrixMath.multiply(MatrixMath.transpose(self.R),[[self.u],[self.v],[self.w]])
        self.chi = math.atan2(pdotned[1][0],pdotned[0][0])
        return

    def __repr__(self):
        return "{1.__name__}(pn={0.pn}, pe={0.pe}, pd={0.pd}, u={0.u}, v={0.v}, w={0.w}," \
               " yaw={0.yaw}, pitch={0.pitch}, roll={0.roll}, p={0.p}, q={0.q}, r={0.r}, dcm={0.R})".format(self, type(self))

    def __str__(self):
        posvel = "(pn={}, pe={}, pd={})\n(u={}, v={}, w={})\n".format(self.pn, self.pe, self.pd, self.u, self.v, self.w)
        eulrate = "(yaw={}, pitch={}, roll={})\n(p={}, q={}, r={})\n".format(math.degrees(self.yaw), math.degrees(self.pitch),
                                                                             math.degrees(self.roll), math.degrees(self.p),
                                                                             math.degrees(self.q), math.degrees(self.r))
        Rdump = "R = [" + '\t'.join(['{: 7.14f}'.format(r) for r in self.R[0]]) + "]\n" + \
                      '    [' + '\t'.join(['{: 7.14f}'.format(r) for r in self.R[1]]) + "]\n" + \
                        '    [' + '\t'.join(['{: 7.14f}'.format(r) for r in self.R[2]]) + "]\n"
        alphaChi = "(Va={}, alpha={}, beta={}, chi={})".format(self.Va, math.degrees(self.alpha), math.degrees(self.beta),
                                                              math.degrees(self.chi))
        return posvel + eulrate + Rdump + alphaChi

    def __eq__(self, other):
        if isinstance(other, type(self)):
            if not all([math.isclose(getattr(self, member), getattr(other, member),abs_tol=testingAbs_tol) for
                        member in ['pn', 'pe', 'pd', 'u', 'v', 'w', 'p', 'q', 'r', 'yaw','pitch','roll']]):
                return False
            for myRow, otherRow in zip(self.R, other.R):
                if not all([math.isclose(x, y, abs_tol=testingAbs_tol) for x, y in zip(myRow, otherRow)]):
                    return False
            return True
        else:
            return NotImplemented


class windState:
    def __init__(self, Wn=0.0, We=0.0, Wd=0.0, Wu=0.0, Wv=0.0, Ww=0.0):
        """
        Defines the wind states which are composed of the overall constant wind (which is defined in the NED coordinate
        frame), and the gusts (which are defined in the body frame). Gusts are created from a random process. Wind components
        are all in [m/s]

        :param Wn: Constant wind velocity in inertial North direction [m/s]
        :param We: Constant wind velocity in inertial East direction [m/s]
        :param Wd: Constant wind velocity in inertial Down direction [m/s]
        :param Wu: Gust wind velocity in static wind x direction [m/s]
        :param Wv: Gust wind velocity in static wind y direction [m/s]
        :param Ww: Gust wind velocity in static wind z direction [m/s]
        """
        self.Wn = Wn
        self.We = We
        self.Wd = Wd
        self.Wu = Wu    # Stochastic wing gust in body x-axis
        self.Wv = Wv    # Stochastic wing gust in body y-axis
        self.Ww = Ww    # Stochastic wing gust in body z-axis
        return

    def __repr__(self):
        return "{0.__name__}(Wn={1.Wn}, We={1.We}, Wd={1.Wd}, Wu={1.Wu}, Wv={1.Wv}, Ww={1.Ww})".format(type(self), self)

    def __eq__(self, other):
        if isinstance(other, type(self)):
            if not all([math.isclose(getattr(self, member), getattr(other, member), abs_tol=testingAbs_tol) for member in ['Wn', 'We', 'Wd',
                                                                                                   'Wu', 'Wv', 'Ww']]):
                return False
            else:
                return True
        else:
            return NotImplemented
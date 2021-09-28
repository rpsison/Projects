import math
from ..Containers import States
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import PayloadPhysicalConstants as PPC
from ..Utilities import MatrixMath as mm
import numpy as np

class PayloadDynamicsModel:
    def __init__(self):
        #instantiates the state and derivative objects
        self.state = States.vehicleState()
        self.dot =States.vehicleState()
        self.dT = PPC.dT
    
    def reset(self):
        #resets the state and derivative to new state object
        self.state = States.vehicleState()
        self.dot = States.vehicleState()
        self.dT = PPC.dT

    def getVehicleState(self):
        #returns self.state
        return self.state
    
    def getVehicleDerivative(self):
        #returns self.dot
        return self.dot

    def setVehicleState(self, newState: States.vehicleState()):
        #sets vehicle state to argument 
        self.state = newState

    def setVehicleDerivative(self, newDot: States.vehicleState()):
        #sets vehicle derivative to argument 
        self.dot = newDot
    
    def Update(self, forcesMoments):
        #grabs the derivative of the current state
        self.dot = self.derivative(self.getVehicleState(), forcesMoments)
        #updates the state with new integrated state
        self.state = self.IntegrateState(self.dT,self.state, self.dot)
        
    
    def derivative(self, instate: States.vehicleState(), forcesMoments):
        dot = States.vehicleState()
        #computes the derivative of body velocity
        #implements formula 1 from kinematics cheatsheet
        V = [[instate.u],[instate.v],[instate.w]]
        omega = [instate.p,instate.q,instate.r]
        skewsim = mm.skew(omega[0],omega[1],omega[2])
        forces = [[forcesMoments.Fx],[forcesMoments.Fy],[forcesMoments.Fz]]
        x3 = np.matmul(np.array(skewsim, dtype=object)*-1, np.array(V, dtype=object))
        x4 = np.array(forces, dtype=object)*(1/PPC.mass)
        dot.u, dot.v, dot.w = np.add(x3 ,x4 ,dtype = object)

        
        #computes the derivative of angular velocity
        #implements equation 9 from kinematics cheatsheet
        Moments = np.array([forcesMoments.Mx,forcesMoments.My,forcesMoments.Mz], dtype=object)
        Moments = np.transpose(Moments)
        x1 = np.matmul(skewsim, np.matmul(PPC.Jbody,omega))
        x2 = np.subtract(Moments, x1)
        newomega = np.matmul(PPC.JinvBody, x2)
        
        dot.p, dot.q, dot.r = newomega[0],newomega[1],newomega[2]
        
        #converts body velocities to inertial velocities
        V = np.array([instate.u,instate.v,instate.w])
        V = np.transpose(V)
        pos = np.matmul(np.transpose(instate.R),V)
        dot.pn, dot.pe, dot.pd = pos[0],pos[1],pos[2]

        return dot
    def ForwardEuler(self,dT, state: States.vehicleState(), dot: States.vehicleState())->States.vehicleState():
        newState = States.vehicleState()

        #performs forward euler integration on position
        x1 = np.array([state.pn ,state.pe, state.pd],dtype=object)
        x2 = dT*np.array([dot.pn,dot.pe,dot.pd],dtype=object)
        pos = np.add(x1 , x2)
        newState.pn ,newState.pe, newState.pd = pos[0], pos[1], pos[2]

        #performs foward euler integration on linear
        oldvel = np.array([state.u, state.v, state.w], order='C')
        velchange = np.array([dot.u,dot.v,dot.w], order='C')*dT
        velchange = np.squeeze(velchange)
        vel = np.add(oldvel ,velchange)
        newState.u, newState.v, newState.w = vel[0],vel[1],vel[2]

        #performs forward euler integration on angular velocity
        b =  np.add(np.array([state.p, state.q, state.r],dtype=object) , np.array([dot.p, dot.q, dot.r])*dT)
        newState.p, newState.q, newState.r = b[0],b[1],b[2]
        
        return newState
    def Rexp(self,dT, state:States.vehicleState(), dot:States.vehicleState()):
        #returns the left hand side for formula 22 in the attitude cheatsheet
        #performs maclauren series expansion for ||omega|| close to 0 (formula 24,25)
        I = np.identity(3)
        omega = [state.p+dT*dot.p/2, state.q+dT*dot.q/2, state.r+dT*dot.r/2]
        skewsim = mm.skew(omega[0], omega[1], omega[2])
        norm2 = np.linalg.norm(omega,2)

        if norm2 > 0.00001:
            sinside =  np.sin(norm2*dT)/norm2 
            cosside = ((1-np.cos(norm2*dT))/norm2**2)
        else:
            sinside = dT-((dT**3)*(norm2**2))/6 + ((dT**5)*(norm2**4))/120
            cosside = dT**2 -((dT**4)*(norm2**2))/24 + ((dT**6)*(norm2**4))/720
        term2 = mm.scalarMultiply(sinside,skewsim)
        term3 = mm.scalarMultiply(cosside, mm.multiply(skewsim, skewsim))
        expform = I - term2 + term3
        return expform
    
    def IntegrateState(self,dT, state:States.vehicleState(), dot:States.vehicleState()):
        #runs forward euler and gets newstate
        #computes newR and populates other values
        newState = self.ForwardEuler(dT, state, dot)
        
        newState.alpha = state.alpha
        newState.beta = state.beta
        newState.Va = state.Va
        newState.chi = math.atan2(dot.pe,dot.pn)

        #doesnt work, ignores drag
        #pDot = np.array([dot.pn,dot.pe,dot.pd])
        #print(pDot)
        #normalizedPdot = pDot/np.linalg.norm(pDot)
        #newState.yaw, newState.pitch = math.atan2(normalizedPdot[1],normalizedPdot[0]), math.asin(normalizedPdot[2])
        #newState.R = Rotations.euler2DCM(newState.yaw,newState.pitch,0)

        
        newState.yaw = state.yaw
        newState.pitch = state.pitch
        newState.R = Rotations.euler2DCM(newState.yaw,newState.pitch,0)

        
        #newState.R = np.matmul(self.Rexp(dT, state, dot), state.R)
        #newState.yaw, newState.pitch, newState.roll = Rotations.dcm2Euler(newState.R)

        return newState

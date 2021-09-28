import math
import random
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Utilities import MatrixMath
from ..Containers import Sensors
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC
from ..Modeling import VehicleAerodynamicsModel

"""
Matthew Bennett mabennet@ucsc.edu
SensorModel.Py
Code to model the onboard sensors of the plane.
Made up of 3 classes:
    GaussMarkov
    GaussMarkovXYZ
    SensorsModel
"""


class GaussMarkov:
    def __init__(self, dT=0.01, tau=1000000.0, eta=0.0):
        self.dT = dT
        self.tau = tau
        self.eta = eta
        self.v = 0

        return
    def reset(self):
        v = 0
        return

    def update(self, vnoise=None):
        if vnoise != None:
            if self.tau == 0:
                self.v = (math.e) * self.v + vnoise
            else:
                self.v = (math.e ** (-self.dT / self.tau)) * self.v + vnoise
        else:
            if self.tau == 0:
                self.v = (math.e) * self.v + random.gauss(0, self.eta)
            else:
                self.v = (math.e ** (-self.dT/self.tau))*self.v + random.gauss(0, self.eta)
        return self.v


class GaussMarkovXYZ:
    def __init__(self, dT=0.01, tauX=1000000.0, etaX=0.0, tauY=None, etaY=None, tauZ=None, etaZ=None):

        self.tauX = tauX
        if tauY == None:
            self.tauY = tauX
        else:
            self.tauY = tauY


        if tauZ == None:
            if tauY != None:
                self.tauZ = tauY
            else:
                self.tauZ = tauX
        else:
            self.tauZ = tauZ
# Set the eta value
        self.etaX = etaX
        if etaY == None:
            self.etaY = etaX
        else:
            self.etaY = etaY

        if etaZ == None:
            if etaY != None:
                self.etaZ = etaY
            else:
                self.etaZ = etaX
        else:
            self.etaZ = etaZ

        self.dT = dT

        self.vX = GaussMarkov(dT=dT, tau=self.tauX, eta=self.etaX)
        self.vY = GaussMarkov(dT=dT, tau=self.tauY, eta=self.etaY)
        self.vZ = GaussMarkov(dT=dT, tau=self.tauZ, eta=self.etaZ)



        return

    def reset(self):
        self.vX.reset()
        self.vY.reset()
        self.vZ.reset()
        return

    def update(self, vXnoise=None, vYnoise=None, vZnoise=None):

        if vXnoise == None:
            self.vX.v = self.vX.update()
        else:
            self.vX.v = self.vX.update(vXnoise)

        if vXnoise == None:
            self.vY.v = self.vY.update(vYnoise)
        else:
            self.vY.v = self.vY.update(vYnoise)

        if vXnoise == None:
            self.vZ.v = self.vZ.update(vZnoise)
        else:
            self.vZ.v = self.vZ.update(vZnoise)


        return self.vX.v, self.vY.v, self.vZ.v



class SensorsModel:
    def __init__(self, aeroModel=VehicleAerodynamicsModel.VehicleAerodynamicsModel(),
                 taugyro=VSC.gyro_tau,
                 etagyro=VSC.gyro_eta,
                 tauGPS=VSC.GPS_tau,
                 etaGPSHorizontal=VSC.GPS_etaHorizontal,
                 etaGPSVertical=VSC.GPS_etaVertical,
                 gpsUpdateHz=VSC.GPS_rate):

        self.aeroModel = aeroModel

        # 4 sensor model sections
        self.sensorsTrue = Sensors.vehicleSensors()
        self.sensorsBiases = self.initializeBiases()
        self.sensorsSigmas = self.initializeSigmas()
        self.sensorsNoisy = Sensors.vehicleSensors()

        # Since the gyro and the GPS will have drifting biases in our model, we should initialize the appropriate Gauss-
        # Markov processes here.

        self.gyroGM = GaussMarkovXYZ(taugyro, etagyro)
        self.gpsGM = GaussMarkovXYZ(tauX=tauGPS, etaX=etaGPSHorizontal, tauY=tauGPS, etaY=etaGPSHorizontal, tauZ=tauGPS, etaZ=etaGPSVertical)

        # Timer
        self.dT = 0.01
        self.updateTicks = 0
        self.gpsUpdateHz = gpsUpdateHz
        self.gpsTickUpdate = self.gpsUpdateHz/self.dT

        return

    def initializeSigmas(self, gyroSigma=VSC.gyro_sigma,
                         accelSigma=VSC.accel_sigma,
                         magSigma=VSC.mag_sigma,
                         baroSigma=VSC.baro_sigma,
                         pitotSigma=VSC.pitot_sigma,
                         gpsSigmaHorizontal=VSC.GPS_sigmaHorizontal,
                         gpsSigmaVertical=VSC.GPS_sigmaVertical,
                         gpsSigmaSOG=VSC.GPS_sigmaSOG,
                         gpsSigmaCOG=VSC.GPS_sigmaCOG):

        sensorSigmas = Sensors.vehicleSensors()

        sensorSigmas.gyro_x = gyroSigma
        sensorSigmas.gyro_y = gyroSigma
        sensorSigmas.gyro_z = gyroSigma
        sensorSigmas.accel_x = accelSigma
        sensorSigmas. accel_y = accelSigma
        sensorSigmas.accel_z = accelSigma
        sensorSigmas.mag_x = magSigma
        sensorSigmas. mag_y = magSigma
        sensorSigmas. mag_z = magSigma
        sensorSigmas.baro = baroSigma
        sensorSigmas.pitot = pitotSigma
        sensorSigmas.gps_n = gpsSigmaHorizontal
        sensorSigmas.gps_e = gpsSigmaHorizontal
        sensorSigmas.gps_alt = gpsSigmaVertical
        sensorSigmas.gps_sog = gpsSigmaSOG
        sensorSigmas.gps_cog = gpsSigmaCOG

        return sensorSigmas




    def initializeBiases(self, gyroBias=VSC.gyro_bias,
                         accelBias=VSC.accel_bias,
                         magBias=VSC.mag_bias,
                         baroBias=VSC.baro_bias,
                         pitotBias=VSC.pitot_bias):
        sensorBiases = Sensors.vehicleSensors()

        sensorBiases.gyro_x = random.uniform(-gyroBias, gyroBias)
        sensorBiases.gyro_y = random.uniform(-gyroBias, gyroBias)
        sensorBiases.gyro_z = random.uniform(-gyroBias, gyroBias)
        sensorBiases.accel_x = random.uniform(-accelBias, accelBias)
        sensorBiases.accel_y = random.uniform(-accelBias, accelBias)
        sensorBiases.accel_z = random.uniform(-accelBias, accelBias)
        sensorBiases.mag_x = random.uniform(-magBias, magBias)
        sensorBiases.mag_y = random.uniform(-magBias, magBias)
        sensorBiases.mag_z = random.uniform(-magBias, magBias)
        sensorBiases.baro = random.uniform(-baroBias, baroBias)
        sensorBiases.pitot = random.uniform(-pitotBias, pitotBias)
        sensorBiases.gps_n = 0.0
        sensorBiases.gps_e = 0.0
        sensorBiases.gps_alt = 0.0
        sensorBiases.gps_sog = 0.0
        sensorBiases.gps_cog = 0.0

        return sensorBiases

    def reset(self):
        # reset the Gauss Markov values
        self.gpsGM.reset()
        self.gyroGM.reset()

        # remake the Sensor Values. Re call the initializeSigmas and initializeBiases methods
        self.sensorsTrue = Sensors.vehicleSensors()
        self.sensorsBiases = self.initializeBiases()
        self.sensorsSigmas = self.initializeSigmas()
        self.sensorsNoisy = Sensors.vehicleSensors()

        # reset the number of ticks
        self.updateTicks = 0


    def getSensorsNoisy(self):
        return self.sensorsNoisy

    def getSensorsTrue(self):
        return self.sensorsTrue

    def updateGyrosTrue(self, state):
        return state.p, state.q, state.r

    def updateAccelsTrue(self, state, dot):
        ax = dot.u + state.q * state.w - state.r * state.v + VPC.g0 * math.sin(state.pitch)
        ay = dot.v + state.r * state.u - state.p * state.w - VPC.g0 * math.cos(state.pitch) * math.sin(state.roll)
        az = dot.w + state.p * state.v - state.q * state.u - VPC.g0 * math.cos(state.pitch) * math.cos(state.roll)
        return ax, ay, az

    def updateGPSTrue(self, state, dot):
        gps_n = state.pn
        gps_e = state.pe
        gps_h = -state.pd

        gps_SOG = math.sqrt((dot.pn ** 2) + (dot.pe ** 2))
        gps_COG = math.atan2(dot.pe,dot.pn)

        return gps_n, gps_e, gps_h, gps_SOG, gps_COG

    def updateMagsTrue(self, state):
        # correctionMatrix = [[math.cos(state.pitch), math.sin(state.pitch) * math.sin(state.roll), math.sin(state.pitch) * math.cos(state.roll)],
        #                     [0, math.cos(state.roll), -math.sin(state.roll)],
        #                     [-math.sin(state.pitch), math.cos(state.pitch) * math.sin(state.roll), math.cos(state.pitch) * math.cos(state.roll)]]
        resultVec = MatrixMath.multiply(state.R, VSC.magfield)
        mag_x = resultVec[0][0]
        mag_y = resultVec[1][0]
        mag_z = resultVec[2][0]

        return mag_x, mag_y, mag_z

    def updatePressureSensorsTrue(self, state):
        hasl = -state.pd
        T0 = 288.15
        L0 = -0.0065
        M = 0.0289644
        R = 8.31432
        baro = VSC.Pground * (T0/(T0 + (hasl * L0)))**((VPC.g0 * M)/(R * L0))
        pitot = (VPC.rho * state.Va**2)/2
        return baro, pitot

    def updateSensorsTrue(self, prevTrueSensors, state, dot):
        trueSensorOutput = Sensors.vehicleSensors()
        trueSensorOutput.gyro_x, trueSensorOutput.gyro_y, trueSensorOutput.gyro_z = self.updateGyrosTrue(state)
        trueSensorOutput.accel_x, trueSensorOutput.accel_y, trueSensorOutput.accel_z = self.updateAccelsTrue(state, dot)
        trueSensorOutput.mag_x, trueSensorOutput.mag_y, trueSensorOutput.mag_z = self.updateMagsTrue(state)
        trueSensorOutput.baro, trueSensorOutput.pitot = self.updatePressureSensorsTrue(state)

        if self.updateTicks == 0:
            trueSensorOutput.gps_n, trueSensorOutput.gps_e, trueSensorOutput.gps_alt, trueSensorOutput.gps_sog, trueSensorOutput.gps_cog = self.updateGPSTrue(state, dot)

        elif self.gpsTickUpdate % self.updateTicks == 0:
            trueSensorOutput.gps_n, trueSensorOutput.gps_e, trueSensorOutput.gps_alt, trueSensorOutput.gps_sog, trueSensorOutput.gps_cog   = self.updateGPSTrue(state, dot)
        else:
            trueSensorOutput.gps_n = prevTrueSensors.gps_n
            trueSensorOutput.gps_e = prevTrueSensors.gps_e
            trueSensorOutput.gps_alt = prevTrueSensors.gps_alt
            trueSensorOutput.gps_sog = prevTrueSensors.gps_sog
            trueSensorOutput.gps_cog = prevTrueSensors.gps_cog

        return trueSensorOutput

    def updateSensorsNoisy(self, trueSensors=Sensors.vehicleSensors(), noisySensors=Sensors.vehicleSensors(), sensorBiases=Sensors.vehicleSensors(), sensorSigmas=Sensors.vehicleSensors()):
        newNoisySensors = Sensors.vehicleSensors()
        sensorBiases = self.sensorsBiases
        sensorSigmas = self.sensorsSigmas

        gyroGaussX, gyroGaussY, gyroGaussZ = self.gyroGM.update()
        GPSGaussX, GPSGaussY, GPSGaussZ = self.gpsGM.update()

        # the gyro will have 4 terms, it will have static sensor biases, random biases,
        newNoisySensors.gyro_x = trueSensors.gyro_x + sensorBiases.gyro_x + random.gauss(0, sensorSigmas.gyro_x) + gyroGaussX
        newNoisySensors.gyro_y = trueSensors.gyro_y + sensorBiases.gyro_y + random.gauss(0,sensorSigmas.gyro_y) + gyroGaussY
        newNoisySensors.gyro_z = trueSensors.gyro_z + sensorBiases.gyro_z + random.gauss(0, sensorSigmas.gyro_z) + gyroGaussZ

        newNoisySensors.accel_x = trueSensors.accel_x + sensorBiases.accel_x + random.gauss(0, sensorSigmas.accel_x)
        newNoisySensors.accel_y = trueSensors.accel_y + sensorBiases.accel_y + random.gauss(0, sensorSigmas.accel_y)
        newNoisySensors.accel_z = trueSensors.accel_z + sensorBiases.accel_z + random.gauss(0, sensorSigmas.accel_z)

        newNoisySensors.mag_x = trueSensors.mag_x + sensorBiases.mag_x + random.gauss(0, sensorSigmas.mag_x)
        newNoisySensors.mag_y = trueSensors.mag_y + sensorBiases.mag_y + random.gauss(0, sensorSigmas.mag_y)
        newNoisySensors.mag_z = trueSensors.mag_z + sensorBiases.mag_z + random.gauss(0, sensorSigmas.mag_z)

        newNoisySensors.baro = trueSensors.baro + sensorBiases.baro + random.gauss(0, sensorSigmas.baro)
        newNoisySensors.pitot = trueSensors.pitot + sensorBiases.pitot + random.gauss(0, sensorSigmas.pitot)

        if self.updateTicks == 0:
            newNoisySensors.gps_n = trueSensors.gps_n + sensorBiases.gps_n + random.gauss(0, sensorSigmas.gps_n) + GPSGaussX
            newNoisySensors.gps_e = trueSensors.gps_e + sensorBiases.gps_e + random.gauss(0, sensorSigmas.gps_e) + GPSGaussY

            newNoisySensors.gps_sog = trueSensors.gps_sog + sensorBiases.gps_sog + random.gauss(0, sensorSigmas.gps_sog) + GPSGaussZ
            newNoisySensors.gps_cog = trueSensors.gps_cog + sensorBiases.gps_cog + random.gauss(0, sensorSigmas.gps_cog) + GPSGaussZ
        elif self.gpsTickUpdate % self.updateTicks == 0:
            newNoisySensors.gps_n = trueSensors.gps_n + sensorBiases.gps_n + random.gauss(0, sensorSigmas.gps_n) + GPSGaussX
            newNoisySensors.gps_e = trueSensors.gps_e + sensorBiases.gps_e + random.gauss(0, sensorSigmas.gps_e) + GPSGaussY

            newNoisySensors.gps_sog = trueSensors.gps_sog + sensorBiases.gps_sog + random.gauss(0, sensorSigmas.gps_sog) + GPSGaussZ
            newNoisySensors.gps_cog = trueSensors.gps_cog + sensorBiases.gps_cog + random.gauss(0, sensorSigmas.gps_cog) + GPSGaussZ
        else:
            newNoisySensors.gps_n = noisySensors.gps_n
            newNoisySensors.gps_e = noisySensors.gps_e

            newNoisySensors.gps_SOG = noisySensors.gps_sog
            newNoisySensors.gps_COG = noisySensors.gps_cog


        return  newNoisySensors

    def update(self):
        self.sensorsTrue = self.updateSensorsTrue(self.sensorsTrue, self.aeroModel.getVehicleState(), self.aeroModel.VDM.getVehicleDerivative())
        self.sensorsNoisy = self.updateSensorsNoisy(trueSensors=self.sensorsTrue, noisySensors=self.sensorsNoisy, sensorBiases=self.sensorsBiases, sensorSigmas=self.sensorsSigmas)





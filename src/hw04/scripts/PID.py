
import numpy as np

class PID:
    """PID controller for a 3 element vector
    
    @ivar Kp: Set of constants for the Proportional part of the controller
    @type Kp: numpy.ndarray(1_3)
    @ivar Kp: Set of constants for the Integral part of the controller
    @type Kp: numpy.ndarray(1_3)
    @ivar Kp: Set of constants for the Derivative part of the controller
    @type Kp: numpy.ndarray(1_3)
    @ivar setPoint: Desired point to reach for the PID controller
    @type setPoint: numpy.ndarray(1_3)

    Methods
    -------
    says(sound=None)
        Prints the animals name and what sound it makes
    """


    def __init__(self,P=np.array([1.0,1.0,1.0]),I=np.array([0.0,0.0,0.0]),D=np.array([0.0,0.0,0.0]),):
        """Initialize the controller

        @param P: Proportional constant (default: {[1.0,1.0,1.0]})
        @type P: numpy.ndarray(1_3)
        @param I: Integral constant (default: {[0.0,0.0,0.0]})
        @type I: numpy.ndarray(1_3)
        @param D: Derivative constant (default: {0.0,0.0,0.0})
        @type D: numpy.ndarray(1_3)
        """
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.setPoint = 0.0  # Default set point to the origin
        self.error = np.array([0.0,0.0,0.0])   # Initial error

        self.setDerivative(0)   # Initialize variable to store the derivation
        self.setIntegral(0)   # Initialize the variable to store the integration

    def controll(self, currentVector):
        """Get the controlled output depending on the currentVector

        @param currentVector: Current vector to move to the setPoint
        @type currentVector: numpy.ndarray(1_3)
        @returns: Controlled Output
        @rtype: numpy.ndarray(1_3)
        """

        self.error = np.subtract(self.setPoint, currentVector)

        # Proportional part
        self.pComponent = np.transpose(self.Kp) * self.error

        # Derivative part`
        # self.dComponent = np.transpose(self.Kd) * (self.error - self.derivative)/ deltaT
        self.dComponent = np.transpose(self.Kd) * (self.error - self.derivative)
        self.derivative = self.error

        # Integral part
        self.integral = self.integral + self.error
        self.iComponent = np.transpose(self.Ki) * self.integral

        PID = self.pComponent + self.iComponent + self.dComponent

        return PID


    # ----------- Setters and Getters

	def setPoint(self,setPoint):
		""" Initilize the setPoint of PID

        @param currentVector: Current vector to move to the setPoint
        @type currentVector: numpy.ndarray(1_3)
		"""
		self.setPoint = setPoint
		self.setDerivative(0)   # Initialize variable to store the derivation to 0
        self.setIntegral(0)   # Initialize the variable to store the integration to 0

    def getPoint(self):
        """ Get the current value of the Setpoint

        @returns: Current value of the Setpoint
        @rtype: numpy.ndarray(1_3)
		"""
		return self.set_point

	def setIntegral(self, integral):
        """ Set the Integer to the desired value

        @param integral: New value for the Integer
        @type integral: numpy.ndarray(1_3)
		"""
		self.integral = integral

    def getIntegral(self):
        """ Get the current value of the Integer

        @returns: Current value for the Integer
        @rtype: numpy.ndarray(1_3)
		"""
		return self.integral

	def setDerivative(self, derivative):
        """ Set the Derivative to the desired value

        @param derivative: New value for the Derivative
        @type derivative: numpy.ndarray(1_3)
		"""
		self.derivative = derivative

	def getDerivative(self):
        """ Get the current value of the derivative

        @returns: Current value for the derivative
        @rtype: numpy.ndarray(1_3)
		"""
		return self.derivative

	def setKp(self,P):
        """ Set the Proportional Constants to the desired value

        @param P: New value for the Proportional constants
        @type P: numpy.ndarray(1_3)
		"""
		self.Kp=P

    def getKp(self):
        """ Get the Proportional Constants

        @returns: Proportional constants
        @rtype: numpy.ndarray(1_3)
		"""
		return self.Kp

	def setKi(self,I):
        """ Set the Integral Constants to the desired value

        @param I: New value for the Integral constants
        @type I: numpy.ndarray(1_3)
		"""
		self.Ki=I
    
    def getKi(self):
        """ Get the Integral Constants

        @returns: Integral constants
        @rtype: numpy.ndarray(1_3)
		"""
		return self.Ki

	def setKd(self,D):
        """ Set the Derivative Constants to the desired value

        @param D: New value for the Derivative constants
        @type D: numpy.ndarray(1_3)
		"""
		self.Kd=D

    def getKd(self):
        """ Get the Derivative Constants

        @returns: Derivative constants
        @rtype: numpy.ndarray(1_3)
		"""
		return self.Kd

	def getError(self):
        """ Get the Last error from the controller

        @returns: error
        @rtype: numpy.ndarray(1_3)
		"""
		return self.error

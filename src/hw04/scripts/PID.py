import numpy as np

class PID:
    """
    PID controler for a 3 element vector
    
    Attributes
    ----------
    Kp : numpy.ndarray<float>[1,3]
        Set of constants for the Proportional part of the controller
    Ki : numpy.ndarray<float>[1,3]
        Set of constants for the Integral part of the controller
    Kd : npArray<float>[1,3]
        Set of constants for the Derivative part of the controller
    setPoint : npArray<float>[1,3]
        Desired point i=to reach for the PID controller

    Methods
    -------
    says(sound=None)
        Prints the animals name and what sound it makes
    """


    def __init__(self,PX=1.0, PY=1.0, PZ=1.0,IX=0.0, IY=0.0, IZ=0.0,DX=0.0, DY=0.0, DZ=0.0):
        """Constructor for the PID contoller
        
        @param PX: Este es el que quiero constant for the X (default: {1.0})        
        """


    # PY {float} -- Proportional constant for the Y (default: {1.0})
    #         PZ {float} -- Proportional constant for the Z (default: {1.0})
    #         IX {float} -- Integral constant for the X (default: {0.0})
    #         IY {float} -- Integral constant for the Y (default: {0.0})
    #         IZ {float} -- Integral constant for the Z (default: {0.0})
    #         DX {float} -- Derivative constant for the X (default: {0.0})
    #         DY {float} -- Derivative constant for the Y (default: {0.0})
    #         DZ {float} -- Derivative constant for the Z (default: {0.0})
        self.Kp = np.array([PX, PY, PZ])
        self.Ki = np.array([IX, IY, IZ])
        self.Kd = np.array([DX, DY, DZ])

        self.setPoint = 0.0  # Default set point to the origin
        self.error = None   # Initial error

        self.derivator = 0    # Initialize variable to store the derivation
        self.integrator = 0   # Initialize the variable to store the integration

    def controll(self, currentVector,bla):
        """Prueba
        
        Arguments:
            currentVector {Int} -- [description]
            bla {[type]} -- [description]
        
        Returns:
            [type] -- [description]
        """

        self.error = self.setPoint - currentVector

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * (self.error - self.Derivator)
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        PID = self.P_value + self.I_value + self.D_value

        return PID

	# def setPoint(self,set_point):
	# 	"""
	# 	Initilize the setpoint of PID
	# 	"""
	# 	self.set_point = set_point
	# 	self.Integrator=0
	# 	self.Derivator=0

	# def setIntegrator(self, Integrator):
	# 	self.Integrator = Integrator

	# def setDerivator(self, Derivator):
	# 	self.Derivator = Derivator

	# def setKp(self,P):
	# 	self.Kp=P

	# def setKi(self,I):
	# 	self.Ki=I

	# def setKd(self,D):
	# 	self.Kd=D

	# def getPoint(self):
	# 	return self.set_point

	# def getError(self):
	# 	return self.error

	# def getIntegrator(self):
	# 	return self.Integrator

	# def getDerivator(self):
	# 	return self.Derivator




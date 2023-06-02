from HelperFuncs import InterpolateArray
import numpy as np
from Inputs import inputs

class Solver:
    # Class Properties
    MaxLevelInbound = None
    MaxLevelOutBound = None

    def __init__(self, inputs: dict) -> None:
        self.inputs = inputs
        self.nts = int(inputs["TT"] / inputs["dt"]) # Time Step
        self.x = np.array(self.nts)
        self.V = np.array(self.nts)
        self.a = np.array(self.nts)


    def InitializeStiffness(self):        
        self.MaxLevelInbound =1
        self.MaxLevelOutBound = 1


        C = (self.inputs["C_in"] / 100) * 2 * abs(k * M) ** 0.5
        pass

    def GetMassFromRegion(self, region: int) -> float:
        return self.inputs["M_in"][0][region]
    
    def GetStiffnessFromRegion(self, region: int) -> float:
        return self.inputs["K_LM"][0][region]
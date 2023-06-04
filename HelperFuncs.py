from typing import Tuple
import numpy as np

# Function shifts all the deflection limits for all inbound and rebound stiffness areas once unloading begins. In otherwords, moves the entire resistance deflection curve left or 
# right on the x-axis
def SetCurrentBounds(PlasticDeformation: float, XBounds: np.array, n: int, CurrentLevel: int, Reg: int, MaxLevelInbound: int, MaxLevelRebound: int, XMax: np.array) -> Tuple[np.array]:
    """_summary_

    Args:
        PlasticDeformation (float): Keeps track of deformations that exceed the elastic area. Where the elastic area is simplistically defined as the first area of resistance/stiffness.
        XBounds (np.array): Inbound/rebound deflection limits between stiffness areas updated during sdof analysis
        n (int): Number of stiffness areas
        CurrentLevel (int): Current area within the resistance displacement curve the sdof engine is in. Tells the sdof engine what stiffness, resistance, and load-mass factors to use.
        Reg (int): Short for region and tells the sdof engine what region the resistance displacement is in. 1 = Inbound; 2 = Rebound
        MaxLevelInbound (int): Flag that lets the sdof engine know what the max CurrentLevel that has been achieved during inbound response (Reg = 1). This flag is reset once unloading begins.
        MaxLevelRebound (int): Flag that lets the sdof engine know what the max CurrentLevel that has been achieved during rebound response (Reg = 2). This flag is reset once unloading begins
        XMax (np.array): Inbound/rebound initial deflection limits between stiffness areas

    Returns:
        Tuple[np.array]: Inbound/rebound deflection limits between stiffness areas updated during sdof analysis
    """
    if Reg==1:
        if np.any([np.all([CurrentLevel == MaxLevelInbound, MaxLevelInbound != 1]),   np.all([CurrentLevel != MaxLevelInbound, CurrentLevel == 1])]):
            # Change bounds only for Level less than MaxLevelInbound
            for jj in range(n):
                XBounds[Reg][jj] = XMax[Reg][jj] + PlasticDeformation
            for jj in range(n):
                XBounds[2][jj] = XMax[2][jj] + PlasticDeformation
    elif Reg==2:
        # Change bounds only for Level less than MaxLevelInbound
        if np.any([np.all([CurrentLevel == MaxLevelRebound, MaxLevelRebound != -1]),   np.all([CurrentLevel != MaxLevelRebound, CurrentLevel == -1])]):
            for jj in range(n):
                XBounds[Reg][jj] = XMax[Reg][jj] + PlasticDeformation
            for jj in range(n):
                XBounds[2][jj] = XMax[1][jj] + PlasticDeformation
    return (XBounds)
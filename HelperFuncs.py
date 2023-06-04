from typing import Tuple
import numpy as np

# Function shifts all the deflection limits for all inbound and rebound stiffness areas once unloading begins. In otherwords, moves the entire resistance deflection curve left or right on the x-axis
def SetCurrentBounds(PlasticDeformation: float, XBounds: np.array, n: int, CurrentLevel: int, Reg: int, MaxLevelInbound: int, MaxLevelRebound: int, XMax: np.array) -> Tuple[np.array]:
    """_summary_

    Args:
        PlasticDeformation (float): Keeps track of deformations that exceed the elastic area. Where the elastic area is simplistically defined as the first area of resistance/stiffness.
        XBounds (np.array): Inbound/rebound deflection limits between stiffness areas updated during sdof analysis
        n (int): Number of stiffness areas
        CurrentLevel (int): Current area within the resistance displacement curve the sdof engine is in. Tells the sdof engine what stiffness, resistance, and load-mass factors to use.
        Reg (int): Short for region and tells the sdof engine what region the resistance displacement is in. 1 = Inbound; 2 = Rebound.
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


#Function sets the deflection limits for the current resistance and stiffness area that the sdof engine is in based on the total calculated displacement.
def SetCurrentLevelMaxAndMin(CurrentLevelXmax: float, CurrentLevelXmin: float, PlasticDeformation: float, CurrentLevel: int, XBounds: np.array, x: list, Reg: int) -> Tuple[float, float]:
    """_summary_

    Args:
        CurrentLevelXmax (float): Maximum displacement for the current area that that the sdof engine. If the sdof displacement is greater than CurrentLevelXmax then the sdof engine increases to the next highest CurrentLevel (area).
        CurrentLevelXmin (float): Minimum displacement for the current area that that the sdof engine. If the sdof displacement is less than CurrentLevelXmin then the sdof engine decreases to the next lowest CurrentLevel (area).
        PlasticDeformation (float): Keeps track of deformations that exceed the elastic area. Where the elastic area is simplistically defined as the first area of resistance/stiffness.
        CurrentLevel (int): Current area within the resistance displacement curve the sdof engine is in. Tells the sdof engine what stiffness, resistance, and load-mass factors to use.
        XBounds (np.array): Inbound/rebound deflection limits between stiffness areas updated during sdof analysis
        x (list): Current total deflection calculated by the sdof engine for loop.
        Reg (int): Short for region and tells the sdof engine what region the resistance displacement is in. 1 = Inbound; 2 = Rebound.

    Returns:
        Tuple[float, float]: [CurrentLevelXmax, CurrentLevelXmin] (see description in arguments above).
    """

    if CurrentLevel > 1:
        CurrentLevelXmin = x
        CurrentLevelXmax = XBounds[Reg][CurrentLevel]
    elif CurrentLevel == 1:
        CurrentLevelXmin = XBounds[Reg][CurrentLevel-1]
        CurrentLevelXmax = XBounds[Reg][CurrentLevel]
    elif CurrentLevel == -11:
        CurrentLevelXmin = XBounds[Reg][np.abs(CurrentLevel)]
        CurrentLevelXmax = XBounds[Reg][np.abs(CurrentLevel+1)]
    elif CurrentLevel < -1:
        CurrentLevelXmin = XBounds[Reg][abs(CurrentLevel)]
        CurrentLevelXmax = x
    return(CurrentLevelXmin, CurrentLevelXmax)


#Function sets the current area integer for load-mass, resistance, and stiffness. CurrentLevel integer helps the sdof engine know what load-mass, resistance, and stiffness values to use in each timestep
def SetCurrentLevel(CurrentLevel: int, x: list, CurrentLevelXmax: float, CurrentLevelXmin: float, Reg: int, MaxLevelInbound: int, MaxLevelRebound: int, n: int, indx: list, ii: int, ii_velocity_change: int, V: list) -> Tuple[int, int]:
    """_summary_

    Args:
        CurrentLevel (int): Current area within the resistance displacement curve the sdof engine is in. Tells the sdof engine what stiffness, resistance, and load-mass factors to use.
        x (list): Current total deflection calculated by the sdof engine for loop.
        CurrentLevelXmax (float): Maximum displacement for the current area that that the sdof engine. If the sdof displacement is greater than CurrentLevelXmax then the sdof engine increases to the next highest CurrentLevel (area).
        CurrentLevelXmin (float): Minimum displacement for the current area that that the sdof engine. If the sdof displacement is less than CurrentLevelXmin then the sdof engine decreases to the next lowest CurrentLevel (area).
        Reg (int): Short for region and tells the sdof engine what region the resistance displacement is in. 1 = Inbound; 2 = Rebound.
        MaxLevelInbound (int): Flag that lets the sdof engine know what the max CurrentLevel that has been achieved during inbound response (Reg = 1). This flag is reset once unloading begins.
        MaxLevelRebound (int): Flag that lets the sdof engine know what the max CurrentLevel that has been achieved during rebound response (Reg = 2). This flag is reset once unloading begins
        n (int): Number of stiffness areas.
        indx (list): Integer that increase with each occurance of in a change in sign of velocity.
        ii (int): Current loop number in the sdof engine for loop.
        ii_velocity_change (int): Index where velocity change occured in sdof engine for loop to be compared to indx integer.
        V (list): Current velocity calculated by the sdof engine for loop.

    Returns:
        Tuple[int, int]: [CurrentLevel, Reg]
    """

    if CurrentLevel == n:
        if x[ii] == CurrentLevelXmax:
            pass #Exit function due to component failure
        else:
            CurrentLevel = 1
    

    elif CurrentLevel == -n:
        if x[ii] == CurrentLevelXmin:
            pass #Exit function due to component failure
        else:
            CurrentLevel = -1
    
    
    elif CurrentLevel == 1:
        if x[ii] >= CurrentLevelXmax:
            if np.all([MaxLevelInbound == 1,    x[ii] > x[ii-1]]):
                CurrentLevel = CurrentLevel + 1
            elif indx[ii] >= indx[ii_velocity_change + 1]:
                CurrentLevel = 1
            else:
                CurrentLevel = MaxLevelInbound
        else: #x = CurrentLevelXmin
            CurrentLevel = -1
            Reg = 2
    

    elif CurrentLevel == -1:
        if x[ii] <= CurrentLevelXmax:
            if np.all([MaxLevelRebound == -1,    x[ii] < x[ii-1]]):
                CurrentLevel = CurrentLevel - 1
            elif indx[ii] >= indx[ii_velocity_change + 1]:
                CurrentLevel = -1
            else:
                CurrentLevel = MaxLevelRebound
        else: #x = CurrentLevelXmin
            CurrentLevel = 1
            Reg = 1
    


    elif np.all([CurrentLevel > 1, CurrentLevel < n]):
        if x[ii] >= CurrentLevelXmax:
            CurrentLevel = CurrentLevel + 1
        else: #x = CurrentLevelXmin
            CurrentLevel = 1
    


    elif np.all([CurrentLevel < -1, CurrentLevel > -n]):
        if x[ii] <= CurrentLevelXmin:
            CurrentLevel = CurrentLevel - 1
        else: #x = CurrentLevelXmin
            CurrentLevel = 1
    

    return (CurrentLevel, Reg)
from typing import Tuple
from scipy.interpolate import interp1d

# Function to interploate on curve
def InterpolateArray(x: list, y:list, xValue: float) -> Tuple[float, float]:
    """_summary_
    Returns the interpolated array from the given x and y values    
    Args:
        x (list): _description_
        y (list): _description_
    """
    f = interp1d(x, y, kind='linear')
    return f(x)

import numpy as np
from scipy import stats, interpolate

def compare_signals(t1, x1, t2, x2):

    # Konverter til numpy arrays
    t1 = np.asarray(t1, dtype=float)
    t2 = np.asarray(t2, dtype=float)
    x1 = np.asarray(x1, dtype=float)
    x2 = np.asarray(x2, dtype=float)

    # Felles tidsakse (overlappende del)
    t_min = max(t1.min(), t2.min())
    t_max = min(t1.max(), t2.max())
    if t_min >= t_max:
        raise ValueError("No overlap in time")

    # Common grid
    t_common = np.linspace(t_min, t_max, num=1000)

    # Ubterpolate on common grid
    f1 = interpolate.interp1d(t1, x1, kind='linear', fill_value="extrapolate")
    f2 = interpolate.interp1d(t2, x2, kind='linear', fill_value="extrapolate")
    x1_interp = f1(t_common)
    x2_interp = f2(t_common)

    # Calculate metrics
    rmse = np.sqrt(np.mean((x1_interp - x2_interp)**2))
    corr = np.corrcoef(x1_interp, x2_interp)[0, 1]
    mean1 = np.mean(x1_interp)
    mean2 = np.mean(x2_interp)
    mean_error = np.mean(x1_interp - x2_interp)
    kurtosis_diff = stats.kurtosis(x1_interp - x2_interp)

    output = {
        'rmse': rmse,
        'corr': corr,
        # 'mean1': mean1,
        # 'mean2': mean2,
        'mean_error': mean_error,
        'kurtosis_diff': kurtosis_diff,
    }

    return output




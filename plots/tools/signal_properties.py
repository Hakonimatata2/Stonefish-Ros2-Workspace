import numpy as np
from scipy import stats, interpolate

import numpy as np
from scipy import interpolate, stats

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

    # Felles tidsgrid
    t_common = np.linspace(t_min, t_max, num=1000)

    # Interpoler til felles grid
    f1 = interpolate.interp1d(t1, x1, kind='linear', fill_value="extrapolate")
    f2 = interpolate.interp1d(t2, x2, kind='linear', fill_value="extrapolate")
    x1_interp = f1(t_common)
    x2_interp = f2(t_common)

    # Feil (differansesignal)
    e = x1_interp - x2_interp

    # Grunnstatistikk
    rmse = np.sqrt(np.mean(e**2))
    corr = np.corrcoef(x1_interp, x2_interp)[0, 1]
    mean_error = np.mean(e)
    mean1 = np.mean(x1_interp)
    mean2 = np.mean(x2_interp)

    # Varians (sample-varians, ddof=1)
    var1 = np.var(x1_interp, ddof=1)
    var2 = np.var(x2_interp, ddof=1)
    var_diff = var1 - var2
    var_error = np.var(e, ddof=1)
    var_ratio = var1 / var2 if var2 != 0 else np.inf

    # Normalisert spredning (Coefficient of Variation)
    cv1 = np.sqrt(var1) / np.abs(np.mean(x1_interp)) if np.mean(x1_interp) != 0 else np.nan
    cv2 = np.sqrt(var2) / np.abs(np.mean(x2_interp)) if np.mean(x2_interp) != 0 else np.nan

    # Kurtosis (bruk bias=False for bedre estimat, fisher=True gir "excess kurtosis")
    kurt1 = stats.kurtosis(x1_interp, fisher=True, bias=False)
    kurt2 = stats.kurtosis(x2_interp, fisher=True, bias=False)
    kurt_diff = kurt1 - kurt2

    output = {
        'rmse': rmse,
        'corr': corr,
        'mean1': mean1,
        'mean2': mean2,
        'mean_error': mean_error,

        # Varians
        'var_signal1': var1,
        'var_signal2': var2,
        'var_difference': var_diff,
        'var_ratio_s1_over_s2': var_ratio,
        # 'var_error_signal': var_error,   # varians til (x1 - x2)

        # Normalisert spredning
        'cv_signal1': cv1,
        'cv_signal2': cv2,

        # Kurtosis
        'kurtosis_signal1': kurt1,
        'kurtosis_signal2': kurt2,
        'kurtosis_difference': kurt_diff,
    }
    return output




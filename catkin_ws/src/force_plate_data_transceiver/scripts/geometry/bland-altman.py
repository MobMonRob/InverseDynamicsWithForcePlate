# The source is https://stackoverflow.com/a/70652576/7817074

import matplotlib.pyplot as plt
import numpy as np
from numpy.random import random
import pandas as pd

def bland_altman_plot(data1, data2, *args, **kwargs):
    data1     = np.asarray(data1)
    data2     = np.asarray(data2)
    mean      = np.mean([data1, data2], axis=0)
    diff      = data1 - data2                   # Difference between data1 and data2
    md        = np.mean(diff)                   # Mean of the difference
    sd        = np.std(diff, axis=0)            # Standard deviation of the difference
    CI_low    = md - 1.96*sd
    CI_high   = md + 1.96*sd

    plt.scatter(mean, diff, *args, **kwargs)
    plt.axhline(md,           color='black', linestyle='-')
    plt.axhline(md + 1.96*sd, color='gray', linestyle='--')
    plt.axhline(md - 1.96*sd, color='gray', linestyle='--')
    return md, sd, mean, CI_low, CI_high

# Read data from fist source
force_plate_data = pd.read_csv('folded_CoP_force_plate_sma.csv')
data1 = force_plate_data.iloc[:, 2]

# Read data from second source
vicon_data = pd.read_csv('Marker_global_translation_filter_9.csv')
data2 = vicon_data.iloc[:, 2]

md, sd, mean, CI_low, CI_high = bland_altman_plot(data1, data2)
plt.title(r"$\mathbf{Bland-Altman}$" + " " + r"$\mathbf{Plot}$")
plt.xlabel("Means")
plt.ylabel("Difference")
plt.ylim(md - 3.5*sd, md + 3.5*sd)

xOutPlot = np.min(mean) + (np.max(mean)-np.min(mean))*1.14

plt.text(xOutPlot, md - 1.96*sd, 
    r'-1.96SD:' + "\n" + "%.2f" % CI_low, 
    ha = "center",
    va = "center",
    )
plt.text(xOutPlot, md + 1.96*sd, 
    r'+1.96SD:' + "\n" + "%.2f" % CI_high, 
    ha = "center",
    va = "center",
    )
plt.text(xOutPlot, md, 
    r'Mean:' + "\n" + "%.2f" % md, 
    ha = "center",
    va = "center",
    )
plt.subplots_adjust(right=0.85)
plt.show()
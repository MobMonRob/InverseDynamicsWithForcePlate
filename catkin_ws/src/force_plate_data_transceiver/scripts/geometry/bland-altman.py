# The source is https://stackoverflow.com/a/70652576/7817074

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

def bland_altman_plot(data1, data2, *args, **kwargs):
    data1     = np.asarray(data1)
    data2     = np.asarray(data2)
    mean      = np.mean([data1, data2], axis = 0)
    diff      = data1 - data2                   # Difference between data1 and data2
    md        = np.mean(diff)                   # Mean of the difference
    sd        = np.std(diff, axis = 0)          # Standard deviation of the difference
    CI_low    = md - 1.96 * sd
    CI_high   = md + 1.96 * sd

    plt.scatter(mean, diff, *args, **kwargs)
    plt.axhline(md,             color = 'red',   linestyle='-')
    plt.axhline(md + 1.96 * sd, color = 'green', linestyle='--')
    plt.axhline(md - 1.96 * sd, color = 'green', linestyle='--')
    return md, sd, mean, CI_low, CI_high

def read_data(data1Path : str, data2Path : str,
              data1ColumnName, data2ColumnName,
              shouldSynchronize : bool, dataSynchronizationColumnName : str,
              dataScalingFactor : int) -> pd.Series:
    data1Csv = pd.read_csv(data1Path)
    data2Csv = pd.read_csv(data2Path)

    if shouldSynchronize:
        print(f"First data measurement count BEFORE synchronization: {len(data1Csv)}")
        print(f"Second data measurement count BEFORE synchronization: {len(data2Csv)}")

        # Find the first and last values in column columnName in data1 and data2
        beginValue = max(data1Csv[dataSynchronizationColumnName].iloc[0], data2Csv[dataSynchronizationColumnName].iloc[0])
        endValue = min(data1Csv[dataSynchronizationColumnName].iloc[-1], data2Csv[dataSynchronizationColumnName].iloc[-1])

        # Filter data1 and data2 to keep only rows between beginValue and endValue
        data1Csv = data1Csv[(data1Csv[dataSynchronizationColumnName] >= beginValue) & (data1Csv[dataSynchronizationColumnName] <= endValue)]
        data2Csv = data2Csv[(data2Csv[dataSynchronizationColumnName] >= beginValue) & (data2Csv[dataSynchronizationColumnName] <= endValue)]

        print(f"Data was synchronized on column {dataSynchronizationColumnName} between values {beginValue} and {endValue}")

    if (dataScalingFactor != 1):
        data1 : pd.Series = data1Csv[data1ColumnName].apply(lambda x : float(x) * dataScalingFactor)
        data2 : pd.Series = data2Csv[data2ColumnName].apply(lambda x : float(x) * dataScalingFactor)
        print(f"Values under {data1ColumnName} from first data and values under {data2ColumnName} from second data are scaled with the factor of {dataScalingFactor}")
    else:
        data1 : pd.Series = data1Csv[data1ColumnName]
        data2 : pd.Series = data2Csv[data2ColumnName]

    print(f"First data measurement count: {len(data1)}")
    print(f"Second data measurement count: {len(data2)}")

    return data1, data2

def generate_bland_altman_plot(data1Path: str, data2Path: str, data1ColumnName: str, data2ColumnName: str,
                               dataSynchronizationColumnName : str, shouldSynchronize: bool, dataScalingFactor: int,
                               firstDataName: str, secondDataName: str, additionalComment: str, units: str):

    dataset1, dataset2 = read_data(data1Path, data2Path,
                                   data1ColumnName, data2ColumnName,
                                   shouldSynchronize, dataSynchronizationColumnName,
                                   dataScalingFactor)

    md, sd, mean, CI_low, CI_high = bland_altman_plot(dataset1, dataset2, marker=".", linewidths=1)

    meanString = "Mittelwert"
    standardDeviationString = "$\sigma$"
    xLabelString = f"Mittelwert zweier Messungen {units}"
    yLabelString = f"Differenz zweier Messungen {units}"

    plotDescription = f"{firstDataName} vs. {secondDataName}" + f"{additionalComment}"

    plt.title(r"$\mathbf{Bland-Altman-Diagramm}$" + "\n" + plotDescription)
    plt.xlabel(xLabelString)
    plt.ylabel(yLabelString)
    plt.ylim(md - 3.5 * sd, md + 3.5 * sd)

    xOutPlot = np.min(mean) + (np.max(mean) - np.min(mean)) * 1.15

    plt.text(xOutPlot, md + 1.96 * sd,
             rf"+1.96{standardDeviationString}:" + "\n" + "%.4f" % CI_high,
             ha="center",
             va="center",
             )
    plt.text(xOutPlot, md,
             rf'{meanString}:' + "\n" + "%.4f" % md,
             ha="center",
             va="center",
             )
    plt.text(xOutPlot, md - 1.96 * sd,
             rf"-1.96{standardDeviationString}:" + "\n" + "%.4f" % CI_low,
             ha="center",
             va="center",
             )
    plt.subplots_adjust(right=0.85)

    plt.gcf().set_size_inches(6.5, 4.5)
    plt.savefig(f"img/BAD_{plotDescription}.svg", format="svg")
    plt.savefig(f"img/BAD_{plotDescription}.png", format="png")
    plt.show()

def main():
    data1Path = "folded_CoP_force_plate_sma.csv"
    firstDataName = f"CoP Kraftmessplatte"
    data1ColumnName = 'field.x_m'

    data2Path = "Marker_global_translation_filter_9.csv"
    secondDataName = f"CoP Marker an der Spitze"
    data2ColumnName = 'field.x_m'

    additionalComment = ""
    units = "[mm]"
    dataScalingFactor = 1000
    shouldSynchronize = True
    dataSynchronizationColumnName = "field.frameNumber"

    generate_bland_altman_plot(data1Path, data2Path,
                               data1ColumnName, data2ColumnName,
                               dataSynchronizationColumnName, shouldSynchronize,
                               dataScalingFactor, firstDataName, secondDataName, additionalComment, units)

if __name__ == "__main__":
    main()
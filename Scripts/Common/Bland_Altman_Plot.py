# Inspired by https://stackoverflow.com/a/70652576/7817074

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from math import sqrt
from Common.geometry_classes import Point2D, Point3D


def plot_x_and_y(frameNumber_to_CoP_force_plate_corner: "dict[int, Point2D]", frameNumber_to_CoP_marker: "dict[str, Point3D]", plotSaveDir: str):
    # plot x
    CoP_force_plate_list: list[float] = []
    CoP_marker_list: list[float] = []
    for frameNumber in frameNumber_to_CoP_marker.keys():
        CoP_force_plate: Point2D = frameNumber_to_CoP_force_plate_corner.get(frameNumber)
        CoP_marker: Point3D = frameNumber_to_CoP_marker.get(frameNumber)
        CoP_force_plate_list.append(CoP_force_plate.x)
        CoP_marker_list.append(CoP_marker.x_m)
    
    CoP_force_plate_list = scale(CoP_force_plate_list, 1000)
    CoP_marker_list = scale(CoP_marker_list, 1000)

    generate_bland_altman_plot(data1=CoP_force_plate_list, data2=CoP_marker_list, dataName1="CoP Kraftmessplatte", dataName2="CoP Überschneidung", units="[mm]", saveDir=plotSaveDir, additionalComment="(x-Achse)")

    # plot y
    CoP_force_plate_list: list[float] = []
    CoP_marker_list: list[float] = []
    for frameNumber in frameNumber_to_CoP_marker.keys():
        CoP_force_plate: Point2D = frameNumber_to_CoP_force_plate_corner.get(frameNumber)
        CoP_marker: Point3D = frameNumber_to_CoP_marker.get(frameNumber)
        CoP_force_plate_list.append(CoP_force_plate.y)
        CoP_marker_list.append(CoP_marker.y_m)

    CoP_force_plate_list = scale(CoP_force_plate_list, 1000)
    CoP_marker_list = scale(CoP_marker_list, 1000)

    generate_bland_altman_plot(data1=CoP_force_plate_list, data2=CoP_marker_list, dataName1="CoP Kraftmessplatte", dataName2="CoP Überschneidung", units="[mm]", saveDir=plotSaveDir, additionalComment="(y-Achse)")

    return


def scale(data: 'list[float]', factor: int) -> "list[float]":
    if factor % 10 != 0:
        raise RuntimeError("factor % 10 != 0")
    
    df = pd.DataFrame(data=data, columns=["data"])
    series = df["data"].apply(lambda x: float(x) * factor)
    return list(series)


def generate_bland_altman_plot(data1: 'list[float]', data2: "list[float]",
                               dataName1: str, dataName2: str,
                               units: str,
                               saveDir: str,
                               additionalComment: str = ""):

    if (len(data1) != len(data2)):
        raise RuntimeError("len(data1) != len(data2)")

    md, sd, mean, CI_low, CI_high = __bland_altman_plot(data1, data2, marker=".", linewidths=0.1)

    meanString = "Mittelwert"
    standardDeviationString = "$\sigma$"
    xLabelString = f"Mittelwert zweier Messungen {units}"
    yLabelString = f"Differenz zweier Messungen {units}"

    plotDescription = f"{dataName1} vs. {dataName2}"
    if len(additionalComment) > 0:
        plotDescription = plotDescription + f" {additionalComment}"

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

    sizeFactor: float = 5
    plt.gcf().set_size_inches(w=sqrt(2) * sizeFactor, h=1 * sizeFactor)
    plt.savefig(f"{saveDir}BAP_{plotDescription}.svg", format="svg")
    plt.savefig(f"{saveDir}BAP_{plotDescription}.png", format="png")

    # Needed for saving
    plt.ion()
    plt.show()
    plt.close()

    return


def __bland_altman_plot(data1, data2, *args, **kwargs):
    data1 = np.asarray(data1)
    data2 = np.asarray(data2)
    mean = np.mean([data1, data2], axis=0)
    diff = data1 - data2                   # Difference between data1 and data2
    md = np.mean(diff)                   # Mean of the difference
    sd = np.std(diff, axis=0)          # Standard deviation of the difference
    CI_low = md - 1.96 * sd
    CI_high = md + 1.96 * sd

    plt.scatter(mean, diff, *args, **kwargs)
    plt.axhline(md,             color='red',   linestyle='-')
    plt.axhline(md + 1.96 * sd, color='green', linestyle='--')
    plt.axhline(md - 1.96 * sd, color='green', linestyle='--')
    return md, sd, mean, CI_low, CI_high


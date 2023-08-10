# Inspired by https://stackoverflow.com/a/70652576/7817074

import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import pandas as pd
from math import sqrt
from Common.geometry_classes import Point2D, Point3D
from dataclasses import dataclass
from typing import Iterator, Tuple
import math
import statistics
from reloading import reloading


@dataclass
class BAP_set(object):
    # len(x1) == len(x2)
    x1: "list[float]"
    x2: "list[float]"


@dataclass
class BAP_config(object):
    sets: "list[BAP_set]"
    colors: Iterator
    dataName1: str
    dataName2: str
    units: str
    additionalComment: str
    plotSaveDir: str


def prepare_CoP_data(frameNumber_to_CoP_force_plate_corner: "dict[int, Point2D]", frameNumber_to_CoP_marker: "dict[str, Point3D]"):
    CoP_force_plate_x: list[float] = []
    CoP_marker_x: list[float] = []
    CoP_force_plate_y: list[float] = []
    CoP_marker_y: list[float] = []
    for frameNumber in frameNumber_to_CoP_marker.keys():
        CoP_force_plate: Point2D = frameNumber_to_CoP_force_plate_corner.get(frameNumber)
        CoP_marker: Point3D = frameNumber_to_CoP_marker.get(frameNumber)
        CoP_force_plate_x.append(CoP_force_plate.x)
        CoP_marker_x.append(CoP_marker.x_m)
        CoP_force_plate_y.append(CoP_force_plate.y)
        CoP_marker_y.append(CoP_marker.y_m)

    factor: int = 1000
    CoP_force_plate_x = __scale(CoP_force_plate_x, factor)
    CoP_marker_x = __scale(CoP_marker_x, factor)
    CoP_force_plate_y = __scale(CoP_force_plate_y, factor)
    CoP_marker_y = __scale(CoP_marker_y, factor)

    x: BAP_set = BAP_set(x1=CoP_force_plate_x, x2=CoP_marker_x)
    y: BAP_set = BAP_set(x1=CoP_force_plate_y, x2=CoP_marker_y)

    return x, y


def __scale(data: 'list[float]', factor: int) -> "list[float]":
    if factor % 10 != 0:
        raise RuntimeError("factor % 10 != 0")

    df = pd.DataFrame(data=data, columns=["data"])
    series = df["data"].apply(lambda x: float(x) * factor)
    return list(series)


def generate_bland_altman_plot(config: BAP_config, showplot: bool = False):
    means, diffs = __plot_sets(sets=config.sets, colors=config.colors)

    md = np.mean(diffs)          # Mean of the difference
    sd = np.std(diffs, axis=0)   # Standard deviation of the difference
    CI_low = md - 1.96 * sd
    CI_high = md + 1.96 * sd
    plt.axhline(md,             color='k',   linestyle='-')
    plt.axhline(md + 1.96 * sd, color='k', linestyle='--')
    plt.axhline(md - 1.96 * sd, color='k', linestyle='--')

    meanString = "Mittelwert"
    standardDeviationString = "$\sigma$"
    xLabelString = f"Mittelwert zweier Messungen {config.units}"
    yLabelString = f"Differenz zweier Messungen {config.units}"

    plotDescription = f"{config.dataName1} vs. {config.dataName2}"
    if len(config.additionalComment) > 0:
        plotDescription = plotDescription + f" {config.additionalComment}"

    plt.title(r"$\mathbf{Bland-Altman-Diagramm}$" + "\n" + plotDescription)
    plt.xlabel(xLabelString)
    plt.ylabel(yLabelString)
    plt.ylim(md - 3.5 * sd, md + 3.5 * sd)

    xOutPlot = np.min(means) + (np.max(means) - np.min(means)) * 1.15

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

    sizeFactor: float = 8  # 5
    plt.gcf().set_size_inches(w=sqrt(2) * sizeFactor, h=1 * sizeFactor)
    dpi: int = 200
    plt.savefig(f"{config.plotSaveDir}BAP_{plotDescription}.svg", format="svg", dpi=dpi)
    plt.savefig(f"{config.plotSaveDir}BAP_{plotDescription}.png", format="png", dpi=dpi)

    # Needed for saving
    if showplot:
        plt.ioff()
    else:
        plt.ion()
    plt.show()
    plt.close()

    return


@reloading
def __plot_sets(sets: "list[BAP_set]", colors: Iterator):
    for set in sets:
        if len(set.x1) != len(set.x2):
            raise RuntimeError("len(set.x1) != len(set.x2)")

    means_all = list()
    diffs_all = list()
    means_of_means = list()
    means_of_diffs = list()
    color_values = list()
    for set in sets:
        x1 = np.asarray(set.x1)
        x2 = np.asarray(set.x2)
        means = np.mean([x1, x2], axis=0)
        diffs = x1 - x2
        means_all.append(means)
        diffs_all.append(diffs)
        mean_of_means = np.mean(means)
        mean_of_diffs = np.mean(diffs)
        means_of_means.append(mean_of_means)
        means_of_diffs.append(mean_of_diffs)
        seg_means, seg_diffs, alphas = __segment(means=means, diffs=diffs, bin_size_m=1.0, bin_size_d=0.3, max_alpha=0.8, min_alpha=0.3)
        color = next(colors)
        color_values.append(color)
        plt.scatter(x=seg_means, y=seg_diffs, marker="_", color=color, alpha=alphas, s=50)

    plt.scatter(x=means_of_means, y=means_of_diffs, marker=".", color=color_values, alpha=0.6, s=500)

    means_all = np.concatenate(means_all)
    diffs_all = np.concatenate(diffs_all)

    return means_all, diffs_all


def __segment(means, diffs, bin_size_m: float, bin_size_d: float, max_alpha: float, min_alpha: float):
    if max_alpha < min_alpha:
        raise RuntimeError("max_alpha < min_alpha")

    bins: dict[Tuple[int, int], list[Tuple[float, float]]] = dict()

    for m, d in np.nditer([means, diffs]):
        m_bin: int = math.floor(m / bin_size_m)
        d_bin: int = math.floor(d / bin_size_d)
        key: Tuple[int, int] = (m_bin, d_bin)
        bin: list[Tuple[float, float]] = bins.get(key, None)
        if bin == None:
            bin = list()
            bins[key] = bin
        value: Tuple[float, float] = (float(m), float(d))
        bin.append(value)

    max_len = float(max([len(values) for values in bins.values()]))

    seg_means: list[float] = list()
    seg_diffs: list[float] = list()
    alphas: list[float] = list()

    for key, list_md in bins.items():
        seg_mean: float = statistics.fmean([m for m, d in list_md])
        seg_diff: float = statistics.fmean([d for m, d in list_md])
        seg_means.append(seg_mean)
        seg_diffs.append(seg_diff)
        alpha: float = (float(len(list_md)) / max_len) * (max_alpha - min_alpha) + min_alpha
        alphas.append(alpha)

    return seg_means, seg_diffs, alphas

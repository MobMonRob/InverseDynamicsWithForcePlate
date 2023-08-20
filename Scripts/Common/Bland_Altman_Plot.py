# Inspired by
# - https://stackoverflow.com/a/70652576/7817074
# - https://github.com/jaketmp/pyCompare

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
from pathlib import Path
from scipy import stats


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


def generate_bland_altman_plot(config: BAP_config, showplot: bool = False, plot_outliers=False):
    meanString = "Mittelwert"
    standardDeviationString = "$\sigma$"
    xLabelString = f"Mittelwert der Methoden {config.units}"
    yLabelString = f"Differenz der Methoden {config.units}"

    plotDescription = f"{config.dataName1} vs. {config.dataName2}"
    if len(config.additionalComment) > 0:
        plotDescription = plotDescription + f" {config.additionalComment}"

    plt.title(r"$\mathbf{Bland-Altman-Diagramm}$" + "\n" + plotDescription)
    plt.xlabel(xLabelString)
    plt.ylabel(yLabelString)

    md, sd, xOutPlot, diffs_lower_limit, diffs_upper_limit, observations = __plot_sets(sets=config.sets, colors=config.colors, plot_outliers=plot_outliers)

    if not plot_outliers:
        plt.ylim(diffs_lower_limit, diffs_upper_limit)

    limit_of_agreement = 1.96
    limit_of_agreement_low = md - limit_of_agreement * sd
    limit_of_agreement_high = md + limit_of_agreement * sd

    plt.axhline(md, color='k', linestyle='-')
    plt.axhline(limit_of_agreement_high, color='k', linestyle='--')
    plt.axhline(limit_of_agreement_low, color='k', linestyle='--')

    plt.text(xOutPlot, limit_of_agreement_high,
             rf"+1.96{standardDeviationString}:" + "\n" + "%.4f" % limit_of_agreement_high,
             ha="center",
             va="center",
             )
    plt.text(xOutPlot, md,
             rf'{meanString}:' + "\n" + "%.4f" % md,
             ha="center",
             va="center",
             )
    plt.text(xOutPlot, limit_of_agreement_low,
             rf"-1.96{standardDeviationString}:" + "\n" + "%.4f" % limit_of_agreement_low,
             ha="center",
             va="center",
             )
    plt.subplots_adjust(right=0.85)

    ci_lower, ci_mean, ci_upper = __calculateConfidenceIntervals(md=md, sd=sd, observations=observations)

    ci_color = "tab:brown"
    plt.axhspan(ci_lower[0],
                ci_lower[1],
                facecolor=ci_color, alpha=0.3)

    plt.axhspan(ci_mean[0],
                ci_mean[1],
                facecolor=ci_color, alpha=0.3)

    plt.axhspan(ci_upper[0],
                ci_upper[1],
                facecolor=ci_color, alpha=0.3)

    # plt.grid(True)

    plt.legend([i for i in range(len(config.sets))],
               loc="upper right", title="Gelenke")

    sizeFactor: float = 8  # 5
    plt.gcf().set_size_inches(w=sqrt(2) * sizeFactor, h=1 * sizeFactor)
    dpi: int = 200
    # create plotSaveDir if not exists
    Path(config.plotSaveDir).mkdir(parents=True, exist_ok=True)
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


def __calculateConfidenceIntervals(md: float, sd: float, observations: int):
    """
    Approximate method by Bland and Altman.
    """

    limitOfAgreement: float = 1.96
    confidenceInterval: float = 0.95

    ci_mean = stats.t.interval(confidenceInterval, observations-1, loc=md, scale=sd/np.sqrt(observations))

    seLoA = ((1/observations) + (limitOfAgreement**2 / (2 * (observations - 1)))) * (sd**2)
    loARange = np.sqrt(seLoA) * stats.t._ppf((1-confidenceInterval)/2., observations-1)

    ci_upper = ((md + limitOfAgreement*sd) + loARange,
                (md + limitOfAgreement*sd) - loARange)

    ci_lower = ((md - limitOfAgreement*sd) + loARange,
                (md - limitOfAgreement*sd) - loARange)

    return ci_lower, ci_mean, ci_upper


def __calculate_limits(data: "list[float]") -> Tuple[float, float]:
    mean = np.mean(data)
    std = np.std(data)
    ci = 1.96 * std
    ci_visible_factor = 1.0 + 1.0 / 3.0
    lower_limit = mean - ci_visible_factor * ci
    upper_limit = mean + ci_visible_factor * ci
    return lower_limit, upper_limit


@reloading
def __plot_sets(sets: "list[BAP_set]", colors: Iterator, plot_outliers: bool):
    for set in sets:
        if len(set.x1) != len(set.x2):
            raise RuntimeError("len(set.x1) != len(set.x2)")

    means_all: list[list[float]] = list()
    diffs_all: list[list[float]] = list()
    seg_means_all: list[list[float]] = list()
    seg_diffs_all: list[list[float]] = list()
    alphas_all: list[list[float]] = list()
    means_of_means: list[float] = list()
    means_of_diffs: list[float] = list()
    for set in sets:
        x1 = np.asarray(set.x1)
        x2 = np.asarray(set.x2)
        means = np.mean([x1, x2], axis=0)
        means_all.append(means)
        diffs = x1 - x2
        diffs_all.append(diffs)
        mean_of_means = np.mean(means)
        mean_of_diffs = np.mean(diffs)
        means_of_means.append(mean_of_means)
        means_of_diffs.append(mean_of_diffs)

        seg_means, seg_diffs, alphas = __segment(means=means, diffs=diffs, bin_size_m=1.0, bin_size_d=0.3, max_alpha=0.8, min_alpha=0.3)

        seg_means_all.append(seg_means)
        seg_diffs_all.append(seg_diffs)
        alphas_all.append(alphas)

    diffs_all_flat = np.concatenate(diffs_all)
    md_data = np.mean(diffs_all_flat)  # Mean of the difference
    sd_data = np.std(diffs_all_flat)   # Standard deviation of the difference
    diffs_lower_limit, diffs_upper_limit = __calculate_limits(diffs_all_flat)

    limited_seg_means_all = seg_means_all
    limited_seg_diffs_all = seg_diffs_all
    if not plot_outliers:
        limited_seg_means_all = list()
        limited_seg_diffs_all = list()
        for seg_means, seg_diffs in zip(seg_means_all, seg_diffs_all):
            limited_means = list()
            limited_diffs = list()
            m_ll, m_ul = __calculate_limits(seg_means)
            d_ll, d_ul = __calculate_limits(seg_diffs)
            for m, d in zip(seg_means, seg_diffs):
                # Remove means global outliers.
                if not (diffs_lower_limit < d < diffs_upper_limit):
                    continue
                # Remove means local outliers.
                if not (m_ll < m < m_ul):
                    continue
                # Remove diffs local outliers.
                if not (d_ll < d < d_ul):
                    continue
                limited_means.append(m)
                limited_diffs.append(d)
            limited_seg_means_all.append(limited_means)
            limited_seg_diffs_all.append(limited_diffs)

    color_values = list()
    for set, means, diffs, alphas in zip(sets, limited_seg_means_all, limited_seg_diffs_all, alphas_all):
        color = next(colors)
        color_values.append(color)
        plt.scatter(x=means, y=diffs, marker="_", color=color, alpha=alphas, s=50)

    plt.scatter(x=means_of_means, y=means_of_diffs, marker=".", color=color_values, alpha=0.6, s=500)

    limited_seg_means_all_flat = np.concatenate(limited_seg_means_all)
    limited_seg_means_all_flat_min = np.min(limited_seg_means_all_flat)
    limited_seg_means_all_flat_max = np.max(limited_seg_means_all_flat)
    xOutPlot = limited_seg_means_all_flat_min + (limited_seg_means_all_flat_max - limited_seg_means_all_flat_min) * 1.15

    return md_data, sd_data, xOutPlot, diffs_lower_limit, diffs_upper_limit, len(diffs_all_flat)


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

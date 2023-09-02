# Inspired by
# - https://stackoverflow.com/a/70652576/7817074
# - https://github.com/jaketmp/pyCompare

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
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
import sys


@dataclass
class BAP_set(object):
    # len(x1) == len(x2)
    x1: "list[float]"
    x2: "list[float]"


@dataclass
class BAP_legend(object):
    title: str
    color_to_label: dict


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


def generate_bland_altman_plot(config: BAP_config, showplot: bool = False, plot_outliers: bool = False, legend: BAP_legend = None):

    # Those need to be set before invocation of __plot_sets().
    sizeFactor: float = 6  # 8 | Größer <=> Kleinere Schrift
    plt.gcf().set_size_inches(w=sqrt(2) * sizeFactor, h=1 * sizeFactor)
    plt.gcf().set_dpi(300)
    plt.rcParams['figure.constrained_layout.use'] = True
    # plt.tight_layout(pad=0.0, h_pad=0.0, w_pad=0.0)

    meanString = "Mittelwert"
    standardDeviationString = "$\sigma$"
    xLabelString = f"Mittelwerte der Messmethoden [{config.units}]"
    yLabelString = f"Differenzen der Messmethoden [{config.units}]"

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
             rf"+1.96{standardDeviationString}:" + f"\n{limit_of_agreement_high:.1f} {config.units}",
             ha="left",
             va="center",
             )
    plt.text(xOutPlot, md,
             rf'{meanString}:' + f"\n{md:.1f} {config.units}",
             ha="left",
             va="center",
             )
    plt.text(xOutPlot, limit_of_agreement_low,
             rf"-1.96{standardDeviationString}:" + f"\n{limit_of_agreement_low:.1f} {config.units}",
             ha="left",
             va="center",
             )

    # ci_lower, ci_mean, ci_upper = __calculateConfidenceIntervals(md=md, sd=sd, observations=observations)

    # ci_color = "tab:brown"
    # plt.axhspan(ci_lower[0],
    #             ci_lower[1],
    #             facecolor=ci_color, alpha=0.3)

    # plt.axhspan(ci_mean[0],
    #             ci_mean[1],
    #             facecolor=ci_color, alpha=0.3)

    # plt.axhspan(ci_upper[0],
    #             ci_upper[1],
    #             facecolor=ci_color, alpha=0.3)

    plt.grid(visible=True, which="both", linestyle=':', color='k', alpha=0.5)

    if legend != None:
        patches = [mpatches.Patch(color=c, label=l) for c, l in legend.color_to_label.items()]
        plt.legend(handles=patches, loc="best", title=legend.title)

    # create plotSaveDir if not exists
    Path(config.plotSaveDir).mkdir(parents=True, exist_ok=True)
    plt.savefig(f"{config.plotSaveDir}BAP_{plotDescription}.svg", format="svg", transparent=True, pad_inches=0.005, bbox_inches="tight")
    plt.savefig(f"{config.plotSaveDir}BAP_{plotDescription}.png", format="png", transparent=True, pad_inches=0.005, bbox_inches="tight")

    # Needed for saving
    if showplot:
        plt.ioff()
        plt.show()
        plt.close()
    else:
        plt.ioff()
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


# @reloading
def __plot_sets(sets: "list[BAP_set]", colors: Iterator, plot_outliers: bool):
    for set in sets:
        if len(set.x1) != len(set.x2):
            raise RuntimeError("len(set.x1) != len(set.x2)")

    means_all: list[list[float]] = list()
    diffs_all: list[list[float]] = list()
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

    diffs_all_flat = np.concatenate(diffs_all)
    md_data = np.mean(diffs_all_flat)  # Mean of the difference
    sd_data = np.std(diffs_all_flat)   # Standard deviation of the difference
    diffs_lower_limit, diffs_upper_limit = __calculate_limits(diffs_all_flat)

    limited_means_all = means_all
    limited_diffs_all = diffs_all
    if not plot_outliers:
        limited_means_all, limited_diffs_all = remove_outliers(limited_means_all, limited_diffs_all, diffs_lower_limit, diffs_upper_limit)

    limited_means_all_flat = np.concatenate(limited_means_all)
    limited_diffs_all_flat = np.concatenate(limited_diffs_all)
    resolution: float = 100
    # Equal to aspect ratio of: plt.gcf().set_size_inches(w=sqrt(2) * sizeFactor, h=1 * sizeFactor)
    d_to_m_ratio = sqrt(2)
    bin_size_m = (np.max(limited_means_all_flat) - np.min(limited_means_all_flat)) / (resolution * d_to_m_ratio)
    bin_size_d = (np.max(limited_diffs_all_flat) - np.min(limited_diffs_all_flat)) / resolution
    # print(f"binSizes: {bin_size_m}, {bin_size_d}")
    limited_seg_means_all: list[list[float]] = list()
    limited_seg_diffs_all: list[list[float]] = list()
    alphas_all: list[list[float]] = list()
    for means, diffs in zip(limited_means_all, limited_diffs_all):
        seg_means, seg_diffs, alphas = __segment(means=means, diffs=diffs, bin_size_m=bin_size_m, bin_size_d=bin_size_d, max_alpha=0.8, min_alpha=0.3)

        limited_seg_means_all.append(seg_means)
        limited_seg_diffs_all.append(seg_diffs)
        alphas_all.append(alphas)

    limited_seg_means_all_flat = np.concatenate(limited_seg_means_all)
    limited_seg_means_all_flat_min = np.min(limited_seg_means_all_flat)
    limited_seg_means_all_flat_max = np.max(limited_seg_means_all_flat)

    limited_seg_diffs_all_flat = np.concatenate(limited_seg_diffs_all)
    limited_seg_diffs_all_flat_min = np.min(limited_seg_diffs_all_flat)
    limited_seg_diffs_all_flat_max = np.max(limited_seg_diffs_all_flat)

    # Start: Calculate marker_size
    plot_extreme_means = [limited_seg_means_all_flat_min, limited_seg_means_all_flat_min, limited_seg_means_all_flat_max, limited_seg_means_all_flat_max]
    plot_extreme_diffs = [limited_seg_diffs_all_flat_min, limited_seg_diffs_all_flat_max, limited_seg_diffs_all_flat_min, limited_seg_diffs_all_flat_max]

    ax = plt.subplot()

    # Update plot size to get correct bin_size_display_dots.
    ax.scatter(x=plot_extreme_means, y=plot_extreme_diffs, marker="o", edgecolors="none", color="black", alpha=0.0, s=30)
    plt.gcf().canvas.draw()

    dpi = plt.gcf().get_dpi()

    bin_size_display_dots = ax.transData.transform((bin_size_m, bin_size_d))-ax.transData.transform((0, 0))
    bin_size_display_inches = bin_size_display_dots / dpi
    inches_per_point = 1.0 / 72.0
    bin_size_display_points = bin_size_display_inches / inches_per_point
    marker_radius = np.min(bin_size_display_points)
    marker_size = marker_radius ** 2
    # Marker size is correct only for marker="o"
    # Stop: # Start: Calculate marker_size

    color_values = list()
    for set, means, diffs, alphas in zip(sets, limited_seg_means_all, limited_seg_diffs_all, alphas_all):
        color = next(colors)
        color_values.append(color)
        plt.scatter(x=means, y=diffs, marker="o", edgecolors="none", color=color, alpha=alphas, s=marker_size)

    plt.scatter(x=means_of_means, y=means_of_diffs, marker="o", edgecolors="black", color=color_values, alpha=0.7, s=marker_size*3**2)

    xOutPlot = limited_seg_means_all_flat_min + (limited_seg_means_all_flat_max - limited_seg_means_all_flat_min) * 1.065  # 1.15

    return md_data, sd_data, xOutPlot, diffs_lower_limit, diffs_upper_limit, len(diffs_all_flat)


def remove_outliers(means_all: "list[list[float]]", diffs_all: "list[list[float]]", diffs_lower_limit: float, diffs_upper_limit: float) -> "Tuple[list[list[float]], list[list[float]]]":
    limited_means_all: "list[list[float]]" = list()
    limited_diffs_all: "list[list[float]]" = list()
    for means, diffs in zip(means_all, diffs_all):
        limited_means: list[float] = list()
        limited_diffs: list[float] = list()
        m_ll, m_ul = __calculate_limits(means)
        d_ll, d_ul = __calculate_limits(diffs)
        for m, d in zip(means, diffs):
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
        limited_means_all.append(limited_means)
        limited_diffs_all.append(limited_diffs)
    return limited_means_all, limited_diffs_all


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
        # # Test proper alignment
        # seg_mean: float = float(key[0]) * bin_size_m
        # seg_diff: float = float(key[1]) * bin_size_d
        seg_means.append(seg_mean)
        seg_diffs.append(seg_diff)
        alpha: float = (float(len(list_md)) / max_len) * (max_alpha - min_alpha) + min_alpha
        alphas.append(alpha)

    return seg_means, seg_diffs, alphas

def default_plot_width_inches() -> float:
    return 7.25


def size_factor_screen_big() -> float:
    """
    Returns size_factor to get a svg plot with 12.325 cm width displayed on screen with firefox. Please note that this can vary significantly on your machine!
    """
    return 7.25 / default_plot_width_inches()


def size_factor_screen_small() -> float:
    """
    Returns size_factor to get a svg plot with 7.25 cm width displayed on screen with firefox. Please note that this can vary significantly on your machine!
    """
    return 4.2647 / default_plot_width_inches()

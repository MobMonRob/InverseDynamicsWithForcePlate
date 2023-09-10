# Damit das Verhältnis noch stimmt, muss für paper beim default der Wert des großen Papers eingetragen werden. Damit sehen die Plots noch gleich aus, werden nur etwas kleiner angezeigt auf dem Laptop.
# Es geht aber auch, dass man die Plots so vorbereitet, wie sie auf dem Laptop aussehen. Und dann das im Latex so skaliert, dass es in der Realität gleich aussieht.
def default_plot_width_inches() -> float:
    return 7.25  # 6.0


def size_factor_paper_big() -> float:
    """
    Returns size_factor to get a svg plot with 12.325 cm width processed with latex and printed on DINA4 paper. Please note that this can vary significantly on your machine!
    """
    return 5.1 / default_plot_width_inches()


def size_factor_paper_small() -> float:
    """
    Returns size_factor to get a svg plot with 7.25 cm width processed with latex and printed on DINA4 paper. Please note that this can vary significantly on your machine!
    """
    return 3.0 / default_plot_width_inches()


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

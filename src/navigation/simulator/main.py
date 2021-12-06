import typing

import matplotlib as mpl
from matplotlib import animation, patches
import matplotlib.pyplot as plt


class Car:
    # Position
    x: float = 0
    y: float = 0

    # Velocity
    dx: float = 0
    dy: float = 0

    # Acceleration
    ddx: float = 0
    ddy: float = 0

    # Heading
    theta: float = 0

    # Angular Velocity
    omega: float = 0

    # Angular Acceleration
    alpha: float = 0

    width: float = 1250
    height: float = 2793

    _body: patches.Rectangle

    def __init__(self) -> None:
        self._body = patches.Rectangle(
            xy=(self.x, self.y), width=self.width, height=self.height, fc="black")


class Simulator:
    def __init__(self) -> None:
        pass

    def init(self):
        pass

    def update():
        pass


def main():
    sim = Simulator()
    fig = plt.figure()
    anim = animation.FuncAnimation(
        fig, sim.update, init_func=sim.init, interval=1000, blit=True)
    plt.show()


if __name__ == "__main__":
    main()

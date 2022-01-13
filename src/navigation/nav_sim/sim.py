from math import sin, cos, tan

from matplotlib import animation, patches, transforms, pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure

from typing import Tuple

"""
All UNITS are in SI - meters, seconds and radians.
"""

class Wheel:
    x_offset: float  # m
    y_offset: float  # m

    steering: bool

    length: float = 0.4  # m
    width: float = 0.2  # m

    _patch: patches.Rectangle

    def __init__(self, x_offset: float, y_offset: float, steering: bool) -> None:
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.steering = steering
        self._patch = patches.Rectangle(
            xy=(0, 0),
            width=self.length,
            height=self.width,
            fc="black"
        )
    
    def drawing_init(self, ax: Axes) -> Tuple:
        ax.add_patch(self._patch)
        return (self._patch,)
    
    def drawing_update(self, ax: Axes, car_x: float, car_y: float, car_heading: float, steering_angle: float) -> Tuple:
        center_x = car_x + self.x_offset
        center_y = car_y + self.y_offset

        self._patch.set_xy((center_x - (self.length / 2), center_y - (self.width / 2)))

        if self.steering:
            rot_t = transforms.Affine2D().rotate_around(center_x, center_y, car_heading + steering_angle) + ax.transData
            self._patch.set_transform(rot_t)

        return (self._patch,)


class Car:
    # x, y are on the back axel of the car
    x: float = 0  # m
    y: float = 0  # m
    heading: float = 0  # rad

    forward_vel: float = 0.1  # m/s
    steering_angle: float = 0  # rad

    wheel_base: float = 2  # m
    width: float = 0.7  # m

    fl_wheel: Wheel

    _body_patch: patches.Rectangle

    @property
    def drawing_x(self):
        return self.x
    
    @property
    def drawing_y(self):
        return self.y - (self.width / 2)

    def __init__(self) -> None:
        self.fl_wheel = Wheel(x_offset=self.wheel_base, y_offset=self.width/2 + Wheel.width, steering=True)
        
        self._body_patch = patches.Rectangle(
            xy=(self.drawing_x, self.drawing_y),
            width=self.wheel_base,
            height=self.width,
            fc="orange"
        )
    
    def drawing_init(self, ax: Axes) -> Tuple:
        ax.add_patch(self._body_patch)
        val = (self._body_patch,)
        val += self.fl_wheel.drawing_init(ax)
        return val

    def drawing_update(self, dt_: float, ax: Axes) -> Tuple:
        self._update_state(dt_)

        self._body_patch.set_xy((self.drawing_x, self.drawing_y))
        rot_t = transforms.Affine2D().rotate_around(self.x, self.y, self.heading) + ax.transData
        self._body_patch.set_transform(rot_t)

        val = (self._body_patch,)
        val += self.fl_wheel.drawing_update(ax, self.x, self.y, self.heading, self.steering_angle)
        return val


    def _update_state(self, dt_: float):
        # page 101, equation 4.2 of Robotics, Vision and Control
        x_dot = self.forward_vel * cos(self.heading)
        y_dot = self.forward_vel * sin(self.heading)
        heading_dot = (self.forward_vel / self.wheel_base) * tan(self.steering_angle)

        self.x += x_dot * dt_
        self.y += y_dot * dt_
        self.heading += heading_dot * dt_


class SimState:
    dt_: float = 1.0 / 30  # seconds

    fig: Figure
    ax: Axes
    
    car: Car

    def __init__(self) -> None:
        self.fig = plt.figure()  # create a figure object
        self.ax = self.fig.add_subplot(1, 1, 1)  # create an axes object in the figure

        self.ax.set_xlim(-5, 5)  # 10m wide
        self.ax.set_ylim(-5, 5)  # 10m high
        self.ax.set_axisbelow(True)
        self.ax.grid(True)

        self.car = Car()
    
    def drawing_init(self):
        val: Tuple = ()
        val += self.car.drawing_init(self.ax)
        return val
    
    def drawing_update(self, i: int):
        val: Tuple = ()
        val += self.car.drawing_update(self.dt_, self.ax)
        return val

if __name__ == "__main__":
    state = SimState()

    interval = 980 * state.dt_  # run a little quicker than 1000 ms in a second, to account for slow processing
    
    _ = animation.FuncAnimation(
        fig=state.fig,
        func=state.drawing_update,
        init_func=state.drawing_init,
        interval=interval,
        blit=True
    )
    plt.show()

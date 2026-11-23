import asyncio
import contextlib
import datetime
import functools
import typing

import attrs
import builtin_interfaces.msg
import rclpy.callback_groups
import rclpy.clock
import rclpy.node
import rclpy.qos
import rclpy.time
import rclpy.timer
import rosgraph_msgs.msg
from typing_extensions import Self

_CLOCK_QOS = rclpy.qos.QoSProfile(
    depth=1,
    reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
    durability=rclpy.qos.DurabilityPolicy.VOLATILE,
    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
)


@functools.total_ordering
@attrs.define
class Time:
    """
    Wrapper for builtin_interfaces.msg.Time
    """

    sec: int = attrs.field(converter=int, default=0)
    nanosec: int = attrs.field(converter=int, default=0)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, type(self)):
            other = self.parse(other)  # type: ignore
        return self.sec == other.sec and self.nanosec == other.nanosec

    def __lt__(self, other: object) -> bool:
        if not isinstance(other, type(self)):
            other = self.parse(other)  # type: ignore
        return self.sec < other.sec or self.nanosec < other.nanosec

    def __add__(self, other: object) -> Self:
        if not isinstance(other, type(self)):
            other = self.parse(other)  # type: ignore

        return self.from_float(self.to_seconds() + other.to_seconds())

    def __sub__(self, other: object) -> Self:
        if not isinstance(other, type(self)):
            other = self.parse(other)  # type: ignore
        new_time = self.to_seconds() - other.to_seconds()
        if new_time < 0:
            raise ValueError('Subtraction leads to negative time.')
        return self.from_float(new_time)

    # Parsing

    @classmethod
    def from_rclpy(cls, v: rclpy.time.Time) -> Self:
        """
        Create instance from rclpy.time.Time object.
        """
        sec, nanosec = v.seconds_nanoseconds()
        return cls(
            sec=sec,
            nanosec=nanosec,
        )

    @classmethod
    def from_msg(cls, v: builtin_interfaces.msg.Time) -> Self:
        """
        Create instance from builtin_interfaces.msg.Time object.
        """
        return cls(
            sec=v.sec,
            nanosec=v.nanosec,
        )

    @classmethod
    def from_rosgraph_msg(cls, v: rosgraph_msgs.msg.Clock) -> Self:
        """
        Create instance from rosgraph_msgs.msg.Clock object.
        """
        return cls.from_msg(v.clock)

    @classmethod
    def from_float(cls, v: float) -> Self:
        """
        Create instance from float seconds.
        """
        sec = int(v)
        nanosec = int((v - sec) * 1e9)
        return cls(
            sec=sec,
            nanosec=nanosec,
        )

    @classmethod
    def parse(cls, v: builtin_interfaces.msg.Time | rosgraph_msgs.msg.Clock | rclpy.time.Time | float) -> Self:
        """s.fr
        Create instance from either builtin_interfaces.msg.Time or rclpy.time.Time object.
        """
        if isinstance(v, builtin_interfaces.msg.Time):
            return cls.from_msg(v)
        elif isinstance(v, rosgraph_msgs.msg.Clock):
            return cls.from_rosgraph_msg(v)
        elif isinstance(v, rclpy.time.Time):
            return cls.from_rclpy(v)
        elif isinstance(v, (int, float)):
            return cls.from_float(v)
        else:
            raise TypeError(f'Cannot parse Time from type: {type(v)}')

    # Converting

    def to_rclpy(self) -> rclpy.time.Time:
        """
        Create rclpy.time.Time from self.
        """
        return rclpy.time.Time(
            seconds=self.sec,
            nanoseconds=self.nanosec,
        )

    def to_msg(self) -> builtin_interfaces.msg.Time:
        """
        Create builtin_interfaces.msg.Time from self.
        """
        return builtin_interfaces.msg.Time(
            sec=self.sec,
            nanosec=self.nanosec,
        )

    def to_rosgraph_msg(self) -> rosgraph_msgs.msg.Clock:
        """
        Create rosgraph_msgs.msg.Clock from self.
        """
        return rosgraph_msgs.msg.Clock(clock=self.to_msg())

    def to_seconds(self) -> float:
        """
        Convert to seconds
        """
        return self.sec + self.nanosec / 1e9


class TimeNode(rclpy.node.Node):
    """Mixin class to provide clock utilities for rclpy nodes."""

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self.__clock_subscriber = self.create_subscription(
            rosgraph_msgs.msg.Clock,
            '/clock',
            self.__clock_callback,
            _CLOCK_QOS,
        )
        self.__sim_time: Time = Time()
        self.__wall_clock: rclpy.clock.Clock | None = None

    def __clock_callback(self, msg: rosgraph_msgs.msg.Clock):
        """Callback for /clock topic to update node clock when using simulated time.

        Args:
            msg (rosgraph_msgs.msg.Clock): Clock message
        """
        self.__sim_time = Time.from_rosgraph_msg(msg)

    @property
    def sim_time(self) -> Time:
        """Get the latest simulated time received from /clock topic.

        Returns:
            Time: Simulated time
        """
        return Time(self.__sim_time.sec, self.__sim_time.nanosec)

    @property
    def wall_time(self) -> Time:
        """Get the current wall time.

        Returns:
            Time: Wall time
        """
        now = datetime.datetime.now()
        sec = int(now.timestamp())
        nanosec = int(now.microsecond * 1e3)
        return Time(sec=sec, nanosec=nanosec)

    @property
    def time(self) -> Time:
        """Get the current time. Sim time if using ROS time, wall time otherwise.

        Returns:
            Time: Current time
        """
        now = self.get_clock().now()
        sec, nanosec = now.seconds_nanoseconds()
        return Time(sec=sec, nanosec=nanosec)

    @property
    def wall_clock(self) -> rclpy.clock.Clock:
        """Steady wall clock, immune to NTP adjustments. Memoized; reusable across timers."""
        if self.__wall_clock is None:
            self.__wall_clock = rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME)
        return self.__wall_clock

    def wall_timer(
        self,
        period_sec: float,
        callback: typing.Callable[[], None],
        *,
        callback_group: rclpy.callback_groups.CallbackGroup | None = None,
    ) -> rclpy.timer.Timer:
        """Create a timer driven by the steady wall clock, regardless of the node's `use_sim_time`."""
        return self.create_timer(
            period_sec,
            callback,
            clock=self.wall_clock,
            callback_group=callback_group,
        )

    @contextlib.contextmanager
    def sim_time_rate(self, rate: float, lifetime: float | None = None) -> typing.Generator[tuple[asyncio.Event, asyncio.Queue[float]], None, None]:
        """Context manager to perform a task at a given simulated time rate.

        Usage:
            ```
            with node.sim_time_rate(10.0) as (done, rate):
                while not done.is_set():
                    dt = await rate.get()
                    # do something
            ```

        Can be canceled by setting the `done` event or exiting the context.

        Args:
            rate (float): Rate in Hz to perform the task.
            lifetime (float | None): Optional lifetime in seconds for the rate context.

        Yields:
            asyncio.Queue: Queue that yields the time delta in seconds at each rate interval.
        """
        interval = 1.0 / rate
        # Sim time advances in physics steps (e.g. 0.0333 s), so "fire once `interval` has passed since
        # the last tick" rounds every tick up to the next step: 10 Hz became 7.5 Hz (3 steps = 0.0999 s
        # < 0.1 s, so each tick waited for a 4th step) and 30 Hz became 15 Hz. Ticks are instead due on a
        # fixed grid (start + k * interval), so the average rate is exact and only single intervals jitter
        # by up to one step. The tolerance absorbs float error such as 3 * 0.0333 = 0.0999.
        tolerance = 0.01 * interval
        start = self.sim_time.to_seconds()
        last = start
        next_due = start + interval
        finish = start + lifetime if lifetime is not None else None

        done = asyncio.Event()
        events: asyncio.Queue[float] = asyncio.Queue(maxsize=1000)

        async def _rate_loop():
            nonlocal last, next_due, finish
            while not done.is_set():
                await asyncio.sleep(0.01)
                now = self.sim_time.to_seconds()
                if now < last:  # sim time went backwards (sim restart): restart the grid from here
                    if finish is not None:
                        finish = now + (finish - last)
                    last, next_due = now, now + interval
                dt = now - last
                if is_put := now >= next_due - tolerance:
                    last = now
                    next_due += interval
                    if now >= next_due - tolerance:  # more than a whole interval late: skip ahead, don't burst
                        next_due = now + interval
                    await events.put(dt)
                if finish is not None and now >= finish:
                    done.set()
                    if not is_put:
                        await events.put(dt)
                    break

        events.put_nowait(0.0)
        loop_task = asyncio.create_task(_rate_loop())
        try:
            yield done, events
        finally:
            loop_task.cancel()

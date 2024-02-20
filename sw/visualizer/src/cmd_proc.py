from visualizer import Viz, Point, Line, Rectangle, Circle, Rgb
from protocol import ClearScreen, ClearLayer, DrawPoint, DrawLine, DrawRectangle, DrawCircle, Command
from abc import ABC, abstractmethod
from typing import Callable


class Proc(ABC):
    """A command processor."""

    @abstractmethod
    def process(self, visualizer: Viz) -> None:
        """Call the processor.

        Args:
            visualizer: The Visualizer.
        """


class ClearScreenProc(Proc):
    """Clear the screen."""

    def __init__(self, cmd: ClearScreen):
        self.cmd = cmd

    def process(self, visualizer: Viz) -> None:
        """Call the ClearScreen.

        Args:
            visualizer: The Visualizer.
        """

        visualizer.clear_screen()


class ClearLayerProc(Proc):
    """Clear a layer."""

    def __init__(self, cmd: ClearLayer):
        self.cmd = cmd

    def process(self, visualizer: Viz) -> None:
        """Call the ClearLayer.

        Args:
            visualizer: The Visualizer.
        """

        visualizer.clear_layer(self.cmd.layer.value)


class DrawPointProc(Proc):
    """Draw a point."""

    def __init__(self, cmd: DrawPoint):
        self.cmd = cmd

    def process(self, visualizer: Viz) -> None:
        """Call the DrawPoint.

        Args:
            visualizer: The Visualizer.
        """

        visualizer.draw(Point(self.cmd.x.value, self.cmd.y.value, Rgb(self.cmd.color.to_rgb888()), self.cmd.thickness.value),
                        self.cmd.layer.value)


class DrawLineProc(Proc):
    """Draw a line."""

    def __init__(self, cmd: DrawLine):
        self.cmd = cmd

    def process(self, visualizer: Viz) -> None:
        """Call the DrawLine.

        Args:
            visualizer: The Visualizer.
        """

        visualizer.draw(Line(
            Point(self.cmd.x1.value, self.cmd.y1.value),
            Point(self.cmd.x2.value, self.cmd.y2.value),
            Rgb(self.cmd.color.to_rgb888()),
            self.cmd.width.value
        ), self.cmd.layer.value)


class DrawRectangleProc(Proc):
    """Draw a rectangle."""

    def __init__(self, cmd: DrawRectangle):
        self.cmd = cmd

    def process(self, visualizer: Viz) -> None:
        """Call the DrawRectangle.

        Args:
            visualizer: The Visualizer.
        """

        visualizer.draw(Rectangle(
            Point(self.cmd.x1.value, self.cmd.y1.value),
            Point(self.cmd.x2.value, self.cmd.y2.value),
            Rgb(self.cmd.color.to_rgb888()),
            self.cmd.width.value
        ), self.cmd.layer.value)


class DrawCircleProc(Proc):
    """Draw a circle."""

    def __init__(self, cmd: DrawCircle):
        self.cmd = cmd

    def process(self, visualizer: Viz) -> None:
        """Call the DrawCircle.

        Args:
            visualizer: The Visualizer.
        """

        visualizer.draw(Circle(
            Point(self.cmd.x.value, self.cmd.y.value),
            self.cmd.r.value,
            Rgb(self.cmd.color.to_rgb888()),
            self.cmd.width.value
        ), self.cmd.layer.value)


PROCESSORS: dict[type[Command], type[Proc]] = {
    ClearScreen: ClearScreenProc,
    ClearLayer: ClearLayerProc,
    DrawPoint: DrawPointProc,
    DrawLine: DrawLineProc,
    DrawRectangle: DrawRectangleProc,
    DrawCircle: DrawCircleProc,
}


def cmd_proc(cmd: Command) -> Callable[[Viz], None]:
    """Process a command.

    Args:
        cmd: The command to process.
        visualizer: The Visualizer.
    """

    Processor = PROCESSORS.get(type(cmd))
    if Processor:
        proc = Processor(cmd)  # type: ignore
        return proc.process
    else:
        raise Exception(f"Unsupported command: {cmd}")

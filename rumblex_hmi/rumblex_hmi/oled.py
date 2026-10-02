"""Shared monochrome OLED renderer and backend contract (no ROS or hardware imports)."""

from typing import Protocol

from PIL import Image, ImageDraw, ImageFont


class HmiBackend(Protocol):
    """Platform operations used by the shared ROS node."""

    def present(self, image: Image.Image) -> None: ...

    def read_power(self) -> tuple[float | None, float | None]: ...

    def set_relay(self, enabled: bool) -> None: ...

    def shutdown_host(self, logger) -> None: ...

    def close(self) -> None: ...


class OledRenderer:
    SIZE = (128, 64)

    def __init__(self):
        self.font = ImageFont.truetype(
            '/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf', 12)
        self.line_height = self.font.getmask('SYSTEM').size[1] + 3

    def render(self, lines):
        # Rebuild the whole frame so shortened text and page changes cannot leave pixels behind.
        image = Image.new('1', self.SIZE)
        draw = ImageDraw.Draw(image)
        for index, text in enumerate(lines):
            draw.text((0, self.line_height * index), text, font=self.font, fill=255)
        return image

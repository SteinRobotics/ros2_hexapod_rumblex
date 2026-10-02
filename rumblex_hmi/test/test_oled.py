from PIL import ImageChops

from rumblex_hmi.oled import OledRenderer


def test_shorter_text_and_page_change_clear_previous_pixels():
    renderer = OledRenderer()
    renderer.render(['SYSTEM', 'long text across the display', 'movement'])
    short = renderer.render(['SYSTEM', 'IP'])
    reference = OledRenderer().render(['SYSTEM', 'IP'])
    assert ImageChops.difference(short, reference).getbbox() is None
    assert short.mode == '1'
    assert short.size == (128, 64)
    assert short.getbbox() is not None
    assert renderer.render([''] * 6).getbbox() is None
    sensors = renderer.render(['SENSORS', '', 'g: 9.81m/s^2'])
    assert ImageChops.difference(sensors, reference).getbbox() is not None

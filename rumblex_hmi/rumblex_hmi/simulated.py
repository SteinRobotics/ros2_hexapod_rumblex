"""Pygame OLED backend with synthetic power readings and an in-memory relay."""

import os

os.environ.setdefault('PYGAME_HIDE_SUPPORT_PROMPT', '1')
import pygame

from rumblex_hmi.oled import OledRenderer


class SimulatedBackend:
    SCALE = 4

    def __init__(self):
        self.relay_enabled = True
        self.power_values = lambda: (12.0, 0.0)
        try:
            pygame.display.init()
            self.screen = pygame.display.set_mode(
                tuple(value * self.SCALE for value in OledRenderer.SIZE))
            pygame.display.set_caption('RumbleX simulated OLED')
        except BaseException:
            self.close()
            raise

    def present(self, image):
        rgb = image.convert('RGB')
        surface = pygame.image.fromstring(rgb.tobytes(), rgb.size, 'RGB')
        self.screen.blit(pygame.transform.scale(surface, self.screen.get_size()), (0, 0))
        pygame.display.flip()

    def read_power(self):
        return self.power_values()

    def set_relay(self, enabled):
        self.relay_enabled = enabled

    def shutdown_host(self, logger):
        logger.info('Simulated shutdown: relay off; host remains running.')

    def process_events(self):
        return not any(event.type == pygame.QUIT for event in pygame.event.get())

    def close(self):
        self.relay_enabled = False
        pygame.display.quit()

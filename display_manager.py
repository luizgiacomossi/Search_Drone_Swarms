import pygame
from constants import SCREEN_WIDTH, SCREEN_HEIGHT

class DisplayManager(object):
    '''
        Class responsible to represent the canvas variables and zoom/pan logic
    ''' 
    def __init__(self):
        pygame.init()
        self.font16 = pygame.font.SysFont(None, 16)
        self.font20 = pygame.font.SysFont(None, 20)
        self.font24 = pygame.font.SysFont(None, 24)
        self.size = SCREEN_WIDTH, SCREEN_HEIGHT 
        self.clock = pygame.time.Clock()
        self.screen = pygame.display.set_mode(self.size)
        
        # Zoom and Pan variables
        self.zoom_level = 1.0
        self.offset = pygame.math.Vector2(0, 0)
        self.min_zoom = 1.0
        self.max_zoom = 5.0
        
        # World surface to draw everything on before scaling
        self.world_surface = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))

    def handle_zoom(self, event):
        if event.type == pygame.MOUSEWHEEL:
            old_zoom = self.zoom_level
            
            if event.y > 0:
                self.zoom_level *= 1.1
            else:
                self.zoom_level /= 1.1
                
            self.zoom_level = max(self.min_zoom, min(self.max_zoom, self.zoom_level))
            
            # Adjust offset to zoom towards mouse pointer
            mouse_x, mouse_y = pygame.mouse.get_pos()
            
            # Calculate world coordinates of mouse before zoom
            world_x = (mouse_x - self.offset.x) / old_zoom
            world_y = (mouse_y - self.offset.y) / old_zoom
            
            # Calculate new offset to keep mouse at same world coordinates
            self.offset.x = mouse_x - world_x * self.zoom_level
            self.offset.y = mouse_y - world_y * self.zoom_level
            
            # Clamp offset to keep world within screen bounds
            self.offset.x = min(0, max(SCREEN_WIDTH * (1 - self.zoom_level), self.offset.x))
            self.offset.y = min(0, max(SCREEN_HEIGHT * (1 - self.zoom_level), self.offset.y))

    def screen_to_world(self, screen_pos):
        return (screen_pos - self.offset) / self.zoom_level

    def world_to_screen(self, world_pos):
        return world_pos * self.zoom_level + self.offset

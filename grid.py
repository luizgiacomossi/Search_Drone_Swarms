from constants import *
import pygame as pg
from math import atan2, pi, exp, floor
import random
import copy 
import numpy as np
# importing "heapq" to implement heap queue
import heapq


vec = pg.math.Vector2 

NOT_VISITED = 0      
VISITED = 1
OBSTACLE = 2

class GridField(object):
    def __init__(self, resolution):
        self.cols =int(WORLD_WIDTH/resolution)  # Columns of the grid
        self.rows = int(WORLD_HEIGHT/resolution)  # Rows of the grid
        self.cells =  np.ndarray((self.rows+1,self.cols+1), dtype=Cell) # grid memory using numpy array
        self.resolution = resolution # Resolution of grid relative to window width and height in pixels
        print(f' Grid created with  col:{self.cols} row:{self.rows}')
        #self.field = [[vec(random.uniform(0,1),random.uniform(0,1)) for col in range(self.cols)] for row in range(self.rows)] # create matrix 
        self.h_cells = []
        heapq.heapify(self.h_cells)

        self.create_grid_cells()
        
        # Optimization: Create a cached surface for the grid with Colorkey RLE acceleration
        self.surface = pg.Surface((WORLD_WIDTH, WORLD_HEIGHT))
        self.surface.set_colorkey((0, 0, 0), pg.RLEACCEL)

        # Semi-transparent tiles for surveyed area and obstacles
        tile_size = self.resolution - 2
        self.visited_tile = pg.Surface((tile_size, tile_size), pg.SRCALPHA)
        self.visited_tile.fill((0, 230, 90, 52)) # Soft translucent green (alpha ~20%)

        self.obstacle_tile = pg.Surface((tile_size, tile_size), pg.SRCALPHA)
        self.obstacle_tile.fill((220, 50, 40, 52)) # Soft translucent hazard red

        self.visited_positions = []
        self.obstacle_positions = []

        self._redraw_all()
   
    def create_grid_cells(self):
        '''
            Creates grid with cells according to resolution 
        '''
        blockSize = self.resolution
        for x in range(0, WORLD_WIDTH, blockSize):
            for y in range(0, WORLD_HEIGHT, blockSize):
                row = int(y / blockSize)
                col = int(x / blockSize)
                self.cells[row][col] = Cell(vec(x, y), blockSize)
                # Priority queue HEAP: store as (state, row, col) for row-major scan priority
                heapq.heappush(self.h_cells, (self.cells[row][col].state, row, col))

    def _redraw_all(self):
        """Redraws the entire grid onto the cached surface."""
        blockSize = self.resolution
        self.surface.fill((0, 0, 0)) # Clear transparent background
        
        for x in range(0, WORLD_WIDTH, blockSize):
            for y in range(0, WORLD_HEIGHT, blockSize):
                rect = pg.Rect(x, y, blockSize, blockSize)
                pg.draw.rect(self.surface, (120,120,120), rect, 1)
                self.cells[int(y/blockSize)][int(x/blockSize)].draw_center(self.surface)

    def draw(self, screen):
        # 1. Blit semi-transparent surveyed tiles directly over background (camouflage visible through tint)
        for pos in self.visited_positions:
            screen.blit(self.visited_tile, pos)
        for pos in self.obstacle_positions:
            screen.blit(self.obstacle_tile, pos)

        # 2. Blit grid lines and center dots on top with RLE colorkey acceleration
        screen.blit(self.surface, (0, 0))

    def change_state_cell(self, cell, to_state = VISITED):
        '''
            Cell is visited
            cell: (col, row)
        '''
        try:
            col = int(cell[0])
            row = int(cell[1])
            if not (0 <= col <= self.cols and 0 <= row <= self.rows):
                return
            c = self.cells[row][col]
            if c.state != to_state and c.state != OBSTACLE:
                c.change_state(to_state)
                # Update only this cell on the cached surface
                c.draw_center(self.surface)
                # Record position for translucent tile overlay
                tile_pos = (int(c.position[0] + 1), int(c.position[1] + 1))
                if to_state == VISITED:
                    self.visited_positions.append(tile_pos)
                elif to_state == OBSTACLE:
                    self.obstacle_positions.append(tile_pos)
        except Exception:
            pass

    def get_state_cell(self, cell):
        '''
            Get if cell was visited before
            cell: tuple with coordinates (col, row)
            return: state of the cell 
        '''
        try:
            col = int(cell[0])
            row = int(cell[1])
            if 0 <= col <= self.cols and 0 <= row <= self.rows:
                return self.cells[row][col].state
            return VISITED
        except Exception:
            return VISITED

    def get_sucessors(self,cell):
        """
            Obtains a list of the 8-connected successors of the node at (i, j).

            :param cell: position cell .
            :type tuple: int.
           
            :return: list of the 8-connected successors.
            :rtype: list of cells.
        """
        i, j = cell
        successors = []
        
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                if dx == 0 and dy == 0:
                    continue  # Skip the current cell

                x, y = i + dx, j + dy
                if 0 <= x < self.cols and 0 <= y < self.rows and self.get_state_cell((x, y)) == NOT_VISITED:
                    successors.append(self.get_cell_center((x, y)))
        
        return successors

    def get_size(self):
        '''
            Returns a tuple containing sizeof the grid :(#col,#row) 
        '''
        return (self.cols, self.rows)

    def get_cell_center(self, cell):
        col = max(0, min(int(cell[0]), self.cols))
        row = max(0, min(int(cell[1]), self.rows))
        return self.cells[row][col].get_cell_center()   
    
    def get_cell_not_visited(self, near_cell=None):
        '''
            This method will return coordinates of a cell that wasn't visited yet: (col, row)
            If near_cell is provided, finds the closest unvisited cell to maintain locality.
        '''
        if near_cell is not None:
            col, row = int(near_cell[0]), int(near_cell[1])
            best_cell = None
            min_dist_sq = float('inf')
            for r in range(self.rows + 1):
                for c in range(self.cols + 1):
                    if self.cells[r][c].state == NOT_VISITED:
                        d_sq = (c - col) ** 2 + (r - row) ** 2
                        if d_sq < min_dist_sq:
                            min_dist_sq = d_sq
                            best_cell = (c, r)
            if best_cell is not None:
                return best_cell

        while self.h_cells:
            _, r, c = heapq.heappop(self.h_cells)
            if 0 <= c <= self.cols and 0 <= r <= self.rows:
                if self.cells[r][c].state == NOT_VISITED:
                    return (c, r)  # Return (col, row) for consistency with (x, y) coordinates
        return None  # If no unvisited cell is found

    def get_coverage_percentage(self):
        """Calculates the percentage of cells that have been visited."""
        total_cells = (self.rows + 1) * (self.cols + 1)
        visited_count = 0
        for r in range(self.rows + 1):
            for c in range(self.cols + 1):
                if self.cells[r][c].state == VISITED:
                    visited_count += 1
        
        if total_cells == 0:
            return 0.0
        return visited_count / total_cells


class Cell():
    '''
        Represents a cell in the grid
        Every cell represents an area in the map that is being searched
    '''
    def __init__(self, pos, blockSize):
        self.size_block = blockSize
        self.position = pos
        self.state = NOT_VISITED
        self.center_in_coord_global = vec(self.position[0]+ self.size_block/2, self.position[1]+ self.size_block/2)

    def draw_center(self, screen):
        center = vec(self.position[0] + self.size_block / 2, self.position[1] + self.size_block / 2)
        if self.state == NOT_VISITED:
            pg.draw.circle(screen, (255, 0, 0), center, 3)
        elif self.state == VISITED:
            pg.draw.circle(screen, (0, 255, 0), center, 3)
        elif self.state == OBSTACLE:
            pg.draw.circle(screen, (0, 0, 255), center, 3)

    def change_state(self, state = VISITED):
        if self.state != OBSTACLE:
            self.state = state
    
    def get_cell_center(self):
        return self.center_in_coord_global
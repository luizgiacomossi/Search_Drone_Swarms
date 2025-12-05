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

        self.cols =int(SCREEN_WIDTH/resolution)  # Columns of the grid
        self.rows = int(SCREEN_HEIGHT/resolution)  # Rows of the grid
        self.cells =  np.ndarray((self.rows+1,self.cols+1), dtype=Cell) # grid memory using numpy array
        self.resolution = resolution # Resolution of grid relative to window width and height in pixels
        print(f' Grid created with  col:{self.cols} row:{self.rows}')
        #self.field = [[vec(random.uniform(0,1),random.uniform(0,1)) for col in range(self.cols)] for row in range(self.rows)] # create matrix 
        self.h_cells = []
        heapq.heapify(self.h_cells)

        self.create_grid_cells()
        
        # Optimization: Create a surface to cache the grid
        self.grid_surface = pg.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.grid_surface.set_colorkey((0,0,0)) # Transparent background if needed, or just fill
        self.grid_surface.fill(BLACK) # Or whatever background color
        self.draw_static_grid()
   
    def create_grid_cells(self):
        '''
            Creates grid with cells according to resolution 
        '''
        blockSize = self.resolution
        for x in range(0, SCREEN_WIDTH, blockSize):
            for y in range(0, SCREEN_HEIGHT, blockSize):
                row = int(y / blockSize)
                col = int(x / blockSize)
                self.cells[row][col] = Cell(vec(x, y), blockSize)
                # Priority queue HEAP
                heapq.heappush(self.h_cells, (self.cells[row][col].state, (row, col)))

    def draw_static_grid(self):
        """Draws the initial grid to the cached surface."""
        blockSize = self.resolution
        for x in range(0, SCREEN_WIDTH, blockSize):
            for y in range(0, SCREEN_HEIGHT, blockSize):
                rect = pg.Rect(x, y, blockSize, blockSize)
                pg.draw.rect(self.grid_surface, (120,120,120), rect, 1)
                self.cells[int(y/blockSize)][int(x/blockSize)].draw_center(self.grid_surface)

    def draw(self, screen):
        """Blits the cached grid surface to the screen."""
        screen.blit(self.grid_surface, (0, 0))

    def change_state_cell(self, cell, to_state = VISITED):
        '''
            Cell is visitated
        '''
        try:
            c = self.cells[cell[1]][cell[0]]
            if c.state != to_state and c.state != OBSTACLE:
                c.change_state(to_state)
                # Optimization: Update only this cell on the surface
                c.draw_center(self.grid_surface)
        except:
            pass

    def get_state_cell(self, cell):
        '''
            Get if cell was visisted before
            cell: tuple with coordenates
            return: state of the cell 
        '''
        try:
            return self.cells[cell[1]][cell[0]].state
        except:
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

    def get_cell_center(self,cell):
        return self.cells[cell[1]][cell[0]].get_cell_center()   
    
    def get_cell_not_visited(self):
        '''
            This method will return coordinates of a cell that wasn't visited yet
        '''
        while self.h_cells:
            state, (row, col) = heapq.heappop(self.h_cells)
            if state == NOT_VISITED:
                return (row, col)
        return None  # If no unvisited cell is found


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

    def draw_center(self,screen):
        
        if self.state == NOT_VISITED:
            pg.draw.circle(screen, (255,0,0), vec(self.position[0]+ self.size_block/2, self.position[1]+ self.size_block/2), 3)
        if self.state == VISITED: 
            pg.draw.circle(screen, (0,255,0), vec(self.position[0]+ self.size_block/2, self.position[1]+ self.size_block/2), 3)
        if self.state == OBSTACLE:
            pg.draw.circle(screen, (0,0,255), vec(self.position[0]+ self.size_block/2, self.position[1]+ self.size_block/2), 3)

    def change_state(self, state = VISITED):
        if self.state != OBSTACLE:
            self.state = state
    
    def get_cell_center(self):
        return self.center_in_coord_global
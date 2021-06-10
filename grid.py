from constants import *
import pygame as pg
from math import atan2, pi, exp, floor
import random
import copy 
import numpy as np
vec = pg.math.Vector2 

class GridField(object):
    def __init__(self, resolution):

        self.cols =int(SCREEN_WIDTH/resolution)  # Columns of the grid
        self.rows = int(SCREEN_HEIGHT/resolution)  # Rows of the grid
        self.cells =  np.ndarray((self.rows+1,self.cols+1), dtype=Cell) # grid memory using numpy array
        self.resolution = resolution # Resolution of grid relative to window width and height in pixels
        print(f' Grid created with  col:{self.cols} row:{self.rows}')
        #self.field = [[vec(random.uniform(0,1),random.uniform(0,1)) for col in range(self.cols)] for row in range(self.rows)] # create matrix 
        self.cells_ = {} # Memory using dictionary NOT USED
        self.create_grid_cells()

    def create_grid_cells(self):
        '''
            Creates grid with cells according to resolution 
        '''
        blockSize =  self.resolution
        for x in range(0, SCREEN_WIDTH,  blockSize):
            for y in range(0, SCREEN_HEIGHT,  blockSize):
                #self.cells_[f'{int(x/blockSize)},{int(y/blockSize)}'] = Cell(vec(x,y), blockSize)
                #             row                  col       
                self.cells[int(y/blockSize)][int(x/blockSize)] = Cell(vec(x,y), blockSize)

    def draw(self, screen):

        blockSize = self.resolution #Set the size of the grid block

        for x in range(0, SCREEN_WIDTH, blockSize):
            for y in range(0, SCREEN_HEIGHT, blockSize):
                rect = pg.Rect(x, y, blockSize, blockSize)
                pg.draw.rect(screen, (120,120,120), rect, 1)
                self.cells[int(y/blockSize)][int(x/blockSize)].draw_center(screen)

    def change_state_cell(self, cell):
        '''
            Cell is visitated
        '''
        self.cells[cell[1]][cell[0]].change_state()

    def get_state_cell(self, cell):
        '''
            Get if cell was visisted before
            cell: tuple with coordenates
            return: state of the cell 
        '''
        return self.cells[cell[1]][cell[0]].state

    def get_sucessors(self,cell):
        """
            Obtains a list of the 8-connected successors of the node at (i, j).

            :param cell: position cell .
            :type tuple: int.
           
            :return: list of the 8-connected successors.
            :rtype: list of cells.
        """
        i = cell[0]
        j = cell[1]
        successors = []

        for di in range(-1, 2):
            for dj in range(-1, 2):

                if di != 0 and dj != 0:
                    
                    x = i + di
                    y = j + dj

                    # if not visited
                    if x > 0 and y > 0:
                        if not self.get_state_cell((x, y)):
                            successors.append((x, y))
        
        print(successors)
        input()

        return successors
        

class Cell():
    '''
        Represents a cell in the grid
        Every cell represents an area in the map that is being searched
    '''
    def __init__(self, pos, blockSize):
        self.size_block = blockSize
        self.position = pos
        self.state = False
        self.center_in_coord_global = vec(self.position[0]+ self.size_block/2, self.position[1]+ self.size_block/2)

    def draw_center(self,screen):
        
        if self.state == False:
            pg.draw.circle(screen, (255,0,0), vec(self.position[0]+ self.size_block/2, self.position[1]+ self.size_block/2), 3)
        else:
            pg.draw.circle(screen, (0,255,0), vec(self.position[0]+ self.size_block/2, self.position[1]+ self.size_block/2), 3)

    def change_state(self):
        self.state = True
    
    def get_cell_center(self):
        return self.center_in_coord_global
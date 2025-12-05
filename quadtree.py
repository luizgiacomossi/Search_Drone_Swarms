import pygame

class Rect:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def contains(self, point):
        return (point.x >= self.x - self.w and
                point.x <= self.x + self.w and
                point.y >= self.y - self.h and
                point.y <= self.y + self.h)

    def intersects(self, range):
        return not (range.x - range.w > self.x + self.w or
                    range.x + range.w < self.x - self.w or
                    range.y - range.h > self.y + self.h or
                    range.y + range.h < self.y - self.h)

class Quadtree:
    def __init__(self, boundary, capacity):
        self.boundary = boundary
        self.capacity = capacity
        self.points = []
        self.divided = False

    def insert(self, point, data):
        if not self.boundary.contains(point):
            return False

        if len(self.points) < self.capacity:
            self.points.append((point, data))
            return True
        else:
            if not self.divided:
                self.subdivide()

            if self.northeast.insert(point, data):
                return True
            elif self.northwest.insert(point, data):
                return True
            elif self.southeast.insert(point, data):
                return True
            elif self.southwest.insert(point, data):
                return True

    def subdivide(self):
        x = self.boundary.x
        y = self.boundary.y
        w = self.boundary.w / 2
        h = self.boundary.h / 2

        ne = Rect(x + w, y - h, w, h)
        self.northeast = Quadtree(ne, self.capacity)
        nw = Rect(x - w, y - h, w, h)
        self.northwest = Quadtree(nw, self.capacity)
        se = Rect(x + w, y + h, w, h)
        self.southeast = Quadtree(se, self.capacity)
        sw = Rect(x - w, y + h, w, h)
        self.southwest = Quadtree(sw, self.capacity)
        self.divided = True

    def query(self, range, found):
        if not self.boundary.intersects(range):
            return

        for p in self.points:
            if range.contains(p[0]):
                found.append(p[1])

        if self.divided:
            self.northwest.query(range, found)
            self.northeast.query(range, found)
            self.southwest.query(range, found)
            self.southeast.query(range, found)

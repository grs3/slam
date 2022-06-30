import math
import pygame
import numpy as np

class buildEnvironment:
    def __init__(self, MapDimensions):
        pygame.init()
        self.pointCloud = []
        self.maph, self.mapw = MapDimensions
        self.MapName = 'SLAM visualisation'
        pygame.display.set_caption(self.MapName)
        self.map = pygame.display.set_mode((self.maph, self.mapw))

        self.black = (0,0,0)
        self.grey = (70,70,70)
        self.blue = (0,0,255)
        self.green = (0,255,0)
        self.red = (255,0,0)
        self.white = (255,255,255)

    def Add_point(self,distance,angle,robotpos):
        x = distance * math.cos(robotpos[2] + np.radians(angle)) + robotpos[0]
        y = distance * math.sin(robotpos[2] + np.radians(angle)) + robotpos[1]
        return (int(x*100),int(y*100))

    def dataStorage(self,distances,position):
        for element in distances:
            if element:
                point = self.Add_point(element,distances.index(element),position)
                if point not in self.pointCloud:
                    self.pointCloud.append(point)

    def show_sensorData(self):
        for point in self.pointCloud:
            self.infomap.set_at((int(point[0]) + 600,int(point[1]) + 600),self.red)
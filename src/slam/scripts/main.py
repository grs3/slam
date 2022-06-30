#!/usr/bin/env python3
import pygame 
import env, features,lidar
import math
import rospy

if __name__ == '__main__':
    rospy.init_node('main')
    lidar = lidar.Lidar('/scan','/odom')
    FeatureMap = features.feturesDetection()
    environment = env.buildEnvironment((1200,1200))
    environment.infomap = environment.map.copy() 

    running = True
    FEATURES_DETECTION = True

    while running:
        environment.infomap = environment.map.copy()
        FEATURES_DETECTION = True
        BREAK_POINT_IND = 0
        ENDPOINTS = [0,0]
        PREDICTED_POINTS_TODRAW = []

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        if lidar:
            position = lidar.position
            distances = lidar.distances
            #environment.dataStorage(distances,position)
            #environment.show_sensorData()
            FeatureMap.laser_points_set(distances,position)

            while BREAK_POINT_IND < (FeatureMap.NP - FeatureMap.PMIN):
                seedSeg = FeatureMap.seed_segment_detection(position, BREAK_POINT_IND)
                if seedSeg == False:
                    break
                else:
                    seedSegment = seedSeg[0]
                    PREDICTED_POINTS_TODRAW = seedSeg[1]
                    INDICES = seedSeg[2]
                    result = FeatureMap.seed_segment_growing(INDICES,BREAK_POINT_IND)
                    if result == False:
                        BREAK_POINT_IND = INDICES[1]
                        continue
                    else:
                        line_eq = result[1]
                        m,c = result[5]
                        line_seg = result[0]
                        OUTERMOST = result[2]
                        BREAK_POINT_IND = result[3]

                        #ENDPOINTS[0] = FeatureMap.projection_point2line(OUTERMOST[0],m,c)
                        #ENDPOINTS[1] = FeatureMap.projection_point2line(OUTERMOST[1],m,c)

                        for point in line_seg:
                        #     #environment.infomap.set_at((int(point[0][0]*100)+ 600 ,int(point[0][1]*100)+ 600),environment.white)
                            #pygame.draw.circle(environment.infomap,environment.blue,(int(point[0][0]* 100) + 600,int(point[0][1]* 100) + 600),2,0)
                        # pygame.draw.line(environment.infomap,environment.green,(OUTERMOST[0][0]*100 +600,OUTERMOST[0][1]*100 +600),(OUTERMOST[1][0]*100 +600,OUTERMOST[1][1]*100 +600),2)
                            FeatureMap.FEATURES.append([[m,c],OUTERMOST])
                        #pygame.draw.line(environment.infomap,environment.green,(OUTERMOST[0][0]*100 +600,OUTERMOST[0][1]*100 +600),(OUTERMOST[1][0]*100 +600,OUTERMOST[1][1]*100 +600),2)
                        environment.dataStorage(distances,position)

                        FeatureMap.FEATURES = FeatureMap.lineFeats2point()
                        features.landmark_association(FeatureMap.FEATURES)

            #for landmark in features.Landmarks:
            #    pygame.draw.line(environment.infomap,environment.green,(landmark[1][0][0]*100 +600,landmark[1][0][1]*100 +600),(landmark[1][1][0]*100 +600,landmark[1][1][1]*100 +600),2)
            environment.show_sensorData()
            print(len(features.Landmarks))
            environment.map.blit(environment.infomap,(0,0))
            pygame.display.update()

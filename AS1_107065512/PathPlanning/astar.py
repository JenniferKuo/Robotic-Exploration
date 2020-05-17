# Reference: https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2

import cv2
import numpy as np

class AStar():
    def __init__(self,m):
        self.map = m
        self.initialize()

    def initialize(self):
        self.queue = []
        self.parent = {}
        self.h = {} # Distance from start to node
        self.g = {} # Distance from node to goal
        self.goal_node = None

    # estimation
    def _distance(self, a, b):
        # Diagonal distance
        d = np.max([np.abs(a[0]-b[0]), np.abs(a[1]-b[1])])
        return d

    def planning(self, start=(100,200), goal=(375,520), inter=10, img=None):
        # Initialize 
        self.initialize()
        # 把起點放進queue
        self.queue.append(start)
        # 起點沒有parent
        self.parent[start] = None
        # Dijkstra，從起點到自己的節點的距離
        self.g[start] = 0
        # BFS的預測的cost，這裡用距離來當作heuristic function，從節點到終點的距離
        self.h[start] = self._distance(start, goal)
        node_goal = None
        while(1):
            min_dist = 99999
            min_id = -1
            # 重複的檢查當前的queue中的節點
            for i, node in enumerate(self.queue):
                # todo
                #####################################################
                # In a*  we need to add something in this function
                # a*中，f = g + h，g是Dijkstra，h是BFS
                f = self.g[node] + self.h[node]
                #####################################################
                # 如果找到的f是目前最小的，就選中目前這個節點
                if f < min_dist:
                    min_dist = f
                    min_id = i

            # pop the nearest node，p是目前拿出的節點
            p = self.queue.pop(min_id)

            # meet obstacle, skip，如果目前節點存在於地圖中
            if self.map[p[1],p[0]]<0.5:
                
                continue
            # find goal，用距離判斷已經到終點
            if self._distance(p,goal) < inter:
                self.goal_node = p
                break

            # eight direction
            # 目前節點周圍的八個節點，下右上左，右下右上左上左下
            pts_next1 = [(p[0]+inter,p[1]), (p[0],p[1]+inter), (p[0]-inter,p[1]), (p[0],p[1]-inter)]
            pts_next2 = [(p[0]+inter,p[1]+inter), (p[0]-inter,p[1]+inter), (p[0]-inter,p[1]-inter), (p[0]+inter,p[1]-inter)]
            pts_next = pts_next1 + pts_next2
            
            for pn in pts_next:
                # 如果鄰近的節點，不是任何節點的parent
                if pn not in self.parent:
                    # 就把他放進queue
                    self.queue.append(pn)
                    # 並把鄰近節點的parent設為剛剛選中的節點
                    self.parent[pn] = p
                    # 設置鄰近節點的g
                    self.g[pn] = self.g[p] + inter
                    ##############################################
                    
                    # update the estimation
                    self.h[pn] = self._distance(pn, goal)
                    ##############################################
                # 如果鄰近節點離起點的距離，比目前節點加上間距的距離還大，代表有往前移動，則更新鄰近節點的g
                elif self.g[pn]>self.g[p] + inter:
                    self.parent[pn] = p
                    self.g[pn] = self.g[p] + inter
            
            # if img is not None:
            #     cv2.circle(img,(start[0],start[1]),5,(0,0,1),3)
            #     cv2.circle(img,(goal[0],goal[1]),5,(0,1,0),3)
            #     cv2.circle(img,p,2,(0,0,1),1)
            #     img_ = cv2.flip(img,0)
            #     cv2.imshow("A* Test",img_)
            #     k = cv2.waitKey(1)
            #     if k == 27:
            #         break
        
        # Extract path
        path = []
        p = self.goal_node
        # 從終點一直往上數parent，取出規劃的路徑
        while(True):
            path.insert(0,p)
            if self.parent[p] == None:
                break
            p = self.parent[p]
        if path[-1] != goal:
            path.append(goal)
        return path

smooth = True
if __name__ == "__main__":
    img = cv2.flip(cv2.imread("../Maps/map2.png"),0)
    img[img>128] = 255
    img[img<=128] = 0
    # m是創建地圖
    m = np.asarray(img)
    m = cv2.cvtColor(m, cv2.COLOR_RGB2GRAY)
    m = m.astype(float) / 255.
    # 膨脹函式(source, kernel大小)
    m = 1-cv2.dilate(1-m, np.ones((20,20)))
    img = img.astype(float)/255.

    start=(100,200)
    goal=(380,520)
    astar = AStar(m)
    path = astar.planning(start=start, goal=goal, img=img, inter=20)
    print(path)

    cv2.circle(img,(start[0],start[1]),5,(0,0,1),3)
    cv2.circle(img,(goal[0],goal[1]),5,(0,1,0),3)
    # Extract Path
    if not smooth:
        for i in range(len(path)-1):
            cv2.line(img, path[i], path[i+1], (1,0,0), 2)
    else:
        from cubic_spline import *
        path = np.array(cubic_spline_2d(path, interval=1))
        for i in range(len(path)-1):
            cv2.line(img, pos_int(path[i]), pos_int(path[i+1]), (1,0,0), 1)
    
    img_ = cv2.flip(img,0)
    cv2.imshow("A* Test",img_)
    k = cv2.waitKey(0)
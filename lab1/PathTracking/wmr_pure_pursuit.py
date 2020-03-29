import numpy as np 

class PurePursuitControl:
    def __init__(self, kp=1, Lfc=10):
        self.path = None
        self.kp = kp # 速度P控制系數
        self.Lfc = Lfc # 車的可視距離

    def set_path(self, path):
        self.path = path.copy()

    def _search_nearest(self, pos):
        min_dist = 99999999
        min_id = -1
        for i in range(self.path.shape[0]):
            dist = (pos[0] - self.path[i,0])**2 + (pos[1] - self.path[i,1])**2
            if dist < min_dist:
                min_dist = dist
                min_id = i
        return min_id, min_dist

    def calculateDistance(self, pos1, pos2):
        return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def feedback(self, state):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None
        
        # Extract State 
        x, y, yaw, v = state["x"], state["y"], state["yaw"], state["v"]

        # Search Front Target
        min_idx, min_dist = self._search_nearest((x,y))
        
        # todo
        #####################################################################
        
        # all parameter name (ex:alpha) comes from the Slides
        # You need to finish the pure pursuit control algo

        # step by step
        # first, you need to calculate the look ahead distance Ld by formula
        Ld = self.kp * v + self.Lfc
        # second, you need to find a point(target) on the path which distance between the path and model is as same as the Ld
        ### hint: (you first need to find the nearest point and then find the point(target) backward, this will make your model won't go back)
        ### hint: (if you can not find a point(target) on the path which distance between the path and model is as same as the Ld, you need to find a similar one)
        if Ld >= min_dist:
            i = min_idx
            while self.calculateDistance((x,y), self.path[i]) < Ld and i < len(self.path) - 2:
                i = i + 1
            target = self.path[i]
        elif Ld < min_dist:
            Ld = min_dist
            target = self.path[min_idx]

        # third, you need to calculate alpha
        alpha = np.arctan2(target[1]-y, target[0]-x) - np.deg2rad(yaw)
        # now, you can calculate the ω
        next_w = np.rad2deg(2 * v * np.sin(alpha) / Ld)

        # The next_w is Pure Pursuit Control's output
        # The target is the point on the path which you find
        #####################################################################
        return next_w, target

if __name__ == "__main__":
    import cv2
    import path_generator
    import sys
    sys.path.append("../")
    from wmr_model import KinematicModel

    # Path
    path = path_generator.path2()
    img_path = np.ones((600,600,3))
    for i in range(path.shape[0]-1):
        cv2.line(img_path, (int(path[i,0]), int(path[i,1])), (int(path[i+1,0]), int(path[i+1,1])), (1.0,0.5,0.5), 1)

    # Initialize Car
    car = KinematicModel()
    car.init_state((50,300,0))
    controller = PurePursuitControl()
    controller.set_path(path)

    while(True):
        print("\rState: "+car.state_str(), end="\t")

        # ================= Control Algorithm ================= 
        # PID Longitude Control
        end_dist = np.hypot(path[-1,0]-car.x, path[-1,1]-car.y)
        target_v = 40 if end_dist > 265 else 0
        next_a = 0.1*(target_v - car.v)

        # Pure Pursuit Lateral Control
        state = {"x":car.x, "y":car.y, "yaw":car.yaw, "v":car.v}
        next_delta, target = controller.feedback(state)
        car.control(next_a,next_delta)
        # =====================================================
        
        # Update & Render
        car.update()
        img = img_path.copy()
        cv2.circle(img,(int(target[0]),int(target[1])),3,(1,0.3,0.7),2) # target points
        img = car.render(img)
        img = cv2.flip(img, 0)
        cv2.imshow("Pure-Pursuit Control Test", img)
        k = cv2.waitKey(1)
        if k == ord('r'):
            _init_state(car)
        if k == 27:
            print()
            break
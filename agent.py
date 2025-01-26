from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point


class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)
        self.keep_tracking = True
        self.created_points = [False, False, False, False, False, False, False, False, False, False, False,
                               False, False, False, False, False, False, False, False, False, False]
        self.dodge_points = [Point(0, 0), Point(0, 0)]
        self.perpendicular_points = [Point(0, 0), Point(0, 0)]
        self.dodge_id = 21
        self.choice = 0
        self.tracking_choice = 0 # 0 = target, 1 = dodge_point[0], 2 = dodge_point[1], 3 = perpendicular_point[0], 4 = perpendicular_point[1]
        self.tracking = Point(0,0)
        self.target = []
        self.rest = False

    def decision(self): # What the agent will do to reach the target
        obs_dist = 0.5 # Distance from the obstacle when it starts to dodge
        point_dist = 0.3 # Distance btween the obstacle and the points to dodge

        if len(self.targets) == 0:
            return
        
        self.select_target()
        self.create_points(obs_dist, point_dist)
        self.choose_dodge_point()

        if self.keep_tracking:
            self.tracking_choice = 0
            self.tracking = self.target[0]
        
        else:
            if self.pos.dist_to(self.dodge_points[0]) <= 0.05 or self.pos.dist_to(self.dodge_points[1]) <= 0.05:
                self.created_points[self.dodge_id - 1] = True
                self.keep_tracking = True
                self.tracking_choice = 0
                self.tracking = self.target[0]
                self.choice = 0

            elif self.choice == 1:
                self.tracking_choice = 1
                self.tracking = self.dodge_points[0]

            elif self.choice == 2:
                self.tracking_choice = 2
                self.tracking = self.dodge_points[1]

            elif self.choice == 3:
                self.tracking_choice = 3
                self.tracking = self.perpendicular_points[0]

            else:
                self.tracking_choice = 4
                self.tracking = self.perpendicular_points[1]

        return

    def post_decision(self):
        if not self.rest:
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.tracking)
            self.set_vel(target_velocity)
            self.set_angle_vel(target_angle_velocity)
        else:
            self.reset()
        return
    
    def create_points(self, obs_dist, point_dist): # Creates the dodge points and perpendicular points for the nearest obstacle
        adjustment_factor = point_dist / 3
        speed_tolerance = 0.1
        for robot_id in self.opponents:
            obs = Point(self.opponents[robot_id].x, self.opponents[robot_id].y)
            min_dist_obs = Point(self.opponents[self.dodge_id].x, self.opponents[self.dodge_id].y)
            if self.pos.dist_to(obs) < obs_dist and not self.created_points[robot_id - 1]:
                change0 = 0
                change1 = 0
                self.reset()
                self.keep_tracking = False
                if self.pos.dist_to(obs) <= self.pos.dist_to(min_dist_obs):
                    # adapt the dodge points to the obstacle velocity
                    if abs(self.opponents[robot_id].v_x) > speed_tolerance or abs(self.opponents[robot_id].v_y) > speed_tolerance:
                        obs_velocity = Point(self.opponents[robot_id].v_x, self.opponents[robot_id].v_y)
                        obs_dodge1 = (self.dodge_points[0] - obs)
                        obs_dodge2 = (self.dodge_points[1] - obs)
                        escalar1 = obs_dodge1.x * obs_velocity.x + obs_dodge1.y * obs_velocity.y
                        escalar2 = obs_dodge2.x * obs_velocity.x + obs_dodge2.y * obs_velocity.y
                        if escalar1 < escalar2:
                            change1 = adjustment_factor
                            change0 = -adjustment_factor
                        elif escalar1 > escalar2:
                            change0 = adjustment_factor
                            change1 = -adjustment_factor
                    # create the dodge points
                    vector = obs - self.target[0]
                    unit_vector = Point(vector.x, vector.y).normalize()
                    perpendicular_vector = Point(-unit_vector.y, unit_vector.x)
                    self.dodge_points[0] = (perpendicular_vector * (point_dist + change0)) + obs
                    self.dodge_points[1] = (perpendicular_vector * -(point_dist + change1)) + obs
                    # create the perpendicular points
                    vector = obs - self.pos
                    unit_vector = Point(vector.x, vector.y).normalize()
                    perpendicular_vector = Point(-unit_vector.y, unit_vector.x)
                    self.perpendicular_points[0] = (perpendicular_vector * (point_dist + change0)) + obs
                    self.perpendicular_points[1] = (perpendicular_vector * -(point_dist + change1)) + obs
                    self.dodge_id = robot_id
        return

    def choose_dodge_point(self): # Chooses the best dodge point
        next_to = False
        min_dist = 0.2
        if self.dodge_points[0].dist_to(self.pos) < self.dodge_points[1].dist_to(self.pos):
            if self.pos.dist_to(self.target[0]) <= self.dodge_points[0].dist_to(self.target[0]):
                self.keep_tracking = True
            else:
                self.choice = 1
        else:
            if self.pos.dist_to(self.target[0]) <= self.dodge_points[1].dist_to(self.target[0]):
                self.keep_tracking = True
            else:
                self.choice = 2
        
        if not self.keep_tracking:
            for robot_id in self.opponents:
                if robot_id != self.dodge_id:
                    obs = Point(self.opponents[robot_id].x, self.opponents[robot_id].y)
                    if abs(self.dodge_points[0].x - obs.x) < min_dist and abs(self.dodge_points[0].y - obs.y) < min_dist and self.choice == 1:
                        next_to = True
                    elif abs(self.dodge_points[1].x - obs.x) < min_dist and abs(self.dodge_points[1].y - obs.y) < min_dist and self.choice == 2:
                        next_to = True
        
        if next_to:
            self.choose_perpendicular_point()
        return

    def choose_perpendicular_point(self): # Chooses the best perpendicular point (farthest from the target)
        if self.tracking_choice == 3 or self.tracking_choice == 4:
            self.choice = self.tracking_choice
            return
        
        if self.perpendicular_points[0].dist_to(self.target[0]) < self.perpendicular_points[1].dist_to(self.target[0]):
            self.choice = 4
        else:
            self.choice = 3
        return
    
    def clear_points(self):
        self.created_points = [False, False, False, False, False, False, False, False, False, False, False,
                               False, False, False, False, False, False, False, False, False, False]
        return

    def select_target(self):
        self.target = sorted(self.targets, key=lambda x: self.pos.dist_to(x))
        return
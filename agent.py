from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point


class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)
        self.keep_tracking = True
        self.created_points = [False, False, False, False, False, False, False, False, False, False, False,
                               False, False, False, False, False, False, False, False, False, False]
        self.dodge_point1 = Point(0, 0)
        self.dodge_point2 = Point(0, 0)
        self.dodge_id = 21
        self.choice = 0
        self.tracking = Point(0,0)
        self.target = Point(0,0)
        self.rest = False

    def decision(self): # What the agent will do to reach the target
        obs_dist = 0.9 # Distance from the obstacle when it starts to dodge
        point_dist = 0.3 # Distance btween the obstacle and the points to dodge
        min_dist_obs = Point(self.opponents[21].x, self.opponents[21].y) # Sets the last obstacle as the closest

        if len(self.targets) == 0:
            return
        
        if self.new_targets:
            self.select_target()
            self.clear_points()
            self.new_targets = False
        
        for robot_id in self.opponents:
            obs = Point(self.opponents[robot_id].x, self.opponents[robot_id].y)
            min_dist_obs = Point(self.opponents[self.dodge_id].x, self.opponents[self.dodge_id].y)

            if self.pos.dist_to(obs) < obs_dist and not self.created_points[robot_id - 1]:
                if self.pos.dist_to(obs) <= self.pos.dist_to(min_dist_obs):
                    vector = obs - self.target
                    unit_vector = Point(vector.x, vector.y).normalize()
                    perpendicular_vector = Point(-unit_vector.y, unit_vector.x)
                    self.dodge_point1 = (perpendicular_vector * point_dist) + obs
                    self.dodge_point2 = (perpendicular_vector * -point_dist) + obs
                    self.dodge_id = robot_id
                    self.keep_tracking = False

        for robot_id in self.opponents:
            obs = Point(self.opponents[robot_id].x, self.opponents[robot_id].y)
            if self.dodge_point1.dist_to(obs) < 0.15:
                self.choice = 2
            elif self.dodge_point2.dist_to(obs) < 0.15:
                self.choice = 1

        if self.keep_tracking:
            self.tracking = self.target
        
        else:
            if self.pos.dist_to(self.dodge_point1) <= 0.05 or self.pos.dist_to(self.dodge_point2) <= 0.05:
                self.created_points[self.dodge_id - 1] = True
                self.keep_tracking = True
                self.tracking = self.target
                self.choice = 0

            elif self.choice == 1:
                if self.pos.dist_to(self.target) <= self.dodge_point1.dist_to(self.target):
                    self.tracking = self.target
                    self.keep_tracking = True
                else:
                    self.tracking = self.dodge_point1

            elif self.choice == 2:
                if self.pos.dist_to(self.target) <= self.dodge_point2.dist_to(self.target):
                    self.tracking = self.target
                    self.keep_tracking = True
                else:
                    self.tracking = self.dodge_point2

            elif self.dodge_point1.dist_to(self.pos) < self.dodge_point2.dist_to(self.pos):
                if self.pos.dist_to(self.target) <= self.dodge_point1.dist_to(self.target):
                    self.tracking = self.target
                    self.keep_tracking = True
                else:
                    self.tracking = self.dodge_point1

            else:
                if self.pos.dist_to(self.target) <= self.dodge_point2.dist_to(self.target):
                    self.tracking = self.target
                    self.keep_tracking = True
                else:
                    self.tracking = self.dodge_point2
        return

    def post_decision(self):
        if not self.rest:
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.tracking)
            self.set_vel(target_velocity)
            self.set_angle_vel(target_angle_velocity)
        else:
            self.reset()
        return

    def clear_points(self):
        self.created_points = [False, False, False, False, False, False, False, False, False, False, False,
                               False, False, False, False, False, False, False, False, False, False]

    def select_target(self):
        self.target = self.targets[self.id]
        return
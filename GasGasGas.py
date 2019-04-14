import math
import time
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

class Vector3:
    def __init__(self,a,b,c):
        self.data = [a,b,c]
    def __getitem__(self,key):
        return self.data[key]
    def __str__(self):
        return str(self.data)
    def __add__(self,value):
        return Vector3(self[0]+value[0], self[1]+value[1], self[2]+value[2])
    def __sub__(self,value):
        return Vector3(self[0]-value[0],self[1]-value[1],self[2]-value[2])
    def __mul__(self,value):
        return Vector3(self[0]*value, self[1]*value, self[2]*value)
    __rmul__ = __mul__
    def __div__(self,value):
        return Vector3(self[0]/value, self[1]/value, self[2]/value)
    def magnitude(self):
        return math.sqrt((self[0]*self[0]) + (self[1] * self[1]) + (self[2]* self[2]))
    def normalize(self):
        mag = self.magnitude()
        if mag != 0:
            return Vector3(self[0]/mag, self[1]/mag, self[2]/mag)
        else:
            return Vector3(0,0,0)
    def dot(self,value):
        return self[0]*value[0] + self[1]*value[1] + self[2]*value[2]
    def cross(self,value):
        return Vector3((self[1]*value[2]) - (self[2]*value[1]),(self[2]*value[0]) - (self[0]*value[2]),(self[0]*value[1]) - (self[1]*value[0]))
    def flatten(self):
        return Vector3(self[0],self[1],0)

class carobject:
    def __init__(self, i):
        self.index = i
        self.loc = Vector3(0,0,0)
        self.vel = Vector3(0,0,0)
        self.rot = Vector3(0,0,0)
        self.Rotvel = Vector3(0,0,0)
        self.matrix = Matrix3D(self.rot)
        self.goals = 0
        self.saves = 0
        self.name = "Gas Gas Gas"
        self.jumped = False
        self.doublejumped = False
        self.team = 0
        self.boostAmount = 0
        self.wheelcontact = False
        self.supersonic = False

    def update(self,TempVar):
        self.loc.data = [TempVar.physics.location.x,TempVar.physics.location.y,TempVar.physics.location.z]
        self.vel.data = [TempVar.physics.velocity.x,TempVar.physics.velocity.y,TempVar.physics.velocity.z]
        self.rot.data = [TempVar.physics.rotation.pitch,TempVar.physics.rotation.yaw,TempVar.physics.rotation.roll]
        self.matrix = Matrix3D(self.rot)
        TempRot = Vector3(TempVar.physics.angular_velocity.x,TempVar.physics.angular_velocity.y,TempVar.physics.angular_velocity.z)
        self.Rotvel = self.matrix.dot(TempRot)
        self.goals = TempVar.score_info.goals
        self.saves = TempVar.score_info.saves
        self.name = TempVar.name
        self.jumped = TempVar.jumped
        self.doublejumped = TempVar.double_jumped
        self.team = TempVar.team
        self.boostAmount = TempVar.boost
        self.wheelcontact = TempVar.has_wheel_contact
        self.supersonic = TempVar.is_super_sonic
    
class Matrix3D:
    def __init__(self,r):
        CR = math.cos(r[2])
        SR = math.sin(r[2])
        CP = math.cos(r[0])
        SP = math.sin(r[0])
        CY = math.cos(r[1])
        SY = math.sin(r[1])        
        self.data = [Vector3(CP*CY, CP*SY, SP),Vector3(CY*SP*SR-CR*SY, SY*SP*SR+CR*CY, -CP * SR),Vector3(-CR*CY*SP-SR*SY, -CR*SY*SP+SR*CY, CP*CR)]

    def dot(self,vector):
        return Vector3(self.data[0].dot(vector),self.data[1].dot(vector),self.data[2].dot(vector))

class ballobject:
    def __init__(self):
        self.loc = Vector3(0,0,0)
        self.vel = Vector3(0,0,0)
        self.rot = Vector3(0,0,0)
        self.Rotvel = Vector3(0,0,0)

    def update(self,TempVar):
        self.loc.data = [TempVar.physics.location.x,TempVar.physics.location.y,TempVar.physics.location.z]
        self.vel.data = [TempVar.physics.velocity.x,TempVar.physics.velocity.y,TempVar.physics.velocity.z]
        self.rot.data = [TempVar.physics.rotation.pitch,TempVar.physics.rotation.yaw,TempVar.physics.rotation.roll]
        self.Rotvel.data = [TempVar.physics.angular_velocity.x,TempVar.physics.angular_velocity.y,TempVar.physics.angular_velocity.z]


class GasGasGas(BaseAgent):

    def initialize_agent(self):
        #This runs once before the bot starts up
        self.controller_state = SimpleControllerState()
        self.car = carobject(self.index)
        self.ball = ballobject()
        self.start = time.time()
        self.teammates = []
        self.opponents = []

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        self.preprocess(packet)

        self.renderer.begin_rendering()
        self.renderer.end_rendering()

        return self.Brain()

    def preprocess(self, gamepacket):
        self.car.update(gamepacket.game_cars[self.index])
        self.ball.update(gamepacket.game_ball)
        for i in range(gamepacket.num_cars):
            car = gamepacket.game_cars[i]
            if i == self.index:
                self.car.update(car)
            elif car.team == self.team:
                flag = True
                for teammate in self.teammates:
                    if i == teammate.index:
                        teammate.update(car)
                        flag = False
                if flag:
                    temp = carobject(i)
                    temp.update(car)
                    self.teammates.append(temp)
            else:
                flag = True
                for opponent in self.opponents:
                    if i == opponent.index:
                        opponent.update(car)
                        flag = False
                if flag:
                    temp = carobject(i)
                    temp.update(car)
                    self.opponents.append(temp)
    
    def Brain(self):
        if self.ball.loc[1]==0:
            return Kick_off(self)
        elif self.car.wheelcontact == False:
            if self.car.doublejumped and self.car.loc[2] > 300:
                return Air_roll(self)
            else:
                return frontflip(self)
        elif (self.ball.loc-self.car.loc)[1] * side(self.team) > 0:
            return Recovery(self)
        else:
            return Shooting(self)

def Kick_off(agent):
    agent.controller_state.yaw = 0
    agent.controller_state.pitch = 0
    agent.controller_state.roll = 0
    target = agent.ball.loc
    if len(agent.teammates)>0 and  abs(agent.teammates[0].loc[1]) <= abs(agent.car.loc[1]):
        speed = 0
        return Controller_output(agent,target,speed)
    else:
        speed = 2300
        return Controller_output(agent,target,speed)

def Air_roll(agent):
    target = agent.ball.loc
    local = agent.car.matrix.dot(target)
    yaw_to = math.atan2(local[1],local[0])
    pitch_to = math.atan2(local[2],local[0])
    roll_to = math.atan2(local[1],local[2])
    agent.controller_state.yaw = steerPD(yaw_to, -agent.car.Rotvel[2]/4.5)
    agent.controller_state.pitch = steerPD(pitch_to, agent.car.Rotvel[1]/4.5)
    agent.controller_state.roll = steerPD(roll_to, agent.car.Rotvel[0])
    return agent.controller_state

def Recovery(agent):
    target = Vector3(0,5150*side(agent.team),0)
    speed = 1410
    return Controller_output(agent,target,speed)

def Shooting(agent):
    agent.controller_state.yaw = 0
    agent.controller_state.pitch = 0
    agent.controller_state.roll = 0
    ballgoal = agent.ball.loc - Vector3(0, -5150 * side(agent.team),0)
    target = agent.ball.loc + ((ballgoal + agent.ball.vel).normalize() * ((agent.car.loc - agent.ball.loc).magnitude() / 2))
    speed = 1600
    return Controller_flip(agent,target, speed)

def Controller_output(agent,target,speed):
    Controller = SimpleControllerState()
    LocalTarget = agent.car.matrix.dot(target-agent.car.loc)
    angle_target = math.atan2(LocalTarget[1],LocalTarget[0])
    Controller.steer = steer(angle_target)
    agent_speed = agent.car.matrix.dot(agent.car.vel)[0]
    Controller.throttle, Controller.boost = throttle(speed, agent_speed)
    if abs(angle_target) > 2:
        Controller.handbrake = True
    else:
        Controller.handbrake = False

    if abs(angle_target) < math.radians(10) and (agent.car.loc - agent.ball.loc).magnitude() < 1000:
        return frontflip(agent)
    return Controller

def Controller_flip(agent,target,speed):
    Controller = SimpleControllerState()
    LocalTarget = agent.car.matrix.dot(target-agent.car.loc)
    angle_target = math.atan2(LocalTarget[1],LocalTarget[0])
    Controller.steer = steer(angle_target)
    agent_speed = agent.car.matrix.dot(agent.car.vel)[0]
    turn_radius = 156 + 0.1*agent_speed + 0.000069*agent_speed**2 + 0.000000164*agent_speed**3 + -5.62E-11*agent_speed**4
    pointright = Vector3(agent.car.rot[1], agent.car.rot[0] * -1, 0) * turn_radius
    pointleft = Vector3(agent.car.rot[1] * 1, agent.car.rot[0], 0) * turn_radius
    ball2d = Vector3(agent.ball.loc[0], agent.ball.loc[1], 0)
    if (ball2d - pointright).flatten().magnitude() < turn_radius or (ball2d - pointleft).flatten().magnitude() < turn_radius:
        speed = 0
    else:
        speed = 1410
    Controller.throttle,Controller.boost = throttle(speed, agent_speed)
    if abs(angle_target) > 2:
        Controller.handbrake = True
    else:
        Controller.handbrake = False
    if abs(angle_target) < math.radians(10) and (agent.car.loc - agent.ball.loc).magnitude() < 300:
        return frontflip(agent)
    return Controller

def frontflip(agent):
    agent.controller_state.yaw = 0
    agent.controller_state.pitch = 0
    agent.controller_state.roll = 0
    time_difference = time.time() - agent.start
    if time_difference > 2.2:
        agent.start = time.time()
    elif time_difference <= 0.1:
        agent.controller_state.jump = True
        agent.controller_state.pitch = -1
    elif time_difference >= 0.1 and time_difference <= 0.15:
        agent.controller_state.jump = False
        agent.controller_state.pitch = -1
    elif time_difference > 0.15 and time_difference < 1:
        agent.controller_state.jump = True
        agent.controller_state.yaw = agent.controller_state.steer
        agent.controller_state.pitch = -1
    elif time_difference > 1.00 and time_difference < 2:
        agent.controller_state.jump = False
    return agent.controller_state

def side(x):
    if x <= 0:
        return -1
    return 1

def cap(x, low, high):
    if x < low:
        return low
    elif x > high:
        return high
    else:
        return x
    
def steer(angle):
    final = ((35 * angle)**3) / 20
    return cap(final,-1,1)

def steerPD(angle,rate):
    final = ((35*(angle+rate))**3)/20
    return cap(final,-1,1)

def velocity3D(target_object):
    return math.sqrt(target_object.vel[0]**2 + target_object.vel[1]**2 + target_object.vel[2]**2)

def throttle(speed, agent_speed):  
    final = ((speed - agent_speed)/100)
    if final > 1:
        boost = True
    else:
        boost = False
    if final > 0 and speed > 1400:
        final = 1
    return cap(final,-1,1),boost,
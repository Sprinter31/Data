#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


ev3 = EV3Brick()

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

cs_left = ColorSensor(Port.S2)
cs_right = ColorSensor(Port.S3)
cl = ColorSensor(Port.S4)  # Kreuzungs Sensor
gyro = GyroSensor(Port.S1)

WHEEL_DIAMETER_MM = 56
AXLE_TRACK_MM = 155
drive = DriveBase(left_motor, right_motor, WHEEL_DIAMETER_MM, AXLE_TRACK_MM)

stripe_width = 35  # mm

N, E, S, W = 0, 1, 2, 3
def left_of(d): return (d + 3) % 4
def right_of(d): return (d + 1) % 4
def back_of(d): return (d + 2) % 4


color_codes = [Color.RED, Color.YELLOW]

SPEED = 150
P_GAIN = 3

intersections = []

driven_intersections = []
driven_intersections_new = []

last_intersection = None
last_came_from = None

# came from is the direction the bot came from when entering the intersection
class Intersection:
    def __init__(self, cur_id):
        self.id = cur_id
        self.edges = []

class Edge:
    def __init__(self, next_intersection, direction, length):
        self.next_intersection = next_intersection
        self.direction = direction
        self.length = length


def get_or_create_intersection(cur_id):
    for intersection in intersections:
        if intersection.id == cur_id:
            return intersection

    intersection = Intersection(cur_id)
    intersections.append(intersection)
    return intersection


def get_next_intersection_dir(intersection):
    current_dir = get_cur_dir()
    prio = [
        right_of(current_dir),
        current_dir,
        left_of(current_dir),
        back_of(current_dir)
    ]

    directions = []
    for edge in intersection.edges:
        directions.append(edge.direction)

    for direction in prio:
        if direction not in directions:
            return direction

    return None


def get_dir(angle):
    angle = angle % 360
    return int((angle + 45) // 90) % 4  # // means div


def get_cur_dir():
    return get_dir(gyro.angle())


def turn_to_dir(direction):
    angle_target = direction * 90

    delta = (angle_target - gyro.angle() + 180) % 360 - 180
    drive.turn(delta)


def follow_line():
    left = cs_left.reflection()
    right = cs_right.reflection()

    error = left - right

    turn_rate = P_GAIN * error

    drive.drive(SPEED, turn_rate)


def drive_to_next_intersection():
    while not is_on_intersection():
        follow_line()
    drive.stop()
    wait(100)


def leave_intersection():
    drive.straight(40)


def fully_explored():
    for intersection in intersections:
        if get_next_intersection_dir(intersection) is not None:
            return False

    return True


def edge_exists(intersection, direction):
    for edge in intersection.edges:
        if edge.direction == direction:
            return True
    return False


def find_intersection(cur_id):
    for it in intersections:
        if it.id == cur_id:
            return it
    return None


def scan_intersection_id():
    intersection_id = 0
    for i in range(3):
        try:
            drive.stop()
            wait(500)
            intersection_id += color_codes.index(cl.color()) * 3 ** (2 - i)
            drive.straight(stripe_width)
        except ValueError:
            return None

    print(intersection_id)
    return intersection_id


def get_current_intersection():
    came_from = get_cur_dir()
    current_intersection_id = scan_intersection_id()
    if current_intersection_id is None:
        return None, None
    ev3.speaker.say(str(current_intersection_id))
    current_intersection = get_or_create_intersection(current_intersection_id)
    return current_intersection, came_from


def is_on_intersection():
    color = cl.color()
    return color == Color.YELLOW or color == Color.RED

def drive_to_last_drivable_intersection():
    for i in range(len(driven_intersections) - 1, -1, -1):
        intersection = driven_intersection[i]
        if get_next_intersection_dir(intersection) is not None:
            drive_to_intersection(intersection)
            break

def get_path(cur_intersection, target_intersection):
    return None

def drive_to_intersection(cur_intersection, target_intersection):
    path = get_path(cur_intersection, target_intersection) # should contain the current intersection
    for i, intersection in enumerate(path):
        if i == len(path) - 1: 
            break
        for edge in intersection.edges:
            if edge.next_intersection == path[i + 1]:
                next_dir = edge.direction
    
    turn_to_dir(next_dir)
    drive_to_next_intersection()
    current_intersection = get_current_intersection()


def main():
    global last_intersection
    gyro.reset_angle(0)
    drive.reset()
    wait(500)

    while True:
        if is_on_intersection():
            print(cl.color())
            current_intersection, came_from = get_current_intersection()

            if current_intersection is None: # means scanning failed
                continue

            driven_distance = drive.distance()
            
            # add edges
            if last_intersection is not None:
                dir_from_last_to_current = back_of(came_from)
                print('current dir: ', get_cur_dir())
                if not edge_exists(last_intersection, dir_from_last_to_current):
                    last_intersection.edges.append(
                    Edge(current_intersection, last_came_from), driven_distance)) # here ts
                    print('last intersection: ', last_intersection.id)
                    for edge in last_intersection.edges:
                        print(' ')
                        print('next intersection:', edge.next_intersection.id,
                        'dir:', edge.direction,
                        'length:', edge.length)
                

                if not edge_exists(current_intersection, came_from):
                    current_intersection.edges.append(
                    Edge(last_intersection, back_of(came_from), driven_distance))
                    print('cur intersection: ', current_intersection.id)
                    for edge in current_intersection.edges:
                        print(' ')
                        print('next intersection:', edge.next_intersection.id,
                        'dir:', edge.direction,
                        'length:', edge.length)
                

            last_intersection = current_intersection
            last_came_from = came_from
            driven_intersections.append(current_intersection)



            drive.reset()

            next_dir = get_next_intersection_dir(current_intersection)

            if next_dir is not None:
                drive.straight(30)
                turn_to_dir(next_dir)
                leave_intersection()
                drive.reset()
                last_intersection = current_intersection
                continue
                
            elif (get_next_intersection_dir(current_intersection) is None) and (not fully_explored()):  # intersections can be driven
                drive_to_last_drivable_intersection()  # drive to intersection that can be driven
            else:  # fully explored
                break

        else:
            follow_line()


main()

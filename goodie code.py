#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Color
from pybricks.tools import wait
from pybricks.robotics import DriveBase
import math

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

stripe_width = 25  # mm

N, E, S, W = 0, 1, 2, 3
def left_of(d): return (d + 3) % 4
def right_of(d): return (d + 1) % 4
def back_of(d): return (d + 2) % 4


color_codes = [Color.RED, Color.GREEN, Color.YELLOW]

SPEED = 100
P_GAIN = SPEED / 33

intersections = []

driven_intersections = []
driven_intersections_new = []


class Intersection:
    def __init__(self, cur_id, direction):
        self.id = cur_id
        self.befahreneRichtungen = []
        self.came_from = direction


def get_or_create_intersection(cur_id, came_from):
    for intersection in intersections:
        if intersection.id == cur_id:
            intersection.came_from = came_from
            return intersection

    current_dir = get_cur_dir()
    intersection = Intersection(cur_id, came_from)
    intersection.befahreneRichtungen.append(back_of(current_dir))
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

    for direction in prio:
        if direction not in intersection.befahreneRichtungen:
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
            wait(50)
            intersection_id += color_codes.index(cl.color()) * 3 ** (2 - i)
            drive.straight(stripe_width)
        except ValueError:
            return None

    return intersection_id


def get_current_intersection():
    came_from = back_of(get_cur_dir())
    current_intersection_id = scan_intersection_id()
    if current_intersection_id is None:
        return None
    current_intersection = get_or_create_intersection(current_intersection_id, came_from)
    return current_intersection


def is_on_intersection():
    color = cl.color()
    return color == Color.GREEN or color == Color.YELLOW or color == Color.RED


def drive_to_last_drivable_intersection():
    global driven_intersections
    global driven_intersections_new

    for i in range(len(driven_intersections) - 1, -1, -1):
        last_intersection = driven_intersections[i]
        turn_to_dir(last_intersection.came_from)
        drive_to_next_intersection()
        new_intersection = get_current_intersection()
        driven_intersections_new.append(new_intersection)

        if get_next_intersection_dir(new_intersection) is not None:
            driven_intersections = driven_intersections_new[:]
            driven_intersections_new.clear()
            break

        leave_intersection()


def main():
    gyro.reset_angle(0)
    while True:
        if is_on_intersection():
            current_intersection = get_current_intersection()

            if current_intersection is None:
                continue

            driven_intersections.append(current_intersection)

            if not get_next_intersection_dir(current_intersection) is None:  # if direction found
                next_dir = get_next_intersection_dir(current_intersection)
                turn_to_dir(next_dir)
                current_intersection.befahreneRichtungen.append(next_dir)
                leave_intersection()
            elif (get_next_intersection_dir(current_intersection) is None) and (not fully_explored()):  # intersections can be driven
                drive_to_last_drivable_intersection()  # drive to intersection that can be driven
            else:  # fully explored
                break

        else:
            follow_line()


main()

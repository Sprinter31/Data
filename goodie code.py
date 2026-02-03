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


# color codes
color_codes = [Color.RED, Color.GREEN, Color.YELLOW]

SPEED = 100
P_GAIN = SPEED / 33

intersections = []

driven_intersections = []


class Intersection:
    def __init__(self, cur_id, direction):
        self.id = cur_id
        self.befahreneRichtung = []
        self.last_came_in_dir = direction


def get_or_create_intersection(cur_id):
    for intersection in intersections:
        if intersection.id == cur_id:
            return intersection

    current_dir = get_cur_dir()
    intersection = Intersection(cur_id, current_dir)

    intersection.befahreneRichtung.append(back_of(current_dir))

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
        if direction not in intersection.befahreneRichtung:
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


def scan_intersection_id():
    intersection_id = 0
    for i in range(3):
        try:
            intersection_id += color_codes.index(cl.color()) * 3 ** (2 - i)
            drive.straight(stripe_width)
        except ValueError:
            return None

    return intersection_id


def is_on_intersection():
    color = cl.color()
    return color == Color.GREEN or color == Color.YELLOW or color == Color.RED


def drive_to_last_drivable_intersection():
    for i in range(len(driven_intersections) - 1, -1, -1):
        cur_intersection = driven_intersections[i]
        turn_to_dir(back_of(cur_intersection.last_came_in_dir))
        drive_to_next_intersection()

        if get_next_intersection_dir(cur_intersection) is not None:
            break

        leave_intersection()


def main():
    while True:
        if is_on_intersection():
            current_intersection_id = scan_intersection_id()

            if current_intersection_id is None:
                continue

            current_intersection = get_or_create_intersection(current_intersection_id)
            intersection.last_came_in_dir = back_of(get_cur_dir())
            driven_intersections.append(current_intersection)

            if any(i.id == current_intersection.id for i in intersections):  # known intersection
                if not get_next_intersection_dir(intersection) is None:  # if direction found
                    next_dir = get_next_intersection_dir(intersection)
                    turn_to_dir(next_dir)
                    current_intersection.befahreneRichtung.append(next_dir)
                elif (get_next_intersection_dir(intersection) is None) and (not fully_explored()):  # intersections can be driven
                    drive_to_last_drivable_intersection()  # drive to intersection that can be driven
                else:  # fully explored
                    break

        else:
            follow_line()


main()

#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import heapq


ev3 = EV3Brick()

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

cs_left = ColorSensor(Port.S2)
cs_right = ColorSensor(Port.S3)
cl = ColorSensor(Port.S4)
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

last_intersection = None
last_dir_exited = None

start_intersection = None
goal_intersection_id = 7


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


def get_graph():
    graph = {}
    for intersection in intersections:
        graph[intersection] = {}

    for intersection in intersections:
        for edge in intersection.edges:
            graph[intersection][edge.next_intersection] = edge.length
    return graph


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

    wait(50)
    snap_gyro_to_grid()


#def align_to_line(max_ms=1500):
#    sw = StopWatch()
#    while sw.time() < max_ms:
#        left = cs_left.reflection()
#        right = cs_right.reflection()
#        err = left - right
#
#        if abs(err) < 2:
#            drive.stop()
#            wait(50)
#            return True
#
#        drive.drive(0, 6 * err)
#        wait(10)
#
#    drive.stop()
#    return False


# test ts
def snap_gyro_to_grid():
    a = gyro.angle()
    snapped = int(round(a / 90.0)) * 90
    gyro.reset_angle(snapped)


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


def drive_with_line_following(distance_mm):
    start = drive.distance()
    while drive.distance() - start < distance_mm:
        follow_line()
    drive.stop()
    wait(50)


def find_intersection(cur_id):
    for it in intersections:
        if it.id == cur_id:
            return it
    return None


def scan_intersection_id():
    intersection_id = 0
    for i in range(3):
        drive.stop()
        wait(250)

        c = cl.color()
        if c not in color_codes:
            return None

        intersection_id += color_codes.index(c) * 2 ** (2 - i)
        drive_with_line_following(stripe_width)

    return intersection_id


def get_current_intersection():
    current_intersection_id = scan_intersection_id()
    if current_intersection_id is None:
        return None
    ev3.speaker.say(str(current_intersection_id))
    current_intersection = get_or_create_intersection(current_intersection_id)
    return current_intersection


def is_on_intersection():
    color = cl.color()
    return color == Color.YELLOW or color == Color.RED


def drive_to_last_drivable_intersection(current_intersection):
    for i in range(len(driven_intersections) - 1, -1, -1):
        target_intersection = driven_intersections[i]
        if get_next_intersection_dir(target_intersection) is not None:
            drive_to_intersection(current_intersection, target_intersection)
            break


def get_path(cur_intersection, target_intersection):
    return dijkstra(cur_intersection, target_intersection)


def dijkstra(start, end):
    graph = get_graph()
    distances = {node: float('inf') for node in graph}
    distances[start] = 0

    previous = {node: None for node in graph}

    pq = [(0, start.id, start)]
    visited = set()

    while pq:
        current_dist, _, current = heapq.heappop(pq)

        if current in visited:
            continue

        visited.add(current)

        if current == end:
            break

        for neighbor, weight in graph[current].items():
            distance = current_dist + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous[neighbor] = current
                heapq.heappush(pq, (distance, neighbor.id, neighbor))

    if distances[end] == float('inf'):
        return None

    cur_path = []
    current = end
    while current is not None:
        cur_path.append(current)
        current = previous[current]
    cur_path.reverse()

    return cur_path


def drive_to_intersection(cur_intersection, target_intersection):
    path = get_path(cur_intersection, target_intersection)  # should contain the current intersection
    if path is None:
        ev3.speaker.say('no path')
        return

    for i in range(len(path) - 1):
        it_a = path[i]
        it_b = path[i + 1]

        next_dir = None
        for edge in it_a.edges:
            if edge.next_intersection == it_b:
                next_dir = edge.direction
                break

        if next_dir is None:
            ev3.speaker.say('missing edge')
            return

        turn_to_dir(next_dir)
        leave_intersection()
        drive.reset()
        drive_to_next_intersection()
        current = get_current_intersection()

        if current is None or current.id != it_b.id:
            ev3.speaker.say('wrong intersection')
            return


def main():
    global last_intersection
    global last_dir_exited
    global start_intersection
    gyro.reset_angle(0)
    drive.reset()
    wait(500)

    while True:
        if is_on_intersection():
            current_intersection = get_current_intersection()

            if current_intersection is None:  # means scanning failed
                continue

            if start_intersection is None:
                start_intersection = current_intersection

            driven_distance = drive.distance()
            
            # add edges
            if last_intersection is not None:
                print('current dir: ', get_cur_dir())
                if not edge_exists(last_intersection, last_dir_exited):
                    last_intersection.edges.append(Edge(current_intersection, last_dir_exited, driven_distance))
                    print('last intersection: ', last_intersection.id)
                    for edge in last_intersection.edges:
                        print(' ')
                        print('next intersection:', edge.next_intersection.id,
                        'dir: ', edge.direction,
                        'length: ', edge.length)

                if not edge_exists(current_intersection, back_of(get_cur_dir())):
                    current_intersection.edges.append(Edge(last_intersection, back_of(get_cur_dir()), driven_distance))
                    print('cur intersection: ', current_intersection.id)
                    for edge in current_intersection.edges:
                        print(' ')
                        print('next intersection:', edge.next_intersection.id,
                        'dir:', edge.direction,
                        'length:', edge.length)

            last_intersection = current_intersection
            driven_intersections.append(current_intersection)

            next_dir = get_next_intersection_dir(current_intersection)

            if next_dir is not None:
                last_dir_exited = next_dir

                drive.straight(30)
                turn_to_dir(next_dir)
                leave_intersection()
                drive.reset()
                last_intersection = current_intersection
                continue
                
            elif (get_next_intersection_dir(current_intersection) is None) and (not fully_explored()):  # intersections can be driven
                drive_to_last_drivable_intersection(current_intersection)  # drive to intersection that can be driven
            else:  # fully explored
                drive_to_intersection(current_intersection, start_intersection)

                goal = find_intersection(goal_intersection_id)
                if goal is None:
                    ev3.speaker.say("goal unknown")
                    break

                drive_to_intersection(start_intersection, goal)
                break

        else:
            follow_line()


main()

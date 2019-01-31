import math

DISTANCE_LIMIT = 1000

def closest_object_in_range(range_data, left_boundary_angle, right_boundary_angle, min_angle, angle_increment):
    left_boundary_index = scan_angle_to_range_index(left_boundary_angle, min_angle, angle_increment)
    right_boundary_index = scan_angle_to_range_index(right_boundary_angle, min_angle, angle_increment)

    distance = DISTANCE_LIMIT
    angle_index = (left_boundary_index + right_boundary_index) / 2

    for i in range(left_boundary_index, right_boundary_index):
        current_distance = range_data[i]
        if current_distance < distance:
            distance = current_distance
            angle_index = i

    angle = range_index_to_angle(angle_index, min_angle, angle_increment)
    return (angle, distance)


def scan_angle_to_range_index(angle, min_angle, angle_increment):
    return int(math.floor((angle - min_angle)/angle_increment))


def range_index_to_angle(index, min_angle, angle_increment):
    return math.floor((index * angle_increment) + min_angle)
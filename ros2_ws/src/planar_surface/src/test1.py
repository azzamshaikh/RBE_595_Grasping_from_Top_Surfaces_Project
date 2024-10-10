#!/usr/bin/python3
import math

def check_vector_intersection(vector1, vector2):
    """
    Checks if two normal vectors intersect in 2D space, considering their direction.

    Args:
        vector1: A tuple containing (x, y, theta) of the first normal vector.
        vector2: A tuple containing (x, y, theta) of the second normal vector.

    Returns:
        A tuple containing the slope of the intersection line and the theta values
        between each normal and the intersection line, or None if there's no intersection.
    """

    # Extract values from vectors
    x1, y1, theta1 = vector1
    x2, y2, theta2 = vector2

    # Convert theta to radians
    theta1_rad = math.radians(theta1)
    theta2_rad = math.radians(theta2)

    # Calculate direction vectors
    dx1 = round(math.cos(theta1_rad),5)
    dy1 = round(math.sin(theta1_rad),5)
    dx2 = round(math.cos(theta2_rad),5)
    dy2 = round(math.sin(theta2_rad),5)

    print("x1: ",x1)
    print("y1: ",y1)
    print("theta_1:",theta1,"deg.,",theta1_rad,"rad.")
    print("x2: ",x2)
    print("y2: ",y2)
    print("theta_2:",theta2,"deg.,",theta2_rad,"rad.")
    print("dx1:",dx1)
    print("dy1:",dy1)
    print("dx2:",dx2)
    print("dy2:",dy2)

    # Check if lines are parallel
    # if dx1 * dy2 - dx2 * dy1 == 0:
    #     print("Error: Vectors are parallel and do not intersect.")
    #     return None

    # Check if vectors point away from each other
    if (dx1 * (x2 - x1) + dy1 * (y2 - y1)) * (dx2 * (x1 - x2) + dy2 * (y1 - y2)) > 0:
        print("Error: Vectors point away from each other and do not intersect.")
        return None

    # Calculate intersection point
    t1 = (y2 - y1 + dx2 * x1 - dx1 * x2) / (dx2 * dy1 - dx1 * dy2)
    t2 = (y2 - y1 + dx1 * x2 - dx2 * x1) / (dx1 * dy2 - dx2 * dy1)
    intersection_point = (x1 + t1 * dx1, y1 + t1 * dy1)
    print("Intersection point: (",intersection_point[0],",",intersection_point[1],")")

    # Calculate slope of intersection line
    intersection_slope = (intersection_point[1] - y1) / (intersection_point[0] - x1)
    print("intersection line slope: ", intersection_slope)

    # Calculate slope of the two normals
    normal_1_slope = (dy1-y1)/(dx1-x1)
    normal_2_slope = (dy2-y2)/(dx2-x2)
    print("Normal 1 slope:",normal_1_slope)
    print("Normal 2 slope:",normal_2_slope)

    # Calculate theta values between normals and intersection line
    # theta_intersection1 = math.atan2(intersection_slope, 1)
    # theta_intersection2 = math.atan2(intersection_slope, 1)
    theta_intersection1 = math.atan2(intersection_slope, normal_1_slope)
    theta_intersection2 = math.atan2(intersection_slope, normal_2_slope)
    print("theta intersection 1:",theta_intersection1)
    print("theta intersection 2:",theta_intersection2)

    # Adjust theta values to be between 0 and 2pi
    if theta_intersection1 < 0:
        theta_intersection1 += 2 * math.pi
    if theta_intersection2 < 0:
        theta_intersection2 += 2 * math.pi

    return intersection_slope, theta_intersection1, theta_intersection2


def calculate_angle(point1, theta1, point2, theta2):
    """
    Calculates the angles between each normal and the intersection line of the two points.

    Args:
        point1: The starting point of the first ray line.
        theta1: The angle between the first ray line and the positive x-axis.
        point2: The starting point of the second ray line.
        theta2: The angle between the second ray line and the positive x-axis.

    Returns:
        A tuple containing the two angles.
    """

    # Calculate the intersection point of the two lines
    x1, y1 = point1
    x2, y2 = point2
    m1 = math.tan(math.radians(theta1))
    m2 = math.tan(math.radians(theta2))
    print("m1",m1)
    print("m2:",m2)
    x_intersect = (m1 * x1 - m2 * x2 + y2 - y1) / (m1 - m2)
    y_intersect = m1 * (x_intersect - x1) + y1
    x_intersect = round(x_intersect,8)
    y_intersect = round(y_intersect,8)
    print("x_intersect:",x_intersect)
    print("y_intersect:",y_intersect)

    # Calculate the angles between each normal and the intersection line
    angle1 = math.degrees(math.atan2(y_intersect - y1, x_intersect - x1)) - (math.pi/4) #theta1
    angle2 = math.degrees(math.atan2(y_intersect - y2, x_intersect - x2)) - theta2
    intersection_line_angle = math.degrees(math.atan2(y2-y1,x2-x1))
    print("intersection line angle:",intersection_line_angle)

    angle1_rad = math.radians(angle1)
    angle2_rad = math.radians(angle2)
    if angle1_rad < 0:
        angle1_rad += 2 * math.pi
    if angle2_rad < 0:
        angle2_rad += 2 * math.pi
    return angle1_rad, angle2_rad


def main(args=None):
    # Example usage
    # vector1 = (0, 0, 45)
    # vector2 = (10, 10, 225)
    # vector1 = (0, 0, 90)
    # vector2 = (0, 10, 270)
    vector1 = (0, 0, 0)
    vector2 = (10, 10, 225)

    # result = check_vector_intersection(vector1, vector2)
    # if result:
    #     slope, theta1, theta2 = result
    #     print("Slope of intersection line:", slope)
    #     print("Theta1:", math.degrees(theta1))
    #     print("Theta2:", math.degrees(theta2))

    # p1 = (0,0)
    # theta_1 = 0
    # p2 = (10,10)
    # theta_2 = 225
    p1 = (0,0)
    theta_1 = 45
    p2 = (0,10)
    theta_2 = 270

    result = calculate_angle(p1,theta_1,p2,theta_2)
    print("result [rad.]: ",result)
    angle1_rad,angle2_rad = result
    angle1_deg = math.degrees(angle1_rad)
    angle2_deg = math.degrees(angle2_rad)
    result_deg = (angle1_deg,angle2_deg)
    print("result [deg.]",result_deg)

if __name__ == '__main__':
    main()
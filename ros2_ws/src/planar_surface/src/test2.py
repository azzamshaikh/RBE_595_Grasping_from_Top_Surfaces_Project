#!/usr/bin/python3
import numpy as np


def calculate_angle(v1, v2):
    """

    """
    # Calculate the dot product and the magnitudes of the vectors
    dot_product = np.dot(v1, v2)
    dot_product = round(dot_product,8)

    print("dot_product:",dot_product)
    
    magnitude_v1 = np.linalg.norm(v1)
    magnitude_v2 = np.linalg.norm(v2)

    print("magnitude_v1:",magnitude_v1)
    print("magnitude_v2:",magnitude_v2)
    
    # Calculate the angle in radians
    angle = np.arccos(dot_product / (magnitude_v1 * magnitude_v2))

    print("angle: ",np.degrees(angle),"deg.")
    return np.degrees(angle)  # Convert to degrees if needed

def vector_angle_between_points(p1, p2):
    """

    """
    # Extract the points and angles
    x1, y1, theta1 = p1
    x2, y2, theta2 = p2

    # Convert angles from degrees to radians for calculations
    # theta1_rad = np.radians(theta1)
    # theta2_rad = np.radians(theta2)
    theta1_rad = theta1
    theta2_rad = theta2

    # Create the direction vectors
    v1 = np.array([np.cos(theta1_rad), np.sin(theta1_rad)])
    v2 = np.array([np.cos(theta2_rad), np.sin(theta2_rad)])
    print("v1:",v1)
    print("v2:",v2)
    
    # Create the line vector from p1 to p2
    line_vector = np.array([x2 - x1, y2 - y1])
    print("line_vector:",line_vector)
    
    # Calculate the angles between the vectors and the line vector
    angle1 = calculate_angle(v1, line_vector)
    angle2 = calculate_angle(v2, line_vector)

    return angle1, angle2



def main(args=None):
    # Example usage: (x, y, theta [deg]) <--degrees for now to debug
    # p1 = (0.002298788886335888, 0.49688825522793273, -179.8161212539945)  # (x1, y1, theta1)
    # p2 = (0.9983957862238022, 0.5000850298796488, 0.1806434)  # (x2, y2, theta2)
    
    p1 = (0,0,0)  # angle should be 0 deg. Instead, got 90.
    p2 = (0,10,180) # angle should be 0 deg. Instead, got 143.2403...

    angles = vector_angle_between_points(p1, p2)
    print("Angles between the vectors and the intersecting line:", angles)

if __name__ == '__main__':
    main()
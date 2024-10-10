#!/usr/bin/python3
import numpy as np
import math

def calculate_angle(p1, p2, doSwitch=False):
  """Calculates the angle between the line connecting p1 and p2 and the positive x-axis."""
  
  if doSwitch:
    # Switch direction of intersection line
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
  else:
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
  return math.degrees(math.atan2(dy, dx))


def constrain_angle(theta):
    if theta < 0:
        return theta + 2*pi
    elif theta > 2*pi:
        pass
    else:
        return theta


def main(args=None):
    # Example usage # Given points
    # p1 = (0.002298788886335888, 0.49688825522793273, -179.8161212539945)
    # p2 = (0.9983957862238022, 0.5000850298796488, 0.1806434)
    p1 = (0,0,0)      # should output 0 deg.
    p2 = (0,10,180)
    # p1 = (0,0,0)        # should output 0 deg.
    # p2 = (10,0,180)     # should output 0 deg.

    # Calculate the angle of the vector between the points
    angle_between_points_1 = calculate_angle(p1[:2], p2[:2])
    angle_between_points_2 = calculate_angle(p1[:2], p2[:2],True)

    print("angle_between_points_1:",angle_between_points_1)
    print("angle_between_points_2:",angle_between_points_2)

    # Calculate the angle differences
    angle_p1 = np.abs(p1[2]-angle_between_points_1)
    angle_p2 = np.abs(p2[2]-angle_between_points_2)
    

    print("Angle between the line and p1:", angle_p1)
    print("Angle between the line and p2:", angle_p2)

if __name__ == '__main__':
    main()
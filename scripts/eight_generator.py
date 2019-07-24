#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Point on a circle, defined by an angle
def circle_point_ang(orig, radius, angle):
    x = radius * np.cos(angle) + orig[0]
    y = radius * np.sin(angle) + orig[1]
    return np.array([x, y])

def sample_arc(orig, radius, start_ang, sample_dist, orientation = 1):
    angle = start_ang + orientation*(sample_dist/radius)
    pt = circle_point_ang(orig, radius, angle)
    return pt

def sample_arc_tangent(orig, radius, start_ang, sample_dist, orientation = 1):
    angle = start_ang + orientation*(sample_dist/radius)
    tangent = angle + orientation*np.pi/2.0
    return tangent

def sample_line(line_start, line_end, sample_dist):
    vec = line_end - line_start
    dist = np.linalg.norm(vec)
    dir = vec/dist
    pt = line_start + dir*sample_dist
    return pt

def sample_line_tangent(line_start, line_end, sample_dist):
    vec = line_end - line_start
    tangent = np.arctan2(vec[1], vec[0])
    return tangent

def ypr_to_R(yaw, pitch, roll):
    yaw_mat = np.matrix([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    pitch_mat = np.matrix([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    roll_mat = np.matrix([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    R = roll_mat*pitch_mat*yaw_mat
    return R

def normalize_angle(angle):
    out = np.fmod(angle, 2*np.pi);
    if out > np.pi:
      out -= 2*np.pi
    elif out < -np.pi:
      out += 2*np.pi
    return out

# from https://stackoverflow.com/questions/53033620/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr?rq=1
def ypr_to_quaternion(yaw, pitch, roll):
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        return [qw, qx, qy, qz]

def save_data(data, ofname, col_names):
    with open(ofname, 'w') as of:
        txt = ",".join(col_names)
        of.write("{:s}\n".format(txt))
        rows = data.shape[0]
        for it in range(0, rows):
            row = data[it, :]
            txt = ",".join(map(str, row))
            of.write("{:s}\n".format(txt))

def main():
    # parameters of the pattern
    radius = 5.5 # metres
    width = 25   # metres
    lin_cutoff = 1.0 # metres, how much of the radius will be cut-off in favour of the linear segment

    # parameters of the sampling
    sample_dist = 0.5 # metres

    # parameters of transformation of the eight pattern
    pattern_rotation_ypr = [1.0, 2.0, 3.0] # radians
    pattern_translation  = [5.0, -3.0, 4.0] # metres

    ### ARCS
    # half of the angle of the circle sector, which is cut off by the linear segments
    arc_ang = np.arccos( (radius - lin_cutoff) / radius )

    # starting angle of the left arc
    l_arc_start_ang = arc_ang
    # ending angle of the left arc
    l_arc_end_ang = 2*np.pi - arc_ang
    # center of the left arc
    l_arc_orig = np.array([- width/2 + radius, 0])

    # starting angle of the right arc
    r_arc_start_ang = np.pi - arc_ang
    # ending angle of the right arc
    r_arc_end_ang = -np.pi + arc_ang
    # center of the right arc
    r_arc_orig = np.array([+ width/2 - radius, 0])

    ### LINES
    # starting point of the first line segment
    l_line_start = circle_point_ang(l_arc_orig, radius, l_arc_end_ang)
    # ending point of the first line segment
    l_line_end = circle_point_ang(r_arc_orig, radius, r_arc_start_ang)

    # starting point of the second line segment
    r_line_start = circle_point_ang(r_arc_orig, radius, r_arc_end_ang)
    # ending point of the second line segment
    r_line_end = circle_point_ang(l_arc_orig, radius, l_arc_start_ang)

    ### SAMPLING
    # perimeter of either of the arcs
    arc_len = radius * (2*np.pi - 2*arc_ang)
    # length of either of the lines
    line_len = np.linalg.norm(l_line_end - l_line_start)
    # total length of the eight pattern
    tot_len = 2*arc_len + 2*line_len

    # Recalculate the sample distance to obtain a full number of sample points
    n_pts = int(np.floor(tot_len/sample_dist))
    sample_dist = tot_len/n_pts

    samples = np.zeros((n_pts, 2))
    yaws = np.zeros((n_pts, 1))
    curvatures = np.zeros((n_pts, 1))
    for it in range(0, n_pts):
        sector = 0 # sector 0 is the first (left) arc
        cur_dist = (it*sample_dist) % tot_len # current distance traveled along the eight-pattern
        cur_sector_dist = cur_dist # current distance traveled in the current sector
        if cur_dist > arc_len:
            sector = 1 # sector 1 is the first ("left") line
            cur_sector_dist = cur_dist - arc_len
        if cur_dist > arc_len + line_len:
            sector = 2 # sector 2 is the second (right) arc
            cur_sector_dist = cur_dist - (arc_len + line_len)
        if cur_dist > 2*arc_len + line_len:
            sector = 3 # sector 3 is the second ("right") line
            cur_sector_dist = cur_dist - (2*arc_len + line_len)

        cur_sample = None
        cur_yaw = None
        cur_curvature = None
        if sector == 0:
            cur_sample = sample_arc(l_arc_orig, radius, l_arc_start_ang, cur_sector_dist)
            cur_yaw = sample_arc_tangent(l_arc_orig, radius, l_arc_start_ang, cur_sector_dist)
            cur_curvature = 1.0/radius
        elif sector == 1:
            cur_sample = sample_line(l_line_start, l_line_end, cur_sector_dist)
            cur_yaw = sample_line_tangent(l_line_start, l_line_end, cur_sector_dist)
            cur_curvature = 0.0
        elif sector == 2:
            cur_sample = sample_arc(r_arc_orig, radius, r_arc_start_ang, cur_sector_dist, -1)
            cur_yaw = sample_arc_tangent(r_arc_orig, radius, r_arc_start_ang, cur_sector_dist)
            cur_curvature = -1.0/radius
        elif sector == 3:
            cur_sample = sample_line(r_line_start, r_line_end, cur_sector_dist)
            cur_yaw = sample_line_tangent(r_line_start, r_line_end, cur_sector_dist)
            cur_curvature = 0.0

        samples[it, :] = cur_sample
        yaws[it, :] = normalize_angle(cur_yaw)
        curvatures[it, :] = cur_curvature
    
    samples3D = np.hstack([samples, np.zeros((n_pts, 1))])
    R = ypr_to_R(pattern_rotation_ypr[0], pattern_rotation_ypr[1], pattern_rotation_ypr[2])
    samples3D = np.dot(R, samples3D.transpose())
    trans = np.matrix([pattern_translation]).transpose()
    samples3D += trans
    samples3D = np.array(samples3D.transpose())

    speeds = sample_dist*np.ones((n_pts, 1)) # assuming dt = 1s

    data = np.hstack([samples3D, yaws, speeds, curvatures])
    save_data(data, "data.csv", ["x", "y", "z", "yaw", "s", "c"])

    quat = ypr_to_quaternion(pattern_rotation_ypr[0], pattern_rotation_ypr[1], pattern_rotation_ypr[2])
    print("Corresponding quaternion: [{:f}, {:f}, {:f}, {:f}]".format(quat[0], quat[1], quat[2], quat[3]))

    speed_check = np.linalg.norm(samples3D[1:, :] - samples3D[0:-1, :], axis=1)
    print(speed_check)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(samples3D[:, 0].flatten(), samples3D[:, 1].flatten(), samples3D[:, 2].flatten())
    ax.plot(samples3D[:, 0].flatten(), samples3D[:, 1].flatten(), samples3D[:, 2].flatten(), 'x')
    # ax.axis('equal')
    ax.set_aspect('equal')
    plt.show()

    # for it in range(0, len(samples)):
    #     plt.plot(samples[:it, 0], samples[:it, 1], 'x')
    #     plt.axis('equal')
    #     plt.xlim([-15, 15])
    #     plt.ylim([-10, 10])
    #     plt.show()


if __name__ == "__main__":
    main()

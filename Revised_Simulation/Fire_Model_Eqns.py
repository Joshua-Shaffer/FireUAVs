import pygame
import math


class FireSimulator(object):

    def __init__(self, domain, starting_loc, burn_length, U=0.0, theta=0.0, I_r=0.0, eta_p=0.0, rho_b=0.0, eta=0.0, Q_ig=0.0):
        self.domain = domain
        self.wind_speed = U
        self.wind_direction = theta
        self.I_r = I_r
        self.eta_p = eta_p
        self.rho_b = rho_b
        self.eta = eta
        self.Q_ig = Q_ig
        self.burn_time = burn_length  # min

        '''
        initialize variables for use in simulation (done for testing as of now)
        '''
        self.fire_list = list()
        self.burn_list = dict()
        vertex_hist_temp = list()

        self.number_pts = 15
        self.initial_radius = 3.0
        self.perimeter_resolution = 10.0

        for start in starting_loc:
            vertex_pts1 = list()
            for i in range(0, self.number_pts):
                vertex_pts1.append((self.initial_radius * math.cos(-i * 2 * math.pi / self.number_pts) + start[0],
                                    self.initial_radius * math.sin(-i * 2 * math.pi / self.number_pts) + start[1]))
            vertex_hist_temp.append(vertex_pts1)
            self.fire_list.append(vertex_hist_temp)

    def orientation(self, p, q, r):
        """
        Determines the orientation of point p to q to r (clockwise, counterclockwise, or straight line)
        :param p: tuple of x and y floats
        :param q: tuple of x and y floats
        :param r: tuple of x and y floats
        :return: 1 for clockwise, 2 for counter, 0 for straight
        """
        val = (q[1] - p[1]) * (r[0] - q[0]) - \
              (q[0] - p[0]) * (r[1] - q[1])
        if math.fabs(val) < 0.000001: return 0
        if val > 0.0:
            return 1
        else:
            return 2

    def on_segment(self, p, q, r):
        """
        Determines if point q is on the line segment from p to r. This assumes that q is on the line that intersects through
            p to r
        :param p: tuple of x and y floats
        :param q: tuple of x and y floats
        :param r: tuple of x and y floats
        :return: True or False
        """
        if min(p[0], r[0]) <= q[0] <= max(p[0],r[0]) and min(p[1], r[1]) <= q[1] <= max(p[1], r[1]):
            return True
        return False

    def intersect(self, p1, q1, p2, q2):
        """
        Determines if the line segment from p1 to q1 and the segment from p2 to q2 intersect
        :param p1: tuple of x and y floats
        :param q1: tuple of x and y floats
        :param p2: tuple of x and y floats
        :param q2: tuple of x and y floats
        :return: True or False
        """
        o1 = self.orientation(p1, q1, p2)
        o2 = self.orientation(p1, q1, q2)
        o3 = self.orientation(p2, q2, p1)
        o4 = self.orientation(p2, q2, q1)

        if o1 != o2 and o3 != o4:
            return True

        if o1 == 0 and self.on_segment(p1, p2, q1):
            return True
        if o2 == 0 and self.on_segment(p1, q2, q1):
            return True
        if o3 == 0 and self.on_segment(p2, p1, q2):
            return True
        if o4 == 0 and self.on_segment(p2, q1, q2):
            return True

        return False

    def inside_poly(self, polygon, point):
        """
        Determines if the point lies within the polygon
        :param polygon: List of tuples of x, y floats. The order defines the edges of the polygon (adjacent points in list
        connect)
        :param point: tuple of x, y floats
        :return: True or False
        """
        if len(polygon) < 3: return False

        point_extreme = (float(1e20), point[1])  # Used in creating line from point to infinity

        # Determine number of times the line from point to point_extreme intersects line segments of the polygon
        # MAJORISSUE TODO: if a point from the polygon sits on the point to point_extreme line segment, you get double count
        #            Currently avoiding the issue by initializing fires so that this doesn't happen initially
        count = 0
        next_idx = 1
        for i in range(0, len(polygon)):
            if self.intersect(polygon[i], polygon[next_idx], point, point_extreme):
                if self.orientation(polygon[i], point, polygon[next_idx]) == 0:
                    return self.on_segment(polygon[i], point, polygon[next_idx])
                count = count + 1

            next_idx = next_idx + 1
            if next_idx == len(polygon):
                next_idx = 0

        check = divmod(count, 2)
        if check[1] == 1:
            return True
        else:
            return False

    def combine_fires(self, vertex_pts_1, vertex_pts_2, size_idx):
        """
        Take two fires that are intersected and combine into one. The perimeter formed is passed as the new fire segment
        :param vertex_pts_1:
        :param vertex_pts_2:
        :param size_idx:
        :return:
        """
        intersect_sets = list()
        for idx1, pts1 in enumerate(vertex_pts_1[size_idx]):
            line1 = (pts1, vertex_pts_1[size_idx][idx1 + 1]) if idx1 < len(vertex_pts_1[size_idx]) - 1 else (
            pts1, vertex_pts_1[size_idx][0])
            intersect_pts = list()
            for idx2, pts2 in enumerate(vertex_pts_2[size_idx]):
                line2 = (pts2, vertex_pts_2[size_idx][idx2 + 1]) if idx2 < len(vertex_pts_2[size_idx]) - 1 else (
                pts2, vertex_pts_2[size_idx][0])
                if self.intersect(line1[0], line1[1], line2[0], line2[1]): intersect_pts.append(self.intersection_point(line1, line2))
            intersect_pts.sort(key=lambda p: (p[0] - pts1[0])**2 + (p[1] - pts1[1])**2)
            intersect_sets.append(intersect_pts)

        poly1 = list()
        for idx1, pts1 in enumerate(vertex_pts_1[size_idx]):
            poly1.append(pts1)
            if intersect_sets[idx1]:
                for pts in intersect_sets[idx1]:
                    poly1.append(pts)

        intersect_sets = list()
        for idx1, pts1 in enumerate(vertex_pts_2[size_idx]):
            line1 = (pts1, vertex_pts_2[size_idx][idx1 + 1]) if idx1 < len(vertex_pts_2[size_idx]) - 1 else (
            pts1, vertex_pts_2[size_idx][0])
            intersect_pts = list()
            for idx2, pts2 in enumerate(vertex_pts_1[size_idx]):
                line2 = (pts2, vertex_pts_1[size_idx][idx2 + 1]) if idx2 < len(vertex_pts_1[size_idx]) - 1 else (
                pts2, vertex_pts_1[size_idx][0])
                if self.intersect(line1[0], line1[1], line2[0], line2[1]): intersect_pts.append(self.intersection_point(line1, line2))
            intersect_pts.sort(key=lambda p: (p[0] - pts1[0])**2 + (p[1] - pts1[1])**2)
            intersect_sets.append(intersect_pts)

        poly2 = list()
        for idx1, pts1 in enumerate(vertex_pts_2[size_idx]):
            poly2.append(pts1)
            if intersect_sets[idx1]:
                for pts in intersect_sets[idx1]:
                    poly2.append(pts)

        total_edges = list()

        min = (0.0, 0.0)
        for idx1, pts1 in enumerate(poly1):
            x_assign = min[0]
            y_assign = min[1]
            if pts1[0] < min[0]: x_assign = pts1[0]
            if pts1[1] < min[1]: y_assign = pts1[1]
            min = (x_assign, y_assign)
            edge = (pts1, poly1[idx1+1]) if idx1 < len(poly1) - 1 else (
                pts1, poly1[0])
            total_edges.append(edge)
        for idx2, pts2 in enumerate(poly2):
            x_assign = min[0]
            y_assign = min[1]
            if pts2[0] < min[0]: x_assign = pts2[0]
            if pts2[1] < min[1]: y_assign = pts2[1]
            min = (x_assign, y_assign)
            edge = (pts2, poly2[idx2+1]) if idx2 < len(poly2) - 1 else (
                pts2, poly2[0])
            total_edges.append(edge)

        min_vertex = [float('inf'), 0, 1, (0.0, 0.0)]
        for idx1, pts1 in enumerate(poly1):
            dist = (pts1[0] - min[0])**2 + (pts1[1] - min[1])**2
            if dist < min_vertex[0]: min_vertex = [dist, idx1, 1, pts1]
        for idx2, pts2 in enumerate(poly2):
            dist = (pts2[0] - min[0]) ** 2 + (pts2[1] - min[1]) ** 2
            if dist < min_vertex[0]: min_vertex = [dist, idx2, 2, pts2]

        new_poly = list()
        new_poly.append(min_vertex[3])
        start_edge = (min_vertex[3], (min_vertex[3][0], min_vertex[3][1]-1.0))

        current_edge = start_edge
        next_edge = current_edge
        possible_edges = [edges for edges in total_edges if edges[0] == min_vertex[3] or edges[1] == min_vertex[3]]

        min_angle = 0.0
        for edges in possible_edges:
            calc_edge = edges
            if edges[1] == current_edge[0]:
                calc_edge = (calc_edge[1], calc_edge[0])
            vec1 = (calc_edge[1][0] - calc_edge[0][0], calc_edge[1][1] - calc_edge[0][1])
            vec2 = (current_edge[1][0] - current_edge[0][0], current_edge[1][1] - current_edge[0][1])
            vec1 = (vec1[0] / math.sqrt(vec1[0] ** 2 + vec1[1] ** 2),
                     vec1[1] / math.sqrt(vec1[0] ** 2 + vec1[1] ** 2))
            vec2 = (vec2[0] / math.sqrt(vec2[0] ** 2 + vec2[1] ** 2),
                     vec2[1] / math.sqrt(vec2[0] ** 2 + vec2[1] ** 2))
            calc_angle = math.acos(vec1[0]*vec2[0] + vec1[1]*vec2[1])
            cross_prod = vec2[0]*vec1[1] - vec2[1]*vec1[0]
            if cross_prod <= 0.0: calc_angle = 2*math.pi - calc_angle
            if calc_angle > min_angle:
                next_edge = edges
                min_angle = calc_angle
        total_edges.remove(next_edge)
        current_edge = next_edge
        next_vertex = next_edge[0] if next_edge[0] not in new_poly else next_edge[1]
        new_poly.append(next_vertex)

        count = 1
        while True:
            possible_edges = [edges for edges in total_edges if edges[0] == new_poly[count] or edges[1] == new_poly[count]]
            min_angle = 0.0
            for edges in possible_edges:
                calc_current = current_edge
                calc_edge = edges
                if edges[1] == next_vertex:
                    calc_edge = (calc_edge[1], calc_edge[0])
                if current_edge[1] == next_vertex:
                    calc_current = (calc_current[1], calc_current[0])

                vec1 = (calc_edge[1][0] - calc_edge[0][0], calc_edge[1][1] - calc_edge[0][1])
                vec1 = (vec1[0] / math.sqrt(vec1[0] ** 2 + vec1[1] ** 2),
                        vec1[1] / math.sqrt(vec1[0] ** 2 + vec1[1] ** 2))
                vec2 = (calc_current[1][0] - calc_current[0][0], calc_current[1][1] - calc_current[0][1])

                vec2 = (vec2[0] / math.sqrt(vec2[0] ** 2 + vec2[1] ** 2),
                        vec2[1] / math.sqrt(vec2[0] ** 2 + vec2[1] ** 2))
                dot_prod = vec1[0]*vec2[0] + vec1[1]*vec2[1]
                if dot_prod < -1.0: dot_prod = -1.0
                if dot_prod > 1.0: dot_prod = 1.0
                calc_angle = math.acos(dot_prod)
                cross_prod = vec2[0]*vec1[1] - vec2[1]*vec1[0]
                if cross_prod < 0.0: calc_angle = 2*math.pi - calc_angle
                if calc_angle > min_angle:
                    next_edge = edges
                    min_angle = calc_angle
            total_edges.remove(next_edge)
            if next_edge[0] in new_poly and next_edge[1] in new_poly: break
            current_edge = next_edge
            next_vertex = next_edge[0] if next_edge[0] not in new_poly else next_edge[1]
            new_poly.append(next_vertex)
            count = count + 1

        new_fire = list()
        for idx in range(0, size_idx):
            sub_fire = [vertex_pts_1[idx], vertex_pts_2[idx]]
            new_fire.append(sub_fire)
        new_fire.append(new_poly)

        return new_fire

    def check_crossover(self, polygon):
        """
        Take fire perimeter that has intersected itself and remove inner vertices, creating new outer perimeter made of
        vertices

        New vertices added at outside intersections
        :return:
        """
        delete_index_list = list()
        for idx, pts in enumerate(polygon):
            line = (pts, polygon[idx+1]) if idx < len(polygon)-1 else (pts, polygon[0])
            for idx2, pts2 in enumerate(polygon):
                if idx == 0 and idx2 == len(polygon) - 1: continue
                if idx == len(polygon) - 1 and idx2 == 0: continue
                if idx == idx2 + 1 or idx == idx2 - 1: continue
                if idx == idx2: continue
                line2 = (pts2, polygon[idx2 + 1]) if idx2 < len(polygon) - 1 else (pts2, polygon[0])
                if self.intersect(line[0], line[1], line2[0], line2[1]) and idx2 > idx:
                    count = 0
                    for indexes in range(0, len(polygon)):
                        if idx+1 <= indexes <= idx2:
                            count = count + 1
                    if count > len(polygon)-count:
                        for indexes in range(0, len(polygon)):
                            if indexes < idx+1 or indexes > idx2:
                                delete_index_list.append(indexes)
                    elif count < len(polygon) - count:
                        for indexes in range(0, len(polygon)):
                            if idx < indexes < idx2+1:
                                delete_index_list.append(indexes)
        new_poly = [v for idx, v in enumerate(polygon) if idx not in delete_index_list]
        return new_poly

    def intersection_point(self, line1, line2):
        """
        Determines intersection point of two lines. Up to developer to ensure the two lines actually intersect

        :param line1:
        :param line2:
        :return:
        """
        if line1[0][0] - line1[1][0] == 0.0:
            m1 = float('inf')
        else:
            m1 = (line1[0][1] - line1[1][1]) / (line1[0][0] - line1[1][0])
        if line2[0][0] - line2[1][0] == 0.0:
            m2 = float('inf')
        else:
            m2 = (line2[0][1] - line2[1][1]) / (line2[0][0] - line2[1][0])
        if (m1 == float('inf') and m2 == float('inf')) or math.fabs(m1 - m2) < 0.00001:
            length1 = math.sqrt((line1[0][0] - line1[1][0])**2+(line1[0][1] - line1[1][1])**2)
            length2 = math.sqrt((line2[0][0] - line2[1][0])**2+(line2[0][1] - line2[1][1])**2)
            if length1 < length2:
                return ((line1[0][0] + line1[1][0])/2.0, (line1[0][1] + line1[1][1])/2.0)
            else:
                return ((line2[0][0] + line2[1][0]) / 2.0, (line2[0][1] + line2[1][1]) / 2.0)
        elif m1 == float('inf'):
            x_intersect = line1[0][0]
            b2 = line2[0][1] - m2 * line2[0][0]
            y_intersect = m2*x_intersect + b2
            return (x_intersect, y_intersect)
        elif m2 == float('inf'):
            x_intersect = line2[0][0]
            b1 = line1[0][1] - m1 * line1[0][0]
            y_intersect = m1*x_intersect + b1
            return (x_intersect, y_intersect)
        else:
            b1 = line1[0][1] - m1*line1[0][0]
            b2 = line2[0][1] - m2*line2[0][0]
            x_intersect = (b1 - b2) / (m2 - m1)
            y_intersect = m1*x_intersect + b1
            return (x_intersect, y_intersect)

    def obstacle_intersect(self, poly_before, poly_after, obstacles):

        lines = list()
        for idx, pts in enumerate(poly_before):
            lines.append((pts, poly_after[idx]))

        altered_segments = list()
        for idxs, segments in enumerate(lines):
            #if segments[0] == segments[1]:
            #    altered_segments.append(idx)
            #    continue

            segment_start_pt = (-0.01*(segments[1][0] - segments[0][0]) + segments[0][0],
                                -0.01 * (segments[1][1] - segments[0][1]) + segments[0][1])

            intersect_pts = list()
            for obs in obstacles:
                for idx, pts in enumerate(obs):
                    line_obs = (pts, obs[idx+1]) if idx < len(obs) - 1 else (
                        pts, obs[0])
                    if self.intersect(segment_start_pt, segments[1], line_obs[0], line_obs[1]): intersect_pts.append([
                        self.intersection_point(segments, line_obs), line_obs])
            intersect_pts.sort(key=lambda p: (p[0][0] - segments[0][0])**2 + (p[0][1] - segments[0][1])**2)
            if intersect_pts:
                #print('About to input a revised segment end')
                if math.fabs(intersect_pts[0][0][0] - segments[0][0]) < 0.0001 and \
                        math.fabs(intersect_pts[0][0][1] - segments[0][1]) < 0.0001:
                    segments_rev = (segments[0], segments[0])
                    lines[idxs] = segments_rev
                    #print('Forced the segment to be start and end!')
                    #print(segments)
                    #print(idxs)
                else:
                    #print('Didn\'t force the segments, had to assign closest intersection point')
                    #print(segments)
                    #print(intersect_pts[0][0])
                    segments_rev = (segments[0], intersect_pts[0][0])
                    lines[idxs] = segments_rev
                altered_segments.append(idxs)
                altered_segments.append(intersect_pts[0][1])

        #if altered_segments:
        #    print(altered_segments)

        new_front = list()  # Used to create the new polygon, with each index representing a vertex, its old index on the
                            # previous polygon, and whether or not it's "stuck" now
        for idx, segments in enumerate(lines):
            idx_check = 0 if idx == len(lines)-1 else idx+1
            if idx in altered_segments and idx_check in altered_segments:
                if altered_segments[altered_segments.index(idx)+1] == altered_segments[altered_segments.index(idx_check)+1]:
                    new_front.append(segments[1])
                    continue
            fire_front = (segments[1], lines[idx+1][1]) if idx < len(lines)-1 else (segments[1], lines[0][1])
            intersect_pts = list()
            for obs in obstacles:
                for idx2, pts in enumerate(obs):
                    line_obs = (pts, obs[idx2+1]) if idx2 < len(obs) - 1 else (
                        pts, obs[0])
                    if self.intersect(fire_front[0], fire_front[1], line_obs[0], line_obs[1]):
                        intersect_pts.append(self.intersection_point(fire_front, line_obs))

            intersect_pts.sort(key=lambda p: (p[0] - fire_front[0][0])**2 + (p[1] - fire_front[0][1])**2)
            intersect_pts_rev = [v for v in intersect_pts if
                                 math.sqrt((v[0]-fire_front[0][0])**2 + (v[1]-fire_front[0][1])**2) > 0.01 and
                                 math.sqrt((v[0] - fire_front[1][0]) ** 2 + (v[1] - fire_front[1][1]) ** 2) > 0.01]
            for idx2, pts in enumerate(intersect_pts_rev):
                previous_vertex = segments[1] if idx2 == 0 else intersect_pts_rev[idx2-1]
                segment_front = (previous_vertex, pts)
                halfway_pt = ((segment_front[1][0] + segment_front[0][0])/2.0,
                             (segment_front[1][1] + segment_front[0][1])/2.0)
                split = True
                for obs in obstacles:
                    if self.inside_poly(obs, halfway_pt):
                        split = False
                        break
                #print(split)
                middle_pts = list()
                if split is True:
                    middle_pts = [(val*(segment_front[1][0] - segment_front[0][0])/3.0 + segment_front[0][0],
                             val*(segment_front[1][1] - segment_front[0][1])/3.0 + segment_front[0][1]) for val in range(1, 3)]
                    #print(middle_pts)
                if idx2 == 0:
                    new_front.append(previous_vertex)
                else:
                    new_front.append(previous_vertex)
                if idx2 != 0:
                    for mids in middle_pts:
                        new_front.append(mids)

            segment_end = (intersect_pts_rev[len(intersect_pts_rev)-1], fire_front[1]) if intersect_pts_rev else \
                (fire_front[0], fire_front[1])

            if not intersect_pts_rev:
                new_front.append(segment_end[0])
            else:
                #print(segment_end)
                new_front.append(segment_end[0])
                halfway_pt = ((segment_end[1][0] + segment_end[0][0]) / 2.0,
                              (segment_end[1][1] + segment_end[0][1]) / 2.0)
                split = True
                for obs in obstacles:
                    if self.inside_poly(obs, halfway_pt):
                        split = False
                        break
                #print(split)
                if split is True:
                    middle_pts = [(val*(segment_end[1][0] - segment_end[0][0])/3.0 + segment_end[0][0],
                             val*(segment_end[1][1] - segment_end[0][1])/3.0 + segment_end[0][1]) for val in range(1, 3)]
                    #for mids in middle_pts:
                    #    new_front.append(mids)
                    #print(middle_pts)

        #if len(new_front) > len(poly_before):
        #    print(altered_segments)
        #    print(new_front)
        #    print(lines)
        #    print(intersect_pts_rev)
        return new_front

    def prop_eqn(self, a, b, c, x_sub, y_sub, theta):
        """
        Propagation equations for vertices on fire. Provides the distance change as m/min in x and y directions
        :param a: half semi-minor axis (m/min)
        :param b: half semi-major axis (m/min)
        :param c: focus to center
        :param x_sub: x slope of the point on perimeter
        :param y_sub: y slope of the point on perimeter
        :param theta: azimuth angle of headwind
        :return: (X_t, Y_t) (rates of change)
        """
        x_s = x_sub/(math.sqrt(math.pow(x_sub, 2.0) + math.pow(y_sub, 2.0))) if (x_sub != 0.0 or y_sub != 0.0) else \
            0.0
        y_s = y_sub/(math.sqrt(math.pow(x_sub, 2.0) + math.pow(y_sub, 2.0))) if (x_sub != 0.0 or y_sub != 0.0) else \
            0.0
        theta_rad = theta/180.0*math.pi
        X_t = (a*a*math.cos(theta_rad)*(x_s*math.sin(theta_rad)+y_s*math.cos(theta_rad))-b*b*math.sin(theta_rad)*(x_s*
                math.cos(theta_rad)-y_s*math.sin(theta_rad)))/math.pow((b*b*(x_s*math.cos(theta_rad)-y_s*math.sin(theta_rad))*
                (x_s*math.cos(theta_rad)-y_s*math.sin(theta_rad)) + a*a*(x_s*math.sin(theta_rad)+y_s*math.cos(theta_rad))*
                (x_s * math.sin(theta_rad) + y_s * math.cos(theta_rad))), 0.5) + c*math.sin(theta_rad)
        Y_t = (-a*a*math.sin(theta_rad)*(x_s*math.sin(theta_rad)+y_s*math.cos(theta_rad))-b*b*math.cos(theta_rad)*(x_s*
                math.cos(theta_rad)-y_s*math.sin(theta_rad)))/math.pow((b*b*(x_s*math.cos(theta_rad)-y_s*math.sin(theta_rad))*
                (x_s*math.cos(theta_rad)-y_s*math.sin(theta_rad)) + a*a*(x_s*math.sin(theta_rad)+y_s*math.cos(theta_rad))*
                (x_s * math.sin(theta_rad) + y_s * math.cos(theta_rad))), 0.5) + c*math.cos(theta_rad)
        return X_t, Y_t


    def update_simulation(self, dt_update, obstacle_list):
        #TODO!!!!!!
        # create history list associated with all vertexes, adding to such the vertex closest(?) to the latest one and
        # adopting that vertexes history up to that point. If the first vertex in the history pool contains a time_latest
        # - current_time > burn time, you ditch that history vertex
        """
        Run simulation for run_time in minutes
        dt in seconds
        :param run_time:
        :return:
        """
        dt = dt_update/60.0

        vertex_pts_list = list()
        #time_start = time.time()
        i = len(self.fire_list[0])-1
        for n, vertex_hist in enumerate(self.fire_list):

            vertex_pts = list()
            for idx, pts in enumerate(vertex_hist[i]):

                i_2 = len(vertex_hist[i])-1

                ##### should let these values be generated by user function....
                LB = 0.936 * math.exp(0.2566 * self.wind_speed) + 0.461 * math.exp(-0.1548 * self.wind_speed) - \
                     0.397
                HB = (LB + math.pow((LB * LB - 1), 0.5)) / (LB - math.pow((LB * LB - 1), 0.5))
                R = 0.5  #self.I_r*self.eta_p/self.rho_b*self.eta*self.Q_ig  # m/min
                a = 0.5 * (R + R / HB) / LB
                b = (R + R / HB) / 2.0
                c = b - R / HB
                theta = self.wind_direction  # degrees

                if idx == 0:
                    x_dif = vertex_hist[i][i_2][0] - vertex_hist[i][1][0]
                    y_dif = vertex_hist[i][i_2][1] - vertex_hist[i][1][1]
                elif idx == len(vertex_hist[i])-1:
                    x_dif = vertex_hist[i][i_2-1][0] - vertex_hist[i][0][0]
                    y_dif = vertex_hist[i][i_2-1][1] - vertex_hist[i][0][1]
                else:
                    x_dif = vertex_hist[i][idx-1][0] - vertex_hist[i][idx+1][0]
                    y_dif = vertex_hist[i][idx-1][1] - vertex_hist[i][idx+1][1]

                if pts in self.burn_list and self.burn_list[pts] > self.burn_time:
                    dmdt = (0.0, 0.0)
                else:
                    dmdt = self.prop_eqn(a, b, c, x_dif, y_dif, theta)
                    if abs(dmdt[0]) < 0.01 and abs(dmdt[1]) < 0.01:
                        #print(dmdt)
                        dmdt = (0.0, 0.0)

                vertex_pts.append((pts[0] + dt * dmdt[0], pts[1] + dt * dmdt[1]))

            # Check intersections on the obstacles
            new_poly = self.obstacle_intersect(vertex_hist[i], vertex_pts, obstacle_list)
            vertex_pts_list.append(new_poly)

        total_burn = True
        burn_list_temp = dict()
        for n in range(0, len(self.fire_list)):
            for idx, pts in enumerate(vertex_pts_list[n]):
                val_temp = self.burn_list.get(pts, 0.0) + dt
                burn_list_temp[pts] = val_temp
                if val_temp <= self.burn_time:
                    total_burn = False
            self.fire_list[n].append(self.check_crossover(vertex_pts_list[n]))
        self.burn_list = burn_list_temp

        if total_burn is True:
            print('Burnt OUT!!!')
            return True

        for n in range(0, len(self.fire_list)):
            if n >= len(self.fire_list): break
            vertex_pts_temp = self.fire_list[n]
            for l in range(0, len(self.fire_list)):
                if l >= len(self.fire_list): break
                for idx, pts in enumerate(vertex_pts_temp[i+1]):
                    if l != n and self.inside_poly(self.fire_list[l][i+1], pts):
                        fire_new = self.combine_fires(vertex_pts_temp, self.fire_list[l], i+1)
                        del self.fire_list[n]
                        self.fire_list.insert(n, fire_new)
                        del self.fire_list[l]
                        l = l - 1
                        break

        for n, fires in enumerate(self.fire_list):
            revised_vertexes = list()
            for idx, pts in enumerate(fires[i+1]):
                revised_vertexes.append(pts)
                line = (pts, fires[i+1][idx+1]) if idx < len(fires[i+1]) - 1 else (pts, fires[i+1][0])
                line_seg = (line[1][0] - line[0][0], line[1][1] - line[0][1])
                line_mag = math.sqrt(line_seg[0]**2 + line_seg[1]**2)
                if line_mag > self.perimeter_resolution:
                    revised_vertexes.append(((line[0][0] + line[1][0])/2.0, (line[0][1] + line[1][1])/2.0))
            self.fire_list[n][i+1] = revised_vertexes
        #print(time.time() - time_start)
        return False

    def update_fire_visuals(self, window_size, screen, pygame_instance, resolution, obstacles_list):
        def transform_vertex_to_pixels(ratio, domain, vertex, window_size):
            new_poly = list()
            for pts in vertex:
                #print(pts)
                #print('HELLO')
                new_pts = [int(round((pts[0]-domain[0][0])*ratio[0])),
                           window_size[1] - int(round((pts[1]-domain[1][0])*ratio[1]))]
                new_poly.append(new_pts)
            return new_poly

        m_to_pixel = (window_size[0]/(self.domain[0][1] - self.domain[0][0]),
                      window_size[1]/(self.domain[1][1] - self.domain[1][0]))
        WIDTH = int(round(float(window_size[0]) / (self.domain[0][1] - self.domain[0][0]) * resolution[0]))
        HEIGHT = int(round(float(window_size[1]) / (self.domain[1][1] - self.domain[1][0]) * resolution[1]))

        '''for row in range(int(round((self.domain[1][1] - self.domain[1][0])/resolution[1]))):
    for column in range(int(round((self.domain[0][1] - self.domain[0][0])/resolution[0]))):

        loc = (float(column*resolution[0] + self.domain[0][0]),
               float(self.domain[1][1] - (row*resolution[1] + self.domain[1][0])))

        inside_fire = False
        inside_burnt = False'''
        for n, vertex_hist in enumerate(self.fire_list):
            pygame_instance.draw.polygon(screen, (242, 23, 12),
                                transform_vertex_to_pixels(m_to_pixel, self.domain, vertex_hist[len(vertex_hist)-1],
                                         window_size))
        for obs in obstacles_list:
            pygame_instance.draw.polygon(screen, (94, 97, 102), transform_vertex_to_pixels(m_to_pixel,
                                        self.domain, obs,
                                         window_size))

        #for n, fires in enumerate(fire_list):
        #    for idx, pts in enumerate(fires[i+1]):
        #        pts1 = [WIDTH*int(round(pts[0]/2.0 + 50.0)) + WIDTH/2, HEIGHT*int(round(-pts[1]/2.0 + 50))+HEIGHT/2]
        #        if idx < len(fires[i+1])-1:
        #            pts2 = fires[i+1][idx+1]
        #            pts2 = [WIDTH*int(round(pts2[0]/2.0 + 50.0))+WIDTH/2, HEIGHT*int(round(-pts2[1]/2.0 + 50))+HEIGHT/2]
        #        else:
        #            pts2 = fires[i+1][0]
        #            pts2 = [WIDTH*int(round(pts2[0] / 2.0 + 50.0))+WIDTH/2, HEIGHT*int(round(-pts2[1] / 2.0 + 50))+HEIGHT/2]
        #        pygame.draw.line(screen, (0, 0, 0), pts1, pts2, 1)

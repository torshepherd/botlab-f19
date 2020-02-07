class WaypointFollower():
         self.waypoints = [[0.0,0.0],[1.0,0.0],[1.0,1.0],[0.0,1.0],[0.0,0.0]]
         self.wpt_num = 1
         self.wpt_thresh = 0.1
-        self.angle_thresh = 0.1
+        self.angle_thresh = 0.05
         self.turning = False
         self.trans_v = 0.0
         self.angular_v = 0.0
         self.setpt_theta= 0.0

     def odometry_handler(self, channel, data):
+        if self.wpt_num >= len(self.waypoints):
+            return
         msg = odometry_t().decode(data)
         waypoint = self.waypoints[self.wpt_num]
-        forward_speed = .3
+        forward_speed = .2
         turn_speed = 1.2
+        K_p = 2
         if not self.turning:
             if math.sqrt((msg.x - waypoint[0]) ** 2 + (msg.y - waypoint[1]) ** 2) < self.wpt_thresh:
                 self.wpt_num += 1
+                if self.wpt_num >= len(self.waypoints):
+                    self.angular_v = 0
+                    self.trans_v = 0
+                    return
                 new_waypoint = self.waypoints[self.wpt_num]
                 self.trans_v = 0
                 self.angular_v = turn_speed
                 self.turning = True
                 self.setpt_theta = math.atan2(new_waypoint[1] - msg.y, new_waypoint[0] - msg.x)
+                print("starting turn")
             else:
+                self.setpt_theta = math.atan2(waypoint[1] - msg.y, waypoint[0] - msg.x)
                 self.trans_v = forward_speed
-                self.angular_v = 0
+                angle_diff_1 = self.setpt_theta - msg.theta
+                angle_diff_2 = angle_diff_1 - 2 * math.pi
+               if abs(angle_diff_1) < abs(angle_diff_2):
+                    angle_diff = angle_diff_1
+                else:
+                    angle_diff = angle_diff_2
+                print "angle diff: %f" % angle_diff
+                self.angular_v = K_p * angle_diff
         else:
             angle_diff = abs(msg.theta - self.setpt_theta)
             if angle_diff < self.angle_thresh or 2 * math.pi - angle_diff < self.angle_thresh:
+                print("starting forward")
                 self.trans_v = forward_speed
                 self.angular_v = 0
                 self.turning = False
             else:
                 self.trans_v = 0
                 self.angular_v = turn_speed
+        print("Waypoint:")
+        print(self.waypoints[self.wpt_num])
+        print("Angular vel:")
+        print(self.angular_v)
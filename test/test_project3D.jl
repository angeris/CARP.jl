# test data
position = [-4.,  0.,  8.] 
goal = [5., -17., -10.]

ob_center = [-1.80268303, -9.37331424,  1.65088143]
ob_shape = [
    30. 9.  5.
    9.  10. 7.
    5.  7.  20.
]
projection = [-2.31797371, -0.75414051,  2.35102477,]

a = AgentModel()
set_current_point!(a, position)
set_goal_point!(a, goal)
set_ellipsoid!(a, "test", ob_center, ob_shape)
print(a)
proj = find_projection!(a)
print(proj)

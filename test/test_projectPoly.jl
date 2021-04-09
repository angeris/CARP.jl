# test data
position = [-4.,  0.,  8.] 
vel = [1., 0., 0.]
goal = [5., -17., -10.]
ob_center = [-1.80268303, -9.37331424,  1.65088143]
obj_uncertainty = [
    42.02681838 11.52115633  6.40064241
    11.52115633 16.42424876  8.96089937
    6.40064241  8.96089937 39.87266856
    ]
projection = [
    -3.66666667 -2.5311153  -2.5311153
    0. -0.7493847  -0.7493847
    8. 3.26528213  3.26528213
    ]

a = AgentModel(order=3)
set_current_position!(a, position)
set_current_velocity!(a, vel)
set_goal_point!(a, goal)
set_ellipsoid!(a, "test", ob_center, obj_uncertainty)
# calculate projection
find_projection!(a)
# test values
@test isapprox(projection, a.trajectory, rtol=1e-3)

position = [ 5., -9.]
goal = [-5, 17]
obj_center = [-4.17742707, 0.34695836]
projection = [7.41002297, 1.30955746]
obj_uncertainty = [[1, 1] [1, 4]]
dist = 10.588

a = AgentModel(dims=2)
set_current_position!(a, position)
set_goal_point!(a, goal)
set_ellipsoid!(a, "test", obj_center, obj_uncertainty)
# calculate projection
find_projection!(a)
# test values
@test isapprox(projection, a.trajectory[:,end], rtol=1e-3)
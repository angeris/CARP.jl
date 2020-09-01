position = [0, 0]
goal = [10, 0]

obj_center = [5, 0]
obj_uncertainty = I(2)

a = AgentModel(dims=2)
set_current_point!(a, position)
set_goal_point!(a, goal)
set_ellipsoid!(a, "test", obj_center, obj_uncertainty)
@show find_projection!(a)
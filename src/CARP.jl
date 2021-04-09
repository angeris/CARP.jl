module CARP

export AgentModel, set_ellipsoid!, set_ellipsoid_position!, set_ellipsoid_uncertainty!, set_goal_point!, set_current_position!, set_current_velocity!, find_projection!

using LinearAlgebra
using JuMP, ECOS

include("./utilities.jl")
include("./agent.jl")

end
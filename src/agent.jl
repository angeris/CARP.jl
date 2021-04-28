mutable struct AgentModel
    object_position::Dict{Any, FloatVector}
    object_uncertainty::Dict{Any, Eigen}
    goal_point::FloatVector
    current_position::FloatVector
    current_velocity::FloatVector
    order::Int
    trajectory::FloatMatrix
    solved::Bool
end

# Constructors
AgentModel(;dims::Int=3, order::Int=1) = AgentModel(Dict{Any, FloatVector}(), Dict{Any, FloatMatrix}(), #obs
                                      zeros(dims),  # goal
                                      zeros(dims), zeros(dims), # state
                                      order, #Poly order (1 is a line)
                                      zeros((dims, order)), # outputs
                                      true)

# Ellipsoid functions (we can add other types, too)
function set_ellipsoid_uncertainty!(a::AgentModel, name, uncertainty::FloatMatrix)
    a.solved = false
    symm_uncertainty = Symmetric(uncertainty)
    factor = eigen(symm_uncertainty)
    a.object_uncertainty[name] = factor
end

function set_ellipsoid_position!(a::AgentModel, name, position::FloatVector)
    a.solved = false
    D, U = a.object_uncertainty[name]
    a.object_position[name] = U * ((U' * position) ./ D)
end

function set_ellipsoid!(a::AgentModel, name, position::FloatVector, uncertainty::FloatMatrix)
    set_ellipsoid_uncertainty!(a, name, uncertainty)
    set_ellipsoid_position!(a, name, position)
end

set_ellipsoid!(a::AgentModel, name, position::RealVector, uncertainty) = set_ellipsoid!(a, name, float.(position), Matrix{Float64}(uncertainty))

# Points
function set_goal_point!(a::AgentModel, position::RealVector)
    a.solved = false
    a.goal_point .= float.(position)
end

function set_current_position!(a::AgentModel, position::RealVector)
    a.solved = false
    a.current_position .= float.(position)
end

function set_current_velocity!(a::AgentModel, velocity::RealVector)
    a.solved = false
    a.current_velocity .= float.(velocity)
    #function body
end

# Solving
function find_projection!(a::AgentModel)
    if a.solved
        return a.trajectory
    end

    n = size(a.current_position, 1)
    order = a.order
    model = Model(ECOS.Optimizer)
    set_optimizer_attribute(model, MOI.Silent(), true)
    
    @variable(model, x[1:n, 1:order])
    @variable(model, dist)

    @objective(model, Min, dist)
    # set cone constraint
    @constraint(model, [dist; x[:,end] - a.goal_point] ∈ SecondOrderCone())

    for (name, position) in a.object_position
        D, U = a.object_uncertainty[name]
        for i in 1:order
            # put each control point in the constraint 
            λ = @variable(model, lower_bound=0.0)
            t = quad_over_lin(model, U'*(x[:,i] + λ * position), λ ./ D .+ 1)
            @constraint(model, sum(a.current_position.^2) - 2*(a.current_position' * x[:,i]) + sum(t)
            <= (position' * (U * (D .* (U' * position)))) * λ - λ)
        end        
    end
    # dynamics
    if order > 1
        @constraint(model, 3*(x[:, 1] - a.current_position) .== a.current_velocity)  # ic vel
        @constraint(model, x[:, end] - x[:, end-1] .== zeros(n))  # final vel
    end
    optimize!(model)
    # return solution
    if termination_status(model) in  [MOI.OPTIMAL, MOI.ALMOST_OPTIMAL]
        a.solved = true
        a.trajectory = [a.current_position value.(x)]
    else
        a.solved=false
        a.trajectory = Array{Float64}(undef, n, order)
    end

    return a.trajectory 
end

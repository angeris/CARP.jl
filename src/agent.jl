mutable struct AgentModel
    object_position::Dict{Any, FloatVector}
    object_uncertainty::Dict{Any, Eigen}
    goal_point::FloatVector
    current_point::FloatVector
    projected_point::FloatVector
    solved::Bool
end

# Constructors
AgentModel(;dims::Int=3) = AgentModel(Dict{Any, FloatVector}(), Dict{Any, FloatMatrix}(), zeros(dims), zeros(dims), zeros(dims), true)

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

function set_current_point!(a::AgentModel, position::RealVector)
    a.solved = false
    a.current_point .= float.(position)
end

# Solving
function find_projection!(a::AgentModel)
    if a.solved
        return a.projected_point
    end

    n = size(a.current_point, 1)
    model = Model(ECOS.Optimizer)

    @variable(model, x[1:n])

    objective = quad_over_lin(model, x - a.goal_point, ones(n)) # equivalent to (x-goal_point)^2, componentwise

    @objective(model, Min, sum(objective))

    for (name, position) in a.object_position
        D, U = a.object_uncertainty[name]

        λ = @variable(model, lower_bound=0.0)
        t = quad_over_lin(model, U'*(x + λ * position), 1 ./ D .+ λ)
        @constraint(model, sum(a.current_point.^2) - 2*(a.current_point' * x) + sum(t)
        <= (position' * (U * (D .* (U' * position)))) * λ - λ)
        
    end

    optimize!(model)

    a.solved = true
    a.projected_point = value.(x)

    return a.projected_point
end
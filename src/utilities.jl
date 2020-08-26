# Types
FloatMatrix = Matrix{Float64}
FloatVector = Vector{Float64}
RealMatrix = Matrix{<: Real}
RealVector = Vector{<: Real}

# Functions

"Computes x[i]^2/y[i] <= t[i] for i=1:n"
function quad_over_lin(m, x, y)
    n = size(x, 1)
    t = @variable(m, [1:n])

    for i = 1:n
        @constraint(m, [y[i] + t[i]; y[i] - t[i]; 2*x[i]] ∈ SecondOrderCone())
    end

    return t
end
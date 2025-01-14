import MathOptInterface as MOI
import Ipopt 
import FiniteDiff
import ForwardDiff
using LinearAlgebra

struct ProblemMOI <: MOI.AbstractNLPEvaluator
    n_nlp::Int
    m_nlp::Int
    obj_grad::Bool
    con_jac::Bool
    sparsity_jac
    sparsity_hess
    hessian_lagrangian::Bool
    params::NamedTuple
    cost # ::Function
    con # ::Function
    diff_type # :Symbol
end

function ProblemMOI(n_nlp,m_nlp,params,cost,con,diff_type;
        obj_grad=true,
        con_jac=true,
        sparsity_jac=sparsity_jacobian(n_nlp,m_nlp),
        sparsity_hess=sparsity_hessian(n_nlp,m_nlp),
        hessian_lagrangian=false)

    ProblemMOI(n_nlp,m_nlp,
        obj_grad,
        con_jac,
        sparsity_jac,
        sparsity_hess,
        hessian_lagrangian,
        params,
        cost,
        con,
        diff_type)
end

function row_col!(row,col,r,c)
    for cc in c
        for rr in r
            push!(row,convert(Int,rr))
            push!(col,convert(Int,cc))
        end
    end
    return row, col
end

function sparsity_jacobian(n,m)

    row = []
    col = []

    r = 1:m
    c = 1:n

    row_col!(row,col,r,c)

    return collect(zip(row,col))
end

function sparsity_hessian(n,m)

    row = []
    col = []

    r = 1:m
    c = 1:n

    row_col!(row,col,r,c)

    return collect(zip(row,col))
end

function MOI.eval_objective(prob::MOI.AbstractNLPEvaluator, x)
    prob.cost(prob.params, x)
end

function MOI.eval_objective_gradient(prob::MOI.AbstractNLPEvaluator, grad_f, x)
    _cost(_x) = prob.cost(prob.params, _x)
    if prob.diff_type == :auto 
        ForwardDiff.gradient!(grad_f,_cost,x)
    else
        FiniteDiff.finite_difference_gradient!(grad_f, _cost, x)
    end
    return nothing
end

function MOI.eval_constraint(prob::MOI.AbstractNLPEvaluator,c,x)
    c .= prob.con(prob.params, x)
    return nothing
end

function MOI.eval_constraint_jacobian(prob::MOI.AbstractNLPEvaluator, jac, x)
    _con(_x) = prob.con(prob.params, _x)
    if prob.diff_type == :auto 
        reshape(jac,prob.m_nlp,prob.n_nlp) .= ForwardDiff.jacobian(_con, x)
    else
        reshape(jac,prob.m_nlp,prob.n_nlp) .= FiniteDiff.finite_difference_jacobian(_con, x)
    end
    return nothing
end

function MOI.features_available(prob::MOI.AbstractNLPEvaluator)
    return [:Grad, :Jac]
end

MOI.initialize(prob::MOI.AbstractNLPEvaluator, features) = nothing
MOI.jacobian_structure(prob::MOI.AbstractNLPEvaluator) = prob.sparsity_jac


"""
x = fmincon(cost,equality_constraint,inequality_constraint,x_l,x_u,c_l,c_u,x0,params,diff_type)

This function uses IPOPT to minimize an objective function 

`cost(params, x)` 

With the following three constraints: 

`equality_constraint(params, x) = 0`
`c_l <= inequality_constraint(params, x) <= c_u` 
`x_l <= x <= x_u` 

Problem specific parameters should be loaded into params::NamedTuple (things like 
cost weights, dynamics parameters, etc.). 

args:
    cost::Function                    - objective function to be minimzed (returns scalar)
    equality_constraint::Function     - c_eq(params, x) == 0 
    inequality_constraint::Function   - c_l <= c_ineq(params, x) c_u 
    x_l::Vector                       - x_l <= x <= x_u 
    x_u::Vector                       - x_l <= x <= x_u 
    c_l::Vector                       - c_l <= c_ineq(params, x) <= c_u 
    c_u::Vector                       - c_l <= c_ineq(params, x) <= c_u 
    x0::Vector                        - initial guess 
    params::NamedTuple                - problem parameters for use in costs/constraints 
    diff_type::Symbol                 - :auto for ForwardDiff, :finite for FiniteDiff 

optional args:
    tol                               - optimality tolerance 
    c_tol                             - constraint violation tolerance 
    max_iters                         - max iterations 
    verbose                           - true for IPOPT output, false for nothing 

You should try and use :auto for your `diff_type` first, and only use :finite if you 
absolutely cannot get ForwardDiff to work. 

This function will run a few basic checks before sending the problem off to IPOPT to 
solve. The outputs of these checks will be reported as the following:

---------checking dimensions of everything----------
---------all dimensions good------------------------
---------diff type set to :auto (ForwardDiff.jl)----
---------testing objective gradient-----------------
---------testing constraint Jacobian----------------
---------successfully compiled both derivatives-----
---------IPOPT beginning solve----------------------

If you're getting stuck during the testing of one of the derivatives, try switching 
to FiniteDiff.jl by setting diff_type = :finite. 
"""

function fmincon(cost::Function,
                 equality_constraint::Function,
                 inequality_constraint::Function,
                 x_l::Vector,
                 x_u::Vector,
                 c_l::Vector,
                 c_u::Vector,
                 x0::Vector,
                 params::NamedTuple,
                 diff_type::Symbol;
                 tol = 1e-4,
                 c_tol = 1e-4,
                 max_iters = 1_000,
                 verbose = true)::Vector
    
    n_primals = length(x0)
    n_eq = length(equality_constraint(params, x0))
    n_ineq = length(inequality_constraint(params, x0))
    
    verbose && println("---------checking dimensions of everything----------")
    @assert length(x0) == length(x_l) == length(x_u)
    @assert length(c_l) == length(c_u) == n_ineq
    @assert all(x_u .>= x_l)
    if n_ineq > 0 
        @assert all(c_u .>= c_l)
    end
    verbose && println("---------all dimensions good------------------------")
    
    
    function con(params, x)
        [
            equality_constraint(params, x);
            inequality_constraint(params, x)
        ]
    end
    
    if diff_type == :auto 
        verbose && println("---------diff type set to :auto (ForwardDiff.jl)----")
    else
        verbose && println("---------diff type set to :finite (FiniteDiff.jl)---")
    end
    verbose && println("---------testing objective gradient-----------------")
    if diff_type == :auto 
        ForwardDiff.gradient(_x -> cost(params, _x), x0)
    else
        FiniteDiff.finite_difference_gradient(_x -> cost(params, _x), x0)
    end     
    verbose && println("---------testing constraint Jacobian----------------")
    if diff_type == :auto 
        ForwardDiff.jacobian(_x -> con(params, _x), x0)
    else
        FiniteDiff.finite_difference_jacobian(_x -> con(params, _x), x0)
    end    
    verbose && println("---------successfully compiled both derivatives-----")
    
    prob = ProblemMOI(n_primals, n_eq + n_ineq, params, cost, con, diff_type)

    # add zeros(n_eq) for equality constraint
    nlp_bounds = MOI.NLPBoundsPair.([zeros(n_eq); c_l],[zeros(n_eq); c_u])
    block_data = MOI.NLPBlockData(nlp_bounds, prob, true)

    solver = Ipopt.Optimizer()
    solver.options["max_iter"] = max_iters
    solver.options["tol"] = tol
    solver.options["constr_viol_tol"] = c_tol
    
    if verbose 
        solver.options["print_level"] = 5
    else
        solver.options["print_level"] = 0 
    end

    x = MOI.add_variables(solver,prob.n_nlp)

    for i = 1:prob.n_nlp
        MOI.add_constraint(solver, x[i], MOI.LessThan(x_u[i]))
        MOI.add_constraint(solver, x[i], MOI.GreaterThan(x_l[i]))
        MOI.set(solver, MOI.VariablePrimalStart(), x[i], x0[i])
    end

    # Solve the problem
    verbose && println("---------IPOPT beginning solve----------------------")

    MOI.set(solver, MOI.NLPBlock(), block_data)
    MOI.set(solver, MOI.ObjectiveSense(), MOI.MIN_SENSE)
    MOI.optimize!(solver)

    # Get the solution
    res = MOI.get(solver, MOI.VariablePrimal(), x)
    
    return res 
    
end


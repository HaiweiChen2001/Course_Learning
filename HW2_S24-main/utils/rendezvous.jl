function create_circle(N_circ)
    R = 1.1
    num_circles = 2
    θ_vec = range(0-pi/2,2*pi+pi/2,length = N_circ)
    [([R*cos(θ),-3,R*sin(θ)+3]) for θ in θ_vec]
end
function desired_trajectory(x0,xg,N::Int64,dt)  
    N_circle = 65 
    N_pre = 10 
    N_post = 25
    p_circle = create_circle(N_circle)
    p_pre_circle =  range(x0[1:3],p_circle[1], length = N_pre + 1)
    p_post1 = range(p_circle[end],[xg[1],-3,xg[3]], length = 9)
    p_post2 = range([xg[1],-3,xg[3]],xg[1:3], length = 18)
    p_post_circle = [p_post1..., p_post2[2:end]...]
    positions = [p_pre_circle...,p_circle[2:end-1]...,p_post_circle...]
    velocities = diff(positions)/dt
    push!(velocities, zeros(3))
    X_desired = [[positions[i];velocities[i]] for i = 1:N]
    @assert length(X_desired) == N
    X_desired
end
function desired_trajectory_long(x0,xg,N::Int64,dt)
    if iseven(N)
    # same as desired_trajectory but we pad the end with goal states
        return [desired_trajectory(x0,xg,Int(N/2),dt)...,[xg for i = 1:Int(N/2)]...]
    else
        error("N needs to be even, N: $N")
    end
end
function skew(v)
    [0 -v[3] v[2]; v[3] 0 -v[1]; -v[2] v[1] 0]
end
function dcm_from_phi(ϕ)
    theta = norm(ϕ)
    r = (abs(theta)>1e-12) ? ϕ/theta : zeros(3)
    Q = (I + sin(theta) * skew(r) + (1.0 - cos(theta)) *
    skew(r) * skew(r))
    return Q 
end
function vis_traj!(vis, name, X; R = 0.01, color = mc.RGBA(1.0, 0.0, 0.0, 1.0))
    for i = 1:(length(X)-1)
        a = X[i][1:3]
        b = X[i+1][1:3] + (ones(3) * 1e-10) # add small perturbation to avoid a == b
        cyl = mc.Cylinder(mc.Point(a...), mc.Point(b...), R)
        mc.setobject!(vis[name]["p"*string(i)], cyl, mc.MeshPhongMaterial(color=color))
    end
    for i = 1:length(X)
        a = X[i][1:3]
        sph = mc.HyperSphere(mc.Point(a...), R)
        mc.setobject!(vis[name]["s"*string(i)], sph, mc.MeshPhongMaterial(color=color))
    end
end
function animate_rendezvous(X, Xref, dt;show_reference = true)
    
    vis = mc.Visualizer()
    mc.setprop!(vis["/Background"], "top_color", colorant"transparent")
    mc.setprop!(vis["/Lights/AmbientLight/<object>"], "intensity", 0.85)
    mc.setprop!(vis["/Lights/PointLightPositiveX/<object>"], "intensity", 0.0)
    dragon_obj = mc.MeshFileGeometry(joinpath(@__DIR__,"dragon.obj"))
    mc.setobject!(vis[:dragon][:base], dragon_obj, mc.MeshPhongMaterial(color=mc.RGBA(0.6, 0.6, 1.0, 1.0)))
    mc.settransform!(vis[:dragon][:base], mc.Translation([0,0,-.34]) ∘ mc.LinearMap(0.002*dcm_from_phi(pi/2*[1,0,0])))
    ISS_obj = mc.MeshFileGeometry(joinpath(@__DIR__,"ISS.obj"))
    mc.setobject!(vis[:iss][:base], ISS_obj,mc.MeshPhongMaterial(color=mc.RGBA(0.6, 0.6, 0.6, 1.0)))
    mc.settransform!(vis[:iss][:base], mc.Translation([-8.915,-2.5,4]) ∘ mc.LinearMap(0.023*dcm_from_phi(pi/2*[1,0,0])))

    if show_reference
        vis_traj!(vis, :traj, Xref)
    end

    anim = mc.Animation(vis)
    # anim = mc.Animation(floor(Int,1/dt))
    for k = 1:length(X)
        mc.atframe(anim, k) do
            mc.settransform!(vis[:dragon], mc.Translation(X[k][1:3]) ∘ mc.LinearMap(dcm_from_phi(pi*[0,0,1])))
        end
    end
    mc.setanimation!(vis, anim)
    return mc.render(vis)
end

function state_estimate(xi, xg)
    if norm(xi - xg) < 1 
        return xi 
    else
        position_σ = .01 #  m 
        velocity_σ = .0001 # m/s
        return xi + [position_σ*randn(3);velocity_σ*randn(3)]
    end
end

function thruster_model(xi, xg, u)
    if norm(xi - xg) < 1
        # assume no thruster errors within 1 m 
        return u 
    else
        misalignment = dcm_from_phi(deg2rad(3)*[.3,.6,-.8])
        scale = Diagonal([.95,1.03,1.01])
        return misalignment*scale*u
    end
end

function rotY(theta)
    [cos(theta) 0 sin(theta);
     0 1 0;
     -sin(theta) 0 cos(theta)]
end
        

function update_quad_pose!(vis, name, x)
    px,pz,theta,_,_,_ = x 
    r = [px,0,pz]
    mc.settransform!(vis[name], mc.compose(mc.Translation(r),mc.LinearMap(1.5*rotY(theta))))
end
function vis_traj!(vis, name, X; R = 0.1, color = mc.RGBA(1.0, 0.0, 0.0, 1.0))
    # visualize a trajectory expressed with X::Vector{Vector}
    for i = 1:(length(X)-1)
        a = X[i][1:3]
        b = X[i+1][1:3]
        cyl = mc.Cylinder(mc.Point(a...), mc.Point(b...), R)
        mc.setobject!(vis[name]["p"*string(i)], cyl, mc.MeshPhongMaterial(color=color))
    end
    for i = 1:length(X)
        a = X[i][1:3]
        sph = mc.HyperSphere(mc.Point(a...), R)
        mc.setobject!(vis[name]["s"*string(i)], sph, mc.MeshPhongMaterial(color=color))
    end
end
function convert_2d_to_3d(X)
    [[x[1],0,x[2]] for x in X]
end
function animate_planar_quadrotors(x1,x2,x3, dt)
    # animate quadrotor, show Xref with vis_traj!, and track Xref with the green sphere
    vis = mc.Visualizer()
    robot_obj = mc.MeshFileGeometry(joinpath(@__DIR__,"quadrotor.obj"))
    
    c1 = mc.RGBA(1.0, 0.0, 0.0, 1.0)
    c2 = mc.RGBA(0.0, 1.0, 0.0, 1.0)
    c3 = mc.RGBA(0.0, 0.0, 1.0, 1.0)
    X1 = convert_2d_to_3d(x1)
    X2 = convert_2d_to_3d(x2)
    X3 = convert_2d_to_3d(x3)
    vis_traj!(vis, :traj1, X1; R = 0.01, color = c1)
    vis_traj!(vis, :traj2, X2; R = 0.01, color = c2)
    vis_traj!(vis, :traj3, X3; R = 0.01, color = c3)
    mc.setobject!(vis[:vic1], robot_obj,mc.MeshPhongMaterial(color=c1))
    mc.setobject!(vis[:vic2], robot_obj,mc.MeshPhongMaterial(color=c2))
    mc.setobject!(vis[:vic3], robot_obj,mc.MeshPhongMaterial(color=c3))

    anim = mc.Animation(vis; fps=floor(Int,1/dt))
    for k = 1:length(X1)
        mc.atframe(anim, k) do
            update_quad_pose!(vis, :vic1, x1[k])
            update_quad_pose!(vis, :vic2, x2[k])
            update_quad_pose!(vis, :vic3, x3[k])
        end
    end
    mc.setanimation!(vis, anim)

    return (mc.render(vis))
end

function distance_between_quads(x1,x2,x3)
    [norm(x1 - x2),norm(x1 - x3), norm(x2 - x3)]
end
function rk4(params,ode,x,u,dt)
    # rk4 
    k1 = dt*ode(params,x, u)
    k2 = dt*ode(params,x + k1/2, u)
    k3 = dt*ode(params,x + k2/2, u)
    k4 = dt*ode(params,x + k3, u)
    return x + (1/6)*(k1 + 2*k2 + 2*k3 + k4)
end
function check_dynamic_feasibility(params,X,U)
    
    dyn_err = [norm(X[i+1] - rk4(params, single_quad_dynamics, X[i],U[i],params.dt), Inf) for i = 1:(params.N-1)]
    
    max_err = maximum(dyn_err)
    med_err = median(dyn_err)
    mean_err = mean(dyn_err)
        
    if (max_err < 5e-1) && (med_err < 1e-2) && (mean_err < 1e-2)
        return true 
    else
        return false 
    end
end

function RotX(θ::Real)
    # rotation matrix for rotation about y axis
    # return [cos(θ) 0 sin(θ); 0 1 0; -sin(θ) 0 cos(θ)]
    return [1 0 0;0 cos(θ) -sin(θ); 0 sin(θ) cos(θ)]
end

function meshcat_animate(params, X, dt,N)
    vis = mc.Visualizer()
    mc.render(vis)
    rod1 = mc.Cylinder(mc.Point(0,0,-params.L1/2), mc.Point(0,0,params.L1/2), 0.05)
    rod2 = mc.Cylinder(mc.Point(0,0,-params.L2/2), mc.Point(0,0,params.L2/2), 0.05)
    mc.setobject!(vis[:rod1], rod1)
    mc.setobject!(vis[:rod2], rod2)
    sphere = mc.HyperSphere(mc.Point(0,0,0.0), 0.1)
    mc.setobject!(vis[:s1], sphere)
    mc.setobject!(vis[:s2], sphere)
    anim = mc.Animation(floor(Int,1/dt))
    for k = 1:N
        mc.atframe(anim, k) do
            θ1,θ2 = X[k][[1,3]]
            r1 = [0, params.L1*sin(θ1), -params.L1*cos(θ1) + 2]
            r2 = r1 + [0, params.L2*sin(θ2), -params.L2*cos(θ2)]
            mc.settransform!(vis[:s1], mc.Translation(r1))
            mc.settransform!(vis[:s2], mc.Translation(r2))
            mc.settransform!(vis[:rod1], mc.compose(mc.Translation(0.5*([0,0,2] + r1)),mc.LinearMap(RotX(θ1))))
            mc.settransform!(vis[:rod2], mc.compose(mc.Translation(r1 + 0.5*(r2 - r1)),mc.LinearMap(RotX(θ2))))
        end
    end
    mc.setanimation!(vis, anim)
    mc.render(vis)
end
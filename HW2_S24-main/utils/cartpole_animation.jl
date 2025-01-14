function rotx(θ)
    s, c = sincos(θ)
    return [1 0 0; 0 c -s; 0 s c]
end
function create_cartpole!(vis)
    mc.setobject!(vis[:cart], mc.HyperRectangle(mc.Vec(-.25,-1.0,-.15), mc.Vec(0.5,2,0.3)))
    mc.setobject!(vis[:pole], mc.Cylinder(mc.Point(0,0,-.75), mc.Point(0,0,.75), 0.05))
    mc.setobject!(vis[:a], mc.HyperSphere(mc.Point(0,0,0.0),0.1))
    mc.setobject!(vis[:b], mc.HyperSphere(mc.Point(0,0,0.0),0.1))
end
function update_cartpole_transform!(vis,x)
    pole_o = 0.3
    px = x[1]
    θ = x[2]
    mc.settransform!(vis[:cart], mc.Translation([0,px,0.0]))
    p1 = [pole_o,px,0]
    p2 = p1 + 1.5*[0,sin(θ), -cos(θ)]
    mc.settransform!(vis[:a], mc.Translation(p1))
    mc.settransform!(vis[:b], mc.Translation(p2))
    mc.settransform!(vis[:pole], mc.Translation(0.5*(p1 + p2)) ∘ mc.LinearMap(rotx(θ))) 
end

function animate_cartpole(X, dt)
    vis = mc.Visualizer()
    create_cartpole!(vis)
    # anim = mc.Animation(floor(Int,1/dt))
    anim = mc.Animation(vis)
    for k = 1:length(X)
        mc.atframe(anim, k) do
            update_cartpole_transform!(vis,X[k])
        end
    end
    mc.setanimation!(vis, anim)
    return mc.render(vis)
end
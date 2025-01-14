using MeshCat, GeometryBasics, Colors
function animate_brick(qs)
    dt = 0.01 
    N = length(qs)
    vis = Visualizer()
    setobject!(vis[:brick], Rect3D(Vec(0,0,0f0), 0.5*Vec(2,1,1f0)), MeshPhongMaterial(color=colorant"firebrick"))
    anim = MeshCat.Animation(floor(Int,1/dt))
    for k = 1:N
        atframe(anim, k) do 
            q = qs[k]
            settransform!(vis[:brick], MeshCat.Translation([q[1],0,q[2]]))
        end
    end
    setanimation!(vis, anim)
    render(vis)
end
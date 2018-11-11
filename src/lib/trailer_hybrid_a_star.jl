#
# Hybrid A* path planning for trailer
#
# author: Atsushi Sakai(@Atsushi_twi)
#

module trailer_hybrid_a_star

using PyPlot
using DataFrames
using NearestNeighbors
using DataStructures 

include("rs_path.jl")
include("grid_a_star.jl")
include("trailerlib.jl")

const XY_GRID_RESOLUTION = 2.0 #[m]
const YAW_GRID_RESOLUTION = deg2rad(15.0) #[rad]
const GOAL_TYAW_TH = deg2rad(5.0) #[rad]
const MOTION_RESOLUTION = 0.1 #[m] path interporate resolution
const N_STEER = 20.0 # number of steer command
const EXTEND_AREA= 5.0 #[m] map extend length
const SKIP_COLLISION_CHECK= 4 # skip number for collision check
 
const SB_COST = 100.0 # switch back penalty cost
const BACK_COST = 5.0 # backward penalty cost
const STEER_CHANGE_COST = 5.0 # steer angle change penalty cost
const STEER_COST = 1.0 # steer angle change penalty cost
const JACKKNIF_COST= 200.0 # Jackknif cost
const H_COST = 5.0 # Heuristic cost
 
const WB = trailerlib.WB #[m] Wheel base
const LT = trailerlib.LT #[m] length of trailer
const MAX_STEER = trailerlib.MAX_STEER #[rad] maximum steering angle


struct Node
    xind::Int64 #x index
    yind::Int64 #y index
    yawind::Int64 #yaw index
    direction::Bool # moving direction forword:true, backword:false
    x::Array{Float64} # x position [m]
    y::Array{Float64} # y position [m]
    yaw::Array{Float64} # yaw angle [rad]
    yaw1::Array{Float64} # trailer yaw angle [rad]
    directions::Array{Bool} # directions of each points forward: true, backward:false
    steer::Float64 # steer input
    cost::Float64 # cost
    pind::Int64 # parent index
end

struct Config # config struct for hybrid A* DB
    minx::Int64
    miny::Int64
    minyaw::Int64
    minyawt::Int64
    maxx::Int64
    maxy::Int64
    maxyaw::Int64
    maxyawt::Int64
    xw::Int64
    yw::Int64
    yaww::Int64
    yawtw::Int64
    xyreso::Float64
    yawreso::Float64
end

mutable struct Path
    x::Array{Float64} # x position [m]
    y::Array{Float64} # y position [m]
    yaw::Array{Float64} # yaw angle [rad]
    yaw1::Array{Float64} # trailer angle [rad]
    direction::Array{Bool} # direction forward: true, back false
    cost::Float64 # cost
end


function calc_hybrid_astar_path(sx::Float64, sy::Float64, syaw::Float64, syaw1::Float64,
                                gx::Float64, gy::Float64, gyaw::Float64, gyaw1::Float64,
                                ox::Array{Float64}, oy::Array{Float64},
                                xyreso::Float64, yawreso::Float64,
                                )
    """
    sx: start x position [m]
    sy: start y position [m]
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    xyreso: grid resolution [m]
    yawreso: yaw angle resolution [rad]
    """

    syaw, gyaw = rs_path.pi_2_pi(syaw), rs_path.pi_2_pi(gyaw)
    ox, oy = ox[:], oy[:]

	kdtree = KDTree([ox'; oy'])

    c = calc_config(ox, oy, xyreso, yawreso)
    nstart = Node(round(Int64,sx/xyreso), round(Int64,sy/xyreso), round(Int64, syaw/yawreso),true,[sx],[sy],[syaw],[syaw1],[true],0.0,0.0, -1)
    ngoal = Node(round(Int64,gx/xyreso), round(Int64,gy/xyreso), round(Int64,gyaw/yawreso),true,[gx],[gy],[gyaw],[gyaw1],[true],0.0,0.0, -1)

    h_dp = calc_holonomic_with_obstacle_heuristic(ngoal, ox, oy, xyreso)

    open, closed = Dict{Int64, Node}(), Dict{Int64, Node}()
    fnode = nothing
    open[calc_index(nstart, c)] = nstart
    pq = PriorityQueue{Int64,Float64}()
    enqueue!(pq, calc_index(nstart, c), calc_cost(nstart, h_dp, ngoal, c))

    u, d = calc_motion_inputs()
    nmotion = length(u)

    while true
        if length(open) == 0
            println("Error: Cannot find path, No open set")
            return []
        end

        c_id = dequeue!(pq)
        current = open[c_id]

        #move current node from open to closed
        delete!(open, c_id)
        closed[c_id] = current

        isupdated, fpath = update_node_with_analystic_expantion(current, ngoal, c, ox, oy, kdtree, gyaw1)
        if isupdated # found
            fnode = fpath
            break
        end

        inityaw1 = current.yaw1[1]

        for i in 1:nmotion
            node = calc_next_node(current, c_id, u[i], d[i], c)

            if !verify_index(node, c, ox, oy, inityaw1, kdtree) continue end

            node_ind = calc_index(node, c)

            # If it is already in the closed set, skip it
            if haskey(closed, node_ind)  continue end

            if !haskey(open, node_ind)
                open[node_ind] = node
                enqueue!(pq, node_ind, calc_cost(node, h_dp, ngoal, c))
            else
                if open[node_ind].cost > node.cost
                    # If so, update the node to have a new parent
                    open[node_ind] = node
                end
            end
        end
    end

    println("final expand node:", length(open) + length(closed))

    path = get_final_path(closed, fnode, nstart, c)

    return path
end


function update_node_with_analystic_expantion(current::Node,
                                             ngoal::Node,
                                             c::Config,
                                             ox,
                                             oy,
                                             kdtree,
                                             gyaw1::Float64
                                            )

    apath = analystic_expantion(current, ngoal, c, ox, oy, kdtree)
    if apath != nothing
        fx = apath.x[2:end]
        fy = apath.y[2:end]
        fyaw =  apath.yaw[2:end]
        steps = MOTION_RESOLUTION*apath.directions
        yaw1 = trailerlib.calc_trailer_yaw_from_xyyaw(apath.x, apath.y, apath.yaw, current.yaw1[end], steps)
        if abs(rs_path.pi_2_pi(yaw1[end] - gyaw1)) >= GOAL_TYAW_TH
            return false, nothing #no update
        end
        fcost = current.cost + calc_rs_path_cost(apath, yaw1)
        fyaw1 = yaw1[2:end]
        fpind = calc_index(current, c)

        fd = Bool[]
        for d in apath.directions[2:end]
            if d >= 0
                push!(fd, true)
            else
                push!(fd, false)
            end
        end

        fsteer = 0.0

        fpath = Node(current.xind, current.yind, current.yawind, current.direction, fx, fy, fyaw, fyaw1, fd, fsteer, fcost, fpind)

        return true, fpath
    end

    return false, nothing #no update
end


function calc_rs_path_cost(rspath::rs_path.Path, yaw1)

    cost = 0.0
    for l in rspath.lengths
        if l >= 0 # forward
            cost += l
        else # back
            cost += abs(l) * BACK_COST
        end
    end

    # swich back penalty
    for i in 1:length(rspath.lengths) - 1
        if rspath.lengths[i] * rspath.lengths[i+1] < 0.0 # switch back
            cost += SB_COST
        end
    end

    # steer penalyty
    for ctype in rspath.ctypes
        if ctype != "S" # curve
            cost += STEER_COST*abs(MAX_STEER)
        end
    end

    # ==steer change penalty
    # calc steer profile
    nctypes = length(rspath.ctypes)
    ulist = fill(0.0, nctypes)
    for i in 1:nctypes
        if rspath.ctypes[i] == "R" 
            ulist[i] = - MAX_STEER
        elseif rspath.ctypes[i] == "L"
            ulist[i] = MAX_STEER
        end
    end
 
    for i in 1:length(rspath.ctypes) - 1
        cost += STEER_CHANGE_COST*abs(ulist[i+1] - ulist[i])
    end

    cost += JACKKNIF_COST * sum(abs.(rs_path.pi_2_pi.(rspath.yaw-yaw1)))

    return cost
end


function analystic_expantion(n::Node, ngoal::Node, c::Config, ox, oy, kdtree)

    sx = n.x[end]
    sy = n.y[end]
    syaw = n.yaw[end]

    max_curvature = tan(MAX_STEER)/WB
    paths = rs_path.calc_paths(sx,sy,syaw,ngoal.x[end], ngoal.y[end], ngoal.yaw[end],
                                   max_curvature, step_size=MOTION_RESOLUTION)

    if length(paths) == 0 
        return nothing
    end

    pathqueue = PriorityQueue{rs_path.Path, Float64}()
    for path in paths
        steps = MOTION_RESOLUTION*path.directions
        yaw1 = trailerlib.calc_trailer_yaw_from_xyyaw(path.x, path.y, path.yaw, n.yaw1[end], steps)
        enqueue!(pathqueue, path, calc_rs_path_cost(path, yaw1))
    end

    for i in length(pathqueue)
        path = dequeue!(pathqueue)

        steps = MOTION_RESOLUTION*path.directions
        yaw1 = trailerlib.calc_trailer_yaw_from_xyyaw(path.x, path.y, path.yaw, n.yaw1[end], steps)
        ind = 1:SKIP_COLLISION_CHECK:length(path.x)
        if trailerlib.check_trailer_collision(ox, oy, path.x[ind], path.y[ind], path.yaw[ind], yaw1[ind], kdtree = kdtree)
            # plot(path.x, path.y, "-^b")
            return path # path is ok
        end
    end

    return nothing
end


function calc_motion_inputs()

    up = [i for i in MAX_STEER/N_STEER:MAX_STEER/N_STEER:MAX_STEER]
    u = vcat([0.0], [i for i in up], [-i for i in up]) 
    d = vcat([1.0 for i in 1:length(u)], [-1.0 for i in 1:length(u)]) 
    u = vcat(u,u)

    return u, d
end


function verify_index(node::Node, c::Config, ox, oy, inityaw1, kdtree)

    # overflow map
    if (node.xind - c.minx) >= c.xw
        return false
    elseif (node.xind - c.minx) <= 0
        return false
    end
    if (node.yind - c.miny) >= c.yw
        return false
    elseif (node.yind - c.miny) <= 0
        return false
    end

    # check collisiton
    steps = MOTION_RESOLUTION*node.directions
    yaw1 = trailerlib.calc_trailer_yaw_from_xyyaw(node.x, node.y, node.yaw, inityaw1, steps)
    ind = 1:SKIP_COLLISION_CHECK:length(node.x)
    if !trailerlib.check_trailer_collision(ox, oy, node.x[ind], node.y[ind], node.yaw[ind], yaw1[ind], kdtree = kdtree)
        return false
    end

    return true #index is ok"
end


function calc_next_node(current::Node, c_id::Int64,
                        u::Float64, d::Float64, 
                        c::Config,
                        )

    arc_l = XY_GRID_RESOLUTION*1.5

    nlist = floor(Int64, arc_l/MOTION_RESOLUTION)+1
    xlist = fill(0.0, nlist)
    ylist = fill(0.0, nlist)
    yawlist = fill(0.0, nlist)
    yaw1list = fill(0.0, nlist)

    xlist[1] = current.x[end] + d * MOTION_RESOLUTION*cos(current.yaw[end])
    ylist[1] = current.y[end] + d * MOTION_RESOLUTION*sin(current.yaw[end])
    yawlist[1] = rs_path.pi_2_pi(current.yaw[end] + d*MOTION_RESOLUTION/WB * tan(u))
    yaw1list[1] = rs_path.pi_2_pi(current.yaw1[end] + d*MOTION_RESOLUTION/LT*sin(current.yaw[end]-current.yaw1[end]))
 
    for i in 1:(nlist-1)
        xlist[i+1] = xlist[i] + d * MOTION_RESOLUTION*cos(yawlist[i])
        ylist[i+1] = ylist[i] + d * MOTION_RESOLUTION*sin(yawlist[i])
        yawlist[i+1] = rs_path.pi_2_pi(yawlist[i] + d*MOTION_RESOLUTION/WB * tan(u))
        yaw1list[i+1] = rs_path.pi_2_pi(yaw1list[i] + d*MOTION_RESOLUTION/LT*sin(yawlist[i]-yaw1list[i]))
    end
 
    xind = round(Int64, xlist[end]/c.xyreso)
    yind = round(Int64, ylist[end]/c.xyreso)
    yawind = round(Int64, yawlist[end]/c.yawreso)

    addedcost = 0.0
    if d > 0
        direction = true
        addedcost += abs(arc_l)
    else
        direction = false
        addedcost += abs(arc_l) * BACK_COST
    end

    # swich back penalty
    if direction != current.direction # switch back penalty
        addedcost += SB_COST
    end

    # steer penalyty
    addedcost += STEER_COST*abs(u)

    # steer change penalty
    addedcost += STEER_CHANGE_COST*abs(current.steer - u)

    # jacknif cost
    addedcost += JACKKNIF_COST * sum(abs.(rs_path.pi_2_pi.(yawlist-yaw1list)))

    cost = current.cost + addedcost 

    directions = [direction for i in 1:length(xlist)]
    node = Node(xind, yind, yawind, direction, xlist, ylist, yawlist, yaw1list, directions, u, cost, c_id)

    return node
end


function is_same_grid(node1::Node,node2::Node)

    if node1.xind != node2.xind
        return false
    end
    if node1.yind != node2.yind
        return false
    end
    if node1.yawind != node2.yawind
        return false
    end

    return true

end


function calc_index(node::Node, c::Config)
    ind = (node.yawind - c.minyaw)*c.xw*c.yw+(node.yind - c.miny)*c.xw + (node.xind - c.minx)

    # 4D grid
    yaw1ind = round(Int64, node.yaw1[end]/c.yawreso)
    ind += (yaw1ind - c.minyawt) *c.xw*c.yw*c.yaww

    if ind <= 0
        println("Error(calc_index):", ind)
    end
    return ind
end


function calc_holonomic_with_obstacle_heuristic(gnode::Node, ox::Array{Float64}, oy::Array{Float64}, xyreso::Float64)
    h_dp = grid_a_star.calc_dist_policy(gnode.x[end], gnode.y[end], ox, oy, xyreso, 1.0)
    return h_dp
end


function calc_config(ox::Array{Float64}, oy::Array{Float64},
                     xyreso::Float64, yawreso::Float64
                     )

    min_x_m = minimum(ox) - EXTEND_AREA
    min_y_m = minimum(oy) - EXTEND_AREA
    max_x_m = maximum(ox) + EXTEND_AREA
    max_y_m = maximum(oy) + EXTEND_AREA

    push!(ox, min_x_m)
    push!(oy, min_y_m)
    push!(ox, max_x_m)
    push!(oy, max_y_m)

    minx = round(Int64, min_x_m/xyreso)
    miny = round(Int64, min_y_m/xyreso)
    maxx = round(Int64, max_x_m/xyreso)
    maxy = round(Int64, max_y_m/xyreso)

    xw = round(Int64,(maxx - minx))
    yw = round(Int64,(maxy - miny))

    minyaw = round(Int64, - pi/yawreso) - 1
    maxyaw = round(Int64, pi/yawreso)
    yaww = round(Int64,(maxyaw - minyaw))

    minyawt = minyaw
    maxyawt = maxyaw
    yawtw = yaww

    config = Config(minx, miny, minyaw, minyawt, maxx, maxy, maxyaw, maxyawt, xw, yw, yaww, yawtw,
                    xyreso, yawreso)

    return config
end


function get_final_path(closed::Dict{Int64, Node},
                        ngoal::Node,
                        nstart::Node,
                        c::Config)

    rx, ry, ryaw = Array{Float64}(reverse(ngoal.x)),Array{Float64}(reverse(ngoal.y)),Array{Float64}(reverse(ngoal.yaw))
    ryaw1 = Array{Float64}(reverse(ngoal.yaw1))
    direction = Array{Float64}(reverse(ngoal.directions))
    nid = ngoal.pind
    finalcost = ngoal.cost

    while true
        n = closed[nid]
        rx = vcat(rx, reverse(n.x))
        ry = vcat(ry, reverse(n.y))
        ryaw = vcat(ryaw, reverse(n.yaw))
        ryaw1 = vcat(ryaw1, reverse(n.yaw1))
        direction = vcat(direction, reverse(n.directions))
        nid = n.pind
        if is_same_grid(n, nstart)
            break
        end
    end

    rx = reverse(rx)
    ry = reverse(ry)
    ryaw = reverse(ryaw)
    ryaw1 = reverse(ryaw1)
    direction = reverse(direction)

    # adjuct first direction
    direction[1] = direction[2]

    path = Path(rx, ry, ryaw, ryaw1, direction, finalcost)

    return path
end


function calc_cost(n::Node, h_dp::Array{Float64}, ngoal::Node, c::Config)

   return (n.cost + H_COST*h_dp[n.xind - c.minx, n.yind - c.miny])

end


function main()
    println(PROGRAM_FILE," start!!")

    sx = 14.0  # [m]
    sy = 10.0  # [m]
    syaw0 = deg2rad(00.0)
    syaw1 = deg2rad(00.0)

    gx = 0.0  # [m]
    gy = 0.0  # [m]
    gyaw0 = deg2rad(90.0)
    gyaw1 = deg2rad(90.0)

    ox = Float64[]
    oy = Float64[]

    for i in -25:25
        push!(ox, Float64(i))
        push!(oy, 15.0)
    end
    for i in -25:-4
        push!(ox, Float64(i))
        push!(oy, 4.0)
    end
    for i in -15:4
        push!(ox, -4.0)
        push!(oy, Float64(i))
    end
    for i in -15:4
        push!(ox, 4.0)
        push!(oy, Float64(i))
    end
    for i in 4:25
        push!(ox, Float64(i))
        push!(oy, 4.0)
    end
    for i in -4:4
        push!(ox, Float64(i))
        push!(oy, -15.0)
    end

    oox = ox[:]
    ooy = oy[:]

    @time path = calc_hybrid_astar_path(sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)

    plot(oox, ooy, ".k")
    trailerlib.plot_trailer(sx, sy, syaw0, syaw1, 0.0)
    trailerlib.plot_trailer(gx, gy, gyaw0, gyaw1, 0.0)
    x = path.x
    y = path.y
    yaw = path.yaw
    yaw1 = path.yaw1
    direction = path.direction

    steer = 0.0
    for ii in 1:length(x)
        cla()
        plot(oox, ooy, ".k")
        plot(x, y, "-r", label="Hybrid A* path")

        if ii < length(x)-1
            k = (yaw[ii+1] - yaw[ii])/MOTION_RESOLUTION
            if !direction[ii]
                k *= -1
            end
            steer = atan(WB*k, 1.0)
        else
            steer = 0.0
        end
        trailerlib.plot_trailer.(x[ii], y[ii], yaw[ii], yaw1[ii], steer)
        grid(true)
        axis("equal")
        pause(0.0001)
    end
    println("Done")
    axis("equal")
    show()

    println(PROGRAM_FILE," Done!!")
end


end #module


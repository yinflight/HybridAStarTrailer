#
# Test script
#
# author: Atsushi Sakai
#

using Test

include("./lib/trailer_hybrid_a_star.jl")

function test()
    println("Test Start !!!")

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

    sx = -10.0  # [m]
    sy = 6.0  # [m]
    syaw0 = deg2rad(00.0)
    syaw1 = deg2rad(00.0)


    @time path = trailer_hybrid_a_star.calc_hybrid_astar_path(sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1, ox, oy,
                                                               trailer_hybrid_a_star.XY_GRID_RESOLUTION,
                                                               trailer_hybrid_a_star.YAW_GRID_RESOLUTION)



    @test length(path.x)>=1

    sx = 14.0  # [m]
    sy = 10.0  # [m]
    syaw0 = deg2rad(00.0)
    syaw1 = deg2rad(00.0)

    @time path = trailer_hybrid_a_star.calc_hybrid_astar_path(sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1, ox, oy,
                                                               trailer_hybrid_a_star.XY_GRID_RESOLUTION,
                                                               trailer_hybrid_a_star.YAW_GRID_RESOLUTION)



    @test length(path.x)>=1

    sx = -14.0  # [m]
    sy = 12.0  # [m]
    syaw0 = deg2rad(00.0)
    syaw1 = deg2rad(00.0)
    @time path = trailer_hybrid_a_star.calc_hybrid_astar_path(sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1, ox, oy,
                                                               trailer_hybrid_a_star.XY_GRID_RESOLUTION,
                                                               trailer_hybrid_a_star.YAW_GRID_RESOLUTION)



    @test length(path.x)>=1

    sx = -20.0  # [m]
    sy = 6.0  # [m]
    syaw0 = deg2rad(00.0)
    syaw1 = deg2rad(00.0)
    @time path = trailer_hybrid_a_star.calc_hybrid_astar_path(sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1, ox, oy,
                                                               trailer_hybrid_a_star.XY_GRID_RESOLUTION,
                                                               trailer_hybrid_a_star.YAW_GRID_RESOLUTION)



    @test length(path.x)>=1

    sx = -14.0  # [m]
    sy = 12.0  # [m]
    syaw0 = deg2rad(00.0)
    syaw1 = deg2rad(00.0)
    @time path = trailer_hybrid_a_star.calc_hybrid_astar_path(sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1, ox, oy,
                                                               trailer_hybrid_a_star.XY_GRID_RESOLUTION,
                                                               trailer_hybrid_a_star.YAW_GRID_RESOLUTION)



    @test length(path.x)>=1

    sx = -20.0  # [m]
    sy = 6.0  # [m]
    syaw0 = deg2rad(180.0)
    syaw1 = deg2rad(180.0)
    @time path = trailer_hybrid_a_star.calc_hybrid_astar_path(sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1, ox, oy,
                                                               trailer_hybrid_a_star.XY_GRID_RESOLUTION,
                                                               trailer_hybrid_a_star.YAW_GRID_RESOLUTION)




    @test length(path.x)>=1

    sx = -20.0  # [m]
    sy = 12.0  # [m]
    syaw0 = deg2rad(180.0)
    syaw1 = deg2rad(180.0)

    @time path = trailer_hybrid_a_star.calc_hybrid_astar_path(sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1, ox, oy,
                                                               trailer_hybrid_a_star.XY_GRID_RESOLUTION,
                                                               trailer_hybrid_a_star.YAW_GRID_RESOLUTION
                                                               )


    @test length(path.x)>=1

    println("Test Done !!!")

end


@time test()


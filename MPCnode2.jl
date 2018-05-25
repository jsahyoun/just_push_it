#!/usr/bin/env julia
# Author: Joseph Sahyoun
# This node uses MPC algorithms to calculate the optimal inputs that minimizes 
# the distance between the BARC and the reference trajectory. For this demonstration
# a dummy reference trajectory is fed into this node which then publishes the optimal
# state trajectory.

println("Initializing Packages...")

using RobotOS
@rosimport std_msgs.msg: Float64MultiArray
@rosimport barc.msg: Input, barc_state
@rosimport data_service.msg: TimeData
@rosimport geometry_msgs.msg: Vector3
rostypegen()
using barc.msg
using data_service.msg
using geometry_msgs.msg
using std_msgs.msg
using JuMP
using Ipopt


println("DONE")

# -------------------------------------------------------------------
#                           MPC PARAMETERS
# -------------------------------------------------------------------

println("Defining Parameters...")

# Number of states / Number of inputs
nz = 3
nu = 2

# Horizons
n = 7           # MPC Horizon (CFTOC Horizon)

# Cost matrices
Q = [10 0 0; 0 10 0; 0 0 0]
R = [1 0; 0 0.1]

# Hardware parameters
dt = 0.65
lr = 0.15
lf = 0.15

#Box constraints on states
x_min = -1
x_max = 1
y_min = 0
y_max = 2

# Box constraints on input, A*u <= B
v_max = 3.02
v_min = 0
steer_min = -1
steer_max = 1

vref = 1.5

version = 1

println("DONE")

# -------------------------------------------------------------------
#                         SET UP CFTOC MODEL
# -------------------------------------------------------------------

println("Creating CFTOC Model...")

m = Model(solver = IpoptSolver(print_level=0))

# Variables
@variable(m,z[1:nz,1:n+1])              # State vectors (nz x n+1)
@variable(m,u[1:nu,1:n])                # Input vectors (nu x n)

# Reference Trajectory
@NLparameter(m, xref[i=1:n] == 0)
@NLparameter(m, yref[i=1:n] == 0)
@NLparameter(m, pref[i=1:n] == 0)

# Objective Function

if version == 1
    @NLobjective(m, :Min, sum(Q[1,1]*(z[1,i+1]-xref[i])^2 + Q[2,2]*(z[2,i+1]-yref[i])^2 + Q[3,3]*(z[3,i+1]-pref[i])^2 for i in 1:n) + sum(R[1,1]*(u[1,i]-vref)^2 + R[2,2]*u[2,i]^2 for i in 1:n))
end


# State Constraints

for i in 1:n+1
    @constraint(m, x_min <= z[1,i] <= x_max)
    @constraint(m, y_min <= z[2,i] <= y_max)
    @constraint(m, -3.1415926/2 <= z[3,i] <= 3.1415926/2)
end

for i in 1:n
    @constraint(m, v_min <= u[1,i] <= v_max)
    @constraint(m, steer_min <= u[2,i] <= steer_max)
end

# Initial Condition and Constraints
@NLparameter(m, x0 == 0); @NLconstraint(m, z[1,1] == x0);
@NLparameter(m, y0 == 0); @NLconstraint(m, z[2,1] == y0);
@NLparameter(m, p0 == 0); @NLconstraint(m, z[3,1] == p0);

# Dynamics Constraints
for i in 1:n                            
    @NLconstraint(m, z[1,i+1] == z[1,i] + dt*u[1,i]*cos(z[3,i]+atan(lr*tan(u[2,i])/(lf+lr))))
    @NLconstraint(m, z[2,i+1] == z[2,i] + dt*u[1,i]*sin(z[3,i]+atan(lr*tan(u[2,i])/(lf+lr))))
    @NLconstraint(m, z[3,i+1] == z[3,i] + dt*(u[1,i]/lr)*sin(atan(lr*tan(u[2,i])/(lf+lr))))
end



println("DONE")

# -------------------------------------------------------------------
#                             INITIAL SOLVE
# -------------------------------------------------------------------

println("Initial solve...")
solve(m)
println("DONE")

# -------------------------------------------------------------------
#                             ROS FUNCTIONS
# -------------------------------------------------------------------

println("Running MPC...")


function callback2(msg::barc_state)
    # Assign new reference trajectory
    for i in 1:n
        setvalue(xref[i], msg.x[i])
        setvalue(yref[i], msg.y[i])
        setvalue(pref[i], msg.psi[i])
    end
end

function loop(pub_one, pub_two)
    loop_rate = Rate(.3)
    while ! is_shutdown()

        # -----------------------------------------------------------
        #                      RUNNING MPC
        # -----------------------------------------------------------
       
        solve(m)                                # Solve CFTOC
        zOpt = getvalue(z)
        uOpt = getvalue(u)
        JOpt = getobjectivevalue(m)

        # Publish optimal inputs and optimal state trajectory
        # Only need to publish state traj for this demonstration
        # cmdInt = Input(uOpt[1,1], uOpt[2,1])    # Command to publish
        optTraj = barc_state(zOpt[1,1:end],zOpt[2,1:end],zOpt[3,1:end])
        # publish(pub_one,cmdInt)                 
        publish(pub_two,optTraj)
        rossleep(loop_rate)

    end
end

function main()
    init_node("MPC_node")
    pub = Publisher("uOpt", Input, queue_size=10)
    pub2 = Publisher("optimal_state_trajectory", barc_state, queue_size=10)
    sub2 = Subscriber("reference_trajectory", barc_state, callback2, queue_size=10)
    loop(pub, pub2)
end

if ! isinteractive()
    main()
end

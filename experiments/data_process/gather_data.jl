#!/usr/bin/env julia

using Printf
using RobotOS
using Dates
using DataFrames
using CSV

# include("unscented_transform.jl")

@rosimport mrs_msgs.msg:RangeWithCovarianceArrayStamped, ControlManagerDiagnostics, PoseWithCovarianceArrayStamped
@rosimport geometry_msgs.msg:PoseWithCovarianceStamped
@rosimport nav_msgs.msg:Odometry
rostypegen()

# global data = DataFrame(
#     timestamp=Float64[],
#     range_gt=Float64[],
#     range_ot=Float64[],
#     range_var_ot=Float64[],
#     mh=Float64[],
#     x_gt=Float64[],
#     y_gt=Float64[],
#     z_gt=Float64[],
#     x_ot=Float64[],
#     y_ot=Float64[],
#     z_ot=Float64[]
# )

global distance_data = DataFrame(
    timestamp=Float64[], 
    range_uwb=Float64[], 
    variance=Float64[],
    rtk_distance=Float64[],
    angle=Float64[]
)

global uav39_rtk = [0, 0, 0]
global uav38_rtk = [0, 0, 0]

global START_TIME::Float64 = NaN

# function object_tracker_cb(msg::mrs_msgs.msg.PoseWithCovarianceArrayStamped)
#     try
#         time_stamp::Float64 = (convert(Float64, msg.header.stamp.secs) + msg.header.stamp.nsecs * 1e-9)
#         global START_TIME

#         global tracking_trajectory

#         if !tracking_trajectory
#             if START_TIME == NaN
#                 return
#             end
#             START_TIME = NaN
#             empty!(data)
#             return
#         end

#         @printf "Got data from object tracker\n\r"

#         if (isnan(START_TIME) || time_stamp < START_TIME)
#             START_TIME = time_stamp
#         end

#         time_stamp = time_stamp - START_TIME

#         uav1 = position_to_vector(uav1_gt.pose.pose.position)
#         position_gt_g = position_to_vector(uav2_gt.pose.pose.position)

#         position_gt_r = position_gt_g - uav1
#         ground_truth_dist = (position_gt_r' * position_gt_r)^(1 / 2)

#         uav1_q = uav1_gt.pose.pose.orientation
#         quaternion = quat(uav1_q.w, uav1_q.x, uav1_q.y, uav1_q.z)

#         for measurement in msg.poses
#             position_r_raw = position_to_vector(measurement.pose.position)
#             position_r = rotate_vector(conj(quaternion), position_r_raw)
#             position_g = position_r + uav1
#             range = (position_r' * position_r)^(1 / 2)

#             covariance_raw = reshape(measurement.covariance, (6, 6))
#             covariance_raw = covariance_raw'
#             covariance_raw = covariance_raw[1:3, 1:3]
#             covariance = rotmatrix_from_quat(conj(quaternion)) * covariance_raw
#             mahanalobis = sqrt((position_gt_r - position_r)' * inv(covariance) * (position_gt_r - position_r))

#             σ, Wm, Wc = UT.compute_sigma_pts(position_r_raw, covariance_raw)

#             @. σ ^= 2 
#             σ = sum(σ, dims=2)
#             @. σ ^= 1/2

#             μ = Wm'*σ
#             Σ = zeros(Float64, (1, 1))

#             for (index, value) in enumerate(Wc)
#                 @. Σ += value*(σ[index, :] - μ)*(σ[index, :] - μ)'
#             end

#             push!(data, [time_stamp 
#             ground_truth_dist 
#             μ[1, 1] 
#             Σ[1, 1] 
#             mahanalobis 
#             position_gt_g...
#             position_g...])
#             # filter!(row -> row.timestamp > time_stamp - 20, data[measurement.id])
#         end

#         CSV.write("object_tracker.csv", data)
#     catch y
#         println(y)
#         rethrow(y)
#     end
# end


function uwb_cb(msg::mrs_msgs.msg.RangeWithCovarianceArrayStamped)
    time_stamp::Float64 = (convert(Float64, msg.header.stamp.secs) + msg.header.stamp.nsecs * 1e-9)
    global START_TIME
    global distance_data

    global uav38_rtk
    global uav39_rtk

    angle = atan(uav38_rtk[2] - uav39_rtk[2], uav38_rtk[1] - uav39_rtk[1])

    rtk_distance::Float64 = sqrt((uav39_rtk - uav38_rtk)' * (uav39_rtk - uav38_rtk))

    if rtk_distance < 0.1 || rtk_distance > 200
        return
    end

    if (isnan(START_TIME) || time_stamp < START_TIME)
        START_TIME = time_stamp
    end

    time_stamp = time_stamp - START_TIME

    # rtk_distance::Float64 = 0

    for measurement in msg.ranges
        if (measurement.range.range < 0)
            return
        end

        push!(distance_data, [time_stamp measurement.range.range measurement.variance rtk_distance angle])
    end
end

function ground_truth_cb(msg::nav_msgs.msg.Odometry, uav::Int)
    global uav38_rtk
    global uav39_rtk
    global distance_data

    if uav == 1
        uav38_rtk = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
    end
    if uav == 2
        uav39_rtk = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
    end
end

function uav38_odom_cb(msg::nav_msgs.msg.Odometry)
    ground_truth_cb(msg, 1)
end

function uav39_odom_cb(msg::nav_msgs.msg.Odometry)
    ground_truth_cb(msg, 2)
end

function save_csv()
    global distance_data
    @printf "Saving data\n\r"
    CSV.write("data.csv", distance_data)
end

function main()
    global distance_data

    init_node("gather_data", anonymous=true)

    # CSV.write("object_tracker.csv", data)
    CSV.write("data.csv", distance_data)

    @printf "Gather data\n\r"

    t = Timer((t) -> save_csv(), 1; interval=1) 
    uav38_odom_sub = Subscriber{nav_msgs.msg.Odometry}("/uav38/odometry/odom_main", uav38_odom_cb)
    uav39_odom_sub = Subscriber{nav_msgs.msg.Odometry}("/uav39/odometry/odom_main", uav39_odom_cb)
    # object_tracker_sub = Subscriber{mrs_msgs.msg.PoseWithCovarianceArrayStamped}("/uav39", object_tracker_cb)

    uwb_sub = Subscriber{mrs_msgs.msg.RangeWithCovarianceArrayStamped}("/uav39/uwb_range/range_out", uwb_cb)
    while(true)
      try
        spin()
      catch y
        # nothing
      end
    end
end

main()

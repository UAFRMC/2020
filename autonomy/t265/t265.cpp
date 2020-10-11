//Created by Matt Perry
//Realsense T265 API and transform to usefull robot data.
// This code will write to the lunatic data exchange with update robot data based on t265 pose transforms
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <aurora/lunatic.h>

void quaternion_to_coord3d(rs2_quaternion q, aurora::robot_coord3D & coord){
    float qxx(q.x * q.x);
    float qyy(q.y * q.y);
    float qzz(q.z * q.z);
    float qxz(q.x * q.z);
    float qxy(q.x * q.y);
    float qyz(q.y * q.z);
    float qwx(q.w * q.x);
    float qwy(q.w * q.y);
    float qwz(q.w * q.z);

    coord.X.x = float(1) - float(2) * (qyy + qzz);
    coord.X.y = float(2) * (qxy + qwz);
    coord.X.z = float(2) * (qxz - qwy);

    coord.Y.x = float(2) * (qxy - qwz);
    coord.Y.y = float(1) - float(2) * (qxx + qzz);
    coord.Y.z = float(2) * (qyz + qwx);

    coord.Z.x = float(2) * (qxz + qwy);
    coord.Z.y = float(2) * (qyz - qwx);
    coord.Z.z = float(1) - float(2) * (qxx + qyy);
}

void convert_vec3(vec3 & out, vec3 in){
    out.x = -1 * in.x;
    out.y = in.z;
    out.z = in.y;
}

int main(){
    rs2::pipeline data;
    rs2::config conf;
    conf.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    data.start(conf);
    MAKE_exchange_T265();
    
    while(true){
        auto frames = data.wait_for_frames();
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
        aurora::robot_coord3D & T265 = exchange_T265.write_begin();
        aurora::robot_coord3D raw;
        raw.origin.x = pose_data.translation.x * 100;
        raw.origin.y = pose_data.translation.y * 100;
        raw.origin.z = pose_data.translation.z * 100;
        convert_vec3(T265.origin, raw.origin);
        quaternion_to_coord3d(pose_data.rotation, raw);
        convert_vec3(T265.X, -raw.X);
        convert_vec3(T265.Y, raw.Z);
        convert_vec3(T265.Z, raw.Y);
        T265.percent = pose_data.tracker_confidence * 100 / 3;
        exchange_T265.write_end();
        T265.print();
    }

}
#include "foot_controller.h"

class Foot_Controller : public rclcpp::Node{

    public: Foot_Controller(): Node("foot_controller"){
        cmd.motorType = MotorType::GO_M8010_6;
        serial = new SerialPort("/dev/ttyS7");
        // std::thread leg0(control_leg,0);
        // std::thread leg1(control_leg,1);
        std::thread leg2(control_leg,2);
        std::thread leg3(control_leg,3);
        // leg0.join();
        // leg1.join();
        leg2.join();
        leg3.join();
    }
};

void SendMsg(MotorData* data,MotorCmd* cmd){
    lock.lock();
    serial->sendRecv(cmd,data);
    lock.unlock();
}

float clip(float var,float maxVal){
    if(var < - maxVal){
        return -maxVal;
    }
    if(var > maxVal){
        return maxVal;
    }
    return var;
}

void control_leg(uint id){
    // // start count
    // auto start = std::chrono::system_clock::now();
    // int count = 0;
    VMC_Param param;
    param.kp_x = 1000;
    param.kd_x = 10;
    param.kp_y = 100;
    param.kd_y = 10;
    // init motor data
    MotorData data_outer,data_inner;
    data_outer.motorType = MotorType::GO_M8010_6;
    data_inner.motorType = MotorType::GO_M8010_6;
    // init motor cmd
    MotorCmd cmd_outer,cmd_inner;
    cmd_outer.motorType = MotorType::GO_M8010_6;
    cmd_inner.motorType = MotorType::GO_M8010_6;
    cmd_outer.id = motor_id_for_legs[id][0];
    cmd_outer.mode = 1;
    cmd_outer.tau = 0;
    cmd_outer.kp = 0;
    cmd_outer.kd = 0;
    cmd_outer.q = 0;
    cmd_outer.dq = 0;
    cmd_inner.id = motor_id_for_legs[id][1];
    cmd_inner.mode = 1;
    cmd_inner.tau = 0;
    cmd_inner.kp = 0;
    cmd_inner.kd = 0;
    cmd_inner.q = 0;
    cmd_inner.dq = 0;
    // send init cmd to motors
    SendMsg(&data_outer,&cmd_outer);
    SendMsg(&data_inner,&cmd_inner);
    // save motor offests
    float outer_motor_offest = data_outer.q
        ,inner_motor_offest = data_inner.q;
    // get gear_ratio
    float gear_ratio = queryGearRatio(MotorType::GO_M8010_6);
    // init values
    int dir_outer = motor_dir[motor_id_for_legs[id][0]]
        ,dir_inner = motor_dir[motor_id_for_legs[id][1]];
    float target_pos_x
        ,target_pos_y
        ,angle_alpha
        ,angle_beta
        ,vec_alpha
        ,vec_beta
        ,outer_tau
        ,inner_tau;
    KinematicResult kineRes;
    VMC_Result vmcRes;
    JacobiResult jocRes;
    // set init pos
    kineRes = Kinematic_Solution(OUTER_MOTOR_OFFEST,INNER_MOTOR_OFFEST,0,0);
    target_pos_x = kineRes.pos_x;
    target_pos_y = kineRes.pos_z;
    // target_pos_x = 0;
    // target_pos_y = 0.223;
    // // test target pos out
    // std::cout << target_pos_x << "  " << target_pos_y << std::endl;
    // return;
    while(rclcpp::ok()){
        angle_alpha = ((data_outer.q - outer_motor_offest) / gear_ratio)*dir_outer + OUTER_MOTOR_OFFEST;
        angle_beta = ((data_inner.q - inner_motor_offest) / gear_ratio)*dir_inner + INNER_MOTOR_OFFEST;
        vec_alpha = data_outer.dq / gear_ratio * dir_outer;
        vec_beta = data_inner.dq / gear_ratio * dir_inner;
        kineRes = Kinematic_Solution(angle_alpha,angle_beta,vec_alpha,vec_beta);
        vmcRes = VMC_Calculate(&param,target_pos_x,target_pos_y,kineRes.pos_x,kineRes.pos_z,kineRes.vec_x,kineRes.vec_z);
        jocRes = VMC_Jacobi_Matrix(angle_alpha,angle_beta,vmcRes.force_x,vmcRes.force_z);
        outer_tau = jocRes.tau_alpha / gear_ratio * dir_outer;
        inner_tau = jocRes.tau_beta / gear_ratio * dir_inner;
        // outer_tau = clip(outer_tau,0.1);
        // inner_tau = clip(inner_tau,0.1);
        // printf("force_x:%.3f force_y:%.3f outer_tau:%.3f inner_tau:%.3f\r\n",vmcRes.force_x,vmcRes.force_z,outer_tau,inner_tau);
        cmd_outer.tau = outer_tau;
        cmd_inner.tau = inner_tau;
        SendMsg(&data_outer,&cmd_outer);
        SendMsg(&data_inner,&cmd_inner);
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // // count freq
        // count ++ ;
        // auto time_now = std::chrono::system_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - start);
        // if(duration.count()>1000){
        //     std::cout << "id:" << id << " count:" << count << std::endl;
        //     start = time_now;
        //     count = 0;
        // }
    }
    cmd_outer.tau = 0;
    cmd_inner.tau = 0;
    SendMsg(&data_outer,&cmd_outer);
    SendMsg(&data_inner,&cmd_inner);
}

int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<Foot_Controller>());
	rclcpp::shutdown();
    return 0;
}
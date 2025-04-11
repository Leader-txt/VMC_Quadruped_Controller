#include "foot_controller.h"

class Foot_Controller : public rclcpp::Node{

    public: Foot_Controller(): Node("foot_controller"){
        cmd.motorType = MotorType::GO_M8010_6;
        serial = new SerialPort("/dev/ttyS7");
        std::thread leg0(control_leg,0);
    }
};

MotorData SendMsg(uint id,uint mode,float tau,float kp,float kd,float q,float dq){
    cmd.motorType = MotorType::GO_M8010_6;
    cmd.id = id;
    cmd.mode = mode;
    cmd.tau = tau;
    cmd.kp = kp;
    cmd.kd = kd;
    cmd.q = q;
    cmd.dq = dq;
    MotorData data;
    data.motorType = MotorType::GO_M8010_6;
    serial->sendRecv(&cmd,&data);
    return data;
}

void control_leg(uint id){
    clock_t start=clock();
    int count = 0;
    while(rclcpp::ok()){
        SendMsg(motor_id_for_legs[id][0],1,0,0,0,0,0);
        SendMsg(motor_id_for_legs[id][1],1,0,0,0,0,0);
        count ++ ;
        if(clock() - start > 1000){
            std::cout << count << std::endl;
            count = 0;
            start = clock();
        }
    }
}

int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<Foot_Controller>());
	rclcpp::shutdown();
    return 0;
}
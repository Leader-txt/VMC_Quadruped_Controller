#include "foot_controller.h"

class Foot_Controller : public rclcpp::Node{
    private:
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription;
        rclcpp::Subscription<vmc_quadruped_controller::msg::MoveCmd>::SharedPtr move_cmd_subscription;
        rclcpp::Subscription<yesense_interface::msg::EulerOnly>::SharedPtr euler_subscription;
        bool stand_up_flag = false;
        bool ctrl_by_joy = true;
        bool use_imu_pitch = false;
        float init_pos[4][2]
            ,leg_pos[4][2] // leg_pos {x,y}
            ,period = NORMAL_GAIT_PERIOD
            ,BODY_HEIGHT = NORMAL_GAIT_BODY_HEIGHT
            ,step_x
            ,step_y
            ,imu_pitch=0
            ,imu_yaw=0
            ,target_yaw=360;
        bool inited[4]={0,0,0,0}
            ,running = false
            ,runner_exists = false;
        Cycloid cycloid;
        VMC_Param params[4];
    public: Foot_Controller(): Node("foot_controller"){
        joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>("joy",10,std::bind(&Foot_Controller::joy_callback,this,std::placeholders::_1));
        move_cmd_subscription = this->create_subscription<vmc_quadruped_controller::msg::MoveCmd>("move_cmd",10,std::bind(&Foot_Controller::move_cmd_callback,this,std::placeholders::_1));
        euler_subscription = this->create_subscription<yesense_interface::msg::EulerOnly>("euler_only",10,std::bind(&Foot_Controller::euler_callback,this,std::placeholders::_1));
        cycloid.Length = 0.05;
        cycloid.Height = NORMAL_GAIT_HEIGHT;
        cycloid.FlightPercent = NORMAL_GAIT_FLIGHT_PERCENT;
        cycloid.BodyHeight = NORMAL_GAIT_BODY_HEIGHT;
        for(int i=0;i<4;i++){
            params[i].kp_x = INIT_KP_X;
            params[i].ki_x = INIT_KI_X;
            params[i].kd_x = INIT_KD_X;
            params[i].kp_y = INIT_KP_Y;
            params[i].ki_y = INIT_KI_Y;
            params[i].kd_y = INIT_KD_Y;
        }
        std::thread leg0(&Foot_Controller::control_leg,this,0);
        std::thread leg1(&Foot_Controller::control_leg,this,1);
        std::thread leg2(&Foot_Controller::control_leg,this,2);
        std::thread leg3(&Foot_Controller::control_leg,this,3);
        leg0.detach();
        leg1.detach();
        leg2.detach();
        leg3.detach();
    }
    private: void euler_callback(const yesense_interface::msg::EulerOnly::SharedPtr msg){
        // RCLCPP_INFO(this->get_logger(),"euler:%.3f %.3f %.3f",msg->euler.pitch,msg->euler.roll,msg->euler.yaw);
        imu_pitch = msg->euler.pitch/180.0*M_PI;
        imu_yaw = msg->euler.yaw;
        // RCLCPP_INFO(this->get_logger(),"imu_pitch:%.3f",imu_pitch);
    }
    private: void move_cmd_callback(const vmc_quadruped_controller::msg::MoveCmd::SharedPtr msg){
        if(!ctrl_by_joy){
            step_x = msg->step_x;
            step_y = msg->step_y;
            RCLCPP_INFO(this->get_logger(),"step_x:%.3f step_y:%.3f",step_x,step_y);
            bool can_run = (abs(msg->step_x)>0 || abs(msg->step_y)>0);
            if(stand_up_flag && !running && can_run && !runner_exists){
                RCLCPP_INFO(this->get_logger(),"start run");
                std::thread runner(&Foot_Controller::run,this);
                runner.detach();
            }
            else if(running && !can_run){
                RCLCPP_INFO(this->get_logger(),"end run");
                running = false;
            }
        }
    }

    private: void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
        // // 打印轴和按钮状态
        // RCLCPP_INFO(this->get_logger(), "收到Joy消息:");
        
        // // 打印所有轴
        // RCLCPP_INFO(this->get_logger(), "轴:");
        // for (size_t i = 0; i < msg->axes.size(); ++i) {
        // RCLCPP_INFO(this->get_logger(), "  轴[%zu]: %.2f", i, msg->axes[i]);
        // }
        
        // // 打印所有按钮
        // RCLCPP_INFO(this->get_logger(), "按钮:");
        // for (size_t i = 0; i < msg->buttons.size(); ++i) {
        // RCLCPP_INFO(this->get_logger(), "  按钮[%zu]: %d", i, msg->buttons[i]);
        // }
        if(msg->buttons[STAND_UP_BTN] && !stand_up_flag){
            if(inited[0]&&inited[1]&&inited[2]&&inited[3])
                stand_up();
        }
        if(msg->buttons[SIT_DOWN_BTN] && stand_up_flag){
            sit_down();
        }
        if(msg->buttons[USE_IMU_PITCH_BTN]){
            use_imu_pitch = !use_imu_pitch;
            if(use_imu_pitch)
                RCLCPP_INFO(get_logger(),"enable imu pitch");
            else
                RCLCPP_INFO(get_logger(),"disable imu pitch");
        }
        if(msg->buttons[NORMAL_GAIT_BTN]){
            BODY_HEIGHT = NORMAL_GAIT_BODY_HEIGHT;
            cycloid.Height = NORMAL_GAIT_HEIGHT;
            cycloid.FlightPercent = NORMAL_GAIT_FLIGHT_PERCENT;
            cycloid.BodyHeight = NORMAL_GAIT_BODY_HEIGHT;
            period = NORMAL_GAIT_PERIOD;
            if(stand_up_flag)
                for(int i=0;i<4;i++){
                    leg_pos[i][0]=0;
                    leg_pos[i][1]=BODY_HEIGHT;
                }
            RCLCPP_INFO(this->get_logger(),"switch to normal gait");
        }
        if(msg->buttons[STEP_GAIT_BTN]){
            BODY_HEIGHT = STEP_GAIT_BODY_HEIGHT;
            cycloid.Height = STEP_GAIT_HEIGHT;
            cycloid.FlightPercent = STEP_GAIT_FLIGHT_PERCENT;
            cycloid.BodyHeight = STEP_GAIT_BODY_HEIGHT;
            period = STEP_GAIT_PERIOD;
            if(stand_up_flag)
                for(int i=0;i<4;i++){
                    leg_pos[i][0]=0;
                    leg_pos[i][1]=BODY_HEIGHT;
                }
            RCLCPP_INFO(this->get_logger(),"switch to step gait");
        }
        if(msg->buttons[LOWER_GAIT_BTN]){
            BODY_HEIGHT = LOWER_GAIT_BODY_HEIGHT;
            cycloid.Height = LOWER_GAIT_HEIGHT;
            cycloid.FlightPercent = LOWER_GAIT_FLIGHT_PERCENT;
            cycloid.BodyHeight = LOWER_GAIT_BODY_HEIGHT;
            period = LOWER_GAIT_PERIOD;
            if(stand_up_flag)
                for(int i=0;i<4;i++){
                    leg_pos[i][0]=0;
                    leg_pos[i][1]=BODY_HEIGHT;
                }
            RCLCPP_INFO(this->get_logger(),"switch to lower gait");
        }
        if(msg->buttons[JUMP_BTN] && stand_up_flag && !running && !runner_exists){
            // jump();
        }
        if(msg->buttons[CTR_TYPE_BTN]){
            ctrl_by_joy = !ctrl_by_joy;
            if(ctrl_by_joy)
                RCLCPP_INFO(get_logger(),"switch control type: joy");
            else
                RCLCPP_INFO(get_logger(),"switch control type: auto");
        }
        if(ctrl_by_joy){
            step_x = msg->axes[AXES_LX]/2;
            step_y = -msg->axes[AXES_LY];//( msg->axes[TRIGGER_RIGHT] - 1 )/2.0f - ( msg->axes[TRIGGER_LEFT] - 1 )/2.0f;
            bool can_run = (abs(step_x)>0 || abs(step_y)>0);
            // RCLCPP_INFO(this->get_logger(),"step_x:%.3f step_y:%.3f",step_x,step_y);
            if(stand_up_flag && !running && can_run && !runner_exists){
                RCLCPP_INFO(this->get_logger(),"start run");
                std::thread runner(&Foot_Controller::run,this);
                runner.detach();
            }
            else if(running && !can_run){
                RCLCPP_INFO(this->get_logger(),"end run");
                running = false;
            }
        }
    }

    private: void jump(){
        RCLCPP_INFO(this->get_logger(),"start jump");
        // ready for jump
        for(int i=0;i<4;i++){
            params[i].kp_y = 3000;
            leg_pos[i][0] = 0;
            leg_pos[i][1] = 0.15;
        }
        // jump
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        params[0].kp_y = 4000;
        params[0].kd_y = 40;
        leg_pos[0][0] = -0.1;
        leg_pos[0][1] = BODY_HEIGHT;

        params[1].kp_y = 3500;
        params[1].kd_y = 40;
        leg_pos[1][0] = -0.1;
        leg_pos[1][1] = BODY_HEIGHT;

        params[2].kp_y = 3500;
        params[2].kd_y = 40;
        leg_pos[2][0] = -0.1;
        leg_pos[2][1] = BODY_HEIGHT;

        params[3].kp_y = 4000;
        params[3].kd_y = 40;
        leg_pos[3][0] = -0.1;
        leg_pos[3][1] = BODY_HEIGHT;
        // fall down
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        for(int i=0;i<4;i++){
            params[i].kp_y = 800;
            params[i].kd_y = 40;
            params[i].kp_x = 800;
            params[i].kd_x = 40;
            params[i].kp_x = INIT_KP_X;
            leg_pos[i][0] = 0;
        }
        // release
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        for(int i=0;i<4;i++){
            params[i].kp_y = INIT_KP_Y;
            params[i].kd_y = INIT_KD_Y;
            params[i].kp_x = INIT_KP_X;
            params[i].kd_x = INIT_KD_X;
        }
        RCLCPP_INFO(this->get_logger(),"end jump");
    }

    private: void stand_up(){
        stand_up_flag = true;
        // // print leg init pos
        // for(int i=0;i<4;i++){
        //     RCLCPP_INFO(this->get_logger(),"leg[%d] x:%.3f y:%.3f",i,leg_pos[i][0],leg_pos[i][1]);
        // }
        uint microstep = 200;
        for(uint i=0;i<microstep && rclcpp::ok();i++){
            for(int j=0;j<4;j++){
                leg_pos[j][0] = init_pos[j][0] + (0.0 - init_pos[j][0])*(i/(float)microstep);
                leg_pos[j][1] = init_pos[j][1] + (BODY_HEIGHT - init_pos[j][1])*(i/(float)microstep);
                // RCLCPP_INFO(this->get_logger(),"leg[%d] x:%.3f y:%.3f",j,leg_pos[j][0],leg_pos[j][1]);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        RCLCPP_INFO(this->get_logger(),"stand_up");
    }

    private: void sit_down(){
        stand_up_flag = false;
        // // print leg init pos
        // for(int i=0;i<4;i++){
        //     RCLCPP_INFO(this->get_logger(),"leg[%d] x:%.3f y:%.3f",i,leg_pos[i][0],leg_pos[i][1]);
        // }
        uint microstep = 200;
        for(uint i=0;i<microstep && rclcpp::ok();i++){
            for(int j=0;j<4;j++){
                leg_pos[j][0] = init_pos[j][0] + (0.0 - init_pos[j][0])*(1 - i/(float)microstep);
                leg_pos[j][1] = init_pos[j][1] + (BODY_HEIGHT - init_pos[j][1])*(1 - i/(float)microstep);
                // RCLCPP_INFO(this->get_logger(),"leg[%d] x:%.3f y:%.3f",j,leg_pos[j][0],leg_pos[j][1]);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        RCLCPP_INFO(this->get_logger(),"sit_down");
    }

    private: void run(){
        running = true;
        runner_exists = true;
        float step_length = 0.30;
        double intpart;
        float t;
        bool isFlightPercent[4]={0,0,0,0};
        // float flight = 2000
        //     ,noflight = 2000
        //     ,flight_kd = 80
        //     ,noflight_kd = 80;
        CycloidResult res;
        while(running && rclcpp::ok()){
            t = modf(rclcpp::Clock().now().seconds()/period,&intpart);
            cycloid.Length = step_y * step_length + step_x * step_length;
            res = cycloid.generate(0.25+t);
            // if(res.isFlightPercent != isFlightPercent[2]){
            //     isFlightPercent[2] = res.isFlightPercent;
            //     if(isFlightPercent[2]){
            //         params[2].kp_y=flight;
            //         params[2].kd_y=flight_kd;
            //     }
            //     else{
            //         params[2].kp_y=noflight;
            //         params[2].kd_y=noflight_kd;
            //     }
            // }
            leg_pos[2][0] = res.x;
            leg_pos[2][1] = res.y;
            res = cycloid.generate(0.75+t);
            // if(res.isFlightPercent != isFlightPercent[3]){
            //     isFlightPercent[3] = res.isFlightPercent;
            //     if(isFlightPercent[3]){
            //         params[3].kp_y=flight;
            //         params[3].kd_y=flight_kd;
            //     }
            //     else{
            //         params[3].kp_y=noflight;
            //         params[3].kd_y=noflight_kd;
            //     }
            // }
            leg_pos[3][0] = res.x;
            leg_pos[3][1] = res.y;

            cycloid.Length = step_y * step_length - step_x * step_length;
            res = cycloid.generate(0.25+t);
            // if(res.isFlightPercent != isFlightPercent[0]){
            //     isFlightPercent[0] = res.isFlightPercent;
            //     if(isFlightPercent[0]){
            //         params[0].kp_y=flight;
            //         params[0].kd_y=flight_kd;
            //     }
            //     else{
            //         params[0].kp_y=noflight;
            //         params[0].kd_y=noflight_kd;
            //     }
            // }
            leg_pos[0][0] = res.x;
            leg_pos[0][1] = res.y;
            res = cycloid.generate(0.75+t);
            // if(res.isFlightPercent != isFlightPercent[1]){
            //     isFlightPercent[1] = res.isFlightPercent;
            //     if(isFlightPercent[1]){
            //         params[1].kp_y=flight;
            //         params[1].kd_y=flight_kd;
            //     }
            //     else{
            //         params[1].kp_y=noflight;
            //         params[1].kd_y=noflight_kd;
            //     }
            // }
            leg_pos[1][0] = res.x;
            leg_pos[1][1] = res.y;
            
            // for(int i=0;i<4;i++){
            //     RCLCPP_INFO(this->get_logger(),"leg[%d] x:%.3f y:%.3f",i,leg_pos[i][0],leg_pos[i][1]);
            // }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        for(int i=0;i<4;i++){
            leg_pos[i][0]=0;
            leg_pos[i][1]=BODY_HEIGHT;
            // params[i].kp_y = noflight;
            // params[i].kd_y = noflight_kd;
        }
        runner_exists = false;
    }
    private: void control_leg(uint id){
        // // start count
        // auto start = std::chrono::system_clock::now();
        // int count = 0;
        // VMC_Param param;
        // param.kp_x = 4000;
        // param.kd_x = 50;
        // param.kp_y = 1000;
        // param.kd_y = 50;
        // init motor data
        SerialPort* serial;
        switch(id){
            case 0:
                serial = new SerialPort("/dev/ttyS3");
                break;
            case 1:
                serial = new SerialPort("/dev/ttyS7");
                break;
            case 2:
                serial = new SerialPort("/dev/ttyS6");
                break;
            case 3:
                serial = new SerialPort("/dev/ttyS1");
                break;
        }
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
        leg_pos[id][0] = kineRes.pos_x;
        leg_pos[id][1] = kineRes.pos_z;
        init_pos[id][0] = kineRes.pos_x;
        init_pos[id][1] = kineRes.pos_z;
        target_pos_x = kineRes.pos_x;
        target_pos_y = kineRes.pos_z;
        // target_pos_x = 0;
        // target_pos_y = 0.223;
        // // test target pos out
        // std::cout << target_pos_x << "  " << target_pos_y << std::endl;
        // return;

        // auto stretch legs
        // stretch
        cmd_outer.tau = 0.1 * dir_outer;
        cmd_inner.tau = -0.1 * dir_inner;
        SendMsg(&data_outer,&cmd_outer,serial);
        SendMsg(&data_inner,&cmd_inner,serial);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        // release
        cmd_outer.tau = 0;
        cmd_inner.tau = 0;
        SendMsg(&data_outer,&cmd_outer,serial);
        SendMsg(&data_inner,&cmd_inner,serial);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        // save motor offests
        float outer_motor_offest = data_outer.q
            ,inner_motor_offest = data_inner.q;
        inited[id] = true;
        RCLCPP_INFO(this->get_logger(), "inited leg:%d",id);
        // init ki
        params[id].start = rclcpp::Clock().now().seconds();
        while(rclcpp::ok()){
            target_pos_x = leg_pos[id][0];
            target_pos_y = leg_pos[id][1];
            angle_alpha = ((data_outer.q - outer_motor_offest) / gear_ratio)*dir_outer + OUTER_MOTOR_OFFEST;
            angle_beta = ((data_inner.q - inner_motor_offest) / gear_ratio)*dir_inner + INNER_MOTOR_OFFEST;
            if(use_imu_pitch){
                angle_alpha -= imu_pitch;
                angle_beta -= imu_pitch;
            }
            vec_alpha = data_outer.dq / gear_ratio * dir_outer;
            vec_beta = data_inner.dq / gear_ratio * dir_inner;
            kineRes = Kinematic_Solution(angle_alpha,angle_beta,vec_alpha,vec_beta);
            params[id].period = rclcpp::Clock().now().seconds() - params[id].start;
            vmcRes = VMC_Calculate(&params[id],target_pos_x,target_pos_y,kineRes.pos_x,kineRes.pos_z,kineRes.vec_x,kineRes.vec_z);
            jocRes = VMC_Jacobi_Matrix(angle_alpha,angle_beta,vmcRes.force_x,vmcRes.force_z);
            outer_tau = jocRes.tau_alpha / gear_ratio * dir_outer;
            inner_tau = jocRes.tau_beta / gear_ratio * dir_inner;
            // RCLCPP_INFO(this->get_logger(),"force_x:%.3f force_y:%.3f",vmcRes.force_x,vmcRes.force_z);
            // outer_tau = clip(outer_tau,0.1);
            // inner_tau = clip(inner_tau,0.1);
            // printf("force_x:%.3f force_y:%.3f outer_tau:%.3f inner_tau:%.3f\r\n",vmcRes.force_x,vmcRes.force_z,outer_tau,inner_tau);
            cmd_outer.tau = outer_tau;
            cmd_inner.tau = inner_tau;
            SendMsg(&data_outer,&cmd_outer,serial);
            SendMsg(&data_inner,&cmd_inner,serial);
            // wait for other thread to send messages
            // std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
        cmd_outer.mode = 0;
        cmd_inner.mode = 0;
        SendMsg(&data_outer,&cmd_outer,serial);
        SendMsg(&data_inner,&cmd_inner,serial);
    }

};

void SendMsg(MotorData* data,MotorCmd* cmd,SerialPort* serial){
    serial->sendRecv(cmd,data);
}

int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<Foot_Controller>());
	rclcpp::shutdown();
    return 0;
}

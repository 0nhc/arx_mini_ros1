#include "hardware.h"

/////////////////////////Global HW Handle///////////////////////////////

RobotHardware *HW_Handle = NULL;

RobotHardware *Hardware()
{
    if (HW_Handle == NULL)
    {
        HW_Handle = new RobotHardware();
    }

    return HW_Handle;
}

void ReleaseHardware()
{
    if (HW_Handle == NULL)
    {
        return;
    }

    delete HW_Handle;
    HW_Handle = NULL;
}

//////////////////////////HW Update//////////////////////////////

void RobotHardware::update()
{
    CAN_Motor_Update();
}

///////////////////////////HW Implementation/////////////////////////////

void CAN0_ReceiveHandlerProxy(can_frame_t *frame, void *ptr)
{
    ((RobotHardware *)ptr)->CAN0_ReceiveFrame(frame);
}

RobotHardware::RobotHardware()
{
    ros::NodeHandle node_priv;
    node_priv.getParam("imu_frame_id",imu_frame_id);
    imu_pub = node_priv.advertise<sensor_msgs::Imu>("imu/data", 20);  

    //Initialize HW
    can0_adapter.reception_handler_data = (void *)this;
    can0_adapter.reception_handler = &CAN0_ReceiveHandlerProxy;

    can0_adapter.open(HW_CAN0_ID);
}

RobotHardware::~RobotHardware()
{
    can0_adapter.close();
}

void RobotHardware::Motor_UpdateOffset(moto_measure_t *ptr, can_frame_t *frame)
{
    ptr->last_angle = ptr->angle;
    ptr->angle = (uint16_t)(frame->data[0] << 8 | frame->data[1]);
    ptr->real_current = (int16_t)(frame->data[2] << 8 | frame->data[3]);
    ptr->speed_rpm = ptr->real_current;
    ptr->given_current = (int16_t)(frame->data[4] << 8 | frame->data[5]) / -5;
    ptr->hall = frame->data[6];

    if (ptr->angle - ptr->last_angle > 4096)
        ptr->round_cnt--;
    else if (ptr->angle - ptr->last_angle < -4096)
        ptr->round_cnt++;

    ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

void RobotHardware::CAN0_ReceiveFrame(can_frame_t *frame)
{
    switch (frame->can_id)
    {
    case CAN_3510Moto1_ID:
    case CAN_3510Moto2_ID:
    case CAN_3510Moto3_ID:
    case CAN_3510Moto4_ID:
    case CAN_3510Moto5_ID:
    case CAN_3510Moto6_ID:
    case CAN_3510Moto7_ID:
    case CAN_3510Moto8_ID:
    {
        int idx = frame->can_id - CAN_3510Moto1_ID; //first 8 motors

        if (idx < HW_MOTOR_COUNT)
        {
            Motor_UpdateOffset(&motors[idx], frame);
            motors[idx].msg_cnt++;
        }

        break;
    }


         case 0x501:
        {
            p_imu_yaw[0]=frame->data[0];
            p_imu_yaw[1]=frame->data[1];
            p_imu_yaw[2]=frame->data[2];
            p_imu_yaw[3]=frame->data[3];

            p_imu_g_yaw[0]=frame->data[4];
            p_imu_g_yaw[1]=frame->data[5];
            p_imu_g_yaw[2]=frame->data[6];
            p_imu_g_yaw[3]=frame->data[7];
    
            //ROS_WARN("imu_yaw:%f",imu_use_c.imu_yaw);
            //ROS_WARN("imu_g_yaw:%f",imu_use_c.imu_yaw_g);

            float eu_r = 0;
            float eu_p = 0;
            float eu_y = imu_use_c.imu_yaw;

            float cy = cos(eu_y*0.5);
            float sy = sin(eu_y*0.5);
            float cp = cos(eu_p*0.5);
            float sp = sin(eu_p*0.5);
            float cr = cos(eu_r*0.5);
            float sr = sin(eu_r*0.5);

            sensor_msgs::Imu imu_raw;
            imu_raw.header.stamp = ros::Time::now();
            imu_raw.header.frame_id = imu_frame_id;

            imu_raw.orientation.x = sr * cp * cy - cr * sp * sy; 
            imu_raw.orientation.y = cr * sp * cy + sr * cp * sy; 
            imu_raw.orientation.z = cr * cp * sy - sr * sp * cy; 
            imu_raw.orientation.w = cr * cp * cy + sr * sp * sy; 
            //角速度
            imu_raw.angular_velocity.z = imu_use_c.imu_yaw_g;

            imu_pub.publish(imu_raw);
            
        } break; 

    }
}

void RobotHardware::CAN_Motor_Update()
{
    can_frame_t frame;

    //CAN0 Transmit Frame 1
    frame.can_id = HW_CAN_MOTOR_ID_1;
    frame.can_dlc = 8;

    for (int id = 0; id < std::min(4, HW_MOTOR_COUNT); id++)
    {
        int16_t power = (int16_t)motors[id].power;

        frame.data[2 * id] = (uint8_t)(power >> 8);
        frame.data[2 * id + 1] = (uint8_t)(power);
    }

    if (can0_adapter.is_open()) can0_adapter.transmit(&frame);

    //CAN0 Transmit Frame 2
    if (HW_MOTOR_COUNT > 4)
    {
        frame.can_id = HW_CAN_MOTOR_ID_2;
        frame.can_dlc = 8;

        for (int id = 4; id < std::min(8, HW_MOTOR_COUNT); id++)
        {
            int16_t power = (int16_t)motors[id].power;

            frame.data[2 * (id - 4)] = (uint8_t)(power >> 8);
            frame.data[2 * (id - 4) + 1] = (uint8_t)(power);
        }

        if (can0_adapter.is_open()) can0_adapter.transmit(&frame);
    }
}
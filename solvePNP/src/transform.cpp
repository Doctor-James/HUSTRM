/**
 * @file transform.cpp
 * @brief solve transform of robot and world
 *
 *        camera --------------
 *          |                  \
 *          |                   \
 *        gimbal -- shooter    armor
 *          |
 *        world
 *
 * @note
 *         ^ z
 *         |
 *         |
 * <-------/
 * x      /
 *       / y
 *  y -> positive forward,unit m
 *  pangolin： x:red,y:green,z:blue
 * @author lyc
 */

#include "eigen3/Eigen/Dense"
#include <opencv2/core/eigen.hpp>
#include "transform.h"
#include "../../thread/inc/serialPortWriteThread.h"
#include "../../thread/inc/serialPortReadThread.h"
#include <lcm/lcm-cpp.hpp>

//#define debug

namespace ly
{
    transform::transform(serialPortReadThread *serialPortRead,serialPortWriteThread *serialPortWrite):
            serialPortRead_(serialPortRead),serialPortWrite_(serialPortWrite)
    {
        world_ = Sophus::SE3(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0,0,0));
        imu_ = Sophus::SE3(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0,0,0));
        //辅瞄暂时不使用IMU
        // 如果要用陀螺仪 gimbal_ = imu_;
        gimbal_ = imu_;
        //@TODO 改成用配置文件
        //Eigen::Vector3d shooter_trans(0,0.1,0.05);
        Eigen::Vector3d camera_trans(0,0.1085,0.05941);

        //shooter_ = Sophus::SE3(Eigen::Matrix3d::Identity(),shooter_trans);
        gimbal_to_cam_ = Sophus::SE3(Eigen::Matrix3d::Identity(),camera_trans);

        armor_now = world_;


        KF_ = cv::KalmanFilter(6,6,0,CV_32F);
        measurement_ = cv::Mat::zeros(6, 1, CV_32F);
        KF_.transitionMatrix = (cv::Mat_<float>(6, 6) <<
                1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 1.0,
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.0,0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0        //A 状态转移矩阵
                        );
        setIdentity(KF_.measurementMatrix);
        setIdentity(KF_.processNoiseCov, cv::Scalar::all(0.001));
        setIdentity(KF_.measurementNoiseCov, cv::Scalar::all(1));
        setIdentity(KF_.errorCovPost, cv::Scalar::all(1));
        randn(KF_.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
        sendData_.pitch = 0;
        sendData_.yaw = 0;
        sendData_.distance = 1;
//#ifdef DEBUG_TRANS //visual debug
//        pthread_t tids;
//        int ret = pthread_create(&tids, NULL, visual, (void*)this);
//#endif
        DrawCurve_.ClearSaveData();
    }
    void transform::setimu(float pitch, float yaw, float roll)
    {
        Eigen::Matrix3d rotation_matrix3;
        rotation_matrix3 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX());
        gimbal_ = Sophus::SE3(rotation_matrix3,Eigen::Vector3d(0,0,0));
    }
    void transform::setArmor2Cam(Sophus::SE3 armor2cam,receiveData receiveData_)
    {
        carPose_now.pitch = receiveData_.pitch/100.0f*3.14159f/180.0f;
        carPose_now.yaw = receiveData_.yaw/100.0f*3.14159f/180.0f;
        carPose_now.BeginToNowTime = receiveData_.Data_Time;


        setimu(carPose_now.pitch,carPose_now.yaw,0);
        camera_ = gimbal_*gimbal_to_cam_;
        armor_now = camera_*armor2cam;
#ifndef debug
        cv::Point3f armorPosition = cv::Point3f(armor_now.translation()[0],armor_now.translation()[1],armor_now.translation()[2]);
        shootAngleTime_now = ballistic_equation(carPose_now.pitch,armorPosition);     //计算得击打所需时间

        //std::cout<<"shootAngleTime_now.time: "<<shootAngleTime_now.time<<std::endl;
        set_KF(armorPosition);
        float shootTime = shootAngleTime_now.time/1000; //需要测试各种延时(s)

        cv::Point3f Position_KF = cv::Point3f(KF_.statePost.at<float>(0),KF_.statePost.at<float>(1),KF_.statePost.at<float>(2));
        cv::Point3f v_KF = cv::Point3f(KF_.statePost.at<float>(3),KF_.statePost.at<float>(4),KF_.statePost.at<float>(5));
        cv::Point3f Position = Position_KF + v_KF*shootTime;


        shootAngleTime_pre = ballistic_equation(carPose_now.pitch,Position);

        sendData_.pitch = shootAngleTime_pre.pitch*100;
        sendData_.yaw = shootAngleTime_pre.yaw*100;
        sendData_.distance = shootAngleTime_pre.distance*10;

        //DrawCurve_.InsertData(armorPosition.x,Position.x,"real","predict");

#endif
#ifdef debug
        cv::Point3f armorPosition = cv::Point3f(armor_now.translation()[0],armor_now.translation()[1],armor_now.translation()[2]);

        armorPosition.x = sin(i_debug);
        i_debug+=0.1;
        shootAngleTime_now.time = 15;
        //std::cout<<"shootAngleTime_now.time: "<<shootAngleTime_now.time<<std::endl;
        set_KF(armorPosition);
        float shootTime = shootAngleTime_now.time/1000; //需要测试各种延时(s)

        cv::Point3f Position_KF = cv::Point3f(KF_.statePost.at<float>(0),KF_.statePost.at<float>(1),KF_.statePost.at<float>(2));
        cv::Point3f v_KF = cv::Point3f(KF_.statePost.at<float>(3),KF_.statePost.at<float>(4),KF_.statePost.at<float>(5));
        cv::Point3f Position = Position_KF + v_KF*shootTime;

//        std::cout<<"v_pre: "<<v_pre.x<<std::endl;
        shootAngleTime_pre = ballistic_equation(carPose_now.pitch,Position);


        //float pitch_temp = atan2(armor_now.translation()[2], sqrt(armor_now.translation()[0] * armor_now.translation()[0] + armor_now.translation()[1] * armor_now.translation()[1]))*180.0f/3.14159f;
        //float yaw_temp  = -atan2(armor_now.translation()[0], armor_now.translation()[1])*180.0f/3.14159f;

        sendData_.pitch = shootAngleTime_pre.pitch*100;
        sendData_.yaw = shootAngleTime_pre.yaw*100;
        sendData_.distance = shootAngleTime_pre.distance*10;
        //DrawCurve_.InsertData(KF_.statePost.at<float>(3));
        DrawCurve_.InsertData(armorPosition.x,Position.x,"real","predict");
        std::cout<<"armorPosition.x: "<<armorPosition.x<<"Position.x: "<<Position.x<<std::endl;

        cv::waitKey(10);

#endif
//        std::cout<<" shootAngleTime_now.yaw:"<<shootAngleTime_now.yaw<<" shootAngleTime_now.pitch:"<<shootAngleTime_now.pitch<<" shootAngleTime_now.time:"<<shootAngleTime_now.time<<" shootAngleTime_now.distance:"<<shootAngleTime_now.distance<<std::endl;
            if(sendData_.yaw>-100000&&sendData_.yaw<100000&&sendData_.pitch<100000&&sendData_.pitch>-100000)
            {
//	std::cout<<"send.y:"<<armor_now.translation()[1]<<"send.pitch:"<<sendData_.pitch<<"send.distance:"<<sendData_.distance <<"send.z:"<<armor_now.translation()[2]<<"send.x:"<<armor_now.translation()[0]<<std::endl;


                //if(coutCnt%2 == 0)
                {

//		printf("STM Pitch	%f3	STM Yaw	%f3\n",ppp*180.0f/3.14159f,yyy*180.0f/3.14159f);
//		printf("send Pitch	%f3	sendYaw	%f3\n",pitch,yaw);
//		printf("predictX	%f3	predictY	%f3\n",predictX,predictY);
//		printf("distance	%f3	z	%f3\n",distance,z);

                }
                serialPortWrite_->setSendMsg(sendData_);
            }

    }

    cv::Point3f transform::set_KF(cv::Point3f Position_now)
    {
        if(!first_find_temp)  //@TODO 判断首次发现装甲
        {
            //转移矩阵复位
            KF_.transitionMatrix = (cv::Mat_<float>(6, 6) <<
                    1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 1.0,
                    0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 1.0,0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 1.0        //A 状态转移矩阵
            );
            first_find_temp = true;
            is_nfirst_KF = false;
            carPose_old = carPose_now;
            Position_old = Position_now;
            return (Position_now);
        }
        else
        {
            if(!is_nfirst_KF)  //第二次发现装甲
            {
                First_Filter(Position_now);
            }
            else
            {
                Continuous_Filter(Position_now); //连续滤波
            }
        }

    }


    void transform::First_Filter(cv::Point3f Position_now)
    {
        if(carPose_old.BeginToNowTime == 0)
        {
            return;
        }

        float delta_t = carPose_now.BeginToNowTime-carPose_old.BeginToNowTime; //ms
        if(delta_t == 0){
            delta_t = 15;         //未收到数
        }
        is_nfirst_KF = true;

        //速度测量值(m/s)
        float v_x = (Position_now.x - Position_old.x)/(delta_t/1000);    //(m/s)
        float v_y = (Position_now.y - Position_old.y)/(delta_t/1000);
        float v_z = (Position_now.z - Position_old.z)/(delta_t/1000);

//        KF_.statePost.at<float>(0) = Position_now.x;
//        KF_.statePost.at<float>(1) = Position_now.y;
//        KF_.statePost.at<float>(2) = Position_now.z;
//        KF_.statePost.at<float>(3) = v_x;
//        KF_.statePost.at<float>(4) = v_y;
//        KF_.statePost.at<float>(5) = v_z;

        measurement_ = (cv::Mat_<float>(6, 1)<<
        Position_now.x,Position_now.y,Position_now.z,v_x,v_y,v_z);
        KF_.correct(measurement_);

        //存放旧值
        carPose_old = carPose_now;
        Position_old = Position_now;
        v_old.x = v_x;
        v_old.y = v_y;
        v_old.z = v_z;
    }

    void transform::Continuous_Filter(cv::Point3f Position_now)
    {
        if(carPose_old.BeginToNowTime == 0)
        {
            return;
        }
        float delta_t = carPose_now.BeginToNowTime-carPose_old.BeginToNowTime; //ms
        //float delta_t = 8;
        if(abs(delta_t) < 1e-5){
            delta_t = 20;         //未收到数
        }

        //std::cout<<"delta_t: "<<delta_t<<std::endl;
        KF_.transitionMatrix = (cv::Mat_<float>(6, 6) <<
                1.0, 0.0, 0.0, delta_t/1000, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, delta_t/1000, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, delta_t/1000,
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.0,0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0        //A 状态转移矩阵
        );

        KF_.predict();

        Position_pre.x = KF_.statePre.at<float>(0);
        Position_pre.y = KF_.statePre.at<float>(1);
        Position_pre.z = KF_.statePre.at<float>(2);
        v_pre.x = KF_.statePre.at<float>(3);
        v_pre.y = KF_.statePre.at<float>(4);
        v_pre.z = KF_.statePre.at<float>(5);

        //速度测量值(m/s)
        float v_x = (Position_now.x - Position_old.x)/(delta_t/1000);
        float v_y = (Position_now.y - Position_old.y)/(delta_t/1000);
        float v_z = (Position_now.z - Position_old.z)/(delta_t/1000);


        //DrawCurve_.InsertData(KF_.statePost.at<float>(3));

        measurement_ = (cv::Mat_<float>(6, 1)<<
                                             Position_now.x,Position_now.y,Position_now.z,v_x,v_y,v_z);
        KF_.correct(measurement_);
        //std::cout<<"v_x: "<<v_x<<" position："<<KF_.statePost.at<float>(0)<<std::endl;

        //存放旧值
        carPose_old = carPose_now;
        Position_old = Position_now;
        v_old.x = v_x;
        v_old.y = v_y;
        v_old.z = v_z;
    }


    // 根据物理方程来计算设定pitch和yaw
    Angle_t transform::ballistic_equation(float gim_pitch,cv::Point3f armor_Position)
    {
        Angle_t shootAngleTime_;
        // 先计算yaw轴角度
        shootAngleTime_.yaw = atan(armor_Position.x /armor_Position.y);
        shootAngleTime_.distance =sqrt(armor_Position.x * armor_Position.x + armor_Position.y * armor_Position.y);
        // armor 的位置进行了一定的旋转
        armor_Position = cv::Point3f(0,sqrt(armor_Position.x * armor_Position.x + armor_Position.y * armor_Position.y), armor_Position.z);
        // 旋转到世界坐标系
        armor_Position = cv::Point3f(0,armor_Position.y*cos(gim_pitch)-armor_Position.z*sin(gim_pitch),armor_Position.y*sin(gim_pitch)+armor_Position.z*cos(gim_pitch));
        //std::cout<<"armor[2] "<<armor_Position.z<<"armor[1]"<<armor_Position.y<<std::endl;
        //std::cout<<"receive_pitch"<<gim_pitch/3.1415926*180<<std::endl;
        // 计算pitch轴的初始角度
        shootAngleTime_.pitch = atan2(armor_Position.z, armor_Position.y);
        //std::cout<<"armor[2] "<<armor_Position.z<<"armor[1]"<<armor_Position.y<<std::endl;
        //std::cout<<"pitch"<<pitch/3.1415926*180<<std::endl;
        float err = 100;
        // 迭代计算pitch轴角度
        for(int i = 0;i < 5;i ++)
        {// 多次迭代计算得到目标位置
            // 前向弹道方程
            auto target = forward_ballistic_equation(shootAngleTime_.pitch,armor_Position.y);
            err = armor_Position.z - target;
            if(fabs(err) < 0.001)
            {
                break;
            }
            // 计算导数
            float J = derivation( shootAngleTime_.pitch,armor_Position.y);
            float d_theta = - err / J;
            // 更新设定角度
            shootAngleTime_.pitch += d_theta;
        }
        shootAngleTime_.time = shootAngleTime_.distance/(carPose_now.ShootSpeed*cos(shootAngleTime_.pitch))*1000;
        shootAngleTime_.pitch = shootAngleTime_.pitch/3.1415926*180.0;
        shootAngleTime_.yaw = shootAngleTime_.yaw/3.1415926*180.0f;


        return shootAngleTime_;
    }

// 高中物理公式
    float transform::forward_ballistic_equation(float angle,float x)
    {
        float time = x / (carPose_now.ShootSpeed * cos(angle));
        return carPose_now.ShootSpeed * sin(angle) * time - 0.5 * 9.8 * time * time;
    }


    float transform::derivation(float angle,float x)
    {
        return -x / pow(cos(angle),2) +1.5*x*x/(carPose_now.ShootSpeed*carPose_now.ShootSpeed)*1/pow(cos(angle),3);
    }

#ifdef DEBUG_TRANS
    void* transform::visual(void *__this)
    {

        transform * _this =(transform *)__this;
        pangolin::CreateWindowAndBind("Main");
        glEnable(GL_DEPTH_TEST);

        // Define Projection and initial ModelView matrix
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
                pangolin::ModelViewLookAt(-2,-2,-2, 0,0,0, pangolin::AxisZ)
        );
        // Create Interactive View in window
        pangolin::Handler3D handler(s_cam);
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
                .SetHandler(&handler);
        while(true)
        {
            try
            {
                // Clear screen and activate view to render into
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                d_cam.Activate(s_cam);
                //draw
                _this->glDrawColouredAxis(_this->world_,0.08);
                _this->glDrawColouredAxis(_this->camera_,0.08);
                _this->glDrawColouredAxis(_this->gimbal_,0.5);
                //_this->glDrawColouredAxis(_this->imu_,0.5);
                //_this->glDrawColouredAxis(_this->shooter_,0.08);
                _this->glDrawColouredCuboid(_this->armor_now,0.08);
                // Swap frames and Process Events
                pangolin::FinishFrame();
                usleep(20000);//50hz
            }
            catch (...)
            {
                break;
            }
        }
    }
    void transform::glDrawColouredAxis(const Sophus::SE3 T, float length = 0.8)
    {
        std::vector<Eigen::Vector3d> axis;
        axis.emplace_back(Eigen::Vector3d(0.0f,0.0f,0.0f));
        axis.emplace_back(Eigen::Vector3d(length,0.0f,0.0f));
        axis.emplace_back(Eigen::Vector3d(0.0,length,0.0f));
        axis.emplace_back(Eigen::Vector3d(0.0,0.0f,length));
        std::vector<Eigen::Vector3d> trans_axis(axis);
        for(int i = 0;i<4;i++)
        {
            trans_axis.at(i) = T*axis.at(i);
        }
        glLineWidth(3);
        glColor3f ( 0.8f,0.f,0.f );
        glBegin ( GL_LINES );
        glVertex3d(trans_axis.at(0)[0],trans_axis.at(0)[1],trans_axis.at(0)[2]);
        glVertex3d(trans_axis.at(1)[0],trans_axis.at(1)[1],trans_axis.at(1)[2]);
        glColor3f( 0.f,0.8f,0.f);

        glVertex3d(trans_axis.at(0)[0],trans_axis.at(0)[1],trans_axis.at(0)[2]);
        glVertex3d(trans_axis.at(2)[0],trans_axis.at(2)[1],trans_axis.at(2)[2]);
        glColor3f( 0.2f,0.2f,1.f);

        glVertex3d(trans_axis.at(0)[0],trans_axis.at(0)[1],trans_axis.at(0)[2]);
        glVertex3d(trans_axis.at(3)[0],trans_axis.at(3)[1],trans_axis.at(3)[2]);
        glEnd();
    }
    void transform::glDrawColouredCuboid(const Sophus::SE3 T, GLfloat a,GLfloat b,GLfloat c)
    {
        std::vector<Eigen::Vector3d> points={Eigen::Vector3d(a,b,-c),Eigen::Vector3d(-a,b,-c),
                                             Eigen::Vector3d(a,b,c),Eigen::Vector3d(-a,b,c),
                                             Eigen::Vector3d(a,-b,c),Eigen::Vector3d(-a,-b,c),
                                             Eigen::Vector3d(a,-b,-c),Eigen::Vector3d(-a,-b,-c)};
        std::vector<Eigen::Vector3d> trans_points(points);
        for(int i=0;i<points.size();i++)
        {
            trans_points.at(i) = T*points.at(i);
        }
        GLfloat  verts[] =
                {
                        (float)trans_points.at(0)[0],(float)trans_points.at(0)[1],(float)trans_points.at(0)[2],
                        (float)trans_points.at(1)[0],(float)trans_points.at(1)[1],(float)trans_points.at(1)[2],
                        (float)trans_points.at(2)[0],(float)trans_points.at(2)[1],(float)trans_points.at(2)[2],
                        (float)trans_points.at(3)[0],(float)trans_points.at(3)[1],(float)trans_points.at(3)[2],
                        // FRONT
                        (float)trans_points.at(6)[0],(float)trans_points.at(6)[1],(float)trans_points.at(6)[2],
                        (float)trans_points.at(7)[0],(float)trans_points.at(7)[1],(float)trans_points.at(7)[2],
                        (float)trans_points.at(4)[0],(float)trans_points.at(4)[1],(float)trans_points.at(4)[2],
                        (float)trans_points.at(5)[0],(float)trans_points.at(5)[1],(float)trans_points.at(5)[2],
                        // BACK
                        (float)trans_points.at(0)[0],(float)trans_points.at(0)[1],(float)trans_points.at(0)[2],
                        (float)trans_points.at(2)[0],(float)trans_points.at(2)[1],(float)trans_points.at(2)[2],
                        (float)trans_points.at(6)[0],(float)trans_points.at(6)[1],(float)trans_points.at(6)[2],
                        (float)trans_points.at(4)[0],(float)trans_points.at(4)[1],(float)trans_points.at(4)[2],
                        // LEFT
                        (float)trans_points.at(7)[0],(float)trans_points.at(7)[1],(float)trans_points.at(7)[2],
                        (float)trans_points.at(5)[0],(float)trans_points.at(5)[1],(float)trans_points.at(5)[2],
                        (float)trans_points.at(1)[0],(float)trans_points.at(1)[1],(float)trans_points.at(1)[2],
                        (float)trans_points.at(3)[0],(float)trans_points.at(3)[1],(float)trans_points.at(3)[2],
                        // RIGHT
                        (float)trans_points.at(2)[0],(float)trans_points.at(2)[1],(float)trans_points.at(2)[2],
                        (float)trans_points.at(3)[0],(float)trans_points.at(3)[1],(float)trans_points.at(3)[2],
                        (float)trans_points.at(4)[0],(float)trans_points.at(4)[1],(float)trans_points.at(4)[2],
                        (float)trans_points.at(5)[0],(float)trans_points.at(5)[1],(float)trans_points.at(5)[2],
                        // TOP
                        (float)trans_points.at(0)[0],(float)trans_points.at(0)[1],(float)trans_points.at(0)[2],
                        (float)trans_points.at(6)[0],(float)trans_points.at(6)[1],(float)trans_points.at(6)[2],
                        (float)trans_points.at(1)[0],(float)trans_points.at(1)[1],(float)trans_points.at(1)[2],
                        (float)trans_points.at(7)[0],(float)trans_points.at(7)[1],(float)trans_points.at(7)[2],
                        // BOTTOM
                };

        glVertexPointer(3, GL_FLOAT, 0, verts);//verts中每次取三个数据（一个点的坐标）
        glEnableClientState(GL_VERTEX_ARRAY);

        glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glDrawArrays(GL_TRIANGLE_STRIP, 4, 4);

        glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
        glDrawArrays(GL_TRIANGLE_STRIP, 8, 4);
        glDrawArrays(GL_TRIANGLE_STRIP, 12, 4);

        glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
        glDrawArrays(GL_TRIANGLE_STRIP, 16, 4);
        glDrawArrays(GL_TRIANGLE_STRIP, 20, 4);

        glDisableClientState(GL_VERTEX_ARRAY);

        // draw the original axis
        std::vector<Eigen::Vector3d> axis;
        axis.push_back(Eigen::Vector3d(0.0f,0.0f,0.0f));
        axis.push_back(Eigen::Vector3d(0.8,0.0f,0.0f));
        axis.push_back(Eigen::Vector3d(0.0,0.8f,0.0f));
        axis.push_back(Eigen::Vector3d(0.0,0.0f,0.8f));
        std::vector<Eigen::Vector3d> trans_axis(axis);
        for(int i = 0;i<4;i++)
        {
            trans_axis.at(i) = T*axis.at(i);
        }
        glLineWidth(3);
        glColor3f ( 0.8f,0.f,0.f );
        glBegin ( GL_LINES );
        glVertex3f(trans_axis.at(0)[0],trans_axis.at(0)[1],trans_axis.at(0)[2]);
        glVertex3f(trans_axis.at(1)[0],trans_axis.at(1)[1],trans_axis.at(1)[2]);
        glColor3f( 0.f,0.8f,0.f);

        glVertex3f(trans_axis.at(0)[0],trans_axis.at(0)[1],trans_axis.at(0)[2]);
        glVertex3f(trans_axis.at(2)[0],trans_axis.at(2)[1],trans_axis.at(2)[2]);
        glColor3f( 0.2f,0.2f,1.f);

        glVertex3f(trans_axis.at(0)[0],trans_axis.at(0)[1],trans_axis.at(0)[2]);
        glVertex3f(trans_axis.at(3)[0],trans_axis.at(3)[1],trans_axis.at(3)[2]);
        glEnd();
    }


#endif
}

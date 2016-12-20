#include "motion_compensation.h"
#define PI 3.141592

// using namespace motion_compensation

MatrixXd motion_compensation::FK_MTM(MatrixXd JointAngles)
{
  MatrixXd DH(7,4);
  MatrixXd T(4,4);
  MatrixXd FK(4,4);
  DH(0,0)=JointAngles(0,0)+PI/2;DH(0,1)=0;DH(0,2)= 0;DH(0,3)= -PI/2;
  DH(1,0)=JointAngles(0,1)-PI/2;DH(1,1)=0; DH(1,2)-0.2794;DH(1,3)= 0;
  DH(2,0)=JointAngles(0,2)+PI/2;DH(2,1)=0; DH(2,2)=-0.3048;DH(2,3)=PI/2;
  DH(3,0)=JointAngles(0,3);DH(3,1)=0.1506; DH(3,2)=0;DH(3,3)=-PI/2;
  DH(4,0)=JointAngles(0,4);DH(4,1)=0; DH(4,2)=0;DH(4,3)=PI/2;
  DH(5,0)=JointAngles(0,5)+PI/2;DH(5,1)=0; DH(5,2)=0;DH(5,3)=PI/2;
  DH(6,0)=JointAngles(0,6);DH(6,1)=0; DH(6,2)=0;DH(6,3)= 0;

  FK<<1,0,0,0,
      0,1,0,0,
      0,0,1,0,
      0,0,0,1;

  for (int i=0; i<7; i++)
  {
    T<<cos(DH(i,0)), -sin(DH(i,0))*cos(DH(i,3)), sin(DH(i,0))*sin(DH(i,3)) , DH(i,2)*cos(DH(i,0)),
       sin(DH(i,0)), cos(DH(i,0))*cos(DH(i,3)) , -cos(DH(i,0))*sin(DH(i,3)), DH(i,2)*sin(DH(i,0)),
                 0  , sin(DH(i,3))               , cos(DH(i,3))               , DH(i,1)             ,
                 0  ,     0                       ,     0                       , 1;

    FK=FK*T;
  }
  return FK;
}

MatrixXd motion_compensation::IK_PSM(MatrixXd pos)
{
  MatrixXd JointAngle(1,3);
  if (pos(0,0)==0)
  {
    JointAngle(0,0)=0;
  }

  else
  {
    JointAngle(0,0)=atan2(pos(0,0),-pos(1,0));
  }
  if (PI-abs(JointAngle(0,0))<0.5)
  {
    JointAngle(0,0)=(PI-abs(JointAngle(0,0)))*JointAngle(0,0)/abs(JointAngle(0,0));
  }
  JointAngle(0,1)=atan2(-(pos(2,0)+0.369),sqrt(pow(pos(0,0),2)+pow(pos(1,0),2)));
  if (JointAngle(0,1)<-PI/2)
  {
    JointAngle(0,1)=PI+JointAngle(0,1);
  }
  else if (JointAngle(0,1)>PI/2)
  {
    JointAngle(0,1)=-PI+JointAngle(0,1);
  }
  JointAngle(0,2)=sqrt(pow(pos(0,0),2)+pow(pos(1,0),2)+pow((pos(2,0)+0.369),2));

  return JointAngle;

}

VectorXf motion_compensation::randv(int x)
{
  //  random function goes from -1 to 1
  VectorXf X=VectorXf::Random(x);
  return X.cwiseAbs();
}

void motion_compensation::moveRobot(MatrixXd theta_psm, MatrixXd theta_mtm, MatrixXd theta_pris, double pos)
{

//  cout<<theta<<endl;
 // while (ros::ok())
 // {
    std_msgs::Float64 psm_msg1,psm_msg2,psm_msg3,psm_msg4,psm_msg5,psm_msg6;
    std_msgs::Float64 mtm_msg1,mtm_msg2,mtm_msg3,mtm_msg4,mtm_msg5,mtm_msg6,mtm_msg7,pris_msg,pos_msg;
    // msg1.data = 0;//theta(0,0);
    // msg2.data = 0;//theta(0,1);
    // msg3.data = 0;//theta(0,2);
    // msg4.data = 0;//theta(0,3);
    // msg5.data = 0;//theta(0,4);
    // msg6.data = 0;//theta(0,5);

    psm_msg1.data = theta_psm(0,0);
    psm_msg2.data = theta_psm(0,1);
    psm_msg3.data = theta_psm(0,2);
    psm_msg4.data = theta_psm(0,3);
    psm_msg5.data = theta_psm(0,4);
    psm_msg6.data = theta_psm(0,5);

    mtm_msg1.data = theta_mtm(0,0);
    mtm_msg2.data = theta_mtm(0,1);
    mtm_msg3.data = theta_mtm(0,2);
    mtm_msg4.data = theta_mtm(0,3);
    mtm_msg5.data = theta_mtm(0,4);
    mtm_msg6.data = theta_mtm(0,5);
    mtm_msg7.data = theta_mtm(0,6);

    pris_msg.data = theta_pris(0,0);
    pos_msg.data = pos;
    // ROS_INFO("%f", msg1.data);
    // ROS_INFO("%f", msg2.data);
    // ROS_INFO("%f", msg3.data);
    // ROS_INFO("%f", msg4.data);
    // ROS_INFO("%f", msg5.data);
    // ROS_INFO("%f", msg6.data);

    psm_pub1.publish(psm_msg1);
    psm_pub2.publish(psm_msg2);
    psm_pub3.publish(psm_msg3);
    psm_pub4.publish(psm_msg4);
    psm_pub5.publish(psm_msg5);
    psm_pub6.publish(psm_msg6);


    mtm_pub1.publish(mtm_msg1);
    mtm_pub2.publish(mtm_msg2);
    mtm_pub3.publish(mtm_msg3);
    mtm_pub4.publish(mtm_msg4);
    mtm_pub5.publish(mtm_msg5);
    mtm_pub6.publish(mtm_msg6);
    mtm_pub7.publish(mtm_msg7);

    pris_pub1.publish(pris_msg);
    chatter_pub.publish(pos_msg);
  //}

}


int main(int argc, char**argv)
{
  MatrixXd angles_psm(1,6);
  MatrixXd angles_mtm(1,7);
  MatrixXd angles_pris(1,1);
  ros::init(argc, argv, "motion_compensation");
  ros::NodeHandle n;
  motion_compensation obj(n);
  VectorXf c = obj.randv(101),f = VectorXf::LinSpaced(101,0.0f,5.0f), phi = PI *(obj.randv(101)-obj.randv(101)),t = VectorXf::LinSpaced(10001,0.0f,100.0f);
  MatrixXd P(2,2),Pred_P(2,2),kalman_A(2,2),I(2,2);
  kalman_A<<1,0.1,
            0,1;
  I=MatrixXd::Identity(2,2);
  VectorXf A(10001);
  VectorXf Meas(10001);
  MatrixXd D(2,1),H(1,2),Mean(2,1),Pred_Mean(2,1),Kg(2,1);
  MatrixXd Tip(10001,1);
  MatrixXd R(1,1);
  MatrixXd temp(1,2);
  MatrixXd temp2(1,1);
  MatrixXd temp3(1,1);
  MatrixXd psm_pos(3,1);
  MatrixXd psm_angle(1,3);

  D(0,0)=0.1;
  D(1,0)=1;

  H(0,0)=1;
  H(0,1)=0;

  int qdt=100;
  R(0,0)=4.219;

  for (int i=0;i<101;i++)
  {
    for (int j=0;j<10001;j++)
    A(j)=A(j)+c(i)*sin(2*PI*f(i)*t(j)+phi(i));
  }

  Meas=A+5*(obj.randv(10001)-obj.randv(10001));

  Mean(0,0)=A(0,0);
  Mean(1,0)=0;

  Tip(0,0)=0;
  P<<1000,0,
     0,100;

     double x_m_old=0;
     double y_m_old=0;
     double z_m_old=0;

  for (int iter=1; iter<10001; iter++)
  {
    Pred_Mean=kalman_A*Mean;
    Pred_P=kalman_A*P*kalman_A.transpose()+D*qdt*D.transpose();
    temp=H*Pred_P;
    temp2=temp*H.transpose()+R;
    Kg=Pred_P*H.transpose()/temp2(0,0);
    temp3(0,0)=Meas(iter);
    Mean=Pred_Mean+Kg*(temp3-H*Pred_Mean);
    P=(I-Kg*H)*Pred_P;
    Tip(iter,0)=Mean(0,0);
  }

  ros::Rate loop_rate(10);

  for (int count=0;count<10001;count++)
  {
    double S=sin(count*2*PI/100);
    double S1=A(count)/20;
    double S2=Tip(count)/20;

    for(int j=0;j<7;j++)
    {
      if (j==0)
      {
        angles_mtm(0,j)=0.5*S;
      }
      else
      {
        angles_mtm(0,j) = 0;
      }

    }

    angles_pris(0,0)=0.12*S1;
    MatrixXd H(4,4);
    H=obj.FK_MTM(angles_mtm);
    double x_m_new=H(0,3);
    double y_m_new=H(1,3);
    double z_m_new=H(2,3);

    double s_x_m=x_m_new-x_m_old;
    double s_y_m=y_m_new-y_m_old;
    double s_z_m=z_m_new-z_m_old;
    // cout<<s_x_m<<"\t"<<s_y_m<<"\t"<<s_z_m<<endl;
    psm_pos(0,0)=0+4*s_x_m;
    // psm_pos(0,0)=0;
    psm_pos(1,0)=0.12-0.12*S2+0*s_x_m;
    // psm_pos(1,0)=0.12-0.12*S2;
    psm_pos(2,0)=-0.269+4*s_y_m;
    // psm_pos(2,0)=-0.269;
    psm_angle=obj.IK_PSM(psm_pos);
    cout<<psm_angle<<endl;
    angles_psm(0,0)=psm_angle(0,0);
    angles_psm(0,5)=psm_angle(0,2);

    for(int j=1;j<5;j++)
    {
      if (j==2||j==3)
      {
        angles_psm(0,j)=-psm_angle(0,1);
      }
      else if (j==4)
      {
        angles_psm(0,j)=psm_angle(0,1);
      }
      else
      {
        angles_psm(0,j)=psm_angle(0,1);
      }

    }
    // for(int j=0;j<6;j++)
    // {
    //   if (j==5)
    //   {
    //     angles_psm(0,j)=0.12-0.12*S2;
    //   }
    //   else
    //   {
    //     angles_psm(0,j) = 0;
    //   }
    //
    // }
    x_m_old=x_m_new;
    y_m_old=y_m_new;
    z_m_old=z_m_new;


    // cout<<H<<endl;
    // for (int i=0; i<10; i++)
    // {
      obj.moveRobot(angles_psm, angles_mtm, angles_pris, 0.12-psm_pos(1,0));
      loop_rate.sleep();
    //}
  }
    // ros::spin();
}

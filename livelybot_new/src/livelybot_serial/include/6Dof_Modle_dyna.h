#ifndef _6DOF_MODLE_H_
#define _6DOF_MODLE_H_

#include "../include/eigen/Eigen/Dense"
#include <math.h>
#include <stdio.h>
using namespace std;
using namespace Eigen;

class UR6DOFARM
{
    /*
    >>>>>>>>Using modify DH
    */
public:

float a[6] = {0}, d[6] = {0}, alpha[6] = {0}, offset[6] = {0};

//Modede DH
void SetDH(float set_a[6], float set_d[6], float set_alpha[6])
{
    for (size_t i = 0; i < 6; i++)
    {
        a[i] = set_a[i];
        d[i] = set_d[i];
        alpha[i] = set_alpha[i];
    }
    

}





/*
InPut: End effect positon and orientation px py pz rotx roty rotz 
OutPut: 6 joints angls (rad) joint[6]
*/
void Ik_6Dof(float px, float py, float pz, float rotx,float roty,float rotz, float joint[6])
{
    //float px, py, pz;
    float ax, ay, az, ox, oy, oz, nx, ny, nz;
    float temp_a, temp_b, temp_c, B1, B2, A, B, C;\
    float joint_234, joint_23;

    nx = cos(roty)*cos(rotz);
    ny = sin(rotx)*sin(roty)*cos(rotz) + cos(rotx)*sin(rotz);
    nz = -cos(rotx)*sin(roty)*cos(rotz) + sin(rotx)*sin(rotz);

    ox = -cos(roty)*sin(rotz);
    oy = -sin(rotx)*sin(roty)*sin(rotz) + cos(rotx)*cos(rotz);
    oz = cos(rotx)*sin(roty)*sin(rotz) + sin(rotx)*cos(rotz);

    ax = sin(roty);
    ay = -sin(rotx)*cos(roty);
    az = cos(rotx)*cos(roty);

    temp_a = py - d[5]*ay;
    temp_b = d[5]*ax - px;
    temp_c = -d[3];

    //jonit-1
    joint[0] = atan2(temp_b, temp_a) - atan2(-sqrt(temp_a*temp_a + temp_b*temp_b - temp_c*temp_c) , temp_c);
    //jonit-5
    joint[4] = -acos(ax*sin(joint[0]) - ay*cos(joint[0]));
    //jonit-6
    joint[5] = atan2( ( -ox*sin(joint[0]) + oy*cos(joint[0]) )/sin(joint[4] ) , -( -nx*sin(joint[0]) + ny*cos(joint[0]) )/sin(joint[4]) );

    joint_234 = atan2( -az/sin(joint[4]) , -( ax*cos(joint[0]) + ay*sin(joint[0]) )/sin(joint[4]) );

    B1 = pz - d[0] + d[4]*cos(joint_234) + d[5]*sin(joint[4])*sin(joint_234);
    B2 = cos(joint[0])*px + sin(joint[0])*py - d[4]*sin(joint_234) + d[5]*sin(joint[4])*cos(joint_234);
    A = 2*B2*a[2]; B = 2*B1*a[2]; C = B1*B1 + B2*B2 + a[2]*a[2] - a[3]*a[3]; 
    //jonit-2
    joint[1] = atan2( C , -sqrt( A*A + B*B - C*C )) - atan2(A,B);
    
    joint_23 = atan2( ( B1-a[2]*sin(joint[1]) )/a[3] , ( B2-a[2]*cos(joint[1]) )/a[3] );
    //jonit-3
    joint[2] = joint_23 - joint[1];
    //jonit-4
    joint[3] = joint_234 - joint_23;

    //cout<<"test"<<(A*A + B*B - C*C )<<endl;


    
}



// calculate rotation matrax 3x3
Matrix3f GetR_i1_i(float ai, float alphai, float di, float qi)
{
    Matrix3f R;
   
            R << cos(qi), -sin(qi), 0,

            sin(qi)*cos(alphai), cos(qi)*cos(alphai), -sin(alphai),

            sin(qi)*sin(alphai), cos(qi)*sin(alphai), cos(alphai);

            //cout<<"R=\n"<<R<<endl;

            return R;    

}
// calculate rotation position 1x3
Vector3f GetP_i1_i(float ai, float alphai, float di, float qi)
{
    Matrix3f R;
    Vector3f P;
   
          //   R << cos(qi), -sin(qi), 0,

          //   sin(qi)*cos(alphai), cos(qi)*cos(alphai), -sin(alphai),

          //   sin(qi)*sin(alphai), cos(qi)*sin(alphai), cos(alphai);

             P << ai, -di*sin(alphai), di*cos(alphai) ;

            //cout<<"p=\n"<<P<<endl;

            return P;

          

}
// calculate jacobian matrix in word frame
MatrixXf Get_Jacob0(float ja[6], float jalpha[6], float jd[6], float q[6])
{
     MatrixXf jacon0(6, 6);
     Matrix3f R, R01, R12, R23, R34, R45, R56;
     Matrix3f R02, R03, R04, R05, R06;
     Matrix3f jacob3x3Left, jacob3x3Right, downJacob3x3Left, downJacob3x3Right;
     Vector3f P01, P12, P23, P34, P45, P56, P06, Fq, F, P0, P1, P2, P3, P4, P5, P6;
     Vector3f z00, z01, z02, z03, z04, z05, z06;
     Vector3f j_b1, j_b2, j_b3, j_b4, j_b5, j_b6;


     R01 = GetR_i1_i(ja[0], jalpha[0], jd[0], q[0]);
     R12 = GetR_i1_i(ja[1], jalpha[1], jd[1], q[1]);
     R23 = GetR_i1_i(ja[2], jalpha[2], jd[2], q[2]);
     R34 = GetR_i1_i(ja[3], jalpha[3], jd[3], q[3]);
     R45 = GetR_i1_i(ja[4], jalpha[4], jd[4], q[4]);
     R56 = GetR_i1_i(ja[5], jalpha[5], jd[5], q[5]);
     R06 = R01 * R12 * R23 * R34 * R45 * R56;

     R02 = R01*R12;
     R03 = R02*R23;
     R04 = R03*R34;
     R05 = R04*R45;
     R06 = R05*R56;

     P01 = GetP_i1_i(ja[0], jalpha[0], jd[0], q[0]);
     P12 = GetP_i1_i(ja[1], jalpha[1], jd[1], q[1]);
     P23 = GetP_i1_i(ja[2], jalpha[2], jd[2], q[2]);
     P34 = GetP_i1_i(ja[3], jalpha[3], jd[3], q[3]);
     P45 = GetP_i1_i(ja[4], jalpha[4], jd[4], q[4]);
     P56 = GetP_i1_i(ja[5], jalpha[5], jd[5], q[5]);

     P06 = P01 + R01*P12 + R02*P23 + R03*P34 + R04*P45 + R05*P56;

     z00 << 0, 0, 1;
     z01 = R01 * z00;
     z02 = R02 * z00;
     z03 = R03 * z00;
     z04 = R04 * z00;
     z05 = R05 * z00;
     z06 = R06 * z00;

     //P0 << 0, 0, 0;
     P1 = P01;
     P2 = P1 + R01 * P12;
     P3 = P2 + R02 * P23;
     P4 = P3 + R03 * P34;
     P5 = P4 + R04 * P45;
     P6 = P06;
  


     j_b1 = z01.cross(P6 - P1);
     j_b2 = z02.cross(P6 - P2);
     j_b3 = z03.cross(P6 - P3);

     j_b4 = z04.cross(P6 - P4);
     j_b5 = z05.cross(P6 - P5);
     j_b6 = z06.cross(P6 - P6);

     jacob3x3Left << j_b1, j_b2, j_b3;
     jacob3x3Right << j_b4, j_b5, j_b6;


     j_b1 = z01;
     j_b2 = z02;
     j_b3 = z03;

     j_b4 = z04;
     j_b5 = z05;
     j_b6 = z06;

     downJacob3x3Left << j_b1, j_b2, j_b3;
     downJacob3x3Right << j_b4, j_b5, j_b6;

     jacon0.block(0, 0, 3, 3) = jacob3x3Left;
     jacon0.block(0, 3, 3, 3) = jacob3x3Right;
     jacon0.block(3, 0, 3, 3) = downJacob3x3Left;
     jacon0.block(3, 3, 3, 3) = downJacob3x3Right;



     return jacon0;
}
void Get_endPos(float ja[6], float jalpha[6], float jd[6], float q[6], float endPos[3])
{
     MatrixXf jacon0(6, 6);
     Matrix3f R, R01, R12, R23, R34, R45, R56;
     Matrix3f R02, R03, R04, R05, R06;
     Matrix3f jacob3x3Left, jacob3x3Right, downJacob3x3Left, downJacob3x3Right;
     Vector3f P01, P12, P23, P34, P45, P56, P06, Fq, F, P0, P1, P2, P3, P4, P5, P6;
     Vector3f z00, z01, z02, z03, z04, z05, z06;
     Vector3f j_b1, j_b2, j_b3, j_b4, j_b5, j_b6;


     R01 = GetR_i1_i(ja[0], jalpha[0], jd[0], q[0]);
     R12 = GetR_i1_i(ja[1], jalpha[1], jd[1], q[1]);
     R23 = GetR_i1_i(ja[2], jalpha[2], jd[2], q[2]);
     R34 = GetR_i1_i(ja[3], jalpha[3], jd[3], q[3]);
     R45 = GetR_i1_i(ja[4], jalpha[4], jd[4], q[4]);
     R56 = GetR_i1_i(ja[5], jalpha[5], jd[5], q[5]);
     R06 = R01 * R12 * R23 * R34 * R45 * R56;

     R02 = R01*R12;
     R03 = R02*R23;
     R04 = R03*R34;
     R05 = R04*R45;
     R06 = R05*R56;

     P01 = GetP_i1_i(ja[0], jalpha[0], jd[0], q[0]);
     P12 = GetP_i1_i(ja[1], jalpha[1], jd[1], q[1]);
     P23 = GetP_i1_i(ja[2], jalpha[2], jd[2], q[2]);
     P34 = GetP_i1_i(ja[3], jalpha[3], jd[3], q[3]);
     P45 = GetP_i1_i(ja[4], jalpha[4], jd[4], q[4]);
     P56 = GetP_i1_i(ja[5], jalpha[5], jd[5], q[5]);

     P06 = P01 + R01*P12 + R02*P23 + R03*P34 + R04*P45 + R05*P56;

     z00 << 0, 0, 1;
     z01 = R01 * z00;
     z02 = R02 * z00;
     z03 = R03 * z00;
     z04 = R04 * z00;
     z05 = R05 * z00;
     z06 = R06 * z00;

     //P0 << 0, 0, 0;
     P1 = P01;
     P2 = P1 + R01 * P12;
     P3 = P2 + R02 * P23;
     P4 = P3 + R03 * P34;
     P5 = P4 + R04 * P45;
     P6 = P06;
     endPos[0] = P6(0);
     endPos[1] = P6(1);
     endPos[2] = P6(2);

     
}


//使用标准的Dh
VectorXf Get_Jdot(float ja[6], float jalpha[6], float jd[6], float q[6], float dq[6], float endAcc[6])
{
    // Matrix3f R;
    // Vector3f P;

    MatrixXf jacon0(6, 6);
     Matrix3f R, R01, R12, R23, R34, R45, R56;
     Matrix3f R02, R03, R04, R05, R06;
     //Matrix3f jacob3x3Left, jacob3x3Right, downJacob3x3Left, downJacob3x3Right;
     Vector3f P01, P12, P23, P34, P45, P56, P06, Fq, F, P0, P1, P2, P3, P4, P5, P6;
     Vector3f z00, z01, z02, z03, z04, z05, z06;
     //Vector3f j_b1, j_b2, j_b3, j_b4, j_b5, j_b6;
     Vector3f omega0, omega1, omega2, omega3, omega4, omega5, omega6;
     Vector3f dc1, dc2, dc3, dc4, dc5,dc6;
     Vector3f rd1, rd2, rd3, rd4, rd5, rd6;
     Vector3f r1, r2, r3, r4, r5, r6;
     Vector3f db1, db2, db3, db4, db5, db6;
     VectorXf v6(6), v5(6), v4(6), v3(6), v2(6), v1(6);
     VectorXf jd1(6), jd2(6), jd3(6), jd4(6), jd5(6), jd6(6), dq_vec(6);
     MatrixXf Rv(6, 6);
     Rv<<MatrixXf::Zero(6,6);


     R01 << cos(q[0]), 0, sin(q[0]),
         sin(q[0]), 0, -cos(q[0]),
         0, 1, 0;

     R12 << cos(q[1]), -sin(q[1]), 0,
         sin(q[1]), cos(q[1]), 0,
         0, 0, 1;

     R23 << cos(q[2]), -sin(q[2]), 0,
         sin(q[2]), cos(q[2]), 0,
         0, 0, 1;

     R34 << cos(q[3]), 0, sin(q[3]),
         sin(q[3]), 0, -cos(q[3]),
         0, 1, 0;

     R45 << cos(q[4]), 0, -sin(q[4]),
         sin(q[4]), 0, cos(q[4]),
         0, -1, 0;

     R56 << cos(q[5]), -sin(q[5]), 0,
         sin(q[5]), cos(q[5]), 0,
         0, 0, 1;

          
     R06 = R01 * R12 * R23 * R34 * R45 * R56;

     R02 = R01*R12;
     R03 = R02*R23;
     R04 = R03*R34;
     R05 = R04*R45;
     R06 = R05*R56;

     P01 << ja[0]*cos(q[0]), ja[0]*sin(q[0]), jd[0];
     P12 << ja[1]*cos(q[1]), ja[1]*sin(q[1]), jd[1];
     P23 << ja[2]*cos(q[2]), ja[2]*sin(q[2]), jd[2];

     P34 << ja[3]*cos(q[3]), ja[3]*sin(q[3]), jd[3];
     P45 << ja[4]*cos(q[4]), ja[4]*sin(q[4]), jd[4];
     P56 << ja[5]*cos(q[5]), ja[5]*sin(q[5]), jd[5];


     P06 = P01 + R01*P12 + R02*P23 + R03*P34 + R04*P45 + R05*P56;

     z00 << 0, 0, 1;
     z01 = z00;
     z02 = R02 * z00;
     z03 = R03 * z00;
     z04 = R04 * z00;
     z05 = R05 * z00;
     z06 = R06 * z00;

  
    

     //P0 << 0, 0, 0;
     P1 = P01;
     P2 = P1 + R01 * P12;
     P3 = P2 + R02 * P23;
     P4 = P3 + R03 * P34;
     P5 = P4 + R04 * P45;
     P6 = P06;

     omega0<<0,0,0;
     omega1 = omega0 + z00*dq[0];
     omega2 = R01.transpose()*omega1 + z00*dq[1];
     omega3 = R12.transpose()*omega2 + z00*dq[2];
     omega4 = R23.transpose()*omega3 + z00*dq[3];
     omega5 = R34.transpose()*omega4 + z00*dq[4];
     omega6 = R45.transpose()*omega5 + z00*dq[5];

     dc1 = omega0.cross(z00);
     dc2 = omega2.cross(z02);
     dc3 = omega3.cross(z03);
     dc4 = omega4.cross(z04);
     dc5 = omega5.cross(z05);
     dc6 = omega6.cross(z06);

     rd6 = omega6.cross(P56);
     rd5 = omega5.cross(P45) + R45*rd6;
     rd4 = omega4.cross(P34) + R34*rd5;
     rd3 = omega3.cross(P23) + R23*rd4;
     rd2 = omega2.cross(P12) + R12*rd3;
     rd1 = omega1.cross(P01) + R01*rd2;

     r6 = P56;
     r5 = P45 + R45*r6;
     r4 = P34 + R34*r5;
     r3 = P23 + R23*r4;
     r2 = P12 + R12*r3;
     r1 = P01 + R01*r2;

     db1 = dc1.cross(r1) + z01.cross(rd1);
     db2 = dc2.cross(r2) + z02.cross(rd2);
     db3 = dc3.cross(r3) + z03.cross(rd3);
     db4 = dc4.cross(r4) + z04.cross(rd4);
     db5 = dc5.cross(r5) + z05.cross(rd5);
     db6 = dc6.cross(r6) + z06.cross(rd6);

     jd6 <<db6, dc6;
     jd5 <<db5, dc5;
     jd4 <<db4, dc4;
     jd3 <<db3, dc3;
     jd2 <<db2, dc2;
     jd1 <<db1, dc1;

  
     v6 = dq[5]*jd6;
     Rv.block(0, 0, 3, 3) = R45; Rv.block(3, 3, 3, 3) = R45;
     v5 = dq[4]*jd5 + Rv*v6;
     Rv.block(0, 0, 3, 3) = R34; Rv.block(3, 3, 3, 3) = R34;
     v4 = dq[3]*jd4 + Rv*v5;
     Rv.block(0, 0, 3, 3) = R23; Rv.block(3, 3, 3, 3) = R23;
     v3 = dq[2]*jd3 + Rv*v4;
     Rv.block(0, 0, 3, 3) = R12; Rv.block(3, 3, 3, 3) = R12;
     v2 = dq[1]*jd2 + Rv*v3;
     Rv.block(0, 0, 3, 3) = R01; Rv.block(3, 3, 3, 3) = R01;
     v1 = dq[0]*jd1 + Rv*v2;


// Rv.block(0, 0, 3, 3) = R45; Rv.block(3, 3, 3, 3) = R45;
// v5 = jd5 + Rv*v6;

   

            return v1;

          

}
//使用标准的DH
VectorXf Get_Tor(float ja[6], float jalpha[6], float jd[6], float q[6], float dq[6], float ddq[6])
{
    float m1, m2, m3, m4, m5, m6;
    Matrix3f R, R01, R12, R23, R34, R45, R56;
    Matrix3f I1, I2, I3, I4, I5, I6;
    Vector3f rci, p1sta, p2sta, p3sta, p4sta, p5sta, p6sta, z;
    Vector3f omega00, omega11, omega22, omega33, omega44, omega55, omega66;
    Vector3f epsilon00, epsilon11, epsilon22, epsilon33, epsilon44, epsilon55, epsilon66;
    Vector3f a00, a11, a22, a33, a44, a55, a66;
    Vector3f ac00, ac11, ac22, ac33, ac44, ac55, ac66;
    Vector3f f00, f11, f22, f33, f44, f55, f66, f77;
    Vector3f n00, n11, n22, n33, n44, n55, n66, n77;
    Vector3f g0, g1, g2, g3, g4, g5, g6;
    Vector3f cros1, cros2;
    VectorXf tao(6);
    // Vector3f epsilon00, epsilon11, epsilon22, epsilon33, epsilon44, epsilon55, epsilon66;

    R01 << cos(q[0]), 0, sin(q[0]),
        sin(q[0]), 0, -cos(q[0]),
        0, 1, 0;

    R12 << cos(q[1]), -sin(q[1]), 0,
        sin(q[1]), cos(q[1]), 0,
        0, 0, 1;

    R23 << cos(q[2]), -sin(q[2]), 0,
        sin(q[2]), cos(q[2]), 0,
        0, 0, 1;

    R34 << cos(q[3]), 0, sin(q[3]),
        sin(q[3]), 0, -cos(q[3]),
        0, 1, 0;

    R45 << cos(q[4]), 0, -sin(q[4]),
        sin(q[4]), 0, cos(q[4]),
        0, -1, 0;

    R56 << cos(q[5]), -sin(q[5]), 0,
        sin(q[5]), cos(q[5]), 0,
        0, 0, 1;

    I1 << 0.001, 0, 0,
        0, 0.001, 0,
        0, 0, 0.001;

    I2 << 0.00041, 0, 0,
        0, 0.0391, 0,
        0, 0, 0.0391;

    I3 << 0.00041, 0, 0,
        0, 0.0221, 0,
        0, 0, 0.0221;

    I4 << 0.001, 0, 0,
        0, 0.001, 0,
        0, 0, 0.001;
    I5 << 0.001, 0, 0,
        0, 0.001, 0,
        0, 0, 0.001;

    I6 << 0.0017, 0, 0,
        0, 0.0046, 0,
        0, 0, 0.0031;
/*连杆的质量，默认质心在连杆末端*/
    // m1=0; m2=0.6; m3=0.3; m4=0.2; m5=0.2; m6=0.2;
    // rci<<0,0,0;//质心坐标在当前坐标系表示，也就是在杆末端

    m1=0; m2=0.4; m3=0.32; m4=0.29; m5=0.1; m6=0.25;
    rci<<0,0,0;//质心坐标在当前坐标系表示，也就是在杆末端

    //  a1 = 0; a2 = 0.210; a3 = 0.180;
    //  m1 = 0; m2 = 0.4;  m3 = 1.1;

/*两个坐标系原点产生的向量*/
        p1sta<<0, jd[0], 0;
        p2sta<<ja[1], 0, 0;
        p3sta<<ja[2], 0, 0;
        p4sta<<0, jd[3], 0;
        p5sta<<0, -jd[4], 0;
        p6sta<<0, 0, jd[5];


        omega00<<0,0,0;
        epsilon00<<0,0,0;
        a00<<0,0,0;
        z<<0,0,1;

omega11 = R01.transpose()*(omega00 + z*dq[0]);
omega22 = R12.transpose()*(omega11 + z*dq[1]);
omega33 = R23.transpose()*(omega22 + z*dq[2]);
omega44 = R34.transpose()*(omega33 + z*dq[3]);
omega55 = R45.transpose()*(omega44 + z*dq[4]);
omega66 = R56.transpose()*(omega55 + z*dq[5]);



//正加速度计算

epsilon11=R01.transpose()*( epsilon00 + omega00.cross(z*dq[0]) + z*ddq[0] );
epsilon22=R12.transpose()*( epsilon11 + omega11.cross(z*dq[1]) + z*ddq[1] );
epsilon33=R23.transpose()*( epsilon22 + omega22.cross(z*dq[2]) + z*ddq[2] );
epsilon44=R34.transpose()*( epsilon33 + omega33.cross(z*dq[3]) + z*ddq[3] );
epsilon55=R45.transpose()*( epsilon44 + omega44.cross(z*dq[4]) + z*ddq[4] );
epsilon66=R56.transpose()*( epsilon55 + omega55.cross(z*dq[5]) + z*ddq[5] );



cros1 = omega11.cross(p1sta);
a11 = R01.transpose()*a00 + epsilon11.cross(p1sta) + omega11.cross(cros1);
cros1 = omega22.cross(p2sta);
a22 = R12.transpose()*a11 + epsilon22.cross(p2sta) + omega22.cross(cros1);
cros1 = omega33.cross(p3sta);
a33 = R23.transpose()*a22 + epsilon33.cross(p3sta) + omega33.cross(cros1);
cros1 = omega44.cross(p4sta);
a44 = R34.transpose()*a33 + epsilon44.cross(p4sta) + omega44.cross(cros1);
cros1 = omega55.cross(p5sta);
a55 = R45.transpose()*a44 + epsilon55.cross(p5sta) + omega55.cross(cros1);
cros1 = omega66.cross(p6sta);
a66 = R56.transpose()*a55 + epsilon66.cross(p6sta) + omega66.cross(cros1);



cros1 = omega11.cross(rci);
ac11 = a11+ epsilon11.cross(rci) + omega11.cross(cros1);
cros1 = omega22.cross(rci);
ac22 = a22 + epsilon22.cross(rci) + omega22.cross(cros1);
cros1 = omega33.cross(rci);
ac33 = a33 + epsilon33.cross(rci) + omega33.cross(cros1);
cros1 = omega44.cross(rci);
ac44 = a44 + epsilon44.cross(rci) + omega44.cross(cros1);
cros1 = omega55.cross(rci);
ac55 = a55 + epsilon55.cross(rci) + omega55.cross(cros1);
cros1 = omega66.cross(rci);
ac66 = a66 + epsilon66.cross(rci) + omega66.cross(cros1);



g0 << 0, 0, -9.81;
g1 = R01.transpose()*g0;
R = R01*R12;
g2 = R.transpose()*g0;
R = R01*R12*R23;
g3 = R.transpose()*g0;
R = R01*R12*R23*R34;
g4 = R.transpose()*g0;
R = R01*R12*R23*R34*R45;
g5 = R.transpose()*g0;
R = R01*R12*R23*R34*R45*R56;
g6 = R.transpose()*g0;
// g1 = R

f77<< 0.0,0.0,0.0; n77<<0.0,0.0,0.0;

f66 = m6*(ac66-g6) +   f77;
f55 = m5*(ac55-g5) + R56*f66;
f44 = m4*(ac44-g4) + R45*f55;
f33 = m3*(ac33-g3) + R34*f44;
f22 = m2*(ac22-g2) + R23*f33;
f11 = m1*(ac11-g1) + R12*f22;

cros1 = I6*omega66; cros2 = m6*(ac66-g6);
 n66 = I6*epsilon66 + omega66.cross(cros1) +  n77 + (p6sta.cross(f66)) + rci.cross(cros2);
  cros1 = I5*omega55; cros2 = m5*(ac55-g5);
  n55 = I5*epsilon55 + omega55.cross(cros1) + R56*n66 + (p5sta.cross(f55)) + rci.cross(cros2);

  cros1 = I4*omega44; cros2 = m4*(ac44-g4);
  n44 = I4*epsilon44 + omega44.cross(cros1) + R45*n55 + (p4sta.cross(f44)) + rci.cross(cros2);

  cros1 = I3*omega33; cros2 = m3*(ac33-g3);
  n33 = I3*epsilon33 + omega33.cross(cros1) + R34*n44 + (p3sta.cross(f33)) + rci.cross(cros2);

  cros1 = I2*omega22; cros2 = m2*(ac22-g2);
  n22 = I2*epsilon22 + omega22.cross(cros1) + R23*n33 + (p2sta.cross(f22)) + rci.cross(cros2);

  cros1 = I1*omega11; cros2 = m1*(ac11-g1);
  n11 = I1*epsilon11 + omega11.cross(cros1) + R12*n22 + (p1sta.cross(f11)) + rci.cross(cros2);

tao(5)=(R56.transpose()*z).transpose()*n66;
tao(4)=(R45.transpose()*z).transpose()*n55;
tao(3)=(R34.transpose()*z).transpose()*n44;
tao(2)=(R23.transpose()*z).transpose()*n33;
tao(1)=(R12.transpose()*z).transpose()*n22;
tao(0)=(R01.transpose()*z).transpose()*n11;


//cout<<"tao\n"<<tao<<endl;
//tao<<0,0,0,0,0,0;
return tao;


}



};

#endif

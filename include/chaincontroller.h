#ifndef CHAINCONTROLLER_H
#define CHAINCONTROLLER_H

#include "genericcontroller.h"
#include "Eigen/Dense"
#include "xr1define.h"
#include <vector>

using namespace Eigen;

class ChainController: public GenericController
{
public:
    ChainController(MatrixXd DH_input, u_int8_t id , VectorXd Inertia_Parameters , int num_joint);

    VectorXd getTargetJointCurrents();

    std::vector<double> getTargetJointCurrentsStd();

    //For each joint

    double getTargetJointCurrent(u_int8_t joint_id);


    VectorXd getEFFForce();

    VectorXd getEFFVelocity();

    VectorXd getEFFPosition();

    MatrixXd getEFFPositionMatrix();

    Vector3d Matrix2XYZ(Matrix3d BaseRotation);

    // Update the the base
    void updateBaseTransformation(MatrixXd BaseT);

    // Returns the last calculated Jacobian matrix
    MatrixXd getJacobian(u_int8_t id);

    // Return the last calculated end effector Transformation
    MatrixXd getTransformation(u_int8_t JointID);



    void triggerCalculationPass();

    VectorXd NewtonEuler();


    void setEFFIncrement(const Vector3d& Linear , const Vector3d& Angular);

    void setEFFVelocity(const Vector3d& Linear , const Vector3d& Angular);

    void setEFFCurrent(const Vector3d& Force , const Vector3d& Torque);

    void setEFFIncrement(const VectorXd& twist);

    void setEFFVelocity(const VectorXd& twist);

    void setEFFCurrent(const VectorXd& twist);

    bool DynamicSwitch;

private:

    MatrixXd DH_parameters;

    double shoulder_angle_offset;

    double la1;
    double la2;
    double la3;
    double la4;
    double la5;

    std::vector<MatrixXd> Jacobians;

    std::vector<MatrixXd> Transformations_Collection;
    std::vector<MatrixXd> Trans_Collection;

    VectorXd InertiaParameters;

    VectorXd InertiaParameters4dof;

    VectorXd InertiaParameters7dof;


    // Saves Jacobian matrix as a the member variable
    void Jacobeans(VectorXd Joint_Angles);


    // Saves individual transformation in the transformation collection
    void Transformation(VectorXd Joint_Angles);
    MatrixXd T_DH(double d , double offset , double alpha , double ad , double theta);



    Matrix3d EulerXYZ(double x , double y , double z ) ;

    Matrix3d EulerZYX(double x , double y , double z ) ;

    MatrixXd dothat(Vector3d v);

    MatrixXd pinv(MatrixXd input);

    Matrix3d BaseRotationTransposed;


    VectorXd Dynamic_Compensation;

    VectorXd d ;
    VectorXd ad ;
    VectorXd alpha ;
    VectorXd offset;


    int NUM_PARA;


    u_int8_t Begin_ID;

    double g;
    double PI;

    Vector3d grav;



protected:

    //Intermediate for Newton Euler;
double t2  ;
double t3  ;
double t4  ;
double t5  ;
double t6  ;
double t7  ;
double t8  ;
double t9  ;
double t10 ;
double t11 ;
double t12 ;
double t13 ;
double t14 ;
double t15 ;
double t16 ;
double t17 ;
double t18 ;
double t19 ;
double t20 ;
double t21 ;
double t22 ;
double t133;
double t134;
double t23 ;
double t24 ;
double t25 ;
double t26 ;
double t27 ;
double t28 ;
double t189;
double t190;
double t191;
double t29 ;
double t30 ;
double t31 ;
double t32 ;
double t33 ;
double t132;
double t34 ;
double t35 ;
double t36 ;
double t37 ;
double t38 ;
double t39 ;
double t40 ;
double t194;
double t195;
double t41 ;
double t42 ;
double t43 ;
double t44 ;
double t45 ;
double t46 ;
double t47 ;
double t48 ;
double t49 ;
double t50 ;
double t51 ;
double t52 ;
double t53 ;
double t54 ;
double t55 ;
double t56 ;
double t57 ;
double t180;
double t58 ;
double t59 ;
double t60 ;
double t61 ;
double t62 ;
double t63 ;
double t64 ;
double t65 ;
double t71 ;
double t66 ;
double t67 ;
double t68 ;
double t70 ;
double t69 ;
double t72 ;
double t73 ;
double t74 ;
double t75 ;
double t76 ;
double t77 ;
double t78 ;
double t79 ;
double t80 ;
double t81 ;
double t82 ;
double t83 ;
double t85 ;
double t84 ;
double t86 ;
double t87 ;
double t88 ;
double t89 ;
double t90 ;
double t91 ;
double t92 ;
double t93 ;
double t94 ;
double t95 ;
double t96 ;
double t97 ;
double t98 ;
double t99 ;
double t100;
double t101;
double t102;
double t103;
double t104;
double t105;
double t106;
double t107;
double t108;
double t109;
double t110;
double t111;
double t112;
double t113;
double t114;
double t115;
double t116;
double t117;
double t118;
double t119;
double t120;
double t121;
double t122;
double t123;
double t124;
double t125;
double t126;
double t127;
double t128;
double t129;
double t130;
double t131;
double t135;
double t136;
double t137;
double t138;
double t139;
double t140;
double t141;
double t142;
double t143;
double t144;
double t145;
double t146;
double t147;
double t148;
double t149;
double t150;
double t151;
double t152;
double t153;
double t154;
double t155;
double t156;
double t157;
double t158;
double t159;
double t160;
double t161;
double t162;
double t163;
double t164;
double t165;
double t166;
double t167;
double t168;
double t169;
double t170;
double t171;
double t172;
double t173;
double t174;
double t175;
double t176;
double t177;
double t178;
double t179;
double t181;
double t182;
double t183;
double t184;
double t185;
double t186;
double t187;
double t188;
double t192;
double t193;
double t196;
double t197;
double t198;
double t199;
double t200;
double t201;
double t202;


    double gx ;
    double gy ;
    double gz ;


    MatrixXd Regressor;



    // For Transforms
    double costheta;
    double sintheta;
    double sinalpha;
    double cosalpha;

    // Jacobeans
    Vector3d v;
    Vector3d w_j;
    Vector3d z;

    Vector3d temp_v_1;
    Vector3d temp_v_2;


    //Transformation

    MatrixXd Trans;

    MatrixXd Temp_Trans;




};

#endif // CHAINCONTROLLER_H

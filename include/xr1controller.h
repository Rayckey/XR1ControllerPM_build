#ifndef XR1CONTROLLER_H
#define XR1CONTROLLER_H

#include "Eigen/Dense"
#include <vector>
#include "xr1define.h"
#include "xr1controllerpm.h"
#include <map>

class XR1Controller
{

public:

    XR1Controller();
    u_int8_t isXR1Okay(); //Error Checks


    //-----------------------------------------------------------------


    //--------Joint Control----------------------------------


    //Set the Target Joint Positions for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Angles contained in Eigen::VectorXd
    //Reutrns : void , may add error message in the fulture
    void setJointPosition(u_int8_t control_group , VectorXd JA);


    //Set the Target Joint Positions for a single joint, i.e. LeftShoulderX , RightWristZ
    //Used in the XR1Controller
    //Argu: Control Group ID , Angles contained in std::vector<double>
    //Reutrns : void , may add error message in the fulture
    void setJointPosition(u_int8_t joint_idx ,   double JA);


    //Set the Target Joint Velocity for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Angular Velocity contained in Eigen::VectorXd
    //Reutrns : void , may add error message in the fulture
    void setJointVelocity(u_int8_t control_group , VectorXd JV);


    //Set the Target Joint Velocity for a single joint, i.e. LeftShoulderX , RightWristZ
    //Used in the XR1Controller
    //Argu: Control Group ID , Angular Velocity contained in std::vector<double>
    //Reutrns : void , may add error message in the fulture
    void setJointVelocity(u_int8_t joint_idx ,   double JV);


    //Set the Target Joint Currents for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Target Current
    //Reutrns : void , may add error message in the fulture
    void setJointCurrent(u_int8_t control_group , VectorXd JC);


    //Set the Target Joint Currents for a single joint, i.e. LeftShoulderX , RightWristZ
    //Used in the XR1Controller
    //Argu: Control Group ID , Angular Velocity contained in std::vector<double>
    //Reutrns : void , may add error message in the fulture
    void setJointCurrent(u_int8_t joint_idx ,   double JC);


    //Get the Current Joint Angles for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : JointAngles contained in Eigen::VectorXd
    VectorXd getJointPositions(u_int8_t control_group , bool vanilla = false);


    //Get the Current Joint Angles for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : JointAngles contained in std::vector
    std::vector<double> getJointPositionsStd(u_int8_t control_group);


    //Get the Current Joint Velocites for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : Joint Velocities contained in Eigen::VectorXd
    VectorXd getJointVelocities(u_int8_t control_group , bool vanilla = false);


    //Get the Current Joint Velocites for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : Joint Velocites contained in std::vector
    std::vector<double> getJointVelocitiesStd(u_int8_t control_group);


    //Get the Current Joint Currents for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : Joint Currents contained in Eigen::VectorXd
    VectorXd getJointCurrents(u_int8_t control_group , bool vanilla = false);


    //Get the Current Joint Currents for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : Joint Currents contained in std::vector
    std::vector<double> getJointCurrentsStd(u_int8_t control_group);



    double getTargetJointPosition(u_int8_t joint_id , bool vanilla = false);

    double getTargetJointVelocity(u_int8_t joint_id , bool vanilla = false);

    double getTargetJointCurrent(u_int8_t joint_id , bool vanilla = false);

    double getJointPosition(u_int8_t joint_id , bool vanilla = false);

    double getJointVelocity(u_int8_t joint_id , bool vanilla = false);

    double getJointCurrent(u_int8_t joint_id , bool vanilla = false);

    //Set the Control Method for an entire Control Group , i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Conrol Mode ID
    //Reutrns : void , may add error message in the fulture
    void setControlMode(u_int8_t control_group , u_int8_t option);
    u_int8_t getControlMode(u_int8_t control_group);



    //Get the last calculated Jacobian from XR1Controller
    //Used in XR1Controller
    //Argu: Joint ID (Only From LeftShoulderX to RightWristX)
    //Reutrns : A 6x7 Eigen::MatrixXd of Jacobians or A 6x7 Eigen::MatrixXd of zeros should an error occur
    MatrixXd getJacobian(u_int8_t joint_idx);


    //Get the last calculated Jacobian from XR1Controller for several joints
    //Used in XR1Controller
    //Argu: Vector of Joint ID (Only From LeftShoulderX to RightWristX)
    //Reutrns : A 6x7 Eigen::MatrixXd of Jacobians or A 6x7 Eigen::MatrixXd of zeros should an error occur
    std::vector<MatrixXd> getJacobian(std::vector<u_int8_t> joint_idx_list);


    //-------------------End Effector (Wrist) Control ---------------------------

    //Move the End Effector of LeftArm for a SMALL distance
    //Used in XR1Controller
    //Argu: Linear increment contained in Eigen::Vector3d , Angular increment contained in Eigen::Vector3d
    //Reutrns : void , may add error message in the fulture
    void setLeftArmIncrement(const Vector3d& Linear , const Vector3d& Angular);
    void setLeftArmIncrement(const VectorXd& twist);

    //Move the End Effector of RightArm for a SMALL distance
    //Used in XR1Controller
    //Argu: Linear increment contained in Eigen::Vector3d , Angular increment contained in Eigen::Vector3d
    //Reutrns : void , may add error message in the fulture
    void setRightArmIncrement(const Vector3d& Linear , const Vector3d& Angular);
    void setRightArmIncrement(const VectorXd& twist);

    //Move the End Effector of LeftArm to a target position
    //Used in XR1Controller , But in simulation this moves the Target Dummy to the location
    //Argu: Position  contained in Eigen::Vector3d , Orientation (Euler Angles in the order XYZ) contained in Eigen::Vector3d
    //Reutrns : void , may add error message in the fulture
//    void setLeftArmPosition(const Vector3d& Linear , const Vector3d& Angular);
//    void setLeftArmPosition(const VectorXd& twist);


    //Move the End Effector of RightArm to a target position
    //Used in XR1Controller , But in simulation this moves the Target Dummy to the location
    //Argu: Position  contained in Eigen::Vector3d , Orientation (Euler Angles in the order XYZ) contained in Eigen::Vector3d
    //Reutrns : void , may add error message in the fulture
    //void setRightArmPosition(const Vector3d& Linear , const Vector3d& Angular);
    //void setRightArmPosition(const VectorXd& twist);

    //Set the velocity of the End Effector of LeftArm,
    //Used in XR1Controller , Needs to be refreshed every simulation step
    //Argu: Linear velocity contained in Eigen::Vector3d , Angular velocity (Euler Angles in the order XYZ) contained in Eigen::Vector3d
    //Reutrns : void , may add error message in the fulture
    void setLeftArmVelocity(const Vector3d& Linear , const Vector3d& Angular);
    void setLeftArmVelocity(const VectorXd& twist);

    //Set the velocity of the End Effector of RightArm,
    //Used in XR1Controller , Needs to be refreshed every simulation step when used
    //Argu: Linear velocity contained in Eigen::Vector3d , Angular velocity (Euler Angles in the order XYZ) contained in Eigen::Vector3d
    //Reutrns : void , may add error message in the fulture
    void setRightArmVelocity(const Vector3d& Linear , const Vector3d& Angular);
    void setRightArmVelocity(const VectorXd& twist);

    //Set the velocity of the End Effector of LeftArm , with the addition of dynamic compensation
    //Used in XR1Controller , Needs to be refreshed every simulation step when used
    //Argu: Force contained in Eigen::Vector3d , Torque contained in Eigen::Vector3d
    //Reutrns : void , may add error message in the fulture
    void setLeftArmCurrent(const Vector3d& Force , const Vector3d& Torque);
    void setLeftArmCurrent(const VectorXd& twist);

    //Set the velocity of the End Effector of RightArm , with the addition of dynamic compensation
    //Used in XR1Controller , Needs to be refreshed every simulation step when used
    //Argu: Force contained in Eigen::Vector3d , Torque contained in Eigen::Vector3d
    //Reutrns : void , may add error message in the fulture
    void setRightArmCurrent(const Vector3d& Force , const Vector3d& Torque);
    void setRightArmCurrent(const VectorXd& twist);

    //get the net force on End Effector of the left arm
    //Used in XR1Controller , Needs to be refreshed every simulation step when used
    //Argu: N/A
    //Reutrns : VectorXd of 3 forces and 3 torques , may add error message in the fulture
    VectorXd getLeftArmForce();

    //get the velocity on End Effector of the left arm
    //Used in XR1Controller , Needs to be refreshed every simulation step when used
    //Argu: N/A
    //Reutrns : VectorXd of 3 linear velocity and 3 angular velocity , may add error message in the fulture
    VectorXd getLeftArmVelocity() ;

    //get the position on End Effector of the left arm
    //Used in XR1Controller , Needs to be refreshed every simulation step when used
    //Argu: N/A
    //Reutrns : VectorXd of 3 linear position and 3 angular position , may add error message in the fulture
    VectorXd getLeftArmPosition() ;

    //get the Homogenous transformation End Effector of the left arm
    //Used in XR1Controller , Needs to be refreshed every simulation step when used
    //Argu: N/A
    //Reutrns : A homogenous transformation (4x4)
    MatrixXd getLeftArmPositionMatrix() ;

    //get the net force on End Effector of the right arm
    //Used in XR1Controller , Needs to be refreshed every simulation step when used
    //Argu: N/A
    //Reutrns : VectorXd of 3 forces and 3 torques , may add error message in the fulture
    VectorXd getRightArmForce() ;

    //get the velocity on End Effector of the left arm
    //Used in XR1Controller , Needs to be refreshed every simulation step when used
    //Argu: N/A
    //Reutrns : VectorXd of 3 linear velocity and 3 angular velocity , may add error message in the fulture
    VectorXd getRightArmVelocity() ;

    //get the position on End Effector of the left arm
    //Used in XR1Controller , Needs to be refreshed every simulation step when used
    //Argu: N/A
    //Reutrns : VectorXd of 3 linear position and 3 angular position , may add error message in the fulture
    VectorXd getRightArmPosition() ;

    //get the Homogenous transformation End Effector of the left arm
    //Used in XR1Controller , Needs to be refreshed every simulation step when used
    //Argu: N/A
    //Reutrns : A homogenous transformation (4x4)
    MatrixXd getRightArmPositionMatrix() ;



    //Convert output from XR1IMU to Motor Joint angles
    void setMoCapPosition(std::vector<double> IMU_output);


    //Convert output from Action files to Motor Joint Angles;
    void setActionPosition(std::vector<double> Action_output);


    //Convert output from Data files to Motor Joint Angles;
    void setDataPosition(std::vector<double> Data_output);



    //Trun ON/OFF the dynamic compensation of Left Arm, only affects Current Mode
    //Used in XR1Controller.
    //Argu: option true/false,
    //Reutrns : void , may add error message in the fulture
    void setLeftArmDynamicSwitch(bool option);


    //Trun ON/OFF the dynamic compensation of Right Arm, only affects Current Mode
    //Used in XR1Controller.
    //Argu: option true/false
    //Reutrns : void , may add error message in the fulture
    void setRightArmDynamicSwitch(bool option);


    //Get Target Position for Arms or Body
    VectorXd getTargetPosition(u_int8_t control_group , bool vanilla = false);

    //Get Target velocity for Arms or Body
    VectorXd getTargetVelocity(u_int8_t control_group , bool vanilla = false);

    //Get Target Current for Arms or Body
    VectorXd getTargetCurrent(u_int8_t control_group , bool vanilla = false);


    // Trigger the Calcualtions
    //Subjected to change
    //Argu: N/A
    //Reutrns : void , may add error message in the fulture
    void triggerCalculation();

    void updatingCallback(u_int8_t id , u_int8_t attrId , double value);

    double getActuatorRatio(u_int8_t id);

    bool CollisionDetection(u_int8_t control_group);

    double tinyBezier(double double_index , double pt_s , double pt_1 , double pt_2 , double pt_e);

    void SetOmniWheelsVelocity(Vector3d input, bool skate_mode);

    void setPeriod(u_int8_t control_group , double period);

private:

    XR1ControllerPM * XR1_ptr;

    double Position2Actuator(u_int8_t id , double value);

    double Velocity2Actuator(u_int8_t id , double value);

    double Current2Actuator(u_int8_t id , double value);

    double Actuator2Position(u_int8_t id , double value);

    double Actuator2Velocity(u_int8_t id , double value);

    double Actuator2Current(u_int8_t id , double value);


    void WristReadingHelper(u_int8_t mode);

    void WristCommandHelper(u_int8_t mode ,  VectorXd& input);

    std::map<u_int8_t , double> WristPositions;

    std::map<u_int8_t , double> WristVelocities;

    std::map<u_int8_t , double> WristCurrents;

    std::map<u_int8_t , bool> WristPositionSwitch;

    std::map<u_int8_t , bool> WristVelocitySwitch;

    std::map<u_int8_t , bool> WristCurrentSwitch;

    double PI;

    int num_joint_in_chain;

};

#endif // XR1CONTROLLER_H

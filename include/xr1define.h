#ifndef XR1DEFINE
#define XR1DEFINE

namespace XR1 {

enum BodyGroups{
    MainBody = 4,
    LeftArm = 11,
    RightArm = 18,
    LeftHand = 25,
    RightHand = 30,
    OmniWheels = 1,
};

enum ActuatorID{
    Left_Front_Wheel = 1 ,
    Right_Front_Wheel = 2,
    Back_Wheel = 3,
    Knee_X = 4          ,
    Back_Z = 5          ,
    Back_X = 6          ,
    Back_Y = 7          ,
    Neck_Z = 8          ,
    Neck_X = 9          ,
    Head = 10            ,
    Left_Shoulder_X = 11,
    Left_Shoulder_Y = 12 ,
    Left_Elbow_Z = 13    ,
    Left_Elbow_X = 14    ,
    Left_Wrist_Z = 15    ,
    Left_Wrist_X = 16    ,
    Left_Wrist_Y = 17    ,
    Right_Shoulder_X = 18,
    Right_Shoulder_Y = 19,
    Right_Elbow_Z = 20   ,
    Right_Elbow_X = 21   ,
    Right_Wrist_Z = 22   ,
    Right_Wrist_X = 23   ,
    Right_Wrist_Y = 24   ,
    Left_Thumb = 25      ,
    Left_Index = 26     ,
    Left_Middle = 27     ,
    Left_Ring = 28       ,
    Left_Pinky = 29      ,
    Right_Thumb = 30      ,
    Right_Index = 31      ,
    Right_Middle = 32     ,
    Right_Ring = 33       ,
    Right_Pinky = 34      ,
    Actuator_Total = 35,
};

enum ChainOperationMode{
    PositionMode = 1,
    VelocityMode = 2,
    ForceMode = 3,
    IKMode = 4,
};

enum MetaErrorDefine{
    EVERYTHING_IS_FINE = 0,
    ERR_ID_NUM = 0x806,
    ERR_NOT_LAUNCH = 0x807,
    ERR_CALULATION = 0x808,
};


enum ValuesOptions{
    ActualPosition,
    ActualVelocity,
    ActualCurrent,
    TargetPosition,
    TargetVelocity,
    TargetCurrent,
};

enum AttributeIDs{
    Proportional = 50,
    Integral = 51,
    Derivative = 52,
};

enum PathPlaningMethods{
    RRT = 100,
    EST = 101,
    KPIECE = 102,
    PDST = 103,
};


enum HandGripActions{
    HandGrip = 401,
    HandRelease = 402,
};


}


#endif // XR1DEFINE


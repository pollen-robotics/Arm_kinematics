#include <chain.hpp>
#include "models.hpp"
#include <utilities/utility.h>

namespace KDL{
    Chain Reachy_RightArm(){
        Chain reachy;
        // r_shoulder_pitch modif
        reachy.addSegment(Segment(Joint(Joint::None),
                                   Frame::DH(0.0,PI_2,0.0,PI)));
        // r_shoulder_roll
        reachy.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,-PI_2,-0.19,PI_2)));
        // r_arm_yaw
        reachy.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,-PI_2,0.0,-PI_2)));
        // r_elbow_pitch
        reachy.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,PI_2,-0.28,PI_2)));
        // r_forearm_yaw
        reachy.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,PI_2,0.0,PI)));
        // r_wrist_pitch
        reachy.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,-PI_2,-0.25,0.0)));
        // r_wrist_roll
        reachy.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(-0.0325,-PI_2,-0.012,-PI_2)));
        // r_gripper
        reachy.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(-0.085,0.0,0.0,0.0)));

        return reachy;
    }

    Chain Reachy_LeftArm(){
        Chain reachy;
        // r_shoulder_pitch modif
        reachy.addSegment(Segment(Joint(Joint::None),
                                   Frame::DH(0.0,PI_2,0.0,PI)));
        // r_shoulder_roll
        reachy.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,-PI_2,0.19,PI_2)));
        // r_arm_yaw
        reachy.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,-PI_2,0.0,-PI_2)));
        // r_elbow_pitch
        reachy.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,PI_2,-0.28,PI_2)));
        // r_forearm_yaw
        reachy.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,PI_2,0.0,PI)));
        // r_wrist_pitch
        reachy.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,-PI_2,-0.25,0.0)));
        // r_wrist_roll
        reachy.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(-0.0325,-PI_2,0.012,-PI_2)));
        // r_gripper
        reachy.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(-0.085,0.0,0.0,0.0)));

        return reachy;
    }
}

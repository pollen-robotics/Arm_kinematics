#include <chain.hpp>
#include "models.hpp"
#include <utilities/utility.h>

namespace KDL{
    Chain Reachy(){
        Chain reachy;
        reachy.addSegment(Segment());

        // FIRST TRY
        // reachy.addSegment(Segment(Joint(Joint::RotZ),
        //                            Frame::DH(0.19,-PI_2,0.0,0.0)));
        // reachy.addSegment(Segment(Joint(Joint::RotZ),
        //                            Frame::DH(0.0,-PI_2,0.0,0.0)));
        // reachy.addSegment(Segment(Joint(Joint::RotZ),
        //                            Frame::DH(0.0,-PI_2,0.0,0.0)));
        // reachy.addSegment(Segment(Joint(Joint::RotZ),
        //                            Frame::DH(0.28,PI_2,0.4318,0.0)));
        // reachy.addSegment(Segment(Joint(Joint::RotZ),
        //                            Frame::DH(0.0,-PI_2,0.0,0.0)));
        // reachy.addSegment(Segment(Joint(Joint::RotZ),
        //                            Frame::DH(-0.25,PI_2,0.0,0.0)));
        // reachy.addSegment(Segment(Joint(Joint::RotZ),
        //                            Frame::DH(0.0,-PI_2,-0.0325,0.0)));
        // reachy.addSegment(Segment(Joint(Joint::TransX),
        //                            Frame::DH(0.01,0.0,-0.075,0.0)));

        // SECOND TRY
        // reachy.addSegment(Segment(Joint(Joint::RotY),
        //                            Frame::DH(0.0,0.0,0.0,0.0)));
        // reachy.addSegment(Segment(Joint(Joint::RotX),
        //                            Frame::DH(0.0,0.0,0.0,0.0)));
        // reachy.addSegment(Segment(Joint(Joint::RotZ),
        //                            Frame::DH(0.0,0.0,-0.28,0.0)));

        // THIRD TRY
        // On ignore dans un premier temps le passage du repère reachy au r_shoulder_pitch
        // Les joints sont exprimés dans le repère du r_shoulder_pitch
        // PB de signe à régler, les z sont problablement orientés dans l'autre sens de temps en temps
        // // r_shoulder_roll
        // reachy.addSegment(Segment(Joint(Joint::RotZ),
        //                            Frame::DH(0.0,PI_2,0.0,PI_2)));
        // // r_arm_yaw
        // reachy.addSegment(Segment(Joint(Joint::RotZ),
        //                            Frame::DH(0.0,PI_2,0.0,-PI_2)));
        // // r_elbow_pitch
        // reachy.addSegment(Segment(Joint(Joint::RotZ),
        //                            Frame::DH(0.0,-PI_2,0.28,PI_2)));
        // // r_forearm_yaw
        // reachy.addSegment(Segment(Joint(Joint::RotZ),
        //                            Frame::DH(0.0,PI_2,0.0,0.0)));
        // // r_wrist_pitch
        // reachy.addSegment(Segment(Joint(Joint::RotZ),
        //                            Frame::DH(0.0,-PI_2,0.25,0.0)));
        // // r_wrist_roll
        // reachy.addSegment(Segment(Joint(Joint::RotZ),
        //                            Frame::DH(-0.0325,PI_2,0.0,PI_2)));
        //  // r_gripper
        // reachy.addSegment(Segment(Joint(Joint::RotZ),
        //                            Frame::DH(-0.085,0,0.0,0)));

        // TEST Reperes bien orientés
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
        reachy.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(-0.085,0.0,0.0,0.0)));

        return reachy;
    }

}

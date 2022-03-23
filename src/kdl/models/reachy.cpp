#include <chain.hpp>
#include "models.hpp"
#include <utilities/utility.h>

namespace KDL{
    Chain Reachy_RightArm(){
        Chain reachy;

        // r_shoulder_pitch
        reachy.addSegment(Segment(Joint(Vector(0.0, -0.19, 0.0), Vector(0.0, 1.0, 0.0), Joint::RotAxis),
                                   Frame(Rotation(1.0, 0.0, 0.0, 0.0, 1.0, 0.0 , 0.0, 0.0, 1.0), 
                                        Vector(0.0, -0.19, 0.0))));
        // r_shoulder_roll
        reachy.addSegment(Segment(Joint(Vector(0.0, 0.0, 0.0), Vector(1.0, 0.0, 0.0), Joint::RotAxis),
                                   Frame(Rotation(1.0, 0.0, 0.0, 0.0, 1.0, 0.0 , 0.0, 0.0, 1.0), 
                                        Vector(0.0, 0.0, 0.0))));
        // r_arm_yaw
        reachy.addSegment(Segment(Joint(Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 1.0), Joint::RotAxis),
                                   Frame(Rotation(1.0, 0.0, 0.0, 0.0, 1.0, 0.0 , 0.0, 0.0, 1.0), 
                                        Vector(0.0, 0.0, 0.0))));
        // r_elbow_pitch
        reachy.addSegment(Segment(Joint(Vector(0.0, 0.0, -0.28), Vector(0.0, 1.0, 0.0), Joint::RotAxis),
                                   Frame(Rotation(1.0, 0.0, 0.0, 0.0, 1.0, 0.0 , 0.0, 0.0, 1.0), 
                                        Vector(0.0, 0.0, -0.28))));
        // r_forearm_yaw
        reachy.addSegment(Segment(Joint(Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 1.0), Joint::RotAxis),
                                   Frame(Rotation(1.0, 0.0, 0.0, 0.0, 1.0, 0.0 , 0.0, 0.0, 1.0), 
                                        Vector(0.0, 0.0, 0.0))));
        // r_wrist_pitch
        reachy.addSegment(Segment(Joint(Vector(0.0, 0.0, -0.25), Vector(0.0, 1.0, 0.0), Joint::RotAxis),
                                   Frame(Rotation(1.0, 0.0, 0.0, 0.0, 1.0, 0.0 , 0.0, 0.0, 1.0), 
                                        Vector(0.0, 0.0, -0.25))));
        // r_wrist_roll
        reachy.addSegment(Segment(Joint(Vector(0.0, 0.0, -0.0325), Vector(1.0, 0.0, 0.0), Joint::RotAxis),
                                   Frame(Rotation(1.0, 0.0, 0.0, 0.0, 1.0, 0.0 , 0.0, 0.0, 1.0), 
                                        Vector(0.0, 0.0, -0.0325))));
        // r_gripper
        reachy.addSegment(Segment(Joint(Joint::None),
                                   Frame(Rotation(1.0, 0.0, 0.0, 0.0, 1.0, 0.0 , 0.0, 0.0, 1.0), 
                                        Vector(0.0, -0.012, -0.085))));

        return reachy;
    }

    Chain Reachy_LeftArm(){
        Chain reachy;
        // l_shoulder_pitch
        reachy.addSegment(Segment(Joint(Vector(0.0, 0.19, 0.0), Vector(0.0, 1.0, 0.0), Joint::RotAxis),
                                   Frame(Rotation(1.0, 0.0, 0.0, 0.0, 1.0, 0.0 , 0.0, 0.0, 1.0), 
                                        Vector(0.0, 0.19, 0.0))));
        // l_shoulder_roll
        reachy.addSegment(Segment(Joint(Vector(0.0, 0.0, 0.0), Vector(1.0, 0.0, 0.0), Joint::RotAxis),
                                   Frame(Rotation(1.0, 0.0, 0.0, 0.0, 1.0, 0.0 , 0.0, 0.0, 1.0), 
                                        Vector(0.0, 0.0, 0.0))));
        // l_arm_yaw
        reachy.addSegment(Segment(Joint(Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 1.0), Joint::RotAxis),
                                   Frame(Rotation(1.0, 0.0, 0.0, 0.0, 1.0, 0.0 , 0.0, 0.0, 1.0), 
                                        Vector(0.0, 0.0, 0.0))));
        // l_elbow_pitch
        reachy.addSegment(Segment(Joint(Vector(0.0, 0.0, -0.28), Vector(0.0, 1.0, 0.0), Joint::RotAxis),
                                   Frame(Rotation(1.0, 0.0, 0.0, 0.0, 1.0, 0.0 , 0.0, 0.0, 1.0), 
                                        Vector(0.0, 0.0, -0.28))));
        // l_forearm_yaw
        reachy.addSegment(Segment(Joint(Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 1.0), Joint::RotAxis),
                                   Frame(Rotation(1.0, 0.0, 0.0, 0.0, 1.0, 0.0 , 0.0, 0.0, 1.0), 
                                        Vector(0.0, 0.0, 0.0))));
        // l_wrist_pitch
        reachy.addSegment(Segment(Joint(Vector(0.0, 0.0, -0.25), Vector(0.0, 1.0, 0.0), Joint::RotAxis),
                                   Frame(Rotation(1.0, 0.0, 0.0, 0.0, 1.0, 0.0 , 0.0, 0.0, 1.0), 
                                        Vector(0.0, 0.0, -0.25))));
        // l_wrist_roll
        reachy.addSegment(Segment(Joint(Vector(0.0, 0.0, -0.0325), Vector(1.0, 0.0, 0.0), Joint::RotAxis),
                                   Frame(Rotation(1.0, 0.0, 0.0, 0.0, 1.0, 0.0 , 0.0, 0.0, 1.0), 
                                        Vector(0.0, 0.0, -0.0325))));
        // l_gripper
        reachy.addSegment(Segment(Joint(Joint::None),
                                   Frame(Rotation(1.0, 0.0, 0.0, 0.0, 1.0, 0.0 , 0.0, 0.0, 1.0), 
                                        Vector(0.0, 0.012, -0.085))));

        return reachy;
    }
}

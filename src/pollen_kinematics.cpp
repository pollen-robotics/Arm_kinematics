#include "pollen_kinematics.hpp"
#include "models.hpp"

#include "orocos_kinematics_dynamics/orocos_kdl/src/chain.hpp"
#include "orocos_kinematics_dynamics/orocos_kdl/src/chainiksolverpos_lma.hpp"
#include "orocos_kinematics_dynamics/orocos_kdl/src/chainfksolverpos_recursive.hpp"

static KDL::Chain chain_right;
static KDL::Chain chain_left;
static KDL::ChainFkSolverPos_recursive *fwdkin_right;
static KDL::ChainIkSolverPos_LMA *invkin_right;
static KDL::ChainFkSolverPos_recursive *fwdkin_left;
static KDL::ChainIkSolverPos_LMA *invkin_left;
static int n_joints;

void setup()
{
    chain_right = KDL::Reachy_RightArm();
    chain_left = KDL::Reachy_LeftArm();
    fwdkin_right = new KDL::ChainFkSolverPos_recursive(chain_right);
    fwdkin_left = new KDL::ChainFkSolverPos_recursive(chain_left);
    invkin_right = new KDL::ChainIkSolverPos_LMA(chain_right);
    invkin_left = new KDL::ChainIkSolverPos_LMA(chain_left);
    n_joints = chain_right.getNrOfJoints();
}

void forward(ArmSide side, double *q, int n, double *M)
{
    KDL::JntArray Q(n);
    for (uint8_t i=0; i < n; i++) {
        Q.data[i] = q[i];
    }

    KDL::Frame pos_goal;

    if(side == ArmSide::Right)
    {
        int retval = fwdkin_right->JntToCart(Q, pos_goal);
    }
    else
    {
        int retval = fwdkin_left->JntToCart(Q, pos_goal);
    }

    M[0] = pos_goal.M.data[0];
    M[1] = pos_goal.M.data[1];
    M[2] = pos_goal.M.data[2];
    M[3] = pos_goal.p.data[0];
    M[4] = pos_goal.M.data[3];
    M[5] = pos_goal.M.data[4];
    M[6] = pos_goal.M.data[5];
    M[7] = pos_goal.p.data[1];
    M[8] = pos_goal.M.data[6];
    M[9] = pos_goal.M.data[7];
    M[10] = pos_goal.M.data[8];
    M[11] = pos_goal.p.data[2];
    M[12] = 0.0;
    M[13] = 0.0;
    M[14] = 0.0;
    M[15] = 1.0;
}

void inverse(ArmSide side, double *M, double *q)
{
    KDL::Frame pos_goal;

    pos_goal.p.data[0] = M[3];
    pos_goal.p.data[1] = M[7];
    pos_goal.p.data[2] = M[11];

    pos_goal.M.data[0] = M[0];
    pos_goal.M.data[1] = M[1];
    pos_goal.M.data[2] = M[2];
    pos_goal.M.data[3] = M[4];
    pos_goal.M.data[4] = M[5];
    pos_goal.M.data[5] = M[6];
    pos_goal.M.data[6] = M[8];
    pos_goal.M.data[7] = M[9];
    pos_goal.M.data[8] = M[10];

    KDL::JntArray q_init(n_joints);
    q_init.data[0] = 0.0;
    q_init.data[1] = 0.0;
    q_init.data[2] = 0.0;
    q_init.data[3] = 0.0;
    q_init.data[4] = 0.0;
    q_init.data[5] = 0.0;
    q_init.data[6] = 0.0;

    KDL::JntArray q_sol(n_joints);

    if(side == ArmSide::Right)
    {
        int retval = invkin_right->CartToJnt(q_init, pos_goal, q_sol);
    }
    else
    {
        int retval = invkin_left->CartToJnt(q_init, pos_goal, q_sol);
    }

    for (int i=0; i < n_joints; i++) {
        q[i] = q_sol.data[i];
    }
}

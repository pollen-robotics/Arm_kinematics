#include "pollen_kinematics.hpp"

#include "kdl/chain.hpp"
#include "kdl/models/models.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"

static KDL::Chain chain;
static KDL::ChainFkSolverPos_recursive *fwdkin;
static KDL::ChainIkSolverPos_LMA *invkin;
static int n_joints;

void setup()
{
    chain = KDL::Puma560();
    fwdkin = new KDL::ChainFkSolverPos_recursive(chain);
    invkin = new KDL::ChainIkSolverPos_LMA(chain);
    n_joints = chain.getNrOfJoints();
}

int forward(double *q, int n, double *M)
{
    if (n != n_joints) {
        return -1;
    }
    KDL::JntArray Q(n);
    for (uint8_t i=0; i < n; i++) {
        Q.data[i] = q[i];
    }

    KDL::Frame pos_goal;
    int retval = fwdkin->JntToCart(Q, pos_goal);

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

    return retval;
}

int inverse(double *M, double *q)
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

    KDL::JntArray q_sol(n_joints);

    int retval = invkin->CartToJnt(q_init, pos_goal, q_sol);

    for (int i=0; i < n_joints; i++) {
        q[i] = q_sol.data[i];
    }

    return retval;
}
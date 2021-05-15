/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include "ompl/control/planners/rrt/RRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

ompl::control::RRT::RRT(const SpaceInformationPtr &si) : base::Planner(si, "RRT")
{
    specs_.approximateSolutions = true;
    
    // 输入的是control::spaceInformationPtr
    // 复制一个进来
    siC_ = si.get();

    addIntermediateStates_ = false;
    lastGoalMotion_ = nullptr;

    goalBias_ = 0.05;

    Planner::declareParam<double>("goal_bias", this, &RRT::setGoalBias, &RRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RRT::setIntermediateStates, &RRT::getIntermediateStates);
}

ompl::control::RRT::~RRT()
{
    freeMemory();
}

void ompl::control::RRT::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
    // 这个distanceFunction调用的是 statespace的distance函数
    nn_->setDistanceFunction(std::bind(&RRT::distanceFunction, this,
        std::placeholders::_1, std::placeholders::_2));
}

void ompl::control::RRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::control::RRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            if (motions[i]->control)
                siC_->freeControl(motions[i]->control);
            delete motions[i];
        }
    }
}

ompl::base::PlannerStatus ompl::control::RRT::solve(const base::PlannerTerminationCondition &ptc)
{
    OMPL_INFORM("----This is control::RRT-----");
    checkValidity();
    base::Goal                   *goal = pdef_->getGoal().get();
    // dynamic_cast 把父类转成子类，本来pdef_->getGoal()返回的是 constrainedGoalSampler,是GoalLazySamples的子类
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    OMPL_INFORM("---goal_s->getThreshold(): %d---",goal_s->getThreshold());

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
    {
        OMPL_INFORM("----in RRTc.cpp: start allocStateSampler()-----");
        sampler_ = si_->allocStateSampler();
        OMPL_INFORM("----in RRTc.cpp: end allocStateSampler()-----");
    }
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();
        // controlSampler_ = siC_->allocControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution  = nullptr;
    Motion *approxsol = nullptr;
    double  approxdif = std::numeric_limits<double>::infinity();

    Motion      *rmotion = new Motion(siC_);
    base::State  *rstate = rmotion->state;
    Control       *rctrl = rmotion->control;
    base::State  *xstate = si_->allocState();

    OMPL_INFORM("----RRT: Ready to start sampling-----");
    OMPL_INFORM("----RRT: addIntermediateStates_ is %d-----", addIntermediateStates_);
    OMPL_INFORM("----RRT: getMinControlDuration() %d-----",siC_->getMinControlDuration());
    OMPL_INFORM("----RRT: getMaxControlDuration() %d-----",siC_->getMaxControlDuration());
    OMPL_INFORM("----RRT: goal_s->canSample() %d-----",goal_s->canSample());
    while (ptc == false)
    {
        /* sample random state (with goal biasing) */
        // OMPL_INFORM("----1s-----");
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
        {
            OMPL_INFORM("---try to sample a goal now!----");
            goal_s->sampleGoal(rstate);
            OMPL_INFORM("---try to sample a goal end!----");
        }
        else
        {
            sampler_->sampleUniform(rstate);
        }
        // si_->printState(rstate);
        // OMPL_INFORM("----1e-----");

        /* find closest state in the tree */
        // OMPL_INFORM("----2s-----");
        Motion *nmotion = nn_->nearest(rmotion);
        // OMPL_INFORM("nn size():%d", nn_->size());
        // si_->printState(nmotion->state);
        // OMPL_INFORM("----2e-----");

        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        // OMPL_INFORM("----3s-----");
        unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);
        // siC_->printControl(rctrl);
        // OMPL_INFORM("%d", cd);
        // OMPL_INFORM("----3e-----");
        // 一般为false
        if (addIntermediateStates_)
        {
            // this code is contributed by Jennifer Barry
            std::vector<base::State *> pstates;
            cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);

            if (cd >= siC_->getMinControlDuration())
            {
                Motion *lastmotion = nmotion;
                bool solved = false;
                size_t p = 0;
                for ( ; p < pstates.size(); ++p)
                {
                    /* create a motion */
                    Motion *motion = new Motion();
                    motion->state = pstates[p];
                    //we need multiple copies of rctrl
                    motion->control = siC_->allocControl();
                    siC_->copyControl(motion->control, rctrl);
                    motion->steps = 1;
                    motion->parent = lastmotion;
                    lastmotion = motion;
                    nn_->add(motion);
                    double dist = 0.0;
                    solved = goal->isSatisfied(motion->state, &dist);
                    if (solved)
                    {
                        approxdif = dist;
                        solution = motion;
                        break;
                    }
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = motion;
                    }
                }

                //free any states after we hit the goal
                while (++p < pstates.size())
                    si_->freeState(pstates[p]);
                if (solved)
                    break;
            }
            else
                for (size_t p = 0 ; p < pstates.size(); ++p)
                    si_->freeState(pstates[p]);
        }
        else
        {
            if (cd >= siC_->getMinControlDuration())
            {
                /* create a motion */
                Motion *motion = new Motion(siC_);
                si_->copyState(motion->state, rmotion->state);
                siC_->copyControl(motion->control, rctrl);
                motion->steps = cd;
                motion->parent = nmotion;

                nn_->add(motion);
                double dist = 0.0;
                bool solv = goal->isSatisfied(motion->state, &dist);
                if (solv)
                {
                    approxdif = dist;
                    solution = motion;
                    break;
                }
                if (dist < approxdif)
                {
                    approxdif = dist;
                    approxsol = motion;
                }
            }
        }
    }
    OMPL_INFORM("----RRTControl:Finish while!!-----");
    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion*> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        PathControl *path = new PathControl(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif, getName());
    }

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(xstate);


    OMPL_INFORM("----RRT: addIntermediateStates_ is %d-----", addIntermediateStates_);
    OMPL_INFORM("----RRT: getMinControlDuration() %d-----",siC_->getMinControlDuration());
    OMPL_INFORM("----RRT: solve? %d-----",solved);
    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return base::PlannerStatus(solved, approximate);
}

void ompl::control::RRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        const Motion *m = motions[i];
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state),
                             base::PlannerDataVertex(m->state),
                             control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state),
                             base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}

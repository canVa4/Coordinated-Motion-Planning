/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

#include <moveit/ompl_interface/planning_context_manager.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/profiler/profiler.h>
#include <algorithm>
#include <set>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/ProjEST.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>

#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/sst/SST.h>

//testing
#include <iostream>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/config.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space_factory.h>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>
#include <moveit/ompl_interface/parameterization/work_space/pose_model_state_space_factory.h>

namespace ompl_interface
{
class PlanningContextManager::LastPlanningContext
{
public:
  ModelBasedPlanningContextPtr getContext()
  {
    boost::mutex::scoped_lock slock(lock_);
    return last_planning_context_solve_;
  }

  void setContext(const ModelBasedPlanningContextPtr& context)
  {
    boost::mutex::scoped_lock slock(lock_);
    last_planning_context_solve_ = context;
  }

  void clear()
  {
    boost::mutex::scoped_lock slock(lock_);
    last_planning_context_solve_.reset();
  }

private:
  /* The planning group for which solve() was called last */
  ModelBasedPlanningContextPtr last_planning_context_solve_;
  boost::mutex lock_;
};

struct PlanningContextManager::CachedContexts
{
  std::map<std::pair<std::string, std::string>, std::vector<ModelBasedPlanningContextPtr> > contexts_;
  boost::mutex lock_;
};

}  // namespace ompl_interface

ompl_interface::PlanningContextManager::PlanningContextManager(
    const robot_model::RobotModelConstPtr& kmodel, const constraint_samplers::ConstraintSamplerManagerPtr& csm)
  : kmodel_(kmodel)
  , constraint_sampler_manager_(csm)
  , max_goal_samples_(10)
  , max_state_sampling_attempts_(4)
  , max_goal_sampling_attempts_(1000)
  , max_planning_threads_(4)
  , max_solution_segment_length_(0.0)
  , minimum_waypoint_count_(2)
{
  last_planning_context_.reset(new LastPlanningContext());
  cached_contexts_.reset(new CachedContexts());
  registerDefaultPlanners();
  registerDefaultStateSpaces();
  using_control_ = false;   //设置是否支持control
}

ompl_interface::PlanningContextManager::~PlanningContextManager()
{
}

namespace
{
using namespace ompl_interface;

template <typename T>
static ompl::base::PlannerPtr allocatePlanner(const ompl::base::SpaceInformationPtr& si, const std::string& new_name,
                                              const ModelBasedPlanningContextSpecification& spec)
{
  ompl::base::PlannerPtr planner(new T(si));
  if (!new_name.empty())
    planner->setName(new_name);
  planner->params().setParams(spec.config_, true);
  planner->setup();
  return planner;
}

template <typename T2>
static ompl::base::PlannerPtr allocateControlPlanner(const ompl::control::SpaceInformationPtr& si, const std::string& new_name,
                                              const ModelBasedPlanningContextSpecification& spec)
{
  ompl::base::PlannerPtr planner(new T2(si));
  if (!new_name.empty())
    planner->setName(new_name);
  planner->params().setParams(spec.config_, true);
  planner->setup();
  return planner;
}

}

ompl_interface::ConfiguredPlannerAllocator
ompl_interface::PlanningContextManager::plannerSelector(const std::string& planner) const
{
  std::map<std::string, ConfiguredPlannerAllocator>::const_iterator it = known_planners_.find(planner);
  if (it != known_planners_.end())
    return it->second;
  else
  {
    ROS_ERROR_NAMED("planning_context_manager", "Unknown planner: '%s'", planner.c_str());
    return ConfiguredPlannerAllocator();
  }
}

ompl_interface::ConfiguredControlPlannerAllocator
ompl_interface::PlanningContextManager::controlPlannerSelector(const std::string& planner) const
{
  auto it = known_control_planners_.find(planner);
  if (it != known_control_planners_.end())
    return it->second;
  else
  {
    ROS_ERROR_NAMED("planning_context_manager", "Unknown planner: '%s'", planner.c_str());
    return ConfiguredPlannerAllocator();
  }
}

void ompl_interface::PlanningContextManager::registerDefaultPlanners()
{
  registerPlannerAllocator("geometric::RRT", boost::bind(&allocatePlanner<og::RRT>, _1, _2, _3));
  registerPlannerAllocator("geometric::RRTConnect", boost::bind(&allocatePlanner<og::RRTConnect>, _1, _2, _3));
  registerPlannerAllocator("geometric::LazyRRT", boost::bind(&allocatePlanner<og::LazyRRT>, _1, _2, _3));
  registerPlannerAllocator("geometric::TRRT", boost::bind(&allocatePlanner<og::TRRT>, _1, _2, _3));
  registerPlannerAllocator("geometric::EST", boost::bind(&allocatePlanner<og::EST>, _1, _2, _3));
  registerPlannerAllocator("geometric::SBL", boost::bind(&allocatePlanner<og::SBL>, _1, _2, _3));
  registerPlannerAllocator("geometric::KPIECE", boost::bind(&allocatePlanner<og::KPIECE1>, _1, _2, _3));
  registerPlannerAllocator("geometric::BKPIECE", boost::bind(&allocatePlanner<og::BKPIECE1>, _1, _2, _3));
  registerPlannerAllocator("geometric::LBKPIECE", boost::bind(&allocatePlanner<og::LBKPIECE1>, _1, _2, _3));
  registerPlannerAllocator("geometric::RRTstar", boost::bind(&allocatePlanner<og::RRTstar>, _1, _2, _3));
  registerPlannerAllocator("geometric::PRM", boost::bind(&allocatePlanner<og::PRM>, _1, _2, _3));
  registerPlannerAllocator("geometric::PRMstar", boost::bind(&allocatePlanner<og::PRMstar>, _1, _2, _3));
  registerPlannerAllocator("geometric::FMT", boost::bind(&allocatePlanner<og::FMT>, _1, _2, _3));
  registerPlannerAllocator("geometric::BFMT", boost::bind(&allocatePlanner<og::BFMT>, _1, _2, _3));
  registerPlannerAllocator("geometric::PDST", boost::bind(&allocatePlanner<og::PDST>, _1, _2, _3));
  registerPlannerAllocator("geometric::STRIDE", boost::bind(&allocatePlanner<og::STRIDE>, _1, _2, _3));
  registerPlannerAllocator("geometric::BiTRRT", boost::bind(&allocatePlanner<og::BiTRRT>, _1, _2, _3));
  registerPlannerAllocator("geometric::LBTRRT", boost::bind(&allocatePlanner<og::LBTRRT>, _1, _2, _3));
  registerPlannerAllocator("geometric::BiEST", boost::bind(&allocatePlanner<og::BiEST>, _1, _2, _3));
  registerPlannerAllocator("geometric::ProjEST", boost::bind(&allocatePlanner<og::ProjEST>, _1, _2, _3));
  registerPlannerAllocator("geometric::LazyPRM", boost::bind(&allocatePlanner<og::LazyPRM>, _1, _2, _3));
  registerPlannerAllocator("geometric::LazyPRMstar", boost::bind(&allocatePlanner<og::LazyPRMstar>, _1, _2, _3));
  registerPlannerAllocator("geometric::SPARS", boost::bind(&allocatePlanner<og::SPARS>, _1, _2, _3));
  registerPlannerAllocator("geometric::SPARStwo", boost::bind(&allocatePlanner<og::SPARStwo>, _1, _2, _3));

  // 注册控制算法
  // TODO
  // boost::bind(&allocateControlPlanner<ompl::control::RRT>, _1, _2, _3);
  registerControlPlannerAllocator("control::RRT", boost::bind(&allocateControlPlanner<ompl::control::RRT>, _1, _2, _3));
}

// 创建constrol space
ompl::control::ControlSpacePtr ompl_interface::PlanningContextManager::createControlSpace(ompl::base::StateSpacePtr &stateSpace, unsigned int dim){
  auto control_space(std::make_shared<ompl::control::RealVectorControlSpace>(stateSpace, dim));
  ompl::base::RealVectorBounds cbounds(7);
  cbounds.setLow(-0.5);
  cbounds.setHigh(0.5);
  return control_space;
}

// 定义系统的ODE //使用boost::odeint 作为ODE solver
void TB3M_ODE(const ompl::control::ODESolver::StateType &q, const ompl::control::Control *c,
                  ompl::control::ODESolver::StateType &qdot) {
  // TODO
  // control value的第0个为线速度，第1个为角速度
  const double *u =
      c->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
  const double velocity = u[0];
  const double steeringAngle = u[1];

  // Retrieve the current orientation of the car.  The memory for
  // ompl::base::SE2StateSpace is mapped as:
  // 0: x
  // 1: y
  // 2: theta
  const double theta = q[2];

  // Ensure qdot is the same size as q.  Zero out all values.
  qdot.resize(q.size(), 0);

  qdot[0] = velocity * cos(theta);         // x-dot
  qdot[1] = velocity * sin(theta);         // y-dot
  qdot[2] = velocity * tan(steeringAngle); // theta-dot
  qdot[3] = u[2];
  qdot[4] = u[3];
  qdot[5] = u[4];
  qdot[6] = u[5];
}

void TB3B_ODE(const oc::ODESolver::StateType &q, const oc::Control *c,
                  oc::ODESolver::StateType &qdot) {
  // Retrieve control values.  Velocity is the first entry, steering angle is
  // second.
  const double *u =
      c->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
  const double velocity = u[0];
  const double steeringAngle = u[1];

  // Retrieve the current orientation of the car.  The memory for
  // ompl::base::SE2StateSpace is mapped as:
  // 0: x
  // 1: y
  // 2: theta
  const double theta = q[2];

  // Ensure qdot is the same size as q.  Zero out all values.
  qdot.resize(q.size(), 0);

  qdot[0] = velocity * cos(theta);         // x-dot
  qdot[1] = velocity * sin(theta);         // y-dot
  qdot[2] = velocity * tan(steeringAngle); // theta-dot
} 

void propagate(const ompl::base::State *start, const ompl::control::Control *control,
               const double duration, ompl::base::State *result) {

  const auto *input_state = start->as<ompl_interface::ModelBasedStateSpace::StateType>()->values;
  // const double *pos =
  //     se2state->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
  // const double rot = se2state->as<ompl::base::SO2StateSpace::StateType>(1)->value;
  const double *ctrl =
      control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

//   result->as<ob::SE2StateSpace::StateType>()->setXY(
//       pos[0] + ctrl[0] * duration * cos(rot),
//       pos[1] + ctrl[0] * duration * sin(rot));
//   result->as<ob::SE2StateSpace::StateType>()->setYaw(rot + ctrl[1] * duration);

  // result->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[0] = input_state[0] + ctrl[0]* duration;
  // result->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[1] = input_state[1] + ctrl[1]* duration;
  // result->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[2] = input_state[2];

  result->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[0] = input_state[0] + ctrl[0] * duration * cos(input_state[2]);
  result->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[1] = input_state[1] + ctrl[0] * duration * sin(input_state[2]);
  result->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[2] = input_state[2] + ctrl[1] * duration;
}

void TB3M_propagate(const ompl::base::State *start, const ompl::control::Control *control,
               const double duration, ompl::base::State *result) {

  const auto *input_state = start->as<ompl_interface::ModelBasedStateSpace::StateType>()->values;
  // const double *pos =
  //     se2state->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
  // const double rot = se2state->as<ompl::base::SO2StateSpace::StateType>(1)->value;
  const double *ctrl =
      control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

//   result->as<ob::SE2StateSpace::StateType>()->setXY(
//       pos[0] + ctrl[0] * duration * cos(rot),
//       pos[1] + ctrl[0] * duration * sin(rot));
//   result->as<ob::SE2StateSpace::StateType>()->setYaw(rot + ctrl[1] * duration);

  // result->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[0] = input_state[0] + ctrl[0]* duration;
  // result->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[1] = input_state[1] + ctrl[1]* duration;
  // result->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[2] = input_state[2];

  result->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[0] = input_state[0] + ctrl[0] * duration * cos(input_state[2]);
  result->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[1] = input_state[1] + ctrl[0] * duration * sin(input_state[2]);
  result->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[2] = input_state[2] + ctrl[1] * duration;
  result->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[3] = ctrl[2] * duration;
  result->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[4] = ctrl[3] * duration;
  result->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[5] = ctrl[4] * duration;
  result->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[6] = ctrl[5] * duration;
}


void ompl_interface::PlanningContextManager::registerDefaultStateSpaces()
{
  registerStateSpaceFactory(ModelBasedStateSpaceFactoryPtr(new JointModelStateSpaceFactory()));
  registerStateSpaceFactory(ModelBasedStateSpaceFactoryPtr(new PoseModelStateSpaceFactory()));
}

ompl_interface::ConfiguredPlannerSelector ompl_interface::PlanningContextManager::getPlannerSelector() const
{
  return boost::bind(&PlanningContextManager::plannerSelector, this, _1);
}

ompl_interface::ConfiguredControlPlannerSelector ompl_interface::PlanningContextManager::getControlPlannerSelector() const
{
  return boost::bind(&PlanningContextManager::controlPlannerSelector, this, _1);
}

void ompl_interface::PlanningContextManager::setPlannerConfigurations(
    const planning_interface::PlannerConfigurationMap& pconfig)
{
  planner_configs_ = pconfig;
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::PlanningContextManager::getPlanningContext(
    const std::string& config, const std::string& factory_type) const
{
  planning_interface::PlannerConfigurationMap::const_iterator pc = planner_configs_.find(config);

  if (pc != planner_configs_.end())
  {
    moveit_msgs::MotionPlanRequest req;  // dummy request with default values
    return getPlanningContext(pc->second,
                              boost::bind(&PlanningContextManager::getStateSpaceFactory1, this, _1, factory_type), req);
  }
  else
  {
    ROS_ERROR_NAMED("planning_context_manager", "Planning configuration '%s' was not found", config.c_str());
    return ModelBasedPlanningContextPtr();
  }
}

ompl_interface::ControlPlanningContextPtr ompl_interface::PlanningContextManager::getControlPlanningContext(
    const std::string& config, const std::string& factory_type) const
{
  planning_interface::PlannerConfigurationMap::const_iterator pc = planner_configs_.find(config);

  if (pc != planner_configs_.end())
  {
    moveit_msgs::MotionPlanRequest req;  // dummy request with default values
    return getControlPlanningContext(pc->second,
                              boost::bind(&PlanningContextManager::getStateSpaceFactory1, this, _1, factory_type), req);
  }
  else
  {
    ROS_ERROR_NAMED("planning_context_manager", "Planning configuration '%s' was not found", config.c_str());
    return ControlPlanningContextPtr();
  }
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::PlanningContextManager::getPlanningContext(
    const planning_interface::PlannerConfigurationSettings& config,
    const StateSpaceFactoryTypeSelector& factory_selector, const moveit_msgs::MotionPlanRequest& req) const
{
  const ompl_interface::ModelBasedStateSpaceFactoryPtr& factory = factory_selector(config.group);

  // Check for a cached planning context
  ModelBasedPlanningContextPtr context;

  {
    boost::mutex::scoped_lock slock(cached_contexts_->lock_);
    std::map<std::pair<std::string, std::string>, std::vector<ModelBasedPlanningContextPtr> >::const_iterator cc =
        cached_contexts_->contexts_.find(std::make_pair(config.name, factory->getType()));
    if (cc != cached_contexts_->contexts_.end())
    {
      for (std::size_t i = 0; i < cc->second.size(); ++i)
        if (cc->second[i].unique())
        {
          ROS_DEBUG_NAMED("planning_context_manager", "Reusing cached planning context");
          context = cc->second[i];
          break;
        }
    }
  }

  // Create a new planning context
  if (!context)
  {
    ModelBasedStateSpaceSpecification space_spec(kmodel_, config.group);
    ModelBasedPlanningContextSpecification context_spec;
    context_spec.config_ = config.config;
    context_spec.planner_selector_ = getPlannerSelector();
    context_spec.constraint_sampler_manager_ = constraint_sampler_manager_;
    // 这一步会输出构建了什么state space
    context_spec.state_space_ = factory->getNewStateSpace(space_spec);

    ROS_INFO_STREAM("**********you r in here!!!**********");
    // context_spec.using_control = using_control_; //额外增加变量至context_spec
    // if (!using_control_)
    // {   
      // Choose the correct simple setup type to load
      context_spec.ompl_simple_setup_.reset(new ompl::geometric::SimpleSetup(context_spec.state_space_));
    // }
    // else{
    //   // // 新增：当使用control时
    //   // // auto control_space = createControlSpace(context_spec.state_space_, 7);

    //   // auto control_space(std::make_shared<ompl::control::RealVectorControlSpace>(context_spec.state_space_, 7));
    //   // ompl::base::RealVectorBounds cbounds(7);
    //   // cbounds.setLow(-0.5);
    //   // cbounds.setHigh(0.5);

    //   // context_spec.ompl_simple_control_setup_.reset(new ompl::control::SimpleSetup(control_space));
    //   // auto si = context_spec.ompl_simple_control_setup_->getSpaceInformation();
    //   // // ompl::control::ODESolverPtr odeSolver (new ompl::control::ODEBasicSolver<> (si, &TB3M_ODE));
    //   // // si->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver)); 
    //   // // si->setStatePropagator(propagate);     
    // }

    bool state_validity_cache = true;
    if (config.config.find("subspaces") != config.config.end())
    {
      context_spec.config_.erase("subspaces");
      // if the planner operates at subspace level the cache may be unsafe
      state_validity_cache = false;
      boost::char_separator<char> sep(" ");
      boost::tokenizer<boost::char_separator<char> > tok(config.config.at("subspaces"), sep);
      for (boost::tokenizer<boost::char_separator<char> >::iterator beg = tok.begin(); beg != tok.end(); ++beg)
      {
        const ompl_interface::ModelBasedStateSpaceFactoryPtr& sub_fact = factory_selector(*beg);
        if (sub_fact)
        {
          ModelBasedStateSpaceSpecification sub_space_spec(kmodel_, *beg);
          context_spec.subspaces_.push_back(sub_fact->getNewStateSpace(sub_space_spec));
        }
      }
    }

    ROS_DEBUG_NAMED("planning_context_manager", "Creating new planning context");
    context.reset(new ModelBasedPlanningContext(config.name, context_spec));
    context->useStateValidityCache(state_validity_cache);
    {
      boost::mutex::scoped_lock slock(cached_contexts_->lock_);
      cached_contexts_->contexts_[std::make_pair(config.name, factory->getType())].push_back(context);
    }
  }

  context->setMaximumPlanningThreads(max_planning_threads_);
  context->setMaximumGoalSamples(max_goal_samples_);
  context->setMaximumStateSamplingAttempts(max_state_sampling_attempts_);
  context->setMaximumGoalSamplingAttempts(max_goal_sampling_attempts_);
  if (max_solution_segment_length_ > std::numeric_limits<double>::epsilon())
    context->setMaximumSolutionSegmentLength(max_solution_segment_length_);
  context->setMinimumWaypointCount(minimum_waypoint_count_);

  context->setSpecificationConfig(config.config);

  last_planning_context_->setContext(context);
  return context;
}

ompl_interface::ControlPlanningContextPtr ompl_interface::PlanningContextManager::getControlPlanningContext(
    const planning_interface::PlannerConfigurationSettings& config,
    const StateSpaceFactoryTypeSelector& factory_selector, const moveit_msgs::MotionPlanRequest& req) const
{
  const ompl_interface::ModelBasedStateSpaceFactoryPtr& factory = factory_selector(config.group);

  // Check for a cached planning context
  ControlPlanningContextPtr context;
  // {
  //   boost::mutex::scoped_lock slock(cached_contexts_->lock_);
  //   std::map<std::pair<std::string, std::string>, std::vector<ControlPlanningContextPtr> >::const_iterator cc =
  //       cached_contexts_->contexts_.find(std::make_pair(config.name, factory->getType()));
  //   if (cc != cached_contexts_->contexts_.end())
  //   {
  //     for (std::size_t i = 0; i < cc->second.size(); ++i)
  //       if (cc->second[i].unique())
  //       {
  //         ROS_DEBUG_NAMED("planning_context_manager", "Reusing cached planning context");
  //         context = cc->second[i];
  //         break;
  //       }
  //   }
  // }  
  // Create a new planning context
  if (!context)
  {
    ModelBasedStateSpaceSpecification space_spec(kmodel_, config.group);
    ModelBasedPlanningContextSpecification context_spec;
    context_spec.config_ = config.config;
    context_spec.control_planner_selector_ = getControlPlannerSelector();
    context_spec.constraint_sampler_manager_ = constraint_sampler_manager_;
    context_spec.state_space_ = factory->getNewStateSpace(space_spec);
  
    // Load ompl::control::simpleSetup
    // 修改的地方！！！
    context_spec.ompl_simple_setup_.reset(new ompl::geometric::SimpleSetup(context_spec.state_space_));

    auto control_space(std::make_shared<ompl::control::RealVectorControlSpace>(context_spec.state_space_, 6));
    ompl::base::RealVectorBounds cbounds(6);
    cbounds.setLow(-0.3);
    cbounds.setHigh(0.3);
    control_space->setBounds(cbounds);

    context_spec.ompl_simple_control_setup_.reset(new ompl::control::SimpleSetup(control_space));
    auto si = context_spec.ompl_simple_control_setup_->getSpaceInformation();
    // ompl::control::ODESolverPtr odeSolver (new ompl::control::ODEBasicSolver<> (si, &TB3M_ODE));
    // ompl::control::ODESolverPtr odeSolver (new ompl::control::ODEBasicSolver<> (si, &TB3B_ODE));
    // si->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));
    // si->setStatePropagator(propagate);
    si->setStatePropagator(TB3M_propagate); 
    //set Planner
    context_spec.ompl_simple_control_setup_->setPlanner(std::make_shared<oc::RRT>(si));
    // context_spec.ompl_simple_control_setup_->setPlanner(std::make_shared<oc::SST>(si));
    // context_spec.ompl_simple_control_setup_->setPlanner(std::make_shared<oc::KPIECE1>(si));         
    // 修改的结束

    bool state_validity_cache = true;
    if (config.config.find("subspaces") != config.config.end())
    {
      context_spec.config_.erase("subspaces");
      // if the planner operates at subspace level the cache may be unsafe
      state_validity_cache = false;
      boost::char_separator<char> sep(" ");
      boost::tokenizer<boost::char_separator<char> > tok(config.config.at("subspaces"), sep);
      for (boost::tokenizer<boost::char_separator<char> >::iterator beg = tok.begin(); beg != tok.end(); ++beg)
      {
        const ompl_interface::ModelBasedStateSpaceFactoryPtr& sub_fact = factory_selector(*beg);
        if (sub_fact)
        {
          ModelBasedStateSpaceSpecification sub_space_spec(kmodel_, *beg);
          context_spec.subspaces_.push_back(sub_fact->getNewStateSpace(sub_space_spec));
        }
      }
    }

    ROS_DEBUG_NAMED("planning_context_manager", "Creating new planning context");
    context.reset(new ControlPlanningContext(config.name, context_spec));
    context->useStateValidityCache(state_validity_cache);
    // {
    //   boost::mutex::scoped_lock slock(cached_contexts_->lock_);
    //   cached_contexts_->contexts_[std::make_pair(config.name, factory->getType())].push_back(context);
    // }
  }

  context->setMaximumPlanningThreads(max_planning_threads_);
  context->setMaximumGoalSamples(max_goal_samples_);
  context->setMaximumStateSamplingAttempts(max_state_sampling_attempts_);
  context->setMaximumGoalSamplingAttempts(max_goal_sampling_attempts_);
  if (max_solution_segment_length_ > std::numeric_limits<double>::epsilon())
    context->setMaximumSolutionSegmentLength(max_solution_segment_length_);
  context->setMinimumWaypointCount(minimum_waypoint_count_);

  context->setSpecificationConfig(config.config);

  // last_planning_context_->setContext(context);
  return context;
}

const ompl_interface::ModelBasedStateSpaceFactoryPtr& ompl_interface::PlanningContextManager::getStateSpaceFactory1(
    const std::string& /* dummy */, const std::string& factory_type) const
{
  ROS_INFO_STREAM("---getStateSpaceFactory1---");
  std::map<std::string, ModelBasedStateSpaceFactoryPtr>::const_iterator f =
      factory_type.empty() ? state_space_factories_.begin() : state_space_factories_.find(factory_type);
  if (f != state_space_factories_.end())
    return f->second;
  else
  {
    ROS_ERROR_NAMED("planning_context_manager", "Factory of type '%s' was not found", factory_type.c_str());
    static const ModelBasedStateSpaceFactoryPtr empty;
    return empty;
  }
}

const ompl_interface::ModelBasedStateSpaceFactoryPtr& ompl_interface::PlanningContextManager::getStateSpaceFactory2(
    const std::string& group, const moveit_msgs::MotionPlanRequest& req) const
{
  ROS_INFO_STREAM("---getStateSpaceFactory2---");
  // find the problem representation to use
  std::map<std::string, ModelBasedStateSpaceFactoryPtr>::const_iterator best = state_space_factories_.end();
  int prev_priority = -1;
  for (std::map<std::string, ModelBasedStateSpaceFactoryPtr>::const_iterator it = state_space_factories_.begin();
       it != state_space_factories_.end(); ++it)
  {
    int priority = it->second->canRepresentProblem(group, req, kmodel_);
    if (priority > 0)
      if (best == state_space_factories_.end() || priority > prev_priority)
      {
        best = it;
        prev_priority = priority;
      }
  }

  if (best == state_space_factories_.end())
  {
    ROS_ERROR_NAMED("planning_context_manager", "There are no known state spaces that can represent the given planning "
                                                "problem");
    static const ModelBasedStateSpaceFactoryPtr empty;
    return empty;
  }
  else
  {
    ROS_DEBUG_NAMED("planning_context_manager", "Using '%s' parameterization for solving problem", best->first.c_str());
    return best->second;
  }
}

ompl_interface::ModelBasedPlanningContextPtr
ompl_interface::PlanningContextManager::getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                           const moveit_msgs::MotionPlanRequest& req,
                                                           moveit_msgs::MoveItErrorCodes& error_code) const
{
  if (req.group_name.empty())
  {
    ROS_ERROR_NAMED("planning_context_manager", "No group specified to plan for");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return ModelBasedPlanningContextPtr();
  }

  error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;

  if (!planning_scene)
  {
    ROS_ERROR_NAMED("planning_context_manager", "No planning scene supplied as input");
    return ModelBasedPlanningContextPtr();
  }

  // identify the correct planning configuration
  planning_interface::PlannerConfigurationMap::const_iterator pc = planner_configs_.end();
  if (!req.planner_id.empty())
  {
    pc = planner_configs_.find(req.planner_id.find(req.group_name) == std::string::npos ?
                                   req.group_name + "[" + req.planner_id + "]" :
                                   req.planner_id);
    if (pc == planner_configs_.end())
      ROS_WARN_NAMED("planning_context_manager",
                     "Cannot find planning configuration for group '%s' using planner '%s'. Will use defaults instead.",
                     req.group_name.c_str(), req.planner_id.c_str());
  }

  if (pc == planner_configs_.end())
  {
    pc = planner_configs_.find(req.group_name);
    if (pc == planner_configs_.end())
    {
      ROS_ERROR_NAMED("planning_context_manager", "Cannot find planning configuration for group '%s'",
                      req.group_name.c_str());
      return ModelBasedPlanningContextPtr();
    }
  }

  // Check if sampling in JointModelStateSpace is enforced for this group by user.
  // This is done by setting 'enforce_joint_model_state_space' to 'true' for the desired group in ompl_planning.yaml.
  //
  // Some planning problems like orientation path constraints are represented in PoseModelStateSpace and sampled via IK.
  // However consecutive IK solutions are not checked for proximity at the moment and sometimes happen to be flipped,
  // leading to invalid trajectories. This workaround lets the user prevent this problem by forcing rejection sampling
  // in JointModelStateSpace.
  StateSpaceFactoryTypeSelector factory_selector;
  std::map<std::string, std::string>::const_iterator it = pc->second.config.find("enforce_joint_model_state_space");

  if (it != pc->second.config.end() && boost::lexical_cast<bool>(it->second))
    factory_selector = boost::bind(&PlanningContextManager::getStateSpaceFactory1, this, _1,
                                   JointModelStateSpace::PARAMETERIZATION_TYPE);
  else
    factory_selector = boost::bind(&PlanningContextManager::getStateSpaceFactory2, this, _1, req);

  ModelBasedPlanningContextPtr context = getPlanningContext(pc->second, factory_selector, req);

  if (context)
  {
    context->clear();

    robot_state::RobotStatePtr start_state = planning_scene->getCurrentStateUpdated(req.start_state);

    // Setup the context
    context->setPlanningScene(planning_scene);
    context->setMotionPlanRequest(req);
    context->setCompleteInitialState(*start_state);

    context->setPlanningVolume(req.workspace_parameters);
    if (!context->setPathConstraints(req.path_constraints, &error_code))
      return ModelBasedPlanningContextPtr();

    if (!context->setGoalConstraints(req.goal_constraints, req.path_constraints, &error_code))
      return ModelBasedPlanningContextPtr();

    try
    {
      context->configure();
      ROS_DEBUG_NAMED("planning_context_manager", "%s: New planning context is set.", context->getName().c_str());
      error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
    catch (ompl::Exception& ex)
    {
      ROS_ERROR_NAMED("planning_context_manager", "OMPL encountered an error: %s", ex.what());
      context.reset();
    }
  }

  return context;
}


ompl_interface::ControlPlanningContextPtr
ompl_interface::PlanningContextManager::getControlPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                           const moveit_msgs::MotionPlanRequest& req,
                                                           moveit_msgs::MoveItErrorCodes& error_code) const
{
  if (req.group_name.empty())
  {
    ROS_ERROR_NAMED("planning_context_manager", "No group specified to plan for");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return ControlPlanningContextPtr();
  }

  error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;

  if (!planning_scene)
  {
    ROS_ERROR_NAMED("planning_context_manager", "No planning scene supplied as input");
    return ControlPlanningContextPtr();
  }

  // identify the correct planning configuration
  planning_interface::PlannerConfigurationMap::const_iterator pc = planner_configs_.end();
  if (!req.planner_id.empty())
  {
    pc = planner_configs_.find(req.planner_id.find(req.group_name) == std::string::npos ?
                                   req.group_name + "[" + req.planner_id + "]" :
                                   req.planner_id);
    if (pc == planner_configs_.end())
      ROS_WARN_NAMED("planning_context_manager",
                     "Cannot find planning configuration for group '%s' using planner '%s'. Will use defaults instead.",
                     req.group_name.c_str(), req.planner_id.c_str());
  }

  if (pc == planner_configs_.end())
  {
    pc = planner_configs_.find(req.group_name);
    if (pc == planner_configs_.end())
    {
      ROS_ERROR_NAMED("planning_context_manager", "Cannot find planning configuration for group '%s'",
                      req.group_name.c_str());
      return ControlPlanningContextPtr();
    }
  }

  // Check if sampling in JointModelStateSpace is enforced for this group by user.
  // This is done by setting 'enforce_joint_model_state_space' to 'true' for the desired group in ompl_planning.yaml.
  //
  // Some planning problems like orientation path constraints are represented in PoseModelStateSpace and sampled via IK.
  // However consecutive IK solutions are not checked for proximity at the moment and sometimes happen to be flipped,
  // leading to invalid trajectories. This workaround lets the user prevent this problem by forcing rejection sampling
  // in JointModelStateSpace.
  StateSpaceFactoryTypeSelector factory_selector;
  std::map<std::string, std::string>::const_iterator it = pc->second.config.find("enforce_joint_model_state_space");

  if (it != pc->second.config.end() && boost::lexical_cast<bool>(it->second))
    factory_selector = boost::bind(&PlanningContextManager::getStateSpaceFactory1, this, _1,
                                   JointModelStateSpace::PARAMETERIZATION_TYPE);
  else
    factory_selector = boost::bind(&PlanningContextManager::getStateSpaceFactory2, this, _1, req); //调用的是这个

  ControlPlanningContextPtr context = getControlPlanningContext(pc->second, factory_selector, req);


  // 提示信息

  ROS_INFO_STREAM("----Already create an control context!----");
  if (context)
  {
    context->clear();

    robot_state::RobotStatePtr start_state = planning_scene->getCurrentStateUpdated(req.start_state);

    // Setup the context
    context->setPlanningScene(planning_scene);
    context->setMotionPlanRequest(req);
    context->setCompleteInitialState(*start_state);

    context->setPlanningVolume(req.workspace_parameters);
    if (!context->setPathConstraints(req.path_constraints, &error_code))
      return ControlPlanningContextPtr();

    if (!context->setGoalConstraints(req.goal_constraints, req.path_constraints, &error_code))
      return ControlPlanningContextPtr();

    try
    {
      context->configure();
      ROS_INFO("----Finish Configure! %s: New planning context is set.----", context->getName().c_str());
      ROS_DEBUG_NAMED("planning_context_manager", "%s: New planning context is set.", context->getName().c_str());
      error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
    catch (ompl::Exception& ex)
    {
      ROS_ERROR_NAMED("planning_context_manager", "OMPL encountered an error: %s", ex.what());
      context.reset();
    }
  }

  return context;
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::PlanningContextManager::getLastPlanningContext() const
{
  return last_planning_context_->getContext();
}

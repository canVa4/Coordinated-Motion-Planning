#ifndef MOVEIT_OMPL_INTERFACE_CONTROL_PLANNING_CONTEXT_
#define MOVEIT_OMPL_INTERFACE_CONTROL_PLANNING_CONTEXT_

#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <moveit/ompl_interface/detail/constrained_valid_state_sampler.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/planning_interface/planning_interface.h>

//需要include modelBasedPlanningContext 因为需要ModelBasedPlanningContextSpecification这个结构体
#include <moveit/ompl_interface/model_based_planning_context.h>

// #include <ompl/geometric/SimpleSetup.h>
// change to control simplesetup
#include <ompl/control/SimpleSetup.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/tools/multiplan/ParallelPlan.h>
#include <ompl/base/StateStorage.h>

#include <boost/thread/mutex.hpp>

namespace ompl_interface
{
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;
namespace oc = ompl::control;

MOVEIT_CLASS_FORWARD(ControlPlanningContext);

class ControlPlanningContext : public planning_interface::PlanningContext
{
public:
  ControlPlanningContext(const std::string& name, const ModelBasedPlanningContextSpecification& spec);

  virtual ~ControlPlanningContext()
  {
  }

  virtual bool solve(planning_interface::MotionPlanResponse& res);
  virtual bool solve(planning_interface::MotionPlanDetailedResponse& res);

  virtual void clear();
  virtual bool terminate();

  const ModelBasedPlanningContextSpecification& getSpecification() const
  {
    return spec_;
  }

  const std::map<std::string, std::string>& getSpecificationConfig() const
  {
    return spec_.config_;
  }

  void setSpecificationConfig(const std::map<std::string, std::string>& config)
  {
    spec_.config_ = config;
  }

  const robot_model::RobotModelConstPtr& getRobotModel() const
  {
    return spec_.state_space_->getRobotModel();
  }

  const robot_model::JointModelGroup* getJointModelGroup() const
  {
    return spec_.state_space_->getJointModelGroup();
  }

  const robot_state::RobotState& getCompleteInitialRobotState() const
  {
    return complete_initial_robot_state_;
  }

  const ModelBasedStateSpacePtr& getOMPLStateSpace() const
  {
    return spec_.state_space_;
  }

  const oc::SimpleSetupPtr& getOMPLSimpleSetup() const
  {
    return ompl_simple_setup_;
  }

  oc::SimpleSetupPtr& getOMPLSimpleSetup()
  {
    return ompl_simple_setup_;
  }

  const ot::Benchmark& getOMPLBenchmark() const
  {
    return ompl_benchmark_;
  }

  ot::Benchmark& getOMPLBenchmark()
  {
    return ompl_benchmark_;
  }

  const kinematic_constraints::KinematicConstraintSetPtr& getPathConstraints() const
  {
    return path_constraints_;
  }

  /* \brief Get the maximum number of sampling attempts allowed when sampling states is needed */
  unsigned int getMaximumStateSamplingAttempts() const
  {
    return max_state_sampling_attempts_;
  }

  /* \brief Set the maximum number of sampling attempts allowed when sampling states is needed */
  void setMaximumStateSamplingAttempts(unsigned int max_state_sampling_attempts)
  {
    max_state_sampling_attempts_ = max_state_sampling_attempts;
  }

  /* \brief Get the maximum number of sampling attempts allowed when sampling goals is needed */
  unsigned int getMaximumGoalSamplingAttempts() const
  {
    return max_goal_sampling_attempts_;
  }

  /* \brief Set the maximum number of sampling attempts allowed when sampling goals is needed */
  void setMaximumGoalSamplingAttempts(unsigned int max_goal_sampling_attempts)
  {
    max_goal_sampling_attempts_ = max_goal_sampling_attempts;
  }

  /* \brief Get the maximum number of valid goal samples to store */
  unsigned int getMaximumGoalSamples() const
  {
    return max_goal_samples_;
  }

  /* \brief Set the maximum number of valid goal samples to store */
  void setMaximumGoalSamples(unsigned int max_goal_samples)
  {
    max_goal_samples_ = max_goal_samples;
  }

  /* \brief Get the maximum number of planning threads allowed */
  unsigned int getMaximumPlanningThreads() const
  {
    return max_planning_threads_;
  }

  /* \brief Set the maximum number of planning threads */
  void setMaximumPlanningThreads(unsigned int max_planning_threads)
  {
    max_planning_threads_ = max_planning_threads;
  }

  /* \brief Get the maximum solution segment length */
  double getMaximumSolutionSegmentLength() const
  {
    return max_solution_segment_length_;
  }

  /* \brief Set the maximum solution segment length */
  void setMaximumSolutionSegmentLength(double mssl)
  {
    max_solution_segment_length_ = mssl;
  }

  unsigned int getMinimumWaypointCount() const
  {
    return minimum_waypoint_count_;
  }

  /** \brief Get the minimum number of waypoints along the solution path */
  void setMinimumWaypointCount(unsigned int mwc)
  {
    minimum_waypoint_count_ = mwc;
  }

  const constraint_samplers::ConstraintSamplerManagerPtr& getConstraintSamplerManager()
  {
    return spec_.constraint_sampler_manager_;
  }

  void setConstraintSamplerManager(const constraint_samplers::ConstraintSamplerManagerPtr& csm)
  {
    spec_.constraint_sampler_manager_ = csm;
  }

  void setVerboseStateValidityChecks(bool flag);

  void setProjectionEvaluator(const std::string& peval);

  void setPlanningVolume(const moveit_msgs::WorkspaceParameters& wparams);

  void setCompleteInitialState(const robot_state::RobotState& complete_initial_robot_state);

  bool setGoalConstraints(const std::vector<moveit_msgs::Constraints>& goal_constraints,
                          const moveit_msgs::Constraints& path_constraints, moveit_msgs::MoveItErrorCodes* error);
  bool setPathConstraints(const moveit_msgs::Constraints& path_constraints, moveit_msgs::MoveItErrorCodes* error);

  void setConstraintsApproximations(const ConstraintsLibraryConstPtr& constraints_library)
  {
    spec_.constraints_library_ = constraints_library;
  }

  bool useStateValidityCache() const
  {
    return use_state_validity_cache_;
  }

  void useStateValidityCache(bool flag)
  {
    use_state_validity_cache_ = flag;
  }

  bool simplifySolutions() const
  {
    return simplify_solutions_;
  }

  void simplifySolutions(bool flag)
  {
    simplify_solutions_ = flag;
  }

  /* @brief Solve the planning problem. Return true if the problem is solved
     @param timeout The time to spend on solving
     @param count The number of runs to combine the paths of, in an attempt to generate better quality paths
  */
  bool solve(double timeout, unsigned int count);

  /* @brief Benchmark the planning problem. Return true on succesful saving of benchmark results
     @param timeout The time to spend on solving
     @param count The number of runs to average in the computation of the benchmark
     @param filename The name of the file to which the benchmark results are to be saved (automatic names can be
     provided if a name is not specified)
  */
  bool benchmark(double timeout, unsigned int count, const std::string& filename = "");

  /* @brief Get the amount of time spent computing the last plan */
  double getLastPlanTime() const
  {
    return last_plan_time_;
  }

  /* @brief Get the amount of time spent simplifying the last plan */
  double getLastSimplifyTime() const
  {
    return last_simplify_time_;
  }

  /* @brief Apply smoothing and try to simplify the plan
     @param timeout The amount of time allowed to be spent on simplifying the plan*/
  void simplifySolution(double timeout);

  /* @brief Interpolate the solution*/
  void interpolateSolution();

  /* @brief Get the solution as a RobotTrajectory object*/
  bool getSolutionPath(robot_trajectory::RobotTrajectory& traj) const;

//void convertPath(const og::PathGeometric& pg, robot_trajectory::RobotTrajectory& traj) const;
  void convertPath(const oc::PathControl& pc, robot_trajectory::RobotTrajectory& traj) const;

  virtual void configure();

protected:
  void preSolve();
  void postSolve();

  void startSampling();
  void stopSampling();

  virtual ob::ProjectionEvaluatorPtr getProjectionEvaluator(const std::string& peval) const;
  virtual ob::StateSamplerPtr allocPathConstrainedSampler(const ompl::base::StateSpace* ss) const;
  virtual void useConfig();
  virtual ob::GoalPtr constructGoal();

  void registerTerminationCondition(const ob::PlannerTerminationCondition& ptc);
  void unregisterTerminationCondition();

  ModelBasedPlanningContextSpecification spec_;

  robot_state::RobotState complete_initial_robot_state_;

  /// the OMPL planning context; this contains the problem definition and the planner used
  // 将原本的ompl geo 变味了 ompl control 
  oc::SimpleSetupPtr ompl_simple_setup_;

  /// the OMPL tool for benchmarking planners
  ot::Benchmark ompl_benchmark_;

  /// tool used to compute multiple plans in parallel; this uses the problem definition maintained by ompl_simple_setup_
  ot::ParallelPlan ompl_parallel_plan_;

  std::vector<int> space_signature_;

  kinematic_constraints::KinematicConstraintSetPtr path_constraints_;
  moveit_msgs::Constraints path_constraints_msg_;
  std::vector<kinematic_constraints::KinematicConstraintSetPtr> goal_constraints_;

  const ob::PlannerTerminationCondition* ptc_;
  boost::mutex ptc_lock_;

  /// the time spent computing the last plan
  double last_plan_time_;

  /// the time spent simplifying the last plan
  double last_simplify_time_;

  /// maximum number of valid states to store in the goal region for any planning request (when such sampling is
  /// possible)
  unsigned int max_goal_samples_;

  /// maximum number of attempts to be made at sampling a state when attempting to find valid states that satisfy some
  /// set of constraints
  unsigned int max_state_sampling_attempts_;

  /// maximum number of attempts to be made at sampling a goal states
  unsigned int max_goal_sampling_attempts_;

  /// when planning in parallel, this is the maximum number of threads to use at one time
  unsigned int max_planning_threads_;

  /// the maximum length that is allowed for segments that make up the motion plan; by default this is 1% from the
  /// extent of the space
  double max_solution_segment_length_;

  /// the minimum number of points to include on the solution path (interpolation is used to reach this number, if
  /// needed)
  unsigned int minimum_waypoint_count_;

  bool use_state_validity_cache_;

  bool simplify_solutions_;
};
}

#endif

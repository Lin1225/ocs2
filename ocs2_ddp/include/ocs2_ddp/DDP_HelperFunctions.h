/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/penalties/MultidimensionalPenalty.h>
#include <ocs2_oc/oc_data/Metrics.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/oc_solver/PerformanceIndex.h>
#include <ocs2_oc/rollout/RolloutBase.h>

#include "ocs2_ddp/DDP_Data.h"

namespace ocs2 {

/**
 * Computes cost, soft constraints and constraints values of each point in the the primalSolution rollout.
 *
 * @param [in] problem: A reference to the optimal control problem.
 * @param [in] primalSolution: The primal solution.
 * @param [out] metrics: The cost, soft constraints and constraints values of the primalSolution rollout.
 */
void computeRolloutMetrics(OptimalControlProblem& problem, const PrimalSolution& primalSolution, Metrics& metrics);

/**
 * Calculates the PerformanceIndex associated to the input Metrics.
 *
 * @param [in] timeTrajectory: Time stamp of the rollout.
 * @param [in] metrics: The cost, soft constraints and constraints values of the primalSolution rollout.
 *
 * @return The PerformanceIndex of the trajectory.
 */
PerformanceIndex computeRolloutPerformanceIndex(const scalar_array_t& timeTrajectory, const Metrics& metrics);

/**
 * Forward integrate the system dynamics with given controller. It uses the given control policies and initial state,
 * to integrate the system dynamics in time period [initTime, finalTime].
 *
 * @param [in] rollout: A reference to the rollout class.
 * @param [in] initTime: Initial time
 * @param [in] initState: Initial state
 * @param [in] finalTime: Final time
 * @param [in] modeSchedule: The mode schedule
 * @param [in] controller: Control policies.
 * @param [out] primalSolution: The resulting primal solution.
 *
 * @return average time step.
 */
scalar_t rolloutTrajectory(RolloutBase& rollout, const scalar_t initTime, const vector_t& initState, const scalar_t finalTime,
                           const ModeSchedule& modeSchedule, LinearController& controller, PrimalSolution& primalSolution);

/**
 * Initializes the size of the ModelData for the intermediate and event times. Moreover, it extracts the time stamp,
 * and state-input dimnesions.
 *
 * @param [in] primalSolution: The resulting primal solution.
 * @param [out] modelDataTrajectory: Initialized array of ModelData for intermediate times.
 * @param [out] modelDataEventTimes: Initialized array of ModelData for event times.
 */
void initializeModelData(const PrimalSolution& primalSolution, std::vector<ModelData>& modelDataTrajectory,
                         std::vector<ModelData>& modelDataEventTimes);

/**
 * Computes the integral of the squared (IS) norm of the controller update.
 *
 * @param [in] controller: Input controller.
 * @return The integral of the squared (IS) norm of the controller update.
 */
scalar_t computeControllerUpdateIS(const LinearController& controller);

}  // namespace ocs2

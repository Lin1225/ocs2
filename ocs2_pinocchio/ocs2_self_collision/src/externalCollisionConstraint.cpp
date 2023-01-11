/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_self_collision/externalCollisionConstraint.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
externalCollisionConstraint::externalCollisionConstraint(const PinocchioStateInputMapping<scalar_t>& mapping,
                                                 PinocchioGeometryInterface pinocchioGeometryInterface, scalar_t minimumDistance)
    : StateConstraint(ConstraintOrder::Linear),
      externalCollision_(std::move(pinocchioGeometryInterface), minimumDistance),
      mappingPtr_(mapping.clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
externalCollisionConstraint::externalCollisionConstraint(const externalCollisionConstraint& rhs)
    : StateConstraint(rhs), externalCollision_(rhs.externalCollision_), mappingPtr_(rhs.mappingPtr_->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t externalCollisionConstraint::getNumConstraints(scalar_t time) const {
  return externalCollision_.getNumCollisionPairs();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t externalCollisionConstraint::getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
  const auto& pinocchioInterface = getPinocchioInterface(preComputation);
  return externalCollision_.getValue(pinocchioInterface);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation externalCollisionConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                  const PreComputation& preComputation) const {
  const auto& pinocchioInterface = getPinocchioInterface(preComputation);
  mappingPtr_->setPinocchioInterface(pinocchioInterface);

  VectorFunctionLinearApproximation constraint;
  matrix_t dfdq, dfdv;
  std::tie(constraint.f, dfdq) = externalCollision_.getLinearApproximation(pinocchioInterface);
  dfdv.setZero(dfdq.rows(), dfdq.cols());
  std::tie(constraint.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(state, dfdq, dfdv);
  return constraint;
}

}  // namespace ocs2

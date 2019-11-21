#include <ocs2_ddp/DDP_BASE.h>
#include <ocs2_ddp/DDP_Settings.h>

#include <ocs2_ddp/SLQ.h>
#include <ocs2_ddp/SLQ_DataCollector.h>
#include <ocs2_ddp/SLQ_Settings.h>

// riccati equations
#include <ocs2_ddp/riccati_equations/SequentialErrorEquationNormalized.h>
#include <ocs2_ddp/riccati_equations/SequentialRiccatiEquationsNormalized.h>

#include <ocs2_ddp/ILQR.h>
#include <ocs2_ddp/ILQR_Settings.h>

// dummy target for clang toolchain
int main() {
  return 0;
}

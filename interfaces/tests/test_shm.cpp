#include "avva_phase1_shm.hpp"
#include <type_traits>
static_assert(std::is_trivially_copyable_v<avva::shm::WorldStateFrame>);
static_assert(std::is_trivially_copyable_v<avva::shm::NpcControlBatch>);
int main(){}


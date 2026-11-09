#include "avva_phase1.hpp"
#include "avva_hash_v1.hpp"
#include <cassert>
int main(){
  avva::msg::FrameCompleteAck ack{};
  ack.ack_status=avva::msg::AckStatus::ACCEPTED;
  avva::hash_v1::Writer w; w.bounded_id("ego_0");
  auto h=avva::hash_v1::sha256(w.data);
  assert(h[0]==0xfd && h[31]==0x3a);
}


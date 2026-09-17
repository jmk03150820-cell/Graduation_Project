// AVVA canonical serializer and dependency-free SHA-256 reference.
#pragma once
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <span>
#include <stdexcept>
#include <string_view>
#include <tuple>
#include <vector>

namespace avva::hash_v1 {
using Bytes = std::vector<std::uint8_t>;
using Hash256 = std::array<std::uint8_t,32>;
using Uuid128 = std::array<std::uint8_t,16>;

struct Writer {
  Bytes data;
  Writer& raw(std::span<const std::uint8_t> v) { data.insert(data.end(),v.begin(),v.end()); return *this; }
  template<std::size_t N> Writer& ascii(const char (&v)[N]) { data.insert(data.end(),v,v+N-1); return *this; }
  Writer& ascii(std::string_view v) { data.insert(data.end(),v.begin(),v.end()); return *this; }
  Writer& u8(std::uint8_t v) { data.push_back(v); return *this; }
  Writer& u16(std::uint16_t v) { for(int i=0;i<2;++i)data.push_back(v>>(8*i)); return *this; }
  Writer& u32(std::uint32_t v) { for(int i=0;i<4;++i)data.push_back(v>>(8*i)); return *this; }
  Writer& u64(std::uint64_t v) { for(int i=0;i<8;++i)data.push_back(v>>(8*i)); return *this; }
  Writer& i64(std::int64_t v) { return u64(static_cast<std::uint64_t>(v)); }
  Writer& f64(double v) { if(!std::isfinite(v))throw std::invalid_argument("non-finite"); if(v==0)v=0; return u64(std::bit_cast<std::uint64_t>(v)); }
  Writer& uuid(const Uuid128& v) { return raw(v); }
  Writer& hash(const Hash256& v) { return raw(v); }
  Writer& bounded_id(std::string_view v) { if(v.empty()||v.size()>63)throw std::invalid_argument("BoundedId"); return u8(v.size()).ascii(v); }
  Writer& bounded_text(std::string_view v) { if(v.size()>255)throw std::invalid_argument("BoundedText255"); return u16(v.size()).ascii(v); }
};

inline Hash256 sha256(std::span<const std::uint8_t> input) {
  static constexpr std::uint32_t k[64]={
    0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
    0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
    0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
    0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
    0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
    0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
    0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
    0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};
  Bytes b(input.begin(),input.end()); const std::uint64_t bits=b.size()*8ull; b.push_back(0x80);
  while((b.size()%64)!=56) b.push_back(0);
  for(int i=7;i>=0;--i) b.push_back(bits>>(i*8));
  std::uint32_t h[8]={0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};
  auto rr=[](std::uint32_t x,int n){return (x>>n)|(x<<(32-n));};
  for(std::size_t o=0;o<b.size();o+=64){std::uint32_t w[64]{}; for(int i=0;i<16;++i)w[i]=(b[o+4*i]<<24)|(b[o+4*i+1]<<16)|(b[o+4*i+2]<<8)|b[o+4*i+3];
    for(int i=16;i<64;++i){auto s0=rr(w[i-15],7)^rr(w[i-15],18)^(w[i-15]>>3);auto s1=rr(w[i-2],17)^rr(w[i-2],19)^(w[i-2]>>10);w[i]=w[i-16]+s0+w[i-7]+s1;}
    auto a=h[0],c=h[2],d=h[3],e=h[4],f=h[5],g=h[6],hh=h[7],bb=h[1];
    for(int i=0;i<64;++i){auto S1=rr(e,6)^rr(e,11)^rr(e,25);auto ch=(e&f)^((~e)&g);auto t1=hh+S1+ch+k[i]+w[i];auto S0=rr(a,2)^rr(a,13)^rr(a,22);auto maj=(a&bb)^(a&c)^(bb&c);auto t2=S0+maj;hh=g;g=f;f=e;e=d+t1;d=c;c=bb;bb=a;a=t1+t2;}
    h[0]+=a;h[1]+=bb;h[2]+=c;h[3]+=d;h[4]+=e;h[5]+=f;h[6]+=g;h[7]+=hh;}
  Hash256 out{}; for(int i=0;i<8;++i)for(int j=0;j<4;++j)out[4*i+j]=h[i]>>(24-8*j); return out;
}

inline Bytes frame_complete_ack_payload(const Uuid128& decision,std::uint64_t tick,const Hash256& snapshot,std::uint8_t ack,std::uint16_t reason){
  Writer w; w.ascii("AVVA-PAYLOAD-V1\0").u16(23).uuid(decision).u64(tick).hash(snapshot).u8(ack).u16(reason); return w.data;
}
using ControlRow=std::tuple<std::string_view,std::uint8_t,std::uint8_t,std::uint8_t,std::uint8_t>;
inline Bytes control_set_digest_bytes(std::vector<ControlRow> rows){
  std::sort(rows.begin(),rows.end(),[](auto&a,auto&b){return std::get<0>(a)<std::get<0>(b);}); Writer w; w.ascii("AVVA-CONTROL-SET-V1\0").u32(rows.size());
  for(auto&[id,r,o,l,p]:rows) w.bounded_id(id).u8(r).u8(o).u8(l).u8(p);
  return w.data;
}
inline Bytes snapshot_bytes(const Uuid128& run,std::uint64_t epoch,std::string_view sim,std::uint64_t tick,const Hash256& digest,std::vector<Hash256> inputs){
  std::sort(inputs.begin(),inputs.end()); Writer w; w.ascii("AVVA-SNAPSHOT-V1\0").uuid(run).u64(epoch).bounded_id(sim).u64(tick).hash(digest).u32(inputs.size()); for(auto&h:inputs)w.hash(h); return w.data;
}
} // namespace avva::hash_v1

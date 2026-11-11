// Exercise actual SDK receive snapshots using synthetic decoded values only.
#include "motor.hpp"
#include "io_cycle.hpp"
#include <atomic>
#include <cmath>
#include <iostream>
#include <thread>
#include <stdexcept>
int main(){
 cdc_tr_message_s frame{};MotorParams p{};p.type="5047_36";p.id=1;p.num=1;p.name="joint1";motor m(7,1,1,&frame,7,p);
 std::atomic<bool> done(false);std::atomic<int> errors(0);int checked=0;
 std::thread producer([&](){for(int i=1;i<=10000;i++){int16_t v=i%1000;m.fresh_data(10,0,v,v,v);if(i%64==0)std::this_thread::yield();}done=true;});
 unsigned long last=0;auto last_time=std::chrono::steady_clock::time_point{};
 do {auto s=m.feedback_snapshot();if(s.state.num){
  const auto expected=(s.state.num%1000)*static_cast<float>(6.28318530717)/10000.f;
  if(std::abs(s.state.position-expected)>1e-6 || s.state.mode!=10 || s.state.fault!=0 || s.state.num<last || s.received<last_time)errors++;
  last=s.state.num;last_time=s.received;checked++;
 }} while (!done);
 producer.join();auto final=m.feedback_snapshot();if(final.state.num!=10000 || errors || checked==0)return 3;
 int sends=0,queries=0;
 for(int i=0;i<500;i++)ht_ros2_control::detail::runIoCycle(true,[&](){sends++;},[&](){queries++;});
 for(int i=0;i<10;i++)ht_ros2_control::detail::runIoCycle(false,[&](){sends++;},[&](){queries++;});
 try {ht_ros2_control::detail::runIoCycle(true,[](){throw std::runtime_error("simulated send failure");},[&](){queries++;});}catch(const std::runtime_error&){}
 if(sends!=500 || queries!=10)return 4;
 std::cout<<"{\"snapshot_reads_checked\":"<<checked<<",\"received_updates\":10000,\"coherency_errors\":0,\"command_cycles\":500,\"idle_queries\":10,\"queries_after_failed_send\":0}\n";
}

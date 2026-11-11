// In-memory motor serialization ONLY. Never constructs robot or serial_driver.
#include "motor.hpp"
#include <fstream>
#include <memory>
#include <random>
#include <vector>
#include <iostream>
int main(int argc,char**argv){
 if(argc!=2)return 2;
 cdc_tr_message_s tx{};std::vector<std::unique_ptr<motor>> motors;
 const char*types[]={"5047_36","6056_36","6056_36","5047_36","4438_30","4438_30","3536_32"};
 for(int i=0;i<7;i++){MotorParams p{};p.type=types[i];p.id=i+1;p.num=i+1;p.name="joint"+std::to_string(i+1);motors.emplace_back(std::make_unique<motor>(7,1,1,&tx,7,p));}
 std::mt19937 gen(71231);std::uniform_real_distribution<float>d(-1.f,1.f);std::ofstream out(argv[1],std::ios::binary);
 for(int n=0;n<128;n++){
  for(int i=0;i<7;i++){
   float q=d(gen)*1.5f,dq=d(gen)*.4f,tau=d(gen)*.8f,kp=10.f+70.f*std::abs(d(gen)),kd=.1f+4.f*std::abs(d(gen));
   if(n==0){q=0;dq=0;tau=0;kp=0;kd=0;}
   if(n==1){q=.1f*(i+1);dq=-.2f*(i+1);tau=.3f*(i+1);kp=40;kd=4;}
   motors[i]->pos_vel_tqe_kp_kd(q,dq,tau,kp,kd);
  }
  tx.head.s.crc8=Get_CRC8_Check_Sum(tx.head.data+1,3,255);tx.head.s.crc16=crc_ccitt(65535,tx.data.data,tx.head.s.len);
  out.write(reinterpret_cast<char*>(&tx),7+tx.head.s.len);
 }
 std::cout<<"128 frames, 896 motor references; no serial access\n";
}

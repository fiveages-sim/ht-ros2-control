"""No-I/O regression: compile actual SDK send bodies and actual pollOnce prefix.
No robot/serial construction, ROS lifecycle, USB or hardware access.
"""
from pathlib import Path
import argparse,hashlib,json,re,subprocess
p=argparse.ArgumentParser();p.add_argument('--source-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--expected-queries',type=int,default=500);p.add_argument('--cxx',default='g++');a=p.parse_args();root=a.source_root;out=a.output;out.mkdir(parents=True,exist_ok=True)
def extract(text,signature):
 start=text.index(signature);open_=text.index('{',start);depth=1;i=open_+1
 while depth:
  if text[i]=='{':depth+=1
  elif text[i]=='}':depth-=1
  i+=1
 return text[start:i]
sdk=root/'external/motor_cpp';bodies=[]
for file,functions in [('canport.cpp',['send_get_motor_state_cmd','send_get_motor_state_cmd2','motor_send_cmd']),('canboard.cpp',['send_get_motor_state_cmd','send_get_motor_state_cmd2','motor_send_cmd']),('robot.cpp',['send_get_motor_state_cmd','motor_send_cmd'])]:
 text=(sdk/'src/hardware'/file).read_text();cls=file[:-4]
 for fun in functions:bodies.append(extract(text,'void '+cls+'::'+fun+'('))
head='''#include <vector>
#include <map>
#include <iostream>
#include <cstring>
#include "serial_struct.hpp"
struct Sink { std::vector<int> sent; void send_2(cdc_tr_message_s* m) {sent.push_back(m->head.s.cmd);} };
struct canport { cdc_tr_message_s cdc_tr_message{}; Sink* ser; void send_get_motor_state_cmd();void send_get_motor_state_cmd2();void motor_send_cmd();};
struct canboard {std::vector<canport*> CANport;void send_get_motor_state_cmd();void send_get_motor_state_cmd2();void motor_send_cmd();};
struct motor {void velocity(float){}};
struct robot {int fun_v=5; bool motor_position_limit_flag=false,motor_torque_limit_flag=false;std::vector<canboard> CANboards;std::vector<motor*> Motors;void send_get_motor_state_cmd();void motor_send_cmd();};
'''

text=(root/'src/panthera_hardware_interface.cpp').read_text();body=extract(text,'void IoLoop::pollOnce()');prefix=body.split('// 读取电机状态缓存',1)[0];calls=re.findall(r'robot_\.(\w+)\(\);',prefix);assert calls and calls[0]=='send_get_motor_state_cmd'
code=head+'\n'.join(bodies)+'\nint main(){Sink sink;canport port{};port.ser=&sink;canboard board{};board.CANport.push_back(&port);robot robot_;robot_.CANboards.push_back(board);for(int i=0;i<500;i++){port.cdc_tr_message.head.s.cmd=0xb0;robot_.motor_send_cmd();'+''.join('robot_.'+f+'();' for f in calls)+'}std::map<int,int> c;for(int x:sink.sent)c[x]++;std::cout<<c[0xb0]<<" "<<c[0x0a]<<std::endl;}'
cpp=out/'query_send_oracle.cpp';cpp.write_text(code);exe=out/'query_send_oracle'
subprocess.run([a.cxx,'-std=c++17','-I'+str(sdk/'include'),'-I'+str(sdk/'include/crc'),str(cpp),'-o',str(exe)],check=True)
observed=subprocess.check_output([str(exe)],text=True).strip();mit,queries=map(int,observed.split());result={'MIT_frames_per_500_cycles':mit,'query_frames_per_500_cycles':queries,'poll_prefix_calls':calls,'generated_cpp_sha256':hashlib.sha256(cpp.read_bytes()).hexdigest(),'hardware_used':False,'scope':'Actual send function bodies and pollOnce call prefix with a counting serial sink; not hardware throughput, firmware behavior or complete IoLoop execution.'}
(out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result));assert mit==500 and queries==a.expected_queries,(observed,a.expected_queries)

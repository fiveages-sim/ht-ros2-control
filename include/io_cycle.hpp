#pragma once
namespace ht_ros2_control::detail {
// One owner performs exactly one operation per cycle. A failed send must not
// trigger an additional query on the same cycle.
template<class Send, class Query>
void runIoCycle(bool has_command, Send send, Query query) {
  if (has_command) send();
  else query();
}
}

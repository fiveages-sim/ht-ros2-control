# No-I/O query regression

Build with BUILD_TESTING=ON and run `ctest -R '^query_send_oracle$' --output-on-failure` in the package build directory. Or run `python3 tests/query_send_oracle.py --source-root . --output /tmp/query-check-NEW`.

The test compiles actual SDK query/send function bodies and the actual pollOnce call prefix against a counting serial sink. It expects 500 MIT frames and 500 query frames for 500 cycles. It opens no serial port, constructs no SDK robot/serial object and runs no controller. This is not a firmware/timing/full-IoLoop test. Existing baseline produces 1000 queries; `--expected-queries 1000` records that control case. Python3 and a host C++17 compiler are required; CTest timeout is 30 seconds.

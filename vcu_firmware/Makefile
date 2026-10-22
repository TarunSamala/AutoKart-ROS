CXX ?= g++
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra -Iinclude

.PHONY: test clean
test: /tmp/vcu_fw_test /tmp/vcu_fw_unverified_test /tmp/vcu_fw_estop_absent_test /tmp/vcu_fw_estop_fitted_test
	/tmp/vcu_fw_test
	/tmp/vcu_fw_unverified_test
	/tmp/vcu_fw_estop_absent_test
	/tmp/vcu_fw_estop_fitted_test

/tmp/vcu_fw_test: src/protocol.cpp src/vcu_logic.cpp test/test_vcu_logic.cpp include/*.h
	$(CXX) $(CXXFLAGS) -DVCU_OUTPUT_WIRING_VERIFIED=1 test/test_vcu_logic.cpp src/protocol.cpp src/vcu_logic.cpp -o $@

/tmp/vcu_fw_unverified_test: src/vcu_logic.cpp test/test_unverified_wiring.cpp include/*.h
	$(CXX) $(CXXFLAGS) test/test_unverified_wiring.cpp src/vcu_logic.cpp -o $@

/tmp/vcu_fw_estop_absent_test: src/vcu_logic.cpp test/test_estop_mode.cpp include/*.h
	$(CXX) $(CXXFLAGS) -DVCU_OUTPUT_WIRING_VERIFIED=1 -DESTOP_INPUT_FITTED=0 test/test_estop_mode.cpp src/vcu_logic.cpp -o $@

/tmp/vcu_fw_estop_fitted_test: src/vcu_logic.cpp test/test_estop_mode.cpp include/*.h
	$(CXX) $(CXXFLAGS) -DVCU_OUTPUT_WIRING_VERIFIED=1 -DESTOP_INPUT_FITTED=1 test/test_estop_mode.cpp src/vcu_logic.cpp -o $@

clean:
	rm -f /tmp/vcu_fw_test /tmp/vcu_fw_unverified_test /tmp/vcu_fw_estop_absent_test /tmp/vcu_fw_estop_fitted_test

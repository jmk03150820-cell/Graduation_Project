.PHONY: generate test clean
generate:
	python3 tools/generate_bindings.py
test: generate
	python3 tests/test_hash_vectors.py
	python3 -m py_compile generated/python/avva_phase1.py generated/python/avva_hash_v1.py
	$(CXX) -std=c++20 -Wall -Wextra -pedantic -Igenerated/cpp tests/test_cpp.cpp -o /tmp/avva_contract_cpp_test
	/tmp/avva_contract_cpp_test
	$(CXX) -std=c++20 -Wall -Wextra -pedantic -Igenerated/shm tests/test_shm.cpp -o /tmp/avva_contract_shm_test
	/tmp/avva_contract_shm_test
clean:
	rm -f /tmp/avva_contract_cpp_test /tmp/avva_contract_shm_test

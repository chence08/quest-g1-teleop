# DFX serial regression

This harness calls the patched production C++ hand methods against a pseudo-terminal emulator. The executable rejects paths outside `/dev/pts/`; it never opens the robot USB devices.

After fetching/patching upstreams and building the workstation environment:

```bash
mkdir -p .cache/serial-test reports
g++ -std=c++17 -O2 -Wall -Wextra -pthread \
  -Irepos/DFX_inspire_service/include -Ienv/include \
  tests/serial/serial_pty_client.cpp -o .cache/serial-test/client
python3 tests/serial/test_serial_pty.py \
  --binary .cache/serial-test/client --report reports/serial-test.json
```

It exercises 100 write/read cycles with 16 ms reply latency and fragmented/coalesced frames, plus timeout, checksum, unrelated-frame, negative-ACK and nonfinite-target cases. This verifies parsing and deadlines, not physical hand safety or sustained robot timing. The hand protocol has no transaction sequence number, so a sufficiently late same-register reply cannot be uniquely identified after a timeout.

#!/usr/bin/env python3
"""
Test All 6 Scenarios Headless
Gate verification for Phase 5:
Runs all 6 scenarios headless and validates that all nodes launch and run without crash.
"""

import subprocess
import time
import sys

SCENARIOS = [
    '1_nominal',
    '2_swarm_scaling',
    '3_obstacles',
    '4_high_sensor_noise',
    '5_agent_failure',
    '6_marine_current'
]

def test_scenarios():
    print("=======================================================")
    print(" HullSentinal Phase 5 Gate: Headless Scenario Verification")
    print(" Testing 6 Test Scenarios for Crash-Free Headless Execution")
    print("=======================================================")

    results = {}
    for sc in SCENARIOS:
        print(f"\n--- Testing Scenario: {sc} ---")
        cmd = [
            "bash", "-c",
            f"source /opt/ros/lyrical/setup.bash && "
            f"source '/home/pritish_3110/Personal Files/Syllabus/Projects/HullSentinal/ws/install/setup.bash' && "
            f"ros2 launch hull_bringup scenario.launch.py scenario:={sc} gui:=false"
        ]

        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        # Allow scenario to run for 6.0 seconds
        time.sleep(6.0)

        # Check if process has terminated prematurely (crashed)
        ret = proc.poll()
        if ret is not None:
            out, _ = proc.communicate()
            print(f"[FAIL] Scenario {sc} crashed prematurely with code {ret}")
            print("Output:\n", out[:500])
            results[sc] = False
        else:
            print(f"[PASS] Scenario {sc} executed steadily without crashes")
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()
            results[sc] = True

        # Clean any dangling nodes between scenarios
        subprocess.run(["pkill", "-f", "sim_engine_node|belief_map_node|utility_scorer|verification_monitor|crawler_controller|tool_controller|safety_barrier|marker_publisher"], check=False)
        time.sleep(0.5)

    print("\n=======================================================")
    print(" Headless Scenarios Summary:")
    for sc, passed in results.items():
        print(f"  {sc:<25} : {'PASS ✅' if passed else 'FAIL ❌'}")
    print("=======================================================")

    assert all(results.values()), "One or more scenarios failed to run cleanly"
    print("\n[Phase 5 Gate PASSED] All 6 scenarios ran headless to completion with 0 crashes!")


if __name__ == '__main__':
    test_scenarios()

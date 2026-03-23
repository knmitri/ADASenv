"""Run quick controller diagnostics across canonical scenarios."""

from KarlsPlayground.testing import ControllerTestEnv


def main() -> int:
    env = ControllerTestEnv(dt=0.05)

    results = []
    results.append(env.run_test(env.create_straight_path((0.0, 0.0), (10.0, 0.0), 20), "Straight", 500, verbose=True))
    results.append(env.run_test(env.create_s_curve_path((0.0, 0.0), 50), "S-Curve", 600, verbose=True))
    results.append(env.run_test(env.create_parking_maneuver((15.0, 15.0), (0.0, 8.0), 40), "Parking", 800, verbose=True))

    print("\n" + "=" * 70)
    print("TEST SUMMARY")
    print("=" * 70)
    passed = 0
    for result in results:
        ok = result["success"]
        passed += int(ok)
        status = "PASS" if ok else "FAIL"
        print(
            f"{status:4} {result['test_name']:10} steps={result['steps']:4d} "
            f"pos_err={result['final_pos_error']:.3f}m heading_err={result['final_heading_error']:.3f}rad"
        )

    print(f"\nTotal: {passed}/{len(results)} passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())

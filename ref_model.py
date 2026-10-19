
"""
Golden Python Reference Model
Project: Precision-Scalable 16x16 Systolic Tensor Unit

Modes: INT4, INT8, INT16

Software tests only. This does not verify Verilog RTL.
"""

import csv
import random
from pathlib import Path

import numpy as np

# ============================================================
# 1. DESIGN CONFIGURATION
# ============================================================

N = 16
RANDOM_SEED = 2026

PRECISION = {
    "INT4": 4,
    "INT8": 8,
    "INT16": 16,
}

OUTPUT_DIR = Path("golden_vectors")


# ============================================================
# 2. PRECISION AND WIDTH CALCULATIONS
# ============================================================

def get_operand_range(mode):
    if mode not in PRECISION:
        raise ValueError(f"Unsupported precision mode: {mode}")

    bits = PRECISION[mode]
    return -(1 << (bits - 1)), (1 << (bits - 1)) - 1


def get_accumulator_width(mode, terms=N):
    if mode not in PRECISION:
        raise ValueError(f"Unsupported precision mode: {mode}")
    if terms < 1:
        raise ValueError("terms must be at least 1")

    minimum, maximum = get_operand_range(mode)
    max_sum = terms * minimum * minimum
    min_sum = terms * minimum * maximum

    width = 1
    while not (
        -(1 << (width - 1)) <= min_sum
        and max_sum <= (1 << (width - 1)) - 1
    ):
        width += 1

    return width


# ============================================================
# 3. MATRIX AND MULTIPLICATION FUNCTIONS
# ============================================================

def constant_matrix(value):
    return [[value for _ in range(N)] for _ in range(N)]


def identity_matrix():
    return [
        [1 if i == j else 0 for j in range(N)]
        for i in range(N)
    ]


def random_matrix(mode, rng):
    minimum, maximum = get_operand_range(mode)
    return [
        [rng.randint(minimum, maximum) for _ in range(N)]
        for _ in range(N)
    ]


def validate_matrix(matrix, mode, name):
    if len(matrix) != N or any(len(row) != N for row in matrix):
        raise ValueError(f"{name} must be a {N}x{N} matrix")

    minimum, maximum = get_operand_range(mode)
    for row in matrix:
        for value in row:
            if not isinstance(value, int) or isinstance(value, bool):
                raise ValueError(f"{name} contains a non-integer value")
            if not minimum <= value <= maximum:
                raise ValueError(
                    f"{name} value {value} is outside "
                    f"{mode} range [{minimum}, {maximum}]"
                )


def matrix_multiply_python(A, B):
    """Independent reference implementation using Python integers."""
    C = [[0] * N for _ in range(N)]

    for i in range(N):
        for j in range(N):
            total = 0
            for k in range(N):
                total += A[i][k] * B[k][j]
            C[i][j] = total

    return C


def matrix_multiply_numpy(A, B):
    """NumPy checker. int64 is sufficient for these supported modes."""
    A_np = np.asarray(A, dtype=np.int64)
    B_np = np.asarray(B, dtype=np.int64)
    return (A_np @ B_np).tolist()


# ============================================================
# 4. TEST FUNCTIONS
# ============================================================

def run_test(name, A, B, mode, expected=None):
    try:
        validate_matrix(A, mode, "A")
        validate_matrix(B, mode, "B")

        actual = matrix_multiply_python(A, B)
        numpy_result = matrix_multiply_numpy(A, B)

        passed = actual == numpy_result

        if expected is not None:
            passed = passed and actual == expected

        print(f"  {name:<18}: {'PASS' if passed else 'FAIL'}")
        if not passed:
            print("    Custom model and/or expected result differs.")

        return passed, actual

    except (ValueError, TypeError, OverflowError) as error:
        print(f"  {name:<18}: FAIL ({error})")
        return False, None


def run_range_validation_tests(mode):
    minimum, maximum = get_operand_range(mode)
    zero = constant_matrix(0)
    tests_passed = True

    for label, invalid_value in (
        ("Below minimum", minimum - 1),
        ("Above maximum", maximum + 1),
    ):
        bad = [row[:] for row in zero]
        bad[0][0] = invalid_value

        try:
            validate_matrix(bad, mode, "Test matrix")
            print(f"  {label:<18}: FAIL (invalid value accepted)")
            tests_passed = False
        except ValueError:
            print(f"  {label:<18}: PASS")

    return tests_passed


# ============================================================
# 5. TEST SUITE FOR EACH PRECISION
# ============================================================

def run_mode_tests(mode, rng):
    minimum, maximum = get_operand_range(mode)

    Z = constant_matrix(0)
    I = identity_matrix()
    A = random_matrix(mode, rng)
    B = random_matrix(mode, rng)

    positive_expected = constant_matrix(N * 1 * 2)
    negative_expected = constant_matrix(N * (-1) * 2)
    mixed_sign_expected = constant_matrix(N * (-1) * 1)
    maximum_expected = constant_matrix(N * maximum * maximum)
    minimum_expected = constant_matrix(N * minimum * minimum)

    print(f"\n{mode} ({PRECISION[mode]}-bit signed arithmetic)")
    results = []

    passed, _ = run_test("Zero", Z, Z, mode, expected=Z)
    results.append(passed)

    passed, _ = run_test("Identity", A, I, mode, expected=A)
    results.append(passed)

    passed, _ = run_test(
        "Positive", constant_matrix(1), constant_matrix(2),
        mode, expected=positive_expected
    )
    results.append(passed)

    passed, _ = run_test(
        "Negative", constant_matrix(-1), constant_matrix(2),
        mode, expected=negative_expected
    )
    results.append(passed)

    passed, _ = run_test(
        "Mixed signs", constant_matrix(-1), constant_matrix(1),
        mode, expected=mixed_sign_expected
    )
    results.append(passed)

    passed, _ = run_test(
        "Maximum", constant_matrix(maximum), constant_matrix(maximum),
        mode, expected=maximum_expected
    )
    results.append(passed)

    passed, _ = run_test(
        "Minimum", constant_matrix(minimum), constant_matrix(minimum),
        mode, expected=minimum_expected
    )
    results.append(passed)

    passed, random_golden = run_test("Random", A, B, mode)
    results.append(passed)

    results.append(run_range_validation_tests(mode))

    print(f"  Accumulator width  : {get_accumulator_width(mode)} bits")
    return all(results), A, B, random_golden


# ============================================================
# 6. CSV EXPORT
# ============================================================

def save_matrix_csv(matrix, filename):
    with open(filename, "w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerows(matrix)


# ============================================================
# 7. MAIN PROGRAM
# ============================================================

def main():
    rng = random.Random(RANDOM_SEED)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    total_modes_passed = 0

    print("=" * 66)
    print("16x16 SYSTOLIC TENSOR UNIT")
    print("GOLDEN PYTHON REFERENCE MODEL + NUMPY CHECKER")
    print("=" * 66)
    print("These are software checks only; Verilog RTL is not run here.")

    for mode in PRECISION:
        passed, A, B, C_golden = run_mode_tests(mode, rng)

        if passed:
            total_modes_passed += 1

        save_matrix_csv(A, OUTPUT_DIR / f"A_{mode}.csv")
        save_matrix_csv(B, OUTPUT_DIR / f"B_{mode}.csv")
        save_matrix_csv(C_golden, OUTPUT_DIR / f"C_golden_{mode}.csv")

    print("\n" + "=" * 66)
    print(
        f"SOFTWARE SUMMARY: {total_modes_passed}/"
        f"{len(PRECISION)} precision modes passed."
    )
    print(f"Golden vectors saved in: {OUTPUT_DIR.resolve()}")
    print("RTL VERIFICATION: NOT RUN")
    print("=" * 66)

    if total_modes_passed != len(PRECISION):
        raise SystemExit("One or more software test groups failed.")


if __name__ == "__main__":
    main()

"""Tests for the shared slew-time model (the C3/C4 contract)."""

import math

import pytest

from acav.slew_model import (
    SlewLimits,
    is_triangular,
    off_nadir_angle_deg,
    peak_rate_rad_s,
    profile_error,
    slew_between_targets_rad,
    slew_time_s,
)

# ACAV parameters: ixx = 0.123, tau = 2 mNm, h = 20 mNms
ACAV = SlewLimits.from_axis(max_torque_Nm=0.002,
                            momentum_capacity_Nms=0.020,
                            inertia_kg_m2=0.123)


class TestLimits:
    def test_derived_values_match_the_yaml_comments(self):
        """The YAML documents these as [DERIVED]; confirm the code agrees
        so a reader editing inertia and forgetting the comment is still
        served correct numbers."""
        assert ACAV.alpha_rad_s2 == pytest.approx(0.016260, abs=1e-5)
        assert ACAV.omega_max_rad_s == pytest.approx(0.162602, abs=1e-5)
        assert ACAV.crossover_deg == pytest.approx(93.1, abs=0.5)

    def test_zero_inertia_rejected(self):
        with pytest.raises(ValueError):
            SlewLimits.from_axis(0.002, 0.020, 0.0)

    def test_negative_alpha_rejected(self):
        with pytest.raises(ValueError):
            SlewLimits(alpha_rad_s2=-1.0, omega_max_rad_s=1.0)


class TestProfileContinuity:
    def test_triangular_and_trapezoidal_agree_at_crossover(self):
        """A discontinuity here would put a step in the scheduler's
        feasibility boundary that would be very hard to attribute."""
        theta = ACAV.crossover_rad
        triangular = 2.0 * math.sqrt(theta / ACAV.alpha_rad_s2)
        trapezoidal = (ACAV.omega_max_rad_s / ACAV.alpha_rad_s2
                       + theta / ACAV.omega_max_rad_s)
        assert triangular == pytest.approx(trapezoidal, rel=1e-12)
        assert slew_time_s(theta, ACAV) == pytest.approx(triangular, rel=1e-12)

    def test_continuity_across_the_crossover_neighbourhood(self):
        eps = 1e-6
        below = slew_time_s(ACAV.crossover_rad - eps, ACAV)
        above = slew_time_s(ACAV.crossover_rad + eps, ACAV)
        assert below == pytest.approx(above, rel=1e-5)

    def test_all_acav_slews_are_triangular(self):
        """Every slew in this study is below the ~93 deg crossover; if
        this stops holding, the wheel sizing changed and the scenario's
        timing assumptions need revisiting."""
        for degrees in (13.5, 18.3, 18.8, 30.2, 30.5, 42.6, 50.0):
            assert is_triangular(math.radians(degrees), ACAV)


class TestSlewTime:
    def test_zero_slew_takes_zero_time(self):
        assert slew_time_s(0.0, ACAV) == 0.0

    def test_sign_is_irrelevant(self):
        assert slew_time_s(0.5, ACAV) == slew_time_s(-0.5, ACAV)

    def test_monotonic_in_angle(self):
        times = [slew_time_s(math.radians(d), ACAV)
                 for d in range(0, 90, 5)]
        assert times == sorted(times)

    def test_quadrupling_angle_doubles_time_in_triangular_regime(self):
        """t = 2*sqrt(theta/alpha), so 4x angle -> 2x time. A useful
        sanity property for anyone reasoning about the schedule."""
        t1 = slew_time_s(math.radians(10.0), ACAV)
        t4 = slew_time_s(math.radians(40.0), ACAV)
        assert t4 == pytest.approx(2.0 * t1, rel=1e-9)

    def test_representative_acav_slews(self):
        # ~42.6 deg is the largest slew in the scenario (T1 -> T2).
        assert slew_time_s(math.radians(42.6), ACAV) == pytest.approx(
            13.5, abs=0.3)
        # ~18.3 deg is a marginal-pair slew (T2 -> T3).
        assert slew_time_s(math.radians(18.3), ACAV) == pytest.approx(
            8.8, abs=0.3)

    def test_peak_rate_respects_momentum_limit(self):
        for degrees in range(1, 180, 7):
            rate = peak_rate_rad_s(math.radians(degrees), ACAV)
            assert rate <= ACAV.omega_max_rad_s + 1e-12


class TestGeometry:
    def test_off_nadir_zero_at_nadir(self):
        assert off_nadir_angle_deg(0.0, 500.0) == pytest.approx(0.0)

    def test_off_nadir_matches_flat_approximation_closely(self):
        """Spherical vs flat should differ by well under a degree at the
        distances this study uses -- but the spherical form is used
        because the difference grows with range and the scheduler's
        estimates feed a marginal-feasibility decision."""
        exact = off_nadir_angle_deg(240.0, 500.0)
        flat = math.degrees(math.atan(240.0 / 500.0))
        assert exact == pytest.approx(flat, abs=0.5)
        assert exact < flat   # curvature reduces the angle

    def test_opposite_sides_sum_angles(self):
        """T1 (-180 km) to T2 (+210 km) must sum, not difference. Getting
        this wrong systematically under-estimates the hardest slews."""
        combined = math.degrees(
            slew_between_targets_rad(-180.0, 210.0, 500.0))
        left = off_nadir_angle_deg(-180.0, 500.0)
        right = off_nadir_angle_deg(210.0, 500.0)
        assert combined == pytest.approx(left + right, rel=1e-9)
        assert combined == pytest.approx(42.6, abs=1.0)

    def test_same_side_differences_angles(self):
        combined = math.degrees(
            slew_between_targets_rad(-240.0, -60.0, 500.0))
        far = off_nadir_angle_deg(-240.0, 500.0)
        near = off_nadir_angle_deg(-60.0, 500.0)
        assert combined == pytest.approx(far - near, rel=1e-9)


class TestProfileError:
    def test_measurement_matching_model_is_zero_error(self):
        theta = math.radians(30.0)
        assert profile_error(slew_time_s(theta, ACAV), theta, ACAV) == \
            pytest.approx(0.0, abs=1e-12)

    def test_slower_than_model_is_positive(self):
        theta = math.radians(30.0)
        model = slew_time_s(theta, ACAV)
        assert profile_error(model * 1.25, theta, ACAV) == pytest.approx(0.25)

    def test_faster_than_model_is_negative_and_signals_a_bug(self):
        """The model is a lower bound. A negative profile_error means the
        twin exceeded its stated torque/momentum limits -- C5 asserts on
        this rather than celebrating a fast controller."""
        theta = math.radians(30.0)
        model = slew_time_s(theta, ACAV)
        assert profile_error(model * 0.8, theta, ACAV) < 0.0

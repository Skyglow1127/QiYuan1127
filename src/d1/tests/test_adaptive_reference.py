"""Partial commands must stay on the same approved joint-space segment."""
import numpy as np
import pytest

from d1_pick_place.adaptive_reference import admissible_reference


def test_saved_203022_j4_gap_can_take_a_smaller_approved_step():
    # 20261026_203022_380117/execution/01_fixed_pregrasp/executed/result.json
    previous = np.array([-.11361246717240547, -32.48196045269355,
        46.293529638734455, .0904046743362108, -12.88649422979055,
        -.7111718702721532])
    target = np.array([-.04948332828401547, -31.94410398531737,
        46.03769291941064, .043268009345571734, -13.684744921944443,
        -.3164724547206106])
    actual = np.array([-.20000000298023224, -32.20000076293945,
        46.79999923706055, .20000000298023224, -10.800000190734863,
        -1.2999999523162842])
    fraction, candidate = admissible_reference(previous, target, actual)
    assert fraction == pytest.approx(.516762418)
    assert candidate[4] == pytest.approx(-13.299000190734863)
    assert np.allclose(candidate, previous + fraction * (target - previous))
    assert np.max(np.abs(candidate - actual)) == pytest.approx(2.499)
    assert np.max(np.abs(np.round(candidate, 3) - actual)) <= 2.5


@pytest.mark.parametrize('joint', range(6))
@pytest.mark.parametrize('direction', [-1., 1.])
def test_every_axis_and_direction_can_limit_the_same_fraction(joint, direction):
    previous = np.full(6, direction * .3)
    delta = np.full(6, .2)
    delta[joint] = joint + 1.
    target = previous + direction * delta
    actual = previous.copy()
    actual[joint] = previous[joint] - direction * 1.
    fraction, candidate = admissible_reference(previous, target, actual)
    expected = 1.499 / abs(target[joint] - previous[joint])
    if expected >= 1.:
        assert fraction == 1.
        assert np.array_equal(candidate, target)
    else:
        assert fraction == pytest.approx(expected)
        assert np.allclose(candidate, previous + fraction * (target - previous))
        assert np.max(np.abs(candidate - actual)) <= 2.499 + 1e-12


@pytest.mark.parametrize('second_actual, expected', [(4., .3499), (7., None)])
def test_multiple_axes_intersect_both_lower_and_upper_fraction_bounds(second_actual, expected):
    previous = np.zeros(6)
    target = np.array([10., 10., 0., 0., 0., 0.])
    actual = np.array([1., second_actual, 0., 0., 0., 0.])
    result = admissible_reference(previous, target, actual)
    if expected is None:
        assert result is None
    else:
        fraction, candidate = result
        assert fraction == pytest.approx(expected)
        assert candidate[0] == candidate[1] == pytest.approx(3.499)


def test_turn_uses_only_the_current_segment_and_never_actual_as_origin():
    previous = np.array([3., 0., 0., 0., 0., 0.])
    target = np.array([3., 5., 0., 0., 0., 0.])
    actual = np.array([2., 0., .4, 0., 0., 0.])
    fraction, candidate = admissible_reference(previous, target, actual)
    assert fraction == pytest.approx(2.499 / 5.)
    assert np.array_equal(candidate[[0, 2, 3, 4, 5]], previous[[0, 2, 3, 4, 5]])
    assert candidate[1] == pytest.approx(2.499)


def test_stationary_axis_outside_admission_box_has_no_segment_intersection():
    previous = np.zeros(6)
    target = np.array([4., 0., 0., 0., 0., 0.])
    actual = np.array([0., 2.6, 0., 0., 0., 0.])
    assert admissible_reference(previous, target, actual) is None


def test_no_forward_room_does_not_reissue_previous_reference():
    previous = np.zeros(6)
    target = np.array([1., 0., 0., 0., 0., 0.])
    actual = np.array([-2.5, 0., 0., 0., 0., 0.])
    assert admissible_reference(previous, target, actual) is None


def test_subminimum_step_does_not_create_a_quantized_or_tiny_command_loop():
    previous = np.zeros(6)
    target = np.array([4., 0., 0., 0., 0., 0.])
    actual = np.array([-2.45, 0., 0., 0., 0., 0.])
    assert admissible_reference(previous, target, actual) is None


def test_path_can_enter_the_admission_box_even_if_previous_is_outside_it():
    previous = np.zeros(6)
    target = np.array([10., 0., 0., 0., 0., 0.])
    actual = np.array([4., 0., 0., 0., 0., 0.])
    fraction, candidate = admissible_reference(previous, target, actual)
    assert fraction == pytest.approx(.6499)
    assert candidate[0] == pytest.approx(6.499)


@pytest.mark.parametrize('distance', [0., .01, 2.5, 2.500009])
def test_admissible_full_target_is_returned_exactly_even_for_a_tiny_step(distance):
    previous = np.zeros(6)
    target = np.array([distance, 0., 0., 0., 0., 0.])
    fraction, candidate = admissible_reference(previous, target, np.zeros(6))
    assert fraction == 1.
    assert np.array_equal(candidate, target)
    candidate[0] = 99.
    assert target[0] == distance


@pytest.mark.parametrize('field', [0, 1, 2])
@pytest.mark.parametrize('invalid', [[0.] * 5, [[0.] * 6], [0.] * 5 + [np.nan],
    [0.] * 5 + [np.inf]])
def test_rejects_invalid_joint_vectors(field, invalid):
    vectors = [np.zeros(6), np.zeros(6), np.zeros(6)]
    vectors[field] = invalid
    with pytest.raises(ValueError, match='six finite'):
        admissible_reference(*vectors)


@pytest.mark.parametrize('kwargs', [dict(margin_deg=0), dict(margin_deg=np.inf),
    dict(margin_deg=True), dict(minimum_step_deg=0), dict(minimum_step_deg=np.nan),
    dict(rounding_margin_deg=-.1), dict(rounding_margin_deg=2.5)])
def test_rejects_invalid_admission_parameters(kwargs):
    with pytest.raises(ValueError, match='finite and valid'):
        admissible_reference(np.zeros(6), np.ones(6), np.zeros(6), **kwargs)

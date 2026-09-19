from s10_route_nav.common import approach


def test_approach_increases_with_limit():
    assert approach(0.0, 1.0, 0.2) == 0.2


def test_approach_decreases_with_limit():
    assert approach(1.0, 0.0, 0.2) == 0.8


def test_approach_does_not_overshoot():
    assert approach(0.9, 1.0, 0.2) == 1.0

"""Freshness-watchdog tests for the /s10_nav/healthy and /s10_nav/obstacle_state feeds.

The mux must fail closed when either upstream publisher stops sending, even if
its last published value was nominal. feed_is_stale only reads instance
attributes, so it is exercised through a lightweight stub (no rclpy node).
"""

import time

from s10_route_nav.command_mux import CommandMux


def make_stub(feed_timeout=0.5):
    stub = type('Stub', (), {})()
    stub.feed_timeout = feed_timeout
    stub.last_health = None
    stub.last_obstacle_state = None
    return stub


def test_stale_when_no_message_ever_received():
    mux = make_stub()
    assert CommandMux.feed_is_stale(mux)


def test_fresh_when_both_feeds_recent():
    mux = make_stub()
    now = time.monotonic()
    mux.last_health = now
    mux.last_obstacle_state = now
    assert not CommandMux.feed_is_stale(mux)


def test_stale_after_health_feed_stops():
    mux = make_stub()
    now = time.monotonic()
    mux.last_health = now - 1.0  # older than the 0.5 s timeout
    mux.last_obstacle_state = now
    assert CommandMux.feed_is_stale(mux)


def test_stale_after_obstacle_feed_stops():
    mux = make_stub()
    now = time.monotonic()
    mux.last_health = now
    mux.last_obstacle_state = now - 1.0
    assert CommandMux.feed_is_stale(mux)


def test_healthy_value_does_not_rescue_a_stale_feed():
    # A nominal last value (True / CLEAR) must not mask a dead publisher.
    mux = make_stub()
    now = time.monotonic()
    mux.last_health = now
    mux.last_obstacle_state = now - 1.0
    assert CommandMux.feed_is_stale(mux)


def test_boundary_inside_timeout_is_fresh():
    mux = make_stub(feed_timeout=0.5)
    now = time.monotonic()
    mux.last_health = now - 0.4
    mux.last_obstacle_state = now - 0.4
    assert not CommandMux.feed_is_stale(mux)

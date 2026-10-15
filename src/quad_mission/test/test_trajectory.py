"""The reference paths: exact derivatives, top speed, and start and end at rest."""
import numpy as np
import pytest

from quad_mission.trajectory import Trajectory


@pytest.mark.parametrize("shape", ["lemniscate", "circle"])
def test_velocity_and_acceleration_are_the_derivatives_of_the_position(shape):
    tr = Trajectory(shape, 2.0, 4.0, 1, [0.0, 0.0, 1.5])
    ts = np.arange(0.0, tr.duration, 1e-3)
    P, V, A = (np.array([tr.sample(t)[i] for t in ts]) for i in range(3))
    assert np.abs(np.gradient(P, ts, axis=0) - V).max() < 1e-4
    assert np.abs(np.gradient(V, ts, axis=0) - A).max() < 1e-2


@pytest.mark.parametrize("shape", ["lemniscate", "circle"])
def test_top_speed_is_the_requested_one(shape):
    tr = Trajectory(shape, 2.0, 5.0, 2, [0.0, 0.0, 1.5])
    t1, t2 = tr.top_speed_window()
    speeds = [np.linalg.norm(tr.sample(t)[1]) for t in np.linspace(t1, t2, 4000)]
    assert max(speeds) == pytest.approx(5.0, rel=1e-3)


def test_starts_and_ends_at_rest_at_the_start_point():
    start = np.array([1.0, -2.0, 1.5])
    tr = Trajectory("lemniscate", 2.0, 4.0, 2, start)
    for t in (0.0, tr.duration):
        p, v, a = tr.sample(t)
        assert np.allclose(p, start, atol=1e-9)
        assert np.allclose(v, 0.0, atol=1e-9)
        assert np.allclose(a, 0.0, atol=1e-9)


def test_laps_set_the_duration():
    one, three = (Trajectory("circle", 1.0, 2.0, laps, [0, 0, 1]) for laps in (1, 3))
    lap_time = 2 * np.pi * 1.0 / 2.0
    assert three.duration - one.duration == pytest.approx(2 * lap_time)


def test_unknown_shape_is_rejected():
    with pytest.raises(ValueError):
        Trajectory("square", 1.0, 1.0, 1, [0, 0, 1])

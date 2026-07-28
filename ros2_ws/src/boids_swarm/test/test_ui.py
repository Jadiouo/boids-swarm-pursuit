"""Unit tests for the control-panel widgets (v5 M15).

These exercise layout, hit-testing and value mapping only — the parts that
decide *what a click means*. They import boids_swarm.ui, which deliberately
keeps pygame out of module scope, so they run headless (and in CI, where
pygame is not installed).
"""

import pytest

from boids_swarm import ui


def _place(row, x=0, y=0, w=260):
    row.place(x, y, w)
    return row


# --- Cycler -----------------------------------------------------------------

def test_cycler_right_side_advances():
    c = _place(ui.Cycler('k', 'l', ui.AGENTS, ('a', 'b', 'c')))
    assert c.click((250, 10), 'a') == (ui.AGENTS, 'k', 'b')

def test_cycler_left_arrow_goes_back():
    c = _place(ui.Cycler('k', 'l', ui.AGENTS, ('a', 'b', 'c')))
    assert c.click((5, 10), 'b') == (ui.AGENTS, 'k', 'a')

def test_cycler_wraps_both_ways():
    c = _place(ui.Cycler('k', 'l', ui.SIM, ('a', 'b', 'c')))
    assert c.click((250, 10), 'c')[2] == 'a'      # forward off the end
    assert c.click((5, 10), 'a')[2] == 'c'        # backward off the front

def test_cycler_unknown_current_value_snaps_to_head():
    """A node launched with a value the panel doesn't list (or a typo'd
    param) must not raise — it selects the first option instead."""
    c = _place(ui.Cycler('k', 'l', ui.SIM, ('a', 'b')))
    assert c.click((250, 10), 'nonsense') == (ui.SIM, 'k', 'a')

def test_cycler_ignores_clicks_outside_its_row():
    c = _place(ui.Cycler('k', 'l', ui.SIM, ('a', 'b')), y=100)
    assert c.click((120, 10), 'a') is None


# --- Toggle -----------------------------------------------------------------

def test_toggle_inverts():
    t = _place(ui.Toggle('k', 'l', ui.SIM))
    assert t.click((20, 10), False) == (ui.SIM, 'k', True)
    assert t.click((20, 10), True) == (ui.SIM, 'k', False)

def test_toggle_returns_real_bool_not_numpy_or_int():
    """ParamBridge types the outgoing ParameterValue off the Python type,
    so a toggle must emit an actual bool or the receiving node rejects it."""
    t = _place(ui.Toggle('k', 'l', ui.SIM))
    _, _, value = t.click((20, 10), 0)
    assert value is True


# --- Slider -----------------------------------------------------------------

def test_slider_maps_ends_exactly():
    s = _place(ui.Slider('k', 'l', ui.AGENTS, 0.0, 5.0), w=260)
    assert s.value_at(-999) == 0.0
    assert s.value_at(9999) == 5.0

def test_slider_midpoint():
    s = _place(ui.Slider('k', 'l', ui.AGENTS, 0.0, 4.0), w=260)
    bx, bw = s._bar_x()
    assert s.value_at(bx + bw / 2) == pytest.approx(2.0, abs=1e-6)

def test_slider_quantizes_to_step():
    s = _place(ui.Slider('k', 'l', ui.SIM, 1.0, 3.0, step=0.05), w=260)
    bx, bw = s._bar_x()
    v = s.value_at(bx + bw * 0.37)
    assert v == pytest.approx(round(v / 0.05) * 0.05, abs=1e-9)
    assert 1.0 <= v <= 3.0

def test_slider_never_leaves_range_after_stepping():
    """Rounding to a step can push a value past the end; it must clamp."""
    s = _place(ui.Slider('k', 'l', ui.SIM, 0.0, 1.1, step=0.3), w=260)
    assert 0.0 <= s.value_at(9999) <= 1.1

def test_slider_frac_is_clamped():
    s = _place(ui.Slider('k', 'l', ui.SIM, 0.0, 2.0))
    assert s.frac(-5) == 0.0 and s.frac(99) == 1.0

def test_slider_emits_float_for_a_double_param():
    s = _place(ui.Slider('k', 'l', ui.SIM, 0.0, 5.0, step=1.0), w=260)
    assert isinstance(s.value_at(-999), float)


# --- Panel ------------------------------------------------------------------

def _panel():
    rows = [ui.Section('S'),
            ui.Cycler('strategy', 'strategy', ui.AGENTS, ('a', 'b')),
            ui.Toggle('trails', 'trails', ui.SIM),
            ui.Slider('w', 'w', ui.AGENTS, 0.0, 4.0),
            ui.Button('reset_episode', 'RESET')]
    p = ui.Panel(rows, 280)
    p.layout(800, 0)
    return p, rows

def test_layout_stacks_without_overlap_inside_the_panel():
    p, rows = _panel()
    for a, b in zip(rows, rows[1:]):
        assert b.y >= a.y + a.height
    for r in rows:
        assert r.x >= 800 and r.x + r.w <= 800 + p.width

def test_panel_routes_click_to_the_row_under_the_cursor():
    p, rows = _panel()
    toggle = rows[2]
    hit = (toggle.x + 5, toggle.y + 5)
    assert p.mouse_down(hit, lambda s, k: False) == (ui.SIM, 'trails', True)

def test_panel_click_on_a_gap_changes_nothing():
    p, rows = _panel()
    assert p.mouse_down((rows[0].x + 5, rows[0].y + 2),
                        lambda s, k: None) is None

def test_button_emits_an_action_and_needs_no_value():
    p, rows = _panel()
    btn = rows[4]
    def boom(scope, key):                      # must never be called
        raise AssertionError('Button asked for a value')
    assert p.mouse_down((btn.x + 5, btn.y + 5), boom) == \
        (ui.ACTION, 'reset_episode', None)

def test_drag_keeps_emitting_until_mouse_up():
    p, rows = _panel()
    sl = rows[3]
    p.mouse_down((sl.x + 5, sl.y + 5), lambda s, k: 0.0)
    scope, key, v1 = p.mouse_move((sl.x + 200, sl.y + 5))
    assert (scope, key) == (ui.AGENTS, 'w') and v1 > 0.0
    p.mouse_up()
    assert p.mouse_move((sl.x + 40, sl.y + 5)) is None

def test_drag_tracks_the_cursor_outside_the_row():
    """Sliders keep following the mouse once grabbed, even if the pointer
    wanders off the row — otherwise a fast drag silently stops updating."""
    p, rows = _panel()
    sl = rows[3]
    p.mouse_down((sl.x + 5, sl.y + 5), lambda s, k: 0.0)
    assert p.mouse_move((sl.x + 500, sl.y + 400))[2] == 4.0

def test_shipped_layout_has_no_duplicate_controls():
    rows = ui.build_rows(('auto', 'naive'), ('hull',), ('reactive',))
    keys = [(r.scope, r.key) for r in rows if r.interactive]
    assert len(keys) == len(set(keys))

def test_shipped_layout_fits_a_default_window():
    """The panel decides the window height, but it should still fit the
    800px default without the sim having to grow the window."""
    p = ui.Panel(ui.build_rows(('auto',), ('hull',), ('reactive',)), 280)
    assert p.layout(800, 0) <= 800

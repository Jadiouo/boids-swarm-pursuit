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

# --- shipped control panel (tabs, modes, restart fields) ----------------------

from boids_swarm import stack_config as sc  # noqa: E402
from boids_swarm.ui_state import PanelState  # noqa: E402

STRATS = ('auto', 'naive', 'intercept')
ENVS = sc.ENVS


def _shipped(width=360):
    p = ui.build_panel(STRATS, ('hull', 'tag'), sc.ALL_EVADERS, ENVS, width)
    p.layout(800, 0)
    return p


def _all_tab_rows(p):
    for name, rows in p.tabs.items():
        yield name, rows


def _value_for(row):
    """A plausible live value for any row type."""
    if isinstance(row, ui.Cycler):
        return row.options[0]
    if isinstance(row, ui.Toggle):
        return False
    if isinstance(row, ui.Slider):
        return row.lo
    if isinstance(row, ui.ModeBar):
        return {'current': None, 'enabled': True}
    if isinstance(row, ui.ButtonPair):
        return {}
    return 0


def _get_for(p):
    rows = {(r.scope, r.key): r for r in
            p.header + [r for t in p.tabs.values() for r in t] + p.footer}
    return lambda scope, key: _value_for(rows[(scope, key)])


def test_shipped_panel_fits_an_800px_window_in_every_tab():
    p = _shipped()
    assert p.height <= 800, p.height


def test_every_tab_lays_out_without_overlap_inside_the_panel():
    p = _shipped()
    for name, rows in _all_tab_rows(p):
        seq = p.header + rows + p.footer
        for a, b in zip(seq, seq[1:]):
            # the footer is parked below the tallest tab, so allow a gap
            assert b.y >= a.y + a.height, (name, a.label, b.label)
        for r in seq:
            assert r.x >= 800 and r.x + r.w <= 800 + p.width, (name, r.label)
            assert r.y >= 0 and r.y + r.height <= p.height, (name, r.label)


def test_footer_does_not_move_between_tabs():
    p = _shipped()
    y0 = p.footer[0].y
    for name in p.tabs:
        p.set_tab(name)
        p.layout(800, 0)
        assert p.footer[0].y == y0


def test_every_interactive_control_is_hit_by_its_own_centre():
    p = _shipped()
    get = _get_for(p)
    for name in p.tabs:
        p.set_tab(name)
        for r in p.rows:
            if not r.interactive:
                continue
            cx = r.x + r.w // 2 + 1
            if isinstance(r, ui.ButtonPair):         # centre is the gap
                sx, sw = r._slot(0)
                cx = sx + sw // 2
            pos = (cx, r.y + r.height // 2)
            assert p.mouse_down(pos, get) is not None or \
                isinstance(r, ui.TabBar), (name, r.label)
            p.mouse_up()


def test_clicking_a_tab_switches_it_and_sends_nothing():
    p = _shipped()
    names = list(p.tabs)
    tb = p.tab_bar
    sx, sw = tb._slot(2)
    assert p.mouse_down((sx + 3, tb.y + 5), lambda s, k: None) is None
    assert p.active == names[2]
    assert p.rows == p.header + p.tabs[names[2]] + p.footer


def test_controls_of_hidden_tabs_cannot_be_hit():
    p = _shipped()
    p.set_tab('Swarm')
    get = _get_for(p)
    hidden = p.tabs['Sensor'][3]            # a sensor slider/toggle
    hit_pos = (hidden.x + 5, hidden.y + 5)
    res = p.mouse_down(hit_pos, get)
    # whatever sits at that spot now belongs to the Swarm tab
    assert res is None or res[1] not in {r.key for r in p.tabs['Sensor']
                                         if r.interactive}


def test_no_duplicate_controls_across_the_whole_panel():
    p = _shipped()
    keys = [(r.scope, r.key) for rows in p.tabs.values() for r in rows
            if r.interactive]
    assert len(keys) == len(set(keys))
    only_keys = [k for _, k in keys]
    assert len(only_keys) == len(set(only_keys))


def test_restart_fields_are_exactly_the_ones_the_stack_cannot_change_live():
    p = _shipped()
    restart = {r.key for rows in p.tabs.values() for r in rows
               if r.restart}
    assert restart == {'perception', 'sharing_mode',
                       'shared_sighting_qos_depth', 'num_agents', 'env',
                       'seed'}
    assert restart <= set(sc.RESTART_KEYS)


def test_integer_slider_emits_ints_and_formats_without_decimals():
    s = _place(ui.Slider('n', 'n', ui.RESTART, 2, 20, 1, integer=True))
    v = s.value_at(s.x + s.w / 2)
    assert isinstance(v, int) and 2 <= v <= 20
    assert s.format(7) == '7'
    assert s.value_at(-5) == 2 and s.value_at(10**6) == 20


def test_slider_format_appends_the_unit():
    s = _place(ui.Slider('k', 'l', ui.SIM, 0, 5, 0.5, '%.1f', 'm'))
    assert s.format(2.5) == '2.5 m'


def test_mode_bar_click_selects_the_mode_under_the_cursor():
    bar = _place(ui.ModeBar([('baseline', 'B'), ('sensor_ros', 'S'),
                             ('nav2', 'N')]), w=340)
    for i, name in enumerate(('baseline', 'sensor_ros', 'nav2')):
        sx, sw = bar._slot(i)
        assert bar.click((sx + sw // 2, 5), None) == (ui.ACTION, 'mode', name)


def test_button_pair_routes_left_and_right_halves():
    pair = _place(ui.ButtonPair([('a', 'A', None), ('b', 'B', None)]),
                  w=340)
    assert pair.click((10, 5), {}) == (ui.ACTION, 'a', None)
    assert pair.click((330, 5), {}) == (ui.ACTION, 'b', None)


def test_wrap_text_respects_width_and_truncates_with_ellipsis():
    measure = lambda t: len(t) * 7       # noqa: E731  fake fixed-pitch font
    lines = ui.wrap_text('aaa bbb ccc ddd eee', 7 * 8, measure)
    assert all(measure(l_) <= 56 for l_ in lines)
    assert ' '.join(lines) == 'aaa bbb ccc ddd eee'
    cut = ui.wrap_text('word ' * 40, 7 * 20, measure, max_lines=2)
    assert len(cut) == 2 and cut[-1].endswith('...')
    assert measure(cut[-1]) <= 7 * 20


def test_every_mode_description_fits_the_description_box():
    """4 lines of the real panel width at ~7.5 px/char must hold each mode
    text, otherwise the explanation the user asked for gets truncated."""
    measure = lambda t: len(t) * 7.5     # noqa: E731
    width = 360 - 2 * ui.PAD
    for mode, text in sc.MODE_DESCRIPTIONS.items():
        lines = ui.wrap_text(text, width, measure)
        assert len(lines) <= 4, (mode, len(lines))


# --- PanelState: which clicks are sent, which are staged -----------------------

def _state(**kw):
    applied = {'perception': 'perfect', 'sharing_mode': 'legacy',
               'evader': 'reactive', 'shared_sighting_qos_depth': 10,
               'num_agents': 8, 'env': 'custom', 'seed': 7}
    applied.update(kw)
    return PanelState(applied, available=sc.ALL_EVADERS)


def test_live_edit_is_sent_immediately():
    st = _state()
    assert st.handle((ui.AGENTS, 'w_pursuit', 3.0)) == \
        ('send', ui.AGENTS, 'w_pursuit', 3.0)
    assert st.pending_count == 0


def test_restart_field_is_staged_not_sent_before_apply():
    st = _state()
    assert st.handle((ui.RESTART, 'num_agents', 12)) == ('stage',
                                                         'num_agents')
    assert st.staged == {'num_agents': 12}
    assert st.applied['num_agents'] == 8          # the running stack untouched
    assert st.value('num_agents') == 12           # but the panel shows it
    assert st.is_pending('num_agents')


def test_staging_the_running_value_again_clears_the_edit():
    st = _state()
    st.handle((ui.RESTART, 'seed', 9))
    st.handle((ui.RESTART, 'seed', 7))
    assert st.pending_count == 0


def test_apply_restarts_with_nothing_extra_and_commit_clears_staging():
    st = _state()
    st.handle((ui.RESTART, 'num_agents', 12))
    st.handle((ui.RESTART, 'env', 'pillar'))
    assert st.handle((ui.ACTION, 'apply', None)) == ('restart', {})
    cfg = st.target_cfg()
    assert cfg['num_agents'] == 12 and cfg['env'] == 'pillar'
    st.commit(cfg)
    assert st.pending_count == 0 and st.applied['num_agents'] == 12


def test_apply_with_nothing_staged_is_a_notice_not_a_restart():
    st = _state()
    kind, _ = st.handle((ui.ACTION, 'apply', None))
    assert kind == 'notice'


def test_mode_click_restarts_with_the_preset():
    st = _state()
    kind, ov = st.handle((ui.ACTION, 'mode', sc.SENSOR_ROS))
    assert kind == 'restart'
    assert ov['perception'] == 'sensor' and ov['sharing_mode'] == 'ros'


def test_mode_click_also_applies_staged_scene_edits():
    st = _state()
    st.handle((ui.RESTART, 'num_agents', 5))
    _, ov = st.handle((ui.ACTION, 'mode', sc.SENSOR_ROS))
    cfg = st.target_cfg(ov)
    assert cfg['num_agents'] == 5 and cfg['perception'] == 'sensor'


def test_ros_sharing_pulls_perception_to_sensor():
    st = _state()
    st.handle((ui.RESTART, 'sharing_mode', 'ros'))
    assert st.value('perception') == 'sensor'
    assert 'sensor' in st.notice


def test_perfect_perception_drops_ros_sharing():
    st = _state(perception='sensor', sharing_mode='ros')
    st.handle((ui.RESTART, 'perception', 'perfect'))
    assert st.value('sharing_mode') == 'legacy'


def test_switching_evader_between_plain_brains_is_live():
    st = _state()
    assert st.handle((ui.TARGET, 'evader', 'adaptive')) == \
        ('send', ui.TARGET, 'evader', 'adaptive')


def test_choosing_nav2_switches_to_the_nav2_mode():
    st = _state()
    kind, ov = st.handle((ui.TARGET, 'evader', 'nav2'))
    assert kind == 'restart'
    assert ov['evader'] == 'nav2' and ov['perception'] == 'sensor'


def test_leaving_nav2_restarts_because_the_nav2_processes_must_go():
    st = _state(perception='sensor', sharing_mode='ros', evader='nav2')
    assert st.handle((ui.TARGET, 'evader', 'adaptive')) == \
        ('restart', {'evader': 'adaptive'})


def test_unmanaged_sim_refuses_restart_actions_with_a_reason():
    st = _state()
    st.managed = False
    for change in ((ui.ACTION, 'mode', sc.NAV2), (ui.ACTION, 'apply', None)):
        st.staged['seed'] = 1
        kind, text = st.handle(change)
        assert kind == 'notice' and 're-launch' in text
    # live edits still work
    assert st.handle((ui.SIM, 'fov', 1.0))[0] == 'send'


def test_current_mode_follows_the_applied_config():
    st = _state()
    assert st.mode == sc.BASELINE
    st.commit(dict(st.applied, **sc.mode_preset(sc.NAV2, sc.ALL_EVADERS)))
    assert st.mode == sc.NAV2


# --- status text ---------------------------------------------------------------

from boids_swarm.ui_state import status_view  # noqa: E402


def test_status_view_covers_every_supervisor_state():
    base = {'warm_left': 0.0, 'eta': 0.0, 'detail': '', 'label': ''}
    seen = {}
    for st in ('idle', 'starting', 'warming', 'running', 'stopping',
               'failed'):
        v = status_view(dict(base, state=st, detail='boom\nlog line'), True,
                        mode_label='Baseline')
        assert v['text']
        seen[st] = v
    assert 'Running: Baseline' in seen['running']['text']
    assert seen['failed']['text'].startswith('FAILED: boom')
    assert seen['failed']['note'] == 'log line'


def test_status_view_warmup_countdown_and_progress():
    v = status_view({'state': 'starting', 'warm_left': 2.0, 'eta': 8.0,
                     'label': 'nav2', 'detail': ''}, True)
    assert 'Nav2' in v['text'] and '2s' in v['text']
    assert v['frac'] == 0.75


def test_status_view_unmanaged_explains_itself():
    v = status_view({}, False)
    assert v['state'] == 'unmanaged' and 'externally' in v['text']

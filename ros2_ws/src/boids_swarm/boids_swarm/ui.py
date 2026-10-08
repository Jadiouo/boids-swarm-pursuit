"""In-window control panel widgets (v5 / M15, tabbed + modes in the stack work).

The panel is **immediate mode**: it owns no values. Every frame it asks the
sim for the live value of each parameter and draws *that*. So a change made
with the mouse and a change made from a terminal (`ros2 param set`) can
never drift apart — there is exactly one source of truth, the ROS parameter.

Each row declares a `scope` saying *whose* parameter it is:

    SIM         the sim node itself            (capture_mode, render_trails)
    AGENTS      every /agentK/boid_controller  (pursuit_strategy, w_pursuit)
    TARGET      /target_controller             (evader)
    SIM_AGENTS  both sim and controllers       (radio_range)
    SIM_TARGET  both sim and target controller (target_speed_multiplier)
    RESTART     needs a stack restart: edits are only STAGED until the user
                presses "Apply & restart" (marked with a restart tag)
    ACTION      not a parameter — a one-shot command (mode, apply, reset)

Layout: a header (mode buttons, mode description, stack status, tab bar), the
active tab's fields, and a footer (Apply & restart, reset, pause). The
footer sits below the TALLEST tab so it never jumps when tabs change.

No pygame import at module scope: layout, hit-testing and value mapping are
pure math, so they unit-test without a display (the same lazy-import
discipline pygame_sim_node uses for its own drawing). Only `Panel.draw`
touches pygame, and it passes the module down to the rows.
"""

SIM = 'sim'
AGENTS = 'agents'
TARGET = 'target'
SIM_AGENTS = 'sim+agents'
SIM_TARGET = 'sim+target'
RESTART = 'restart'
ACTION = 'action'

PAD = 10
ROW_GAP = 4

# --- palette (matches the arena's dark theme) -------------------------------
BG = (22, 26, 40)
EDGE = (52, 60, 78)
FG = (222, 230, 240)
DIM = (128, 140, 158)
ACCENT = (80, 200, 255)
SECTION_FG = (255, 200, 90)
TRACK = (48, 56, 74)
HOVER = (38, 46, 64)
BTN = (54, 64, 86)
BTN_ON = (150, 90, 60)
WARN = (255, 170, 60)
OK = (90, 210, 130)
BAD = (255, 90, 80)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class _Row:
    """Base row. `place` assigns an absolute rect; `hit` tests it."""

    height = 24
    interactive = False
    needs_value = False

    def __init__(self, label, key=None, scope=None):
        self.label = label
        self.key = key
        self.scope = scope
        self.x = self.y = self.w = 0
        self.pending = False      # restart row with an un-applied edit

    @property
    def restart(self):
        return self.scope == RESTART

    def place(self, x, y, w):
        self.x, self.y, self.w = x, y, w

    def hit(self, pos):
        px, py = pos
        return (self.x <= px <= self.x + self.w
                and self.y <= py <= self.y + self.height)


class Section(_Row):
    """A group heading."""

    height = 26

    def __init__(self, label):
        super().__init__(label)

    def draw(self, pg, surf, fonts, value, hovered):
        surf.blit(fonts['bold'].render(self.label, True, SECTION_FG),
                  (self.x, self.y + 7))
        y = self.y + self.height - 3
        pg.draw.line(surf, EDGE, (self.x, y), (self.x + self.w, y), 1)


class ReadOnly(_Row):
    """A value the panel can show but not change (decided at launch)."""

    height = 22
    needs_value = True

    def __init__(self, key, label, scope=SIM):
        super().__init__(label, key, scope)

    def draw(self, pg, surf, fonts, value, hovered):
        surf.blit(fonts['small'].render(self.label, True, DIM),
                  (self.x, self.y + 4))
        txt = fonts['small'].render(f'{value}', True, DIM)
        surf.blit(txt, (self.x + self.w - txt.get_width(), self.y + 4))


def wrap_text(text, max_px, measure, max_lines=None):
    """Greedy word wrap. `measure(str) -> px` is the font's width function,
    so the same code works for the real font and for a fake in tests. When
    `max_lines` truncates the text the last line ends in '...'."""
    lines, cur = [], ''
    for word in text.split():
        trial = word if not cur else cur + ' ' + word
        if measure(trial) <= max_px or not cur:
            cur = trial
        else:
            lines.append(cur)
            cur = word
    if cur:
        lines.append(cur)
    if max_lines is not None and len(lines) > max_lines:
        lines = lines[:max_lines]
        last = lines[-1]
        while last and measure(last + '...') > max_px:
            last = last[:-1]
        lines[-1] = last.rstrip() + '...'
    return lines


def _restart_tag(pg, surf, fonts, x, y, pending):
    """Amber 'restart' marker (a drawn arrow-circle, not a glyph: a missing
    U+21BB renders as tofu on hosts without the font). Returns its width."""
    col = WARN if pending else DIM
    cx, cy = x + 5, y + 7
    pg.draw.arc(surf, col, (cx - 5, cy - 5, 11, 11), 0.9, 5.6, 1)
    pg.draw.polygon(surf, col, [(cx + 3, cy - 6), (cx + 7, cy - 1),
                                (cx + 1, cy - 1)])
    txt = fonts['tiny'].render('restart' if not pending else 'pending',
                               True, col)
    surf.blit(txt, (x + 14, y + 1))
    return 14 + txt.get_width()


def _label(pg, surf, fonts, row, x, y, color=DIM):
    """Row label plus the restart tag when the row needs one."""
    txt = fonts['small'].render(row.label + (f' [{row.unit}]'
                                             if getattr(row, 'unit', '')
                                             and not isinstance(row, Slider)
                                             else ''), True, color)
    surf.blit(txt, (x, y))
    if row.restart:
        _restart_tag(pg, surf, fonts, x + txt.get_width() + 8, y + 1,
                     row.pending)


class Cycler(_Row):
    """`label` over `◀ value ▶` for a fixed option list. Clicking the left
    arrow steps back, anywhere else steps forward."""

    height = 48
    interactive = True
    needs_value = True
    ARROW_W = 28
    BOX_Y = 19
    BOX_H = 26

    def __init__(self, key, label, scope, options, unit=''):
        super().__init__(label, key, scope)
        self.options = list(options)
        self.unit = unit

    def step(self, value, delta):
        try:
            i = self.options.index(value)
        except ValueError:
            i = 0                       # unknown current value: start at head
            delta = 0
        return self.options[(i + delta) % len(self.options)]

    def click(self, pos, value):
        if not self.hit(pos):
            return None
        delta = -1 if pos[0] <= self.x + self.ARROW_W else 1
        return (self.scope, self.key, self.step(value, delta))

    def draw(self, pg, surf, fonts, value, hovered):
        _label(pg, surf, fonts, self, self.x, self.y + 1)
        by = self.y + self.BOX_Y
        r = (self.x, by, self.w, self.BOX_H)
        pg.draw.rect(surf, HOVER if hovered else TRACK, r, border_radius=4)
        pg.draw.rect(surf, WARN if self.pending else EDGE, r, 1,
                     border_radius=4)
        # Arrows are drawn, not typed: SysFont('monospace') resolves to
        # whatever the host has, and a missing U+25C0 glyph renders as tofu.
        cy = by + self.BOX_H // 2
        pg.draw.polygon(surf, ACCENT, [(self.x + 8, cy),
                                       (self.x + 15, cy - 5),
                                       (self.x + 15, cy + 5)])
        rx = self.x + self.w - 8
        pg.draw.polygon(surf, ACCENT, [(rx, cy), (rx - 7, cy - 5),
                                       (rx - 7, cy + 5)])
        txt = fonts['body'].render(str(value), True,
                                   WARN if self.pending else FG)
        surf.blit(txt, (self.x + (self.w - txt.get_width()) // 2,
                        by + (self.BOX_H - txt.get_height()) // 2))


class Toggle(_Row):
    """`[x] label` boolean."""

    height = 26
    interactive = True
    needs_value = True
    BOX = 14

    def __init__(self, key, label, scope=SIM):
        # Explicit, so every value-carrying row is constructed (key, label,
        # scope). Inheriting _Row's (label, key, scope) silently swapped the
        # first two and shipped a control that set a nonexistent parameter.
        super().__init__(label, key, scope)

    def click(self, pos, value):
        if not self.hit(pos):
            return None
        return (self.scope, self.key, not bool(value))

    def draw(self, pg, surf, fonts, value, hovered):
        by = self.y + (self.height - self.BOX) // 2
        box = (self.x, by, self.BOX, self.BOX)
        pg.draw.rect(surf, ACCENT if value else TRACK, box, border_radius=3)
        pg.draw.rect(surf, EDGE if not hovered else ACCENT, box, 1,
                     border_radius=3)
        if value:
            pg.draw.lines(surf, BG, False,
                          [(self.x + 3, by + 7), (self.x + 6, by + 10),
                           (self.x + 11, by + 4)], 2)
        _label(pg, surf, fonts, self, self.x + self.BOX + 8, self.y + 5,
               FG if value else DIM)


class Slider(_Row):
    """`label ━━●━━ value unit`. The bar spans the full row width.
    `integer=True` emits ints (an int ROS parameter rejects a float)."""

    height = 38
    interactive = True
    needs_value = True
    BAR_Y = 26
    BAR_H = 5
    KNOB_R = 6

    def __init__(self, key, label, scope, lo, hi, step=0.0, fmt='%.2f',
                 unit='', integer=False):
        super().__init__(label, key, scope)
        self.lo, self.hi, self.step, self.fmt = lo, hi, step, fmt
        self.unit, self.integer = unit, integer
        if integer:
            self.fmt = '%d'
            self.step = max(1.0, step)

    def _bar_x(self):
        """Inset by the knob radius so the knob never clips the row edge."""
        return self.x + self.KNOB_R, max(self.w - 2 * self.KNOB_R, 1)

    def frac(self, value):
        span = self.hi - self.lo
        if span <= 0:
            return 0.0
        return _clamp((float(value) - self.lo) / span, 0.0, 1.0)

    def value_at(self, px):
        bx, bw = self._bar_x()
        f = _clamp((px - bx) / bw, 0.0, 1.0)
        v = self.lo + f * (self.hi - self.lo)
        if self.step > 0.0:
            v = round(v / self.step) * self.step
        v = _clamp(v, self.lo, self.hi)
        if self.integer:
            return int(round(v))
        return round(v, 6)

    def click(self, pos, value):
        if not self.hit(pos):
            return None
        return (self.scope, self.key, self.value_at(pos[0]))

    def format(self, value):
        txt = self.fmt % (int(value) if self.integer else float(value))
        return f'{txt} {self.unit}'.rstrip()

    def draw(self, pg, surf, fonts, value, hovered):
        _label(pg, surf, fonts, self, self.x, self.y + 2)
        vtxt = fonts['small'].render(self.format(value), True,
                                     WARN if self.pending else FG)
        surf.blit(vtxt, (self.x + self.w - vtxt.get_width(), self.y + 2))
        bx, bw = self._bar_x()
        by = self.y + self.BAR_Y
        pg.draw.rect(surf, TRACK, (bx, by, bw, self.BAR_H), border_radius=3)
        f = self.frac(value)
        col = WARN if self.pending else ACCENT
        pg.draw.rect(surf, col, (bx, by, int(bw * f), self.BAR_H),
                     border_radius=3)
        pg.draw.circle(surf, FG if hovered else col,
                       (int(bx + bw * f), by + self.BAR_H // 2), self.KNOB_R)


class Button(_Row):
    """One-shot command. `scope` is ACTION and `key` is the action name."""

    height = 30
    interactive = True

    def __init__(self, action, label, toggled_label=None):
        super().__init__(label, action, ACTION)
        self.toggled_label = toggled_label

    def click(self, pos, value):
        if not self.hit(pos):
            return None
        return (ACTION, self.key, None)

    def draw(self, pg, surf, fonts, value, hovered):
        on = bool(value)
        r = (self.x, self.y, self.w, self.height)
        pg.draw.rect(surf, BTN_ON if on else (BTN if not hovered else HOVER),
                     r, border_radius=4)
        pg.draw.rect(surf, EDGE, r, 1, border_radius=4)
        label = self.toggled_label if (on and self.toggled_label) else \
            self.label
        txt = fonts['body'].render(label, True, FG)
        surf.blit(txt, (self.x + (self.w - txt.get_width()) // 2,
                        self.y + (self.height - txt.get_height()) // 2))


class ApplyButton(Button):
    """'Apply & restart (n)': lit when edits are staged. The value is the
    number of pending edits."""

    needs_value = True
    height = 34

    def __init__(self):
        super().__init__('apply', 'APPLY & RESTART')

    def draw(self, pg, surf, fonts, value, hovered):
        n = int(value or 0)
        r = (self.x, self.y, self.w, self.height)
        if n:
            fill = (120, 84, 30) if not hovered else (150, 104, 36)
            edge = WARN
        else:
            fill, edge = (BTN if not hovered else HOVER), EDGE
        pg.draw.rect(surf, fill, r, border_radius=4)
        pg.draw.rect(surf, edge, r, 1, border_radius=4)
        label = f'APPLY & RESTART ({n})' if n else 'APPLY & RESTART'
        txt = fonts['body'].render(label, True, FG if n else DIM)
        surf.blit(txt, (self.x + (self.w - txt.get_width()) // 2,
                        self.y + (self.height - txt.get_height()) // 2))


class ButtonPair(_Row):
    """Two half-width buttons on one row (reset episode / pause). The value
    is a dict {action: toggled?} so a toggle button can show its state."""

    height = 30
    interactive = True
    needs_value = True
    GAP = 6

    def __init__(self, items):
        """items: [(action, label, toggled_label_or_None), x2]"""
        super().__init__('pair', 'buttons', ACTION)
        self.items = list(items)

    def _slot(self, i):
        w = (self.w - self.GAP * (len(self.items) - 1)) // len(self.items)
        return self.x + i * (w + self.GAP), w

    def click(self, pos, value):
        if not self.hit(pos):
            return None
        for i, (action, _, _) in enumerate(self.items):
            sx, sw = self._slot(i)
            if sx <= pos[0] <= sx + sw:
                return (ACTION, action, None)
        return None

    def draw(self, pg, surf, fonts, value, hovered):
        value = value or {}
        for i, (action, label, toggled) in enumerate(self.items):
            sx, sw = self._slot(i)
            on = bool(value.get(action))
            r = (sx, self.y, sw, self.height)
            pg.draw.rect(surf, BTN_ON if on else (
                BTN if not hovered else HOVER), r, border_radius=4)
            pg.draw.rect(surf, EDGE, r, 1, border_radius=4)
            txt = fonts['small'].render(toggled if (on and toggled)
                                        else label, True, FG)
            surf.blit(txt, (sx + (sw - txt.get_width()) // 2,
                            self.y + (self.height - txt.get_height()) // 2))


class ModeBar(_Row):
    """The three mode buttons. Value: {'current': mode|None, 'enabled': bool}.
    A click returns (ACTION, 'mode', <mode name>)."""

    height = 34
    interactive = True
    needs_value = True
    GAP = 4

    def __init__(self, modes):
        """modes: [(name, label), ...]"""
        super().__init__('modes', 'mode_bar', ACTION)
        self.modes = list(modes)

    def _slot(self, i):
        n = len(self.modes)
        w = (self.w - self.GAP * (n - 1)) // n
        return self.x + i * (w + self.GAP), w

    def click(self, pos, value):
        if not self.hit(pos):
            return None
        for i, (name, _) in enumerate(self.modes):
            sx, sw = self._slot(i)
            if sx <= pos[0] <= sx + sw:
                return (ACTION, 'mode', name)
        return None

    def draw(self, pg, surf, fonts, value, hovered):
        value = value or {}
        cur, enabled = value.get('current'), value.get('enabled', True)
        for i, (name, label) in enumerate(self.modes):
            sx, sw = self._slot(i)
            on = name == cur
            r = (sx, self.y, sw, self.height)
            fill = (36, 88, 118) if on else BTN
            if not enabled:
                fill = (34, 38, 50)
            pg.draw.rect(surf, fill, r, border_radius=5)
            pg.draw.rect(surf, ACCENT if on else EDGE, r, 2 if on else 1,
                         border_radius=5)
            txt = fonts['small'].render(label, True,
                                        FG if (enabled and on) else
                                        (DIM if not enabled else FG))
            surf.blit(txt, (sx + (sw - txt.get_width()) // 2,
                            self.y + (self.height - txt.get_height()) // 2))


class TextBlock(_Row):
    """Fixed-height wrapped paragraph; the value is the text."""

    needs_value = True

    def __init__(self, key, lines=4, scope=ACTION, line_h=15, color=DIM):
        super().__init__(key, key, scope)
        self.lines, self.line_h, self.color = lines, line_h, color
        self.height = lines * line_h + 2

    def draw(self, pg, surf, fonts, value, hovered):
        f = fonts['small']
        for i, ln in enumerate(wrap_text(str(value or ''), self.w,
                                         lambda t: f.size(t)[0],
                                         self.lines)):
            surf.blit(f.render(ln, True, self.color),
                      (self.x, self.y + i * self.line_h))


class StatusBar(_Row):
    """Stack status: coloured dot + state text, optional progress bar and a
    one-line note. Value: {'state','text','note','frac'}."""

    height = 40
    needs_value = True
    COLORS = {'running': OK, 'failed': BAD, 'idle': DIM, 'unmanaged': DIM}

    def __init__(self):
        super().__init__('status', 'status', ACTION)

    def draw(self, pg, surf, fonts, value, hovered):
        value = value or {}
        col = self.COLORS.get(value.get('state'), WARN)
        pg.draw.circle(surf, col, (self.x + 6, self.y + 9), 5)
        txt = fonts['small'].render(value.get('text', ''), True, FG)
        surf.blit(txt, (self.x + 18, self.y + 2))
        frac = value.get('frac')
        by = self.y + 20
        if frac is not None:
            pg.draw.rect(surf, TRACK, (self.x, by, self.w, 4),
                         border_radius=2)
            pg.draw.rect(surf, col, (self.x, by, int(self.w *
                                                      _clamp(frac, 0, 1)), 4),
                         border_radius=2)
        note = value.get('note', '')
        if note:
            f = fonts['tiny']
            line = wrap_text(note, self.w, lambda t: f.size(t)[0], 1)
            surf.blit(f.render(line[0] if line else '', True,
                               WARN if value.get('state') != 'failed'
                               else BAD), (self.x, self.y + 26))


class TabBar(_Row):
    """Row of tab headers. Clicking one switches tab (handled by
    ControlPanel; nothing reaches the sim)."""

    height = 28
    interactive = True
    needs_value = False

    def __init__(self, names):
        super().__init__('tabs', 'tab_bar', ACTION)
        self.names = list(names)
        self.active = self.names[0]

    def _slot(self, i):
        n = len(self.names)
        w = self.w // n
        return self.x + i * w, (w if i < n - 1 else self.w - w * (n - 1))

    def tab_at(self, pos):
        for i, name in enumerate(self.names):
            sx, sw = self._slot(i)
            if sx <= pos[0] <= sx + sw:
                return name
        return None

    def click(self, pos, value):
        if not self.hit(pos):
            return None
        return (ACTION, 'tab', self.tab_at(pos))

    def draw(self, pg, surf, fonts, value, hovered):
        for i, name in enumerate(self.names):
            sx, sw = self._slot(i)
            on = name == self.active
            r = (sx, self.y, sw - 2, self.height)
            pg.draw.rect(surf, (40, 52, 74) if on else TRACK, r,
                         border_top_left_radius=5, border_top_right_radius=5)
            pg.draw.line(surf, ACCENT if on else EDGE,
                         (sx, self.y + self.height - 1),
                         (sx + sw - 2, self.y + self.height - 1),
                         3 if on else 1)
            txt = fonts['small'].render(name, True, FG if on else DIM)
            surf.blit(txt, (sx + (sw - 2 - txt.get_width()) // 2,
                            self.y + (self.height - txt.get_height()) // 2))


class Panel:
    """Stacks rows in a fixed-width column and routes mouse events.

    `get(scope, key)` supplies the live value of any row; the panel never
    caches it. Mouse handlers return `(scope, key, value)` change requests
    (or None) for the caller to apply — the panel does no I/O itself.
    """

    def __init__(self, rows, width):
        self.rows = rows
        self.width = width
        self._drag = None
        self._hover = None
        self.height = 0

    def layout(self, x, y):
        cy = y + PAD
        inner = self.width - 2 * PAD
        for r in self.rows:
            r.place(x + PAD, cy, inner)
            cy += r.height + ROW_GAP
        self.height = cy - y + PAD - ROW_GAP
        return self.height

    # --- events ---------------------------------------------------------
    def mouse_down(self, pos, get):
        for r in self.rows:
            if not r.interactive or not r.hit(pos):
                continue
            if isinstance(r, Slider):
                self._drag = r
            value = get(r.scope, r.key) if r.needs_value else None
            return r.click(pos, value)
        return None

    def mouse_up(self):
        self._drag = None

    def mouse_move(self, pos):
        """Track hover; while dragging a slider, keep emitting new values."""
        self._hover = next((r for r in self.rows
                            if r.interactive and r.hit(pos)), None)
        if self._drag is None:
            return None
        return (self._drag.scope, self._drag.key,
                self._drag.value_at(pos[0]))

    # --- drawing --------------------------------------------------------
    def draw(self, pg, surf, x, y, fonts, get, is_pending=None):
        pg.draw.rect(surf, BG, (x, y, self.width, surf.get_height() - y))
        pg.draw.line(surf, EDGE, (x, y), (x, surf.get_height()), 1)
        for r in self.rows:
            if is_pending is not None and r.restart:
                r.pending = bool(is_pending(r.key))
            value = get(r.scope, r.key) if r.needs_value else None
            r.draw(pg, surf, fonts, value, r is self._hover)


class ControlPanel(Panel):
    """Header (modes, description, status, tab bar) + one tab's fields +
    footer. Only the active tab's rows exist for hit-testing and drawing."""

    def __init__(self, header, tabs, footer, width, tab_bar):
        self.header, self.tabs, self.footer = header, tabs, footer
        self.tab_bar = tab_bar
        self.active = tab_bar.active
        super().__init__(self._rows(), width)

    def _rows(self):
        return self.header + self.tabs[self.active] + self.footer

    def set_tab(self, name):
        if name not in self.tabs or name == self.active:
            return
        self.active = self.tab_bar.active = name
        self._drag = self._hover = None
        self.rows = self._rows()

    def _stack_height(self, rows):
        return sum(r.height for r in rows) + ROW_GAP * len(rows)

    def tab_height(self, name):
        return self._stack_height(self.tabs[name])

    def layout(self, x, y):
        inner = self.width - 2 * PAD
        cy = y + PAD
        for r in self.header:
            r.place(x + PAD, cy, inner)
            cy += r.height + ROW_GAP
        top = cy
        for name, rows in self.tabs.items():
            c = top
            for r in rows:
                r.place(x + PAD, c, inner)
                c += r.height + ROW_GAP
        cy = top + max(self.tab_height(n) for n in self.tabs)
        for r in self.footer:
            r.place(x + PAD, cy, inner)
            cy += r.height + ROW_GAP
        self.height = cy - y + PAD - ROW_GAP
        return self.height

    def mouse_down(self, pos, get):
        change = super().mouse_down(pos, get)
        if change is not None and change[:2] == (ACTION, 'tab'):
            self.set_tab(change[2])
            return None
        return change


def _tabs(strategies, capture_modes, evaders, envs):
    """The five tabs. Each field's range/unit comes from params.yaml and
    the code that reads it; `restart` rows (scope RESTART) are staged."""
    sensor = [
        Section('SENSOR  (live, perception=sensor)'),
        Cycler('perception', 'perception', RESTART, ('perfect', 'sensor')),
        Slider('fov', 'field of view (half-angle)', SIM, 0.2, 3.14159, 0.02,
               '%.2f', 'rad'),
        Slider('sensor_range', 'sensor range', SIM, 1.0, 20.0, 0.5, '%.1f',
               'm'),
        Toggle('occlusion_enabled', 'occlusion by bodies / obstacles', SIM),
        Slider('range_sigma', 'range noise', SIM, 0.0, 0.2, 0.005, '%.3f',
               'm/m'),
        Slider('bearing_sigma', 'bearing noise', SIM, 0.0, 0.2, 0.005,
               '%.3f', 'rad'),
        Slider('p_miss', 'miss probability', SIM, 0.0, 0.6, 0.01, '%.2f',
               ''),
    ]
    comms = [
        Section('COMMS / RELAY'),
        Cycler('sharing_mode', 'sharing mode', RESTART,
               ('legacy', 'off', 'oracle', 'ros')),
        Slider('shared_sighting_qos_depth', 'sighting QoS depth', RESTART,
               1, 50, 1, unit='msgs', integer=True),
        Slider('radio_range', 'radio range (ros)', SIM_AGENTS, 1.0, 25.0,
               0.5, '%.1f', 'm'),
        Slider('comm_range', 'comm range (oracle)', SIM, 1.0, 25.0, 0.5,
               '%.1f', 'm'),
        Slider('oracle_max_hops', 'oracle max hops (0=inf)', SIM, 0, 10, 1,
               unit='hops', integer=True),
        Slider('comm_jitter', 'relay jitter', SIM, 0.0, 1.0, 0.01, '%.2f',
               'm'),
        Slider('sighting_timeout', 'sighting timeout', AGENTS, 0.1, 3.0,
               0.05, '%.2f', 's'),
        Toggle('comms_enabled', 'legacy comms mesh', SIM),
    ]
    swarm = [
        Section('SWARM BEHAVIOR  (live, all boids)'),
        Cycler('pursuit_strategy', 'pursuit strategy', AGENTS, strategies),
        Slider('w_separation', 'separation weight', AGENTS, 0.0, 6.0, 0.1,
               '%.1f'),
        Slider('w_alignment', 'alignment weight', AGENTS, 0.0, 3.0, 0.05,
               '%.2f'),
        Slider('w_cohesion', 'cohesion weight', AGENTS, 0.0, 3.0, 0.05,
               '%.2f'),
        Slider('safe_distance', 'safe distance', AGENTS, 0.3, 4.0, 0.1,
               '%.1f', 'm'),
        Slider('sensing_radius', 'neighbour radius', AGENTS, 1.0, 15.0, 0.5,
               '%.1f', 'm'),
        Slider('w_pursuit', 'pursuit weight', AGENTS, 0.0, 5.0, 0.1,
               '%.1f'),
        Slider('lead_time', 'lead time', AGENTS, 0.0, 3.0, 0.1, '%.1f', 's'),
        Slider('ring_radius_start', 'ring radius', AGENTS, 1.0, 8.0, 0.1,
               '%.1f', 'm'),
        Slider('commit_distance', 'commit distance', AGENTS, 0.0, 6.0, 0.1,
               '%.1f', 'm'),
    ]
    target = [
        Section('TARGET / EVADER'),
        Cycler('evader', 'evader brain', TARGET, evaders),
        Slider('target_speed_multiplier', 'speed vs boids', SIM_TARGET, 1.0,
               3.0, 0.05, '%.2f', 'x'),
        Slider('target_omega_max', 'max turn rate', SIM_TARGET, 0.3, 3.0,
               0.05, '%.2f', 'rad/s'),
        Toggle('target_stamina_enabled', 'stamina limits sprinting', SIM),
        Slider('target_stamina_drain', 'stamina drain', SIM, 0.05, 1.0,
               0.01, '%.2f', '/s'),
        Slider('target_stamina_regen', 'stamina regen', SIM, 0.05, 1.0,
               0.01, '%.2f', '/s'),
    ]
    scene = [
        Section('SCENE'),
        Slider('num_agents', 'boids', RESTART, 2, 20, 1, unit='',
               integer=True),
        Cycler('env', 'environment', RESTART, envs),
        Slider('seed', 'seed', RESTART, 0, 99, 1, integer=True),
        Slider('episode_time_limit', 'episode time limit (0=none)', SIM, 0.0,
               300.0, 5.0, '%.0f', 's'),
        Cycler('capture_mode', 'capture rule', SIM, capture_modes),
        Slider('d_capture', 'capture distance', SIM, 0.5, 5.0, 0.1, '%.1f',
               'm'),
        Slider('capture_k', 'boids needed (hull)', SIM, 1, 6, 1,
               unit='boids', integer=True),
        Cycler('game_mode', 'target driver', SIM, ('ai', 'human')),
        Toggle('render_trails', 'draw trails', SIM),
    ]
    return {'Sensor': sensor, 'Comms': comms, 'Swarm': swarm,
            'Target': target, 'Scene': scene}


def build_panel(strategies, capture_modes, evaders, envs, width,
                mode_names=None):
    """The shipped control panel.

    `strategies` comes from behaviors.pursuit.STRATEGIES and `evaders`
    from the installed target_controller, so a new tactic or brain shows up
    in the GUI without touching this file."""
    from . import stack_config as sc
    modes = [(m, sc.MODE_SHORT[m]) for m in (mode_names or sc.MODES)]
    tabs = _tabs(strategies, capture_modes, evaders, envs)
    tab_bar = TabBar(list(tabs))
    header = [ModeBar(modes), TextBlock('mode_desc', lines=5),
              StatusBar(), tab_bar]
    footer = [ApplyButton(),
              ButtonPair([('reset_episode', 'RESET EPISODE', None),
                          ('toggle_pause', 'PAUSE', 'RESUME')])]
    return ControlPanel(header, tabs, footer, width, tab_bar)

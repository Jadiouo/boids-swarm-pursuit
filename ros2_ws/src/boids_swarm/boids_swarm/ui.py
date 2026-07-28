"""In-window control panel widgets (v5 / M15).

The panel is **immediate mode**: it owns no values. Every frame it asks the
sim for the live value of each parameter and draws *that*. So a change made
with the mouse and a change made from a terminal (`ros2 param set`) can
never drift apart — there is exactly one source of truth, the ROS parameter.

Each row declares a `scope` saying *whose* parameter it is:

    SIM      the sim node itself          (capture_mode, render_trails, ...)
    AGENTS   every /agentK/boid_controller (pursuit_strategy, w_pursuit, ...)
    TARGET   /target_controller            (evader)
    ACTION   not a parameter — a one-shot command (reset episode, pause)

No pygame import at module scope: layout, hit-testing and value mapping are
pure math, so they unit-test without a display (the same lazy-import
discipline pygame_sim_node uses for its own drawing). Only `Panel.draw`
touches pygame, and it passes the module down to the rows.
"""

SIM = 'sim'
AGENTS = 'agents'
TARGET = 'target'
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


class Cycler(_Row):
    """`◀ value ▶` over a fixed option list. Clicking the body advances."""

    height = 30
    interactive = True
    needs_value = True
    ARROW_W = 24

    def __init__(self, key, label, scope, options):
        super().__init__(label, key, scope)
        self.options = list(options)

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
        r = (self.x, self.y, self.w, self.height)
        pg.draw.rect(surf, HOVER if hovered else TRACK, r, border_radius=4)
        pg.draw.rect(surf, EDGE, r, 1, border_radius=4)
        # Arrows are drawn, not typed: SysFont('monospace') resolves to
        # whatever the host has, and a missing U+25C0 glyph renders as tofu.
        cy = self.y + self.height // 2
        pg.draw.polygon(surf, ACCENT, [(self.x + 8, cy),
                                       (self.x + 15, cy - 5),
                                       (self.x + 15, cy + 5)])
        rx = self.x + self.w - 8
        pg.draw.polygon(surf, ACCENT, [(rx, cy), (rx - 7, cy - 5),
                                       (rx - 7, cy + 5)])
        txt = fonts['body'].render(str(value), True, FG)
        surf.blit(txt, (self.x + (self.w - txt.get_width()) // 2,
                        self.y + 6))


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
        surf.blit(fonts['small'].render(self.label, True,
                                        FG if value else DIM),
                  (self.x + self.BOX + 8, self.y + 5))


class Slider(_Row):
    """`label ━━●━━ value`. The bar spans the full row width."""

    height = 36
    interactive = True
    needs_value = True
    BAR_Y = 24
    BAR_H = 5
    KNOB_R = 6

    def __init__(self, key, label, scope, lo, hi, step=0.0, fmt='%.2f'):
        super().__init__(label, key, scope)
        self.lo, self.hi, self.step, self.fmt = lo, hi, step, fmt

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
        return round(_clamp(v, self.lo, self.hi), 6)

    def click(self, pos, value):
        if not self.hit(pos):
            return None
        return (self.scope, self.key, self.value_at(pos[0]))

    def draw(self, pg, surf, fonts, value, hovered):
        surf.blit(fonts['small'].render(self.label, True, DIM),
                  (self.x, self.y + 2))
        vtxt = fonts['small'].render(self.fmt % float(value), True, FG)
        surf.blit(vtxt, (self.x + self.w - vtxt.get_width(), self.y + 2))
        bx, bw = self._bar_x()
        by = self.y + self.BAR_Y
        pg.draw.rect(surf, TRACK, (bx, by, bw, self.BAR_H), border_radius=3)
        f = self.frac(value)
        pg.draw.rect(surf, ACCENT, (bx, by, int(bw * f), self.BAR_H),
                     border_radius=3)
        pg.draw.circle(surf, FG if hovered else ACCENT,
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
                        self.y + 6))


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
    def draw(self, pg, surf, x, y, fonts, get):
        pg.draw.rect(surf, BG, (x, y, self.width, surf.get_height() - y))
        pg.draw.line(surf, EDGE, (x, y), (x, surf.get_height()), 1)
        for r in self.rows:
            value = get(r.scope, r.key) if r.needs_value else None
            r.draw(pg, surf, fonts, value, r is self._hover)


def build_rows(strategies, capture_modes, evaders):
    """The shipped panel layout.

    `strategies` comes from behaviors.pursuit.STRATEGIES so a new tactic
    shows up in the GUI without touching this file.
    """
    return [
        Section('PURSUIT'),
        Cycler('pursuit_strategy', 'strategy', AGENTS, strategies),
        Slider('w_pursuit', 'w_pursuit', AGENTS, 0.0, 5.0, 0.1, '%.1f'),
        Slider('commit_distance', 'commit dist', AGENTS, 0.0, 6.0, 0.1,
               '%.1f'),
        Slider('ring_radius_start', 'ring radius', AGENTS, 1.0, 8.0, 0.1,
               '%.1f'),

        Section('TARGET'),
        Cycler('evader', 'brain', TARGET, evaders),
        Slider('target_speed_multiplier', 'speed x', SIM, 1.0, 3.0, 0.05,
               '%.2f'),
        Slider('target_omega_max', 'turn rate', SIM, 0.3, 3.0, 0.05, '%.2f'),
        Toggle('target_stamina_enabled', 'stamina', SIM),

        Section('GAME'),
        Cycler('capture_mode', 'capture', SIM, capture_modes),
        Slider('d_capture', 'd_capture', SIM, 0.5, 5.0, 0.1, '%.1f'),
        Cycler('game_mode', 'driver', SIM, ('ai', 'human')),

        Section('WORLD / VIEW'),
        Toggle('render_trails', 'trails', SIM),
        Toggle('comms_enabled', 'comms mesh', SIM),

        Button('reset_episode', 'RESET EPISODE'),
        Button('toggle_pause', 'PAUSE', toggled_label='RESUME'),

        ReadOnly('perception', 'perception', SIM),
        ReadOnly('env', 'env', SIM),
        ReadOnly('agents', 'agents', SIM),
    ]

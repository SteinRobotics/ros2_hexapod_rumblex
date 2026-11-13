"""Tk desktop editor for sequence_generator. Imported only for GUI sessions."""
import bisect
import copy
import math
from pathlib import Path
import time
import tkinter as tk
from tkinter import filedialog, font, messagebox, simpledialog, ttk

import yaml

from sequence_generator import (
    CURVES, GAITS, LEGS, PRESETS, Sequence, arc, builtin, hold,
    load_yaml, move, pose, preset, repeat, route_preview, sequence_notes, stop,
    travel, turn, validate_action,
)

BG = '#101820'
PANEL = '#17232e'
INPUT = '#213241'
TEXT = '#e4edf3'
MUTED = '#9eb2c3'
ACCENT = '#60dbc4'
GOLD = '#efbb72'

KINDS = ('Travel', 'Turn', 'Arc', 'Velocity', 'Pose offset', 'Hold', 'Save pose',
         'Restore pose', 'Stand up', 'Lay down', 'Stop', 'Say', 'Music',
         'Hold distance', 'Level torso')


def describe(action):
    if 'loop' in action:
        loop = action['loop']
        return f'Repeat ×{loop["times"]} · {len(loop["actions"])} blocks'
    movement = action.get('movement', {})
    name = movement.get('name', '')
    if name == 'SINGLE_POSE':
        if movement.get('restore_pose'):
            return 'Restore saved pose'
        fields = movement.get('pose_delta', movement.get('body_pose', movement.get('joint_targets', {})))
        return ('Save + ' if movement.get('save_pose') else '') + 'Pose · ' + ', '.join(fields)
    if 'velocity' in action and name in GAITS:
        v = action['velocity']
        return (f'{name.removeprefix("MOVE_").title()} · '
                f'x {v.get("linear", {}).get("x", 0):g}, y {v.get("linear", {}).get("y", 0):g} m/s · '
                f'ω {v.get("angular", {}).get("z", 0):g}')
    if name:
        return name.removeprefix('SEQUENCE_').replace('_', ' ').title()
    return ' + '.join(action).title()


def block_duration(action):
    if 'loop' in action:
        return action['loop']['times'] * sum(block_duration(a) for a in action['loop']['actions'])
    return validate_action(action)


class SequenceEditor:
    def __init__(self, root, sequence, path=None):
        self.root = root
        self.sequence = copy.deepcopy(sequence)
        self.path = Path(path) if path else None
        self.saved = self.snapshot()
        self.undo_stack = []
        self.redo_stack = []
        self.selected = None
        self.loading = False
        self.points = []
        self.point_times = []
        self.playing = False
        self.timer = None
        self.last_tick = 0
        self.pending_refresh = None
        self.root.title('RumbleX · Sequence Studio')
        self.root.geometry('1600x1000')
        self.root.minsize(1080, 720)
        self.root.configure(bg=BG)
        self.text_size = tk.StringVar(value='20')
        self._style()
        self._layout()
        self.refresh()
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        for key, command in (('<Control-s>', self.export), ('<Control-o>', self.open_file),
                             ('<Control-n>', self.new), ('<Control-z>', self.undo),
                             ('<Control-y>', self.redo)):
            self.root.bind(key, lambda event, callback=command: self._shortcut(callback))

    def _shortcut(self, callback):
        callback()
        return 'break'

    def _style(self):
        # Geometry and canvas are in pixels. Match fonts to those units even on
        # XWayland desktops where fontconfig DPI and Tk's screen DPI disagree.
        probe = font.Font(self.root, family='DejaVu Sans', size=-13)
        line_height = probe.metrics('linespace')
        if line_height > 20 or line_height < 12:
            scaling = float(self.root.tk.call('tk', 'scaling'))
            self.root.tk.call('tk', 'scaling', scaling * line_height / 16)
        self.ui_font = font.Font(self.root, family='DejaVu Sans', size=-20)
        self.heading_font = font.Font(self.root, family='DejaVu Sans', size=-20, weight='bold')
        self.title_font = font.Font(self.root, family='DejaVu Sans', size=-32, weight='bold')
        self.code_font = font.Font(self.root, family='DejaVu Sans Mono', size=-18)
        self.map_font = font.Font(self.root, family='DejaVu Sans', size=-18)
        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure('.', background=PANEL, foreground=TEXT, font=self.ui_font)
        style.configure('TFrame', background=PANEL)
        style.configure('TLabel', background=PANEL)
        style.configure('Muted.TLabel', foreground=MUTED)
        style.configure('Title.TLabel', font=self.title_font, foreground=ACCENT)
        style.configure('Heading.TLabel', font=self.heading_font)
        style.configure('TButton', padding=(10, 7), background=INPUT, borderwidth=0)
        style.map('TButton', background=[('active', '#345164')])
        style.configure('Accent.TButton', background=ACCENT, foreground=BG)
        style.map('Accent.TButton', background=[('active', '#8bead8')])
        style.configure('TEntry', fieldbackground=INPUT, insertcolor=TEXT, padding=5)
        style.configure('TCombobox', fieldbackground=INPUT, arrowcolor=ACCENT, padding=4)
        style.map('TCombobox', fieldbackground=[('readonly', INPUT)], foreground=[('readonly', TEXT)])
        style.configure('TNotebook', borderwidth=0)
        style.configure('TNotebook.Tab', padding=(13, 8), background=INPUT)
        style.map('TNotebook.Tab', background=[('selected', PANEL)], foreground=[('selected', ACCENT)])
        style.configure('Treeview', background=PANEL, fieldbackground=PANEL, rowheight=34, borderwidth=0)
        style.configure('Treeview.Heading', background=INPUT, padding=6)
        style.map('Treeview', background=[('selected', '#31546a')])
        style.configure('TScale', background=PANEL, troughcolor=INPUT)
        self.root.option_add('*TCombobox*Listbox.background', INPUT)
        self.root.option_add('*TCombobox*Listbox.foreground', TEXT)
        self.resize_text()

    def resize_text(self, event=None):
        size = int(self.text_size.get())
        for name in ('TkDefaultFont', 'TkTextFont', 'TkMenuFont', 'TkHeadingFont'):
            font.nametofont(name).configure(family='DejaVu Sans', size=-size)
        self.ui_font.configure(size=-size)
        self.heading_font.configure(size=-size)
        self.title_font.configure(size=-(size + 12))
        self.code_font.configure(size=-(size - 2))
        self.map_font.configure(size=-(size - 2))
        ttk.Style(self.root).configure('Treeview', rowheight=self.ui_font.metrics('linespace') + 14)
        if hasattr(self, 'canvas'):
            self.root.after_idle(self.draw_route)

    def _layout(self):
        shell = ttk.Frame(self.root, padding=18)
        shell.pack(fill='both', expand=True)
        header = ttk.Frame(shell)
        header.pack(fill='x', pady=(0, 14))
        ttk.Label(header, text='RumbleX / Sequence Studio', style='Title.TLabel').pack(side='left')
        ttk.Label(header, text='OFFLINE AUTHORING', foreground=GOLD).pack(side='right')
        toolbar = ttk.Frame(shell)
        toolbar.pack(fill='x', pady=(0, 12))
        for label, command in (('New', self.new), ('Open YAML', self.open_file),
                               ('Undo', self.undo), ('Redo', self.redo)):
            ttk.Button(toolbar, text=label, command=command).pack(side='left', padx=(0, 6))
        ttk.Label(toolbar, text='Text size').pack(side='left', padx=(12, 6))
        size_choice = ttk.Combobox(toolbar, textvariable=self.text_size,
                                  values=('16', '18', '20', '22', '24', '28'), state='readonly', width=3)
        size_choice.pack(side='left')
        size_choice.bind('<<ComboboxSelected>>', self.resize_text)
        ttk.Button(toolbar, text='Export YAML…', style='Accent.TButton',
                   command=self.export).pack(side='right')
        metadata = ttk.Frame(shell)
        metadata.pack(fill='x', pady=(0, 14))
        self.name = tk.StringVar(value=self.sequence.name)
        self.voice = tk.StringVar(value=(self.sequence.trigger or {}).get('voice', ''))
        self.joystick = tk.StringVar(value=(self.sequence.trigger or {}).get('joystick', ''))
        for index, (label, variable) in enumerate((('Sequence', self.name), ('Voice trigger', self.voice),
                                                   ('Joystick trigger', self.joystick))):
            ttk.Label(metadata, text=label, style='Muted.TLabel').grid(row=0, column=index * 2, padx=(0, 8))
            ttk.Entry(metadata, textvariable=variable).grid(row=0, column=index * 2 + 1, sticky='ew', padx=(0, 16))
            metadata.columnconfigure(index * 2 + 1, weight=1)
            variable.trace_add('write', self.metadata_changed)
        self.status = tk.StringVar()
        ttk.Label(shell, textvariable=self.status, style='Muted.TLabel').pack(side='bottom', anchor='w', pady=(12, 0))
        panes = ttk.Panedwindow(shell, orient='horizontal')
        panes.pack(fill='both', expand=True)
        sidebar = ttk.Frame(panes, width=335)
        middle = ttk.Frame(panes, width=465, padding=(12, 0))
        right = ttk.Frame(panes, width=550)
        panes.add(sidebar, weight=0)
        panes.add(middle, weight=2)
        panes.add(right, weight=3)
        # Long labels should not consume the preview's share on smaller screens.
        def balance_panes(event):
            side = min(335, event.width // 3)
            panes.sashpos(0, side)
            panes.sashpos(1, side + int((event.width - side) * 0.48))
        panes.bind('<Configure>', balance_panes)
        self._composer(sidebar)
        self._timeline(middle)
        self._preview(right)

    def _composer(self, parent):
        tabs = ttk.Notebook(parent)
        tabs.pack(fill='both', expand=True)
        add_page = ttk.Frame(tabs)
        recipe_page = ttk.Frame(tabs)
        tabs.add(add_page, text='Add block')
        tabs.add(recipe_page, text='Presets')
        ttk.Button(add_page, text='+ Insert after selection', style='Accent.TButton',
                   command=self.add_block).pack(side='bottom', fill='x', padx=10, pady=8)
        ttk.Button(recipe_page, text='Load preset', style='Accent.TButton',
                   command=self.load_preset).pack(side='bottom', fill='x', padx=10, pady=8)
        add = self.scroll_frame(add_page)
        recipes = self.scroll_frame(recipe_page)
        ttk.Label(add, text='BUILD A PHRASE', style='Heading.TLabel').pack(anchor='w', pady=(5, 12))
        self.kind = tk.StringVar(value='Travel')
        combo = ttk.Combobox(add, values=KINDS, textvariable=self.kind, state='readonly', width=22)
        combo.pack(fill='x')
        combo.bind('<<ComboboxSelected>>', lambda event: self.compose_fields())
        self.form = ttk.Frame(add)
        self.form.pack(fill='x', pady=12)
        self.fields = {}
        self.compose_fields()
        ttk.Label(add, text='Distances: metres\nPose angles: degrees\nVelocity yaw: radians / second\n+X forward · +Y left · +yaw left',
                  style='Muted.TLabel', justify='left', wraplength=245).pack(anchor='w', pady=15)
        ttk.Label(add, text='Select a block to edit its YAML, including loops, individual toes and joint targets.',
                  style='Muted.TLabel', wraplength=245).pack(anchor='w')
        ttk.Label(recipes, text='START WITH AN IDEA', style='Heading.TLabel').pack(anchor='w', pady=(5, 12))
        self.recipe = tk.StringVar(value='square')
        ttk.Combobox(recipes, textvariable=self.recipe, values=PRESETS, state='readonly', width=22).pack(fill='x')
        self.recipe_fields = {}
        for label, key, value in (('Side / distance / radius (m)', 'size', '0.5'),
                                  ('Linear speed (m/s)', 'speed', '0.03'),
                                  ('Square turn speed (rad/s)', 'yaw_rate', '0.2'),
                                  ('Dance tempo (BPM)', 'bpm', '112'),
                                  ('Gesture / dance amplitude', 'amplitude', '0.6')):
            self.entry(recipes, self.recipe_fields, label, key, value)
        self.entry(recipes, self.recipe_fields, 'Walking gait', 'gait', GAITS[0], GAITS)
        self.clockwise = tk.BooleanVar()
        ttk.Checkbutton(recipes, text='Clockwise route', variable=self.clockwise).pack(anchor='w', pady=10)
        ttk.Label(recipes, text='Routes include stand-up and an explicit stop. Dance presets begin standing.\n\n'
                  'All presets become editable blocks. Size and speed apply to routes; tempo and amplitude to choreography.',
                  wraplength=245, style='Muted.TLabel').pack(anchor='w', pady=16)

    def scroll_frame(self, parent):
        wrapper = ttk.Frame(parent)
        wrapper.pack(fill='both', expand=True)
        canvas = tk.Canvas(wrapper, bg=PANEL, highlightthickness=0, width=310)
        scrollbar = ttk.Scrollbar(wrapper, command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side='right', fill='y')
        canvas.pack(fill='both', expand=True)
        body = ttk.Frame(canvas, padding=10)
        window = canvas.create_window(0, 0, anchor='nw', window=body)
        body.bind('<Configure>', lambda event: canvas.configure(scrollregion=canvas.bbox('all')))
        def resize_form(event):
            canvas.itemconfigure(window, width=event.width)

            def wrap_labels(widget):
                for child in widget.winfo_children():
                    if isinstance(child, ttk.Label):
                        child.configure(wraplength=max(100, event.width - 25))
                    wrap_labels(child)
            wrap_labels(body)
        canvas.bind('<Configure>', resize_form)

        def wheel(event):
            if str(event.widget).startswith(str(wrapper)):
                step = (-1 if event.num == 4 else 1) if event.num in (4, 5) else (-1 if event.delta > 0 else 1)
                canvas.yview_scroll(step * 3, 'units')
        for event in ('<MouseWheel>', '<Button-4>', '<Button-5>'):
            self.root.bind(event, wheel, add='+')
        return body

    def entry(self, parent, fields, label, key, default, options=None):
        ttk.Label(parent, text=label, style='Muted.TLabel').pack(anchor='w', pady=(9, 3))
        variable = tk.StringVar(value=default)
        fields[key] = variable
        widget = (ttk.Combobox(parent, textvariable=variable, values=options, state='readonly', width=22)
                  if options else ttk.Entry(parent, textvariable=variable, width=24))
        widget.pack(fill='x')
        return widget

    def compose_fields(self):
        for child in self.form.winfo_children():
            child.destroy()
        self.fields = {}
        kind = self.kind.get()
        field = lambda label, key, default, options=None: self.entry(
            self.form, self.fields, label, key, default, options)
        if kind in ('Travel', 'Turn', 'Arc', 'Velocity'):
            field('Gait', 'gait', GAITS[0], GAITS)
        if kind == 'Travel':
            field('Signed distance (m)', 'distance', '0.25')
            field('Speed (m/s)', 'speed', '0.03')
            field('Bearing: 0 forward, 90 left (°)', 'bearing', '0')
        elif kind == 'Turn':
            field('Signed angle (°)', 'angle', '90')
            field('Yaw speed (rad/s)', 'rate', '0.2')
        elif kind == 'Arc':
            field('Radius (m)', 'radius', '0.5')
            field('Signed angle (°)', 'angle', '90')
            field('Forward speed (m/s)', 'speed', '0.03')
        else:
            field('Duration (s)', 'duration', '1.0')
        if kind == 'Velocity':
            field('Forward X (m/s)', 'x', '0.03')
            field('Left Y (m/s)', 'y', '0')
            field('Yaw (rad/s)', 'yaw', '0')
        elif kind == 'Pose offset':
            combo = field('Component', 'component', 'head_rotation',
                          ('head_rotation', 'torso_rotation', 'torso_translation', 'toe_translation'))
            field('Leg (for toe translation)', 'leg', LEGS[0], LEGS)
            field('Roll / X', 'a', '0')
            field('Pitch / Y', 'b', '0')
            field('Yaw / Z', 'c', '5')
            field('Interpolation', 'curve', 'sin_squared', CURVES)
            # Translation starts at zero instead of inheriting the angular default.
            combo.bind('<<ComboboxSelected>>', lambda event: [self.fields[k].set('0') for k in ('a', 'b', 'c')])
        elif kind == 'Say':
            field('Text', 'text', 'Hello!')
        elif kind == 'Music':
            field('Song filename or STOP', 'song', 'STOP')
        elif kind == 'Hold distance':
            field('Sensor target distance (m)', 'distance', '0.6')

    def make_block(self):
        f = {key: value.get() for key, value in self.fields.items()}
        number = lambda key: float(f[key])
        kind = self.kind.get()
        if kind == 'Travel':
            return travel(number('distance'), number('speed'), bearing_deg=number('bearing'), gait=f['gait'])
        if kind == 'Turn':
            return turn(number('angle'), number('rate'), gait=f['gait'])
        if kind == 'Arc':
            return arc(number('radius'), number('angle'), number('speed'), gait=f['gait'])
        duration = number('duration')
        if kind == 'Velocity':
            return move(duration, x=number('x'), y=number('y'), yaw=number('yaw'), gait=f['gait'])
        if kind == 'Pose offset':
            axes = ('roll', 'pitch', 'yaw') if f['component'].endswith('rotation') else ('x', 'y', 'z')
            values = dict(zip(axes, [number('a'), number('b'), number('c')]))
            target = {f['leg']: values} if f['component'] == 'toe_translation' else values
            return pose(duration, pose_delta={f['component']: target}, interpolation=f['curve'])
        if kind == 'Hold':
            return hold(duration)
        if kind == 'Save pose':
            return pose(duration, save=True, pose_delta={'head_rotation': {'yaw': 0}})
        if kind == 'Restore pose':
            return pose(duration, restore=True)
        if kind == 'Stop':
            return stop(duration)
        if kind in ('Say', 'Music'):
            action = ({'say': {'text': f['text'], 'minDuration': duration}} if kind == 'Say' else
                      {'music': {'song': f['song'], 'background': True, 'minDuration': duration}})
            validate_action(action)
            return action
        return builtin({'Stand up': 'SEQUENCE_STAND_UP', 'Lay down': 'SEQUENCE_LAYDOWN',
                        'Hold distance': 'HOLD_DISTANCE', 'Level torso': 'LEVEL_TORSO'}[kind], duration,
                       distance_m=number('distance') if kind == 'Hold distance' else None)

    def _timeline(self, parent):
        title = ttk.Frame(parent)
        title.pack(fill='x', pady=(0, 10))
        ttk.Label(title, text='SEQUENCE / BLOCKS', style='Heading.TLabel').pack(anchor='w')
        self.stats = tk.StringVar()
        ttk.Label(title, textvariable=self.stats, foreground=ACCENT).pack(anchor='w', pady=(3, 0))
        split = ttk.Panedwindow(parent, orient='vertical')
        split.pack(fill='both', expand=True)
        top = ttk.Frame(split)
        bottom = ttk.Frame(split)
        split.add(top, weight=3)
        split.add(bottom, weight=2)
        self.tree = ttk.Treeview(top, columns=('block', 'time'), show='headings', selectmode='extended', height=8)
        self.tree.heading('block', text='Block / command')
        self.tree.heading('time', text='Seconds')
        self.tree.column('block', width=330, minwidth=180)
        self.tree.column('time', width=120, minwidth=110, stretch=False, anchor='e')
        scroll = ttk.Scrollbar(top, orient='vertical', command=self.tree.yview)
        self.tree.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right', fill='y')
        self.tree.pack(fill='both', expand=True)
        self.tree.bind('<<TreeviewSelect>>', self.select_block)
        actions = ttk.Frame(bottom)
        actions.pack(fill='x', pady=8)
        for column, (label, command) in enumerate((('↑', lambda: self.shift(-1)), ('↓', lambda: self.shift(1)),
                                                  ('Copy', self.duplicate), ('Loop', self.wrap_loop),
                                                  ('Delete', self.delete))):
            actions.columnconfigure(column, weight=1, uniform='actions')
            ttk.Button(actions, text=label, width=1, command=command).grid(
                row=0, column=column, sticky='ew', padx=(0, 4))
        ttk.Label(bottom, text='BLOCK YAML · select multiple rows to loop',
                  style='Muted.TLabel').pack(anchor='w', pady=(0, 6))
        ttk.Button(bottom, text='Apply block edits', command=self.apply_block).pack(side='bottom', anchor='e', pady=(8, 0))
        self.block_text = self.text_widget(bottom, height=9)
        self.block_text.edit_modified(False)

    def text_widget(self, parent, **kwargs):
        frame = ttk.Frame(parent)
        frame.pack(fill='both', expand=True)
        text = tk.Text(frame, bg=BG, fg=TEXT, insertbackground=ACCENT, selectbackground='#31546a',
                       relief='flat', padx=12, pady=10, font=self.code_font,
                       wrap='none', undo=True, **kwargs)
        vertical = ttk.Scrollbar(frame, orient='vertical', command=text.yview)
        horizontal = ttk.Scrollbar(frame, orient='horizontal', command=text.xview)
        text.configure(yscrollcommand=vertical.set, xscrollcommand=horizontal.set)
        vertical.pack(side='right', fill='y')
        horizontal.pack(side='bottom', fill='x')
        text.pack(fill='both', expand=True)
        return text

    def _preview(self, parent):
        tabs = ttk.Notebook(parent)
        tabs.pack(fill='both', expand=True)
        preview = ttk.Frame(tabs, padding=12)
        output = ttk.Frame(tabs, padding=8)
        tabs.add(preview, text='Route preview')
        tabs.add(output, text='Generated YAML')
        preview.columnconfigure(0, weight=1)
        preview.rowconfigure(2, weight=1)
        preview_title = ttk.Label(preview, text='COMMAND PATH / TOP VIEW', style='Heading.TLabel')
        preview_title.grid(row=0, column=0, sticky='w')
        preview_description = ttk.Label(preview, text='Nominal travel only · pose changes are not rendered',
                                        style='Muted.TLabel')
        preview_description.grid(row=1, column=0, sticky='w', pady=(5, 12))
        self.canvas = tk.Canvas(preview, background=BG, highlightthickness=0, height=180, width=200)
        self.canvas.grid(row=2, column=0, sticky='nsew')
        self.canvas.bind('<Configure>', lambda event: self.draw_route())
        controls = ttk.Frame(preview)
        controls.grid(row=3, column=0, sticky='ew', pady=12)
        self.play_button = ttk.Button(controls, text='▶ Play', command=self.toggle_play, width=6)
        self.play_button.grid(row=0, column=0, sticky='w')
        self.speed = tk.StringVar(value='1×')
        ttk.Combobox(controls, textvariable=self.speed, values=('0.5×', '1×', '2×', '4×', '10×'),
                     state='readonly', width=5).grid(row=0, column=1, sticky='w', padx=8)
        self.time_label = tk.StringVar(value='0.0 / 0.0 s')
        ttk.Label(controls, textvariable=self.time_label, foreground=ACCENT).grid(
            row=1, column=0, columnspan=2, sticky='w', pady=(4, 0))
        self.scrub = tk.DoubleVar()
        self.scale = ttk.Scale(preview, variable=self.scrub, from_=0, to=1, command=self.scrub_changed)
        self.scale.grid(row=4, column=0, sticky='ew')
        self.position = tk.StringVar()
        position_label = ttk.Label(preview, textvariable=self.position, style='Muted.TLabel')
        position_label.grid(row=5, column=0, sticky='w', pady=8)
        self.notes = tk.StringVar()
        self.notes_label = ttk.Label(preview, textvariable=self.notes, foreground=GOLD,
                                     wraplength=490, justify='left')
        self.notes_label.grid(row=6, column=0, sticky='ew', pady=(6, 0))
        def wrap_preview(event):
            for label in (self.notes_label, preview_title, preview_description, position_label):
                label.configure(wraplength=max(160, event.width - 30))
        preview.bind('<Configure>', wrap_preview)
        self.yaml_text = self.text_widget(output)
        ttk.Button(output, text='Copy YAML', command=self.copy_yaml).pack(anchor='e', pady=(8, 0))

    def snapshot(self):
        return copy.deepcopy((self.sequence.name, self.sequence.trigger, self.sequence.actions))

    def checkpoint(self):
        self.undo_stack.append(self.snapshot())
        del self.undo_stack[:-100]
        self.redo_stack.clear()

    def restore_snapshot(self, snapshot):
        self.sequence.name, self.sequence.trigger, self.sequence.actions = copy.deepcopy(snapshot)
        self.selected = None
        self.refresh()

    def metadata_changed(self, *args):
        if self.loading:
            return
        self.checkpoint()
        self.sequence.name = self.name.get()
        self.sequence.trigger = {k: v for k, v in (('voice', self.voice.get()), ('joystick', self.joystick.get())) if v}
        if self.pending_refresh:
            self.root.after_cancel(self.pending_refresh)
        self.pending_refresh = self.root.after(200, self.refresh_preview)

    def refresh(self, selection=None):
        self.pause()
        self.loading = True
        self.name.set(self.sequence.name)
        self.voice.set((self.sequence.trigger or {}).get('voice', ''))
        self.joystick.set((self.sequence.trigger or {}).get('joystick', ''))
        self.tree.delete(*self.tree.get_children())
        for index, action in enumerate(self.sequence.actions):
            self.tree.insert('', 'end', iid=str(index), values=(f'{index + 1:02d}  {describe(action)}',
                                                              f'{block_duration(action):.2f}'))
        self.selected = selection if selection is not None and selection < len(self.sequence.actions) else None
        if self.selected is not None:
            self.tree.selection_set(str(self.selected))
            self.tree.see(str(self.selected))
        self.show_block()
        self.loading = False
        self.refresh_preview()

    def refresh_preview(self):
        self.pending_refresh = None
        self.pause()
        self.points = []
        content = ''
        try:
            count, seconds = self.sequence.summary()
            self.stats.set(f'{count} actions · {seconds:.1f} s')
            self.points = route_preview(self.sequence)
            self.point_times = [p.seconds for p in self.points]
            content = self.sequence.to_yaml(recipe=False)
            notes = sequence_notes(self.sequence)
            self.notes.set('\n\n'.join(notes) or 'Timing excludes scheduler handoffs.')
            self.status.set('Ready to export · Ctrl+S export · Ctrl+Z undo · Ctrl+Y redo')
        except (ValueError, OverflowError) as error:
            self.stats.set('Draft')
            self.notes.set(str(error))
            self.status.set('Draft needs attention before export: ' + str(error))
        self.yaml_text.configure(state='normal')
        self.yaml_text.delete('1.0', 'end')
        self.yaml_text.insert('1.0', content or '# Resolve the draft validation message to export YAML.')
        self.yaml_text.configure(state='disabled')
        self.scale.configure(to=self.points[-1].seconds if self.points else 1)
        self.scrub.set(0)
        self.draw_route()
        dirty = self.snapshot() != self.saved
        self.root.title(f'{"* " if dirty else ""}{self.sequence.name} · RumbleX Sequence Studio')

    def show_block(self):
        self.block_text.delete('1.0', 'end')
        if self.selected is not None:
            self.block_text.insert('1.0', yaml.safe_dump(self.sequence.actions[self.selected], sort_keys=False))
        self.block_text.edit_reset()
        self.block_text.edit_modified(False)

    def settle_block(self):
        if not self.block_text.edit_modified():
            return True
        answer = messagebox.askyesnocancel('Unapplied block edits', 'Apply the block edits before continuing?', parent=self.root)
        if answer is None:
            return False
        if answer:
            return self.apply_block()
        self.block_text.edit_modified(False)
        return True

    def select_block(self, event=None):
        if self.loading:
            return
        rows = self.tree.selection()
        selected = int(rows[0]) if rows else None
        if selected == self.selected:
            return
        if not self.settle_block():
            self.loading = True
            self.tree.selection_set(str(self.selected)) if self.selected is not None else self.tree.selection_remove(*rows)
            self.loading = False
            return
        self.selected = selected
        if rows:
            self.tree.selection_set(*rows)
        self.show_block()

    def apply_block(self):
        if self.selected is None:
            return False
        try:
            action = load_yaml(self.block_text.get('1.0', 'end'))
            # Validate the block's structure/limits independently of save/restore pairing,
            # since users may still be assembling those pairs in the draft.
            trial = Sequence('block').extend([action])
            try:
                trial.summary()
            except ValueError as error:
                if str(error) not in ('Restore requires a saved pose', 'Saved pose must be restored on normal completion'):
                    raise
                # A restore can occur before malformed later actions; inspect every leaf too.
                self.validate_draft_block(action)
            self.checkpoint()
            self.sequence.actions[self.selected] = action
            self.block_text.edit_modified(False)
            self.refresh(self.selected)
            return True
        except (ValueError, TypeError) as error:
            self.error(error)
            return False

    def validate_draft_block(self, action, depth=0, budget=None):
        if budget is None:
            budget = [10000]
        if not isinstance(action, dict):
            raise ValueError('Expected action mapping')
        if 'loop' in action:
            loop = action['loop']
            if set(action) != {'loop'} or not isinstance(loop, dict) or set(loop) != {'times', 'actions'} or depth >= 16:
                raise ValueError('Invalid loop or nesting exceeds 16')
            repeat(loop['times'], loop['actions'])
            if not isinstance(loop['actions'], list):
                raise ValueError('Loop actions must be a list')
            for _ in range(loop['times']):
                for child in loop['actions']:
                    self.validate_draft_block(child, depth + 1, budget)
        else:
            budget[0] -= 1
            if budget[0] < 0:
                raise ValueError('Block exceeds 10000 actions')
            validate_action(action)

    def add_block(self):
        if not self.settle_block():
            return
        try:
            action = self.make_block()
            self.checkpoint()
            index = self.selected + 1 if self.selected is not None else len(self.sequence.actions)
            self.sequence.actions.insert(index, action)
            self.refresh(index)
        except (ValueError, OverflowError) as error:
            self.error(error)

    def indices(self):
        return sorted(int(row) for row in self.tree.selection())

    def duplicate(self):
        if not self.settle_block():
            return
        indices = self.indices()
        if indices:
            self.checkpoint()
            index = indices[-1] + 1
            self.sequence.actions[index:index] = copy.deepcopy([self.sequence.actions[i] for i in indices])
            self.refresh(index)

    def delete(self):
        if not self.settle_block():
            return
        indices = self.indices()
        if indices:
            self.checkpoint()
            for index in reversed(indices):
                del self.sequence.actions[index]
            self.refresh(min(indices[0], len(self.sequence.actions) - 1) if self.sequence.actions else None)

    def shift(self, direction):
        if not self.settle_block():
            return
        indices = self.indices()
        if not indices or indices != list(range(indices[0], indices[-1] + 1)):
            return
        if indices[0] + direction < 0 or indices[-1] + direction >= len(self.sequence.actions):
            return
        self.checkpoint()
        for i in (indices if direction < 0 else reversed(indices)):
            self.sequence.actions[i], self.sequence.actions[i + direction] = (
                self.sequence.actions[i + direction], self.sequence.actions[i])
        self.refresh(indices[0] + direction)
        self.tree.selection_set(*(str(i + direction) for i in indices))

    def wrap_loop(self):
        if not self.settle_block():
            return
        indices = self.indices()
        if not indices:
            return
        if indices != list(range(indices[0], indices[-1] + 1)):
            self.error('Select adjacent blocks to create a loop.')
            return
        times = simpledialog.askinteger('Repeat phrase', 'Repetitions (1–1000)', initialvalue=2,
                                         minvalue=1, maxvalue=1000, parent=self.root)
        if times is None:
            return
        self.checkpoint()
        phrase = repeat(times, self.sequence.actions[indices[0]:indices[-1] + 1])
        self.sequence.actions[indices[0]:indices[-1] + 1] = [phrase]
        self.refresh(indices[0])

    def undo(self):
        if self.settle_block() and self.undo_stack:
            self.redo_stack.append(self.snapshot())
            self.restore_snapshot(self.undo_stack.pop())

    def redo(self):
        if self.settle_block() and self.redo_stack:
            self.undo_stack.append(self.snapshot())
            self.restore_snapshot(self.redo_stack.pop())

    def replace_document(self, sequence, path=None):
        self.sequence = copy.deepcopy(sequence)
        self.path = Path(path) if path else None
        self.saved = self.snapshot() if path else None
        self.undo_stack.clear()
        self.redo_stack.clear()
        self.refresh()

    def may_replace(self):
        if not self.settle_block():
            return False
        if self.snapshot() == self.saved:
            return True
        answer = messagebox.askyesnocancel('Unsaved sequence', 'Export your changes before continuing?', parent=self.root)
        return self.export() if answer else answer is not None

    def new(self):
        if self.may_replace():
            self.replace_document(Sequence('new_sequence'))

    def open_file(self):
        if not self.may_replace():
            return
        path = filedialog.askopenfilename(parent=self.root, title='Open self-contained sequence',
                                          filetypes=[('Sequence YAML', '*.yaml *.yml')])
        if path:
            try:
                self.replace_document(Sequence.from_yaml(Path(path).read_text(encoding='utf-8')), path)
            except (ValueError, OSError) as error:
                self.error(error)

    def load_preset(self):
        try:
            f = self.recipe_fields
            sequence = preset(self.recipe.get(), **{key: float(f[key].get()) for key in
                              ('size', 'speed', 'yaw_rate', 'bpm', 'amplitude')},
                              gait=f['gait'].get(), clockwise=self.clockwise.get())
            if self.may_replace():
                self.replace_document(sequence)
        except (ValueError, OverflowError) as error:
            self.error(error)

    def export(self):
        if not self.settle_block():
            return False
        try:
            self.sequence.summary()
            path = filedialog.asksaveasfilename(parent=self.root, title='Export runtime sequence YAML',
                                                initialdir=str(self.path.parent) if self.path else None,
                                                initialfile=self.path.name if self.path else self.sequence.name + '.yaml',
                                                defaultextension='.yaml', confirmoverwrite=True,
                                                filetypes=[('Sequence YAML', '*.yaml *.yml')])
            if not path:
                return False
            self.sequence.write(path, force=True, recipe=False)
            self.path = Path(path)
            self.saved = self.snapshot()
            self.refresh_preview()
            self.status.set(f'Exported {self.path}')
            return True
        except (ValueError, OSError) as error:
            self.error(error)
            return False

    def copy_yaml(self):
        if not self.settle_block():
            return
        try:
            content = self.sequence.to_yaml(recipe=False)
            self.root.clipboard_clear()
            self.root.clipboard_append(content)
            self.status.set('Validated sequence YAML copied to clipboard')
        except ValueError as error:
            self.error(error)

    def close(self):
        if self.may_replace():
            self.pause()
            if self.pending_refresh:
                self.root.after_cancel(self.pending_refresh)
            self.root.destroy()

    def error(self, error):
        messagebox.showerror('Sequence needs attention', str(error), parent=self.root)

    def pause(self):
        self.playing = False
        if self.timer is not None:
            self.root.after_cancel(self.timer)
            self.timer = None
        if hasattr(self, 'play_button'):
            self.play_button.configure(text='▶ Play')

    def toggle_play(self):
        if self.playing:
            self.pause()
        elif self.points:
            if self.scrub.get() >= self.points[-1].seconds:
                self.scrub.set(0)
            self.playing = True
            self.play_button.configure(text='Ⅱ Pause')
            self.last_tick = time.monotonic()
            self.tick()

    def tick(self):
        self.timer = None
        if not self.playing or not self.points:
            return
        now = time.monotonic()
        value = min(self.points[-1].seconds, self.scrub.get() + (now - self.last_tick) * float(self.speed.get()[:-1]))
        self.last_tick = now
        self.scrub.set(value)
        self.draw_cursor()
        if value >= self.points[-1].seconds:
            self.pause()
        else:
            self.timer = self.root.after(33, self.tick)

    def scrub_changed(self, value):
        self.draw_cursor()

    def draw_route(self):
        self.canvas.delete('all')
        width, height = self.canvas.winfo_width(), self.canvas.winfo_height()
        if width < 10 or height < 10:
            return
        if not self.points:
            self.canvas.create_text(width / 2, height / 2, text='Add blocks to preview your sequence', fill=MUTED)
            self.time_label.set('0.0 / 0.0 s')
            self.position.set('')
            return
        # Screen up is world +X; screen left is world +Y, as seen from behind the robot.
        xs, ys = [p.x for p in self.points], [p.y for p in self.points]
        xmin, xmax = min(xs) - 0.12, max(xs) + 0.12
        ymin, ymax = min(ys) - 0.12, max(ys) + 0.12
        scale = min(max(1, height - 70) / (xmax - xmin), max(1, width - 70) / (ymax - ymin))
        self.transform = lambda x, y: (width / 2 - (y - (ymin + ymax) / 2) * scale,
                                        height / 2 - (x - (xmin + xmax) / 2) * scale)
        span = max(xmax - xmin, ymax - ymin)
        grid = 10 ** math.floor(math.log10(span / 5))
        if span / grid > 12:
            grid *= 5
        for axis in ('x', 'y'):
            low, high = (xmin, xmax) if axis == 'x' else (ymin, ymax)
            for index in range(math.ceil(low / grid), math.floor(high / grid) + 1):
                value = index * grid
                endpoints = ((value, ymin), (value, ymax)) if axis == 'x' else ((xmin, value), (xmax, value))
                a, b = [self.transform(*point) for point in endpoints]
                self.canvas.create_line(*a, *b, fill='#21313d')
        coordinates = [coordinate for p in self.points for coordinate in self.transform(p.x, p.y)]
        if len(coordinates) >= 4:
            self.canvas.create_line(*coordinates, fill=ACCENT, width=3, joinstyle='round')
        sx, sy = self.transform(0, 0)
        self.canvas.create_oval(sx - 5, sy - 5, sx + 5, sy + 5, outline=GOLD, width=2)
        self.canvas.create_text(sx + 12, sy + 13, text='START', fill=GOLD, anchor='w', font=self.map_font)
        self.canvas.create_text(14, 18, text=f'↑ +X    ← +Y     grid {grid:g} m',
                                fill=MUTED, anchor='w', font=self.map_font)
        self.draw_cursor()

    def draw_cursor(self):
        self.canvas.delete('cursor')
        if not self.points or not hasattr(self, 'transform'):
            return
        seconds = self.scrub.get()
        index = min(len(self.points) - 1, bisect.bisect_right(self.point_times, seconds))
        after, before = self.points[index], self.points[max(0, index - 1)]
        fraction = max(0, min(1, (seconds - before.seconds) / (after.seconds - before.seconds))) if after.seconds > before.seconds else 1
        x = before.x + (after.x - before.x) * fraction
        y = before.y + (after.y - before.y) * fraction
        yaw = before.yaw + (after.yaw - before.yaw) * fraction
        sx, sy = self.transform(x, y)
        # A schematic six-legged marker: fixed pixel size, independent of robot geometry.
        def screen(forward, left):
            return (sx - (forward * math.sin(yaw) + left * math.cos(yaw)),
                    sy - (forward * math.cos(yaw) - left * math.sin(yaw)))
        for fore in (-10, 0, 10):
            for side in (-1, 1):
                self.canvas.create_line(*screen(fore, side * 7), *screen(fore * 1.5, side * 19),
                                         fill=TEXT, width=2, tags='cursor')
        body = [v for point in ((17, 0), (7, 9), (-12, 9), (-17, 0), (-12, -9), (7, -9)) for v in screen(*point)]
        self.canvas.create_polygon(*body, fill='#264e5d', outline=ACCENT, width=2, tags='cursor')
        self.canvas.create_line(*screen(2, 0), *screen(25, 0), arrow='last', fill=GOLD, width=2, tags='cursor')
        self.time_label.set(f'{seconds:.1f} / {self.points[-1].seconds:.1f} s')
        self.position.set(f'X {x:+.3f} m   Y {y:+.3f} m   heading {math.degrees(yaw):+.1f}°\n'
                          f'Expanded action {max(1, after.action + 1)}')


def launch(sequence, path=None):
    try:
        root = tk.Tk()
    except tk.TclError as error:
        raise ValueError('Cannot open a display. Run in a desktop session, or use --preset NAME for CLI generation.') from error
    SequenceEditor(root, sequence, path)
    root.mainloop()

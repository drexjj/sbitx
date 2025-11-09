#!/usr/bin/env python3
import gi
import telnetlib
import os
import sys
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, Gdk, GLib

LOCK_FILE = '/tmp/frequency_keypad.lock'
POSITION_FILE = '/home/pi/sbitx/data/keypad_position.txt'

class FrequencyKeypad(Gtk.Window):
    def __init__(self):
        super().__init__(title="Freq Keypad", decorated=False)
        self.set_border_width(6)
        self.set_default_size(300, 300)
        self.load_position()
        self.freq = ""
        self.apply_css()

        main = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        self.add(main)

        # LEFT: Band buttons (vertical)
        band_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=3)
        main.pack_start(band_box, False, False, 0)

        bands = [
            ("80M", "3750"), ("60M", "5357"), ("40M", "7150"),
            ("30M", "10136"), ("20M", "14175"), ("17M", "18118"),
            ("15M", "21225"), ("12M", "24940"), ("10M", "28800")
        ]
        for label, khz in bands:
            b = Gtk.Button(label=label)
            b.set_name("band_button")
            b.connect("clicked", self.tune_band, khz)
            band_box.pack_start(b, True, True, 0)

        # RIGHT: Display + Keypad
        right = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
        main.pack_start(right, True, True, 0)

        # Close button
        close = Gtk.Button(label="X")
        close.set_name("close_button")
        close.connect("clicked", lambda x: self.close_win())
        top = Gtk.Box()
        top.pack_end(close, False, False, 0)
        right.pack_start(top, False, False, 0)

        # Display
        self.disp = Gtk.Label()
        self.disp.set_text("Enter kHz")
        self.disp.set_name("display_label")
        right.pack_start(self.disp, False, False, 0)

        # Keypad
        grid = Gtk.Grid()
        grid.set_row_homogeneous(True)
        grid.set_column_homogeneous(True)
        grid.set_row_spacing(4)
        grid.set_column_spacing(4)
        right.pack_start(grid, True, True, 0)

        layout = [
            ['1','2','3'],
            ['4','5','6'],
            ['7','8','9'],
            ['<-','0','Enter']
        ]
        for r, row in enumerate(layout):
            for c, txt in enumerate(row):
                b = Gtk.Button(label=txt)
                b.set_name("key_button")
                if txt == "Enter":
                    b.get_style_context().add_class("enter_button")
                if txt == "<-":
                    b.get_style_context().add_class("back_button")
                b.connect("clicked", self.key_press)
                grid.attach(b, c, r, 1, 1)

        # Drag
        self.connect("button-press-event", self.drag_start)
        self.connect("button-release-event", self.drag_stop)
        self.connect("motion-notify-event", self.drag_move)
        self.dragging = False
        self.set_keep_above(True)

    def tune_band(self, btn, khz):
        self.send(khz)
        self.disp.set_text(f"Tuned {khz}")
        GLib.timeout_add(600, lambda: self.disp.set_text("Enter kHz"))
        GLib.timeout_add(900, self.close_win)

    def key_press(self, btn):
        t = btn.get_label()
        if t == "Enter":
            if self.freq:
                self.send(self.freq)
                self.freq = ""
                self.close_win()
        elif t == "<-":
            self.freq = self.freq[:-1]
        else:
            self.freq += t
        self.disp.set_text(self.freq or "Enter kHz")

    def send(self, khz):
        try:
            with telnetlib.Telnet("127.0.0.1", 8081) as tn:
                tn.write(f"f {khz}\n".encode())
            print(f"Tuned: {khz} kHz")
        except Exception as e:
            print("Telnet error:", e)

    def drag_start(self, w, e):
        if e.button == 1:
            self.dragging = True
            self.ox, self.oy = e.x, e.y

    def drag_stop(self, w, e):
        if e.button == 1:
            self.dragging = False
            self.save_position()

    def drag_move(self, w, e):
        if self.dragging:
            x, y = self.get_position()
            self.move(x + e.x - self.ox, y + e.y - self.oy)

    def close_win(self):
        self.save_position()
        remove_lock_file()
        Gtk.main_quit()

    def apply_css(self):
        css = """
        window { background-color: #000000; }
        #display_label {
            color: #00ff00;
            font-size: 18px;
            font-weight: bold;
            background-color: #001111;
            padding: 8px;
            border-radius: 6px;
            margin: 4px 8px;
        }
        #band_button {
            background-color: #003333;
            color: #00ff00;
            font-size: 14px;
            font-weight: bold;
            min-width: 48px;
            min-height: 28px;
            border-radius: 6px;
        }
        #band_button:hover {
            background-color: #00ff00;
            color: #000000;
        }
        #key_button {
            background-color: #006666;
            color: #ffffff;
            font-size: 16px;
            font-weight: bold;
            min-width: 56px;
            min-height: 36px;
            border-radius: 6px;
        }
        #key_button:hover {
            background-color: #009999;
        }
        .enter_button {
            background-color: #008800;
        }
        .enter_button:hover {
            background-color: #00bb00;
        }
        .back_button {
            background-color: #aa5500;
        }
        .back_button:hover {
            background-color: #cc7700;
        }
        #close_button {
            background-color: #cc0000;
            color: #ffffff;
            font-size: 14px;
            min-width: 32px;
            border-radius: 6px;
        }
        #close_button:hover {
            background-color: #ff4444;
        }
        """
        provider = Gtk.CssProvider()
        provider.load_from_data(css.encode('utf-8'))
        Gtk.StyleContext.add_provider_for_screen(
            Gdk.Screen.get_default(),
            provider,
            Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
        )

    def save_position(self):
        x, y = self.get_position()
        try:
            with open(POSITION_FILE, "w") as f:
                f.write(f"{x},{y}")
        except: pass

    def load_position(self):
        if os.path.exists(POSITION_FILE):
            try:
                with open(POSITION_FILE) as f:
                    x, y = map(int, f.read().strip().split(','))
                    self.move(x, y)
            except: pass

def create_lock_file():
    try:
        open(LOCK_FILE, 'x').close()
    except FileExistsError:
        print("Another instance running!")
        sys.exit(1)

def remove_lock_file():
    if os.path.exists(LOCK_FILE):
        try: os.remove(LOCK_FILE)
        except: pass

if __name__ == "__main__":
    create_lock_file()
    win = FrequencyKeypad()
    win.connect("destroy", lambda w: (remove_lock_file(), Gtk.main_quit()))
    win.show_all()
    Gtk.main()
    remove_lock_file()

import gi
import telnetlib
import os
import sys

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, Gdk

LOCK_FILE = '/tmp/frequency_keypad.lock'
POSITION_FILE = '/home/pi/sbitx/data/keypad_position.txt'  # File to store the window position

class FrequencyKeypad(Gtk.Window):
    def __init__(self):
        super().__init__(title="Frequency Keypad", decorated=False)  # Hide the title bar
        self.set_border_width(10)
        self.set_default_size(300, 300)

        # Read previous position from file
        self.load_position()

        # Initialize frequency input
        self.frequency_input = ""

        # Apply CSS styles
        self.apply_css()

        # Create a vertical box to arrange widgets
        vbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        self.add(vbox)

        # Create the close button
        close_button = Gtk.Button(label="X")
        close_button.set_name("close_button")
        close_button.connect("clicked", self.on_close_button_clicked)
        vbox.pack_start(close_button, False, True, 0)

        # Create the display label
        self.display = Gtk.Label()
        self.display.set_text("Enter kHz")
        self.display.set_name("display_label")  
        vbox.pack_start(self.display, True, True, 0)

        # Keypad buttons
        buttons = [
            ['1', '2', '3'],
            ['4', '5', '6'],
            ['7', '8', '9'],
            ['←', '0', '↵']
        ]

        for row in buttons:
            hbox = Gtk.Box(spacing=6)
            vbox.pack_start(hbox, True, True, 0)
            for label in row:
                button = Gtk.Button(label=label)
                button.connect("clicked", self.on_button_clicked)
                button.set_name("keypad_button")  
                hbox.pack_start(button, True, True, 0)

        # Connect mouse events for dragging the window
        self.connect("button-press-event", self.on_button_press_event)
        self.connect("button-release-event", self.on_button_release_event)
        self.connect("motion-notify-event", self.on_motion_notify_event)

        self.is_dragging = False
        self.drag_start_x = 0
        self.drag_start_y = 0
        self.set_keep_above(True)

    def on_button_press_event(self, widget, event):
        """Start dragging the window when the mouse button is pressed."""
        if event.button == 1:  # Left mouse button
            self.is_dragging = True
            self.drag_start_x = event.x
            self.drag_start_y = event.y

    def on_button_release_event(self, widget, event):
        """Stop dragging the window when the mouse button is released."""
        if event.button == 1:  # Left mouse button
            self.is_dragging = False
            self.save_position()  # Save the position when dragging stops

    def on_motion_notify_event(self, widget, event):
        """Move the window if dragging is in progress."""
        if self.is_dragging:
            window_x, window_y = self.get_position()
            self.move(window_x + event.x - self.drag_start_x, window_y + event.y - self.drag_start_y)

    def on_close_button_clicked(self, button):
        """Handle the close button click event."""
        self.save_position()  # Save position before closing
        remove_lock_file()  # Ensure the lock file is removed
        Gtk.main_quit()  # Exit the GTK main loop

    def on_button_clicked(self, button):
        label = button.get_label()

        if label == "↵":
            self.send_frequency_to_telnet()
        elif label == "←":
            self.frequency_input = self.frequency_input[:-1]
            self.update_display()
        else:
            self.frequency_input += label
            self.update_display()

    def update_display(self):
        if self.frequency_input == "":
            self.display.set_text("Enter kHz")
        else:
            self.display.set_text(self.frequency_input)

    def send_frequency_to_telnet(self):
        if not self.frequency_input:
            return

        # Sending the 'f <frequency>\n' command to the Telnet server
        frequency_command = f"f {self.frequency_input}\n"

        try:
            # Establishing Telnet connection
            with telnetlib.Telnet("127.0.0.1", 8081) as tn:
                tn.write(frequency_command.encode('utf-8'))
                print(f"Sent frequency command: {frequency_command.strip()}")
        except ConnectionError:
            print("Could not connect to the Telnet server.")

        # Clear input after sending
        self.frequency_input = ""
        self.update_display()

        # Automatically close the window
        self.on_close_button_clicked(None)

    def apply_css(self):
        css = b"""
        window {
            background-color: #000000;
        }
        #display_label {
            color: white;
            font-size: 20px;
            padding: 10px;
        }
        #keypad_button {
            background-color: #008080;
            color: white;
            font-size: 16px;
            border-radius: 10px;
            border-width: 1px;
        }
        #keypad_button:hover {
            background-color: #009999;
        }
        #close_button {
            background-color: #ff3333;  /* Red color for close button */
            color: white;
            font-size: 16px;
            border-radius: 10px;
            border-width: 1px;
            margin-bottom: 10px;  /* Add some space below the close button */
        }
        #close_button:hover {
            background-color: #ff5555;  /* Lighter red on hover */
        }
        """

        style_provider = Gtk.CssProvider()
        style_provider.load_from_data(css)

        Gtk.StyleContext.add_provider_for_screen(
            Gdk.Screen.get_default(),
            style_provider,
            Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
        )

    def save_position(self):
        """Save the current window position to a file."""
        x, y = self.get_position()
        try:
            with open(POSITION_FILE, 'w') as f:
                f.write(f"{x},{y}")
                print(f"Saved position: {x},{y}")
        except Exception as e:
            print(f"Error saving position: {e}")

    def load_position(self):
        """Load the window position from a file."""
        if os.path.exists(POSITION_FILE):
            try:
                with open(POSITION_FILE, 'r') as f:
                    position = f.read().strip()
                    x, y = map(int, position.split(','))
                    self.move(x, y)
                    print(f"Loaded position: {x},{y}")
            except Exception as e:
                print(f"Error loading position: {e}")

def create_lock_file():
    """Create a lock file to prevent multiple instances."""
    try:
        with open(LOCK_FILE, 'x'):
            pass
    except FileExistsError:
        print("Another instance is already running.")
        sys.exit(1)

def remove_lock_file():
    """Remove the lock file when exiting."""
    if os.path.exists(LOCK_FILE):
        try:
            os.remove(LOCK_FILE)
            print("Lock file removed.")
        except Exception as e:
            print(f"Error removing lock file: {e}")

if __name__ == "__main__":
    create_lock_file()
    win = FrequencyKeypad()
    
    # Ensure lock file is removed on exit
    win.connect("destroy", lambda w: (remove_lock_file(), Gtk.main_quit()))
    
    win.show_all()
    Gtk.main()

    # Remove lock file when exiting the main loop
    remove_lock_file()

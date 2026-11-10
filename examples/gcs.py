import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping  # fix for Python 3.10+

import math
import threading
import time
import tkinter as tk
from tkinter import messagebox

from dronekit import connect, VehicleMode

try:
    import tkintermapview
    from PIL import Image, ImageDraw, ImageTk
    MAP_AVAILABLE = True
except ImportError:
    MAP_AVAILABLE = False

MODES = ["GUIDED", "STABILIZE", "LOITER", "LAND", "RTL"]

# RC values in microseconds (PWM)
RC_MIN, RC_MID, RC_MAX = 1000, 1500, 2000
DEFLECT = 300          # how far roll/pitch/yaw move from the middle while a key is held
THROTTLE_STEP = 20     # throttle change per tick while W/S is held
TICK_MS = 100          # 10 Hz: RC override is re-sent and the GUI is refreshed
STICK_SIZE = 130       # size of the stick squares in pixels
MAP_EVERY_N_TICKS = 3  # the map is refreshed every 3rd tick (about 3 times a second)
TRAIL_MIN_STEP_M = 1.0 # a new trail point is added after the drone moves this far
TRAIL_MAX_POINTS = 1000

# Keys that control the drone. Letters are compared in lowercase.
CONTROL_KEYS = {"w", "a", "s", "d", "x", "Left", "Right", "Up", "Down"}


class GCS(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("My GCS")
        self.vehicle = None
        self.connecting = False

        # RC command values (what we send to the drone)
        self.roll = self.pitch = self.yaw = RC_MID
        self.throttle = RC_MIN

        # Keyboard state
        self.pressed = set()
        self.release_jobs = {}

        # Map state
        self.map_widget = None
        self.drone_marker = None
        self.trail = []
        self.trail_path = None
        self.icon_cache = {}
        self.last_icon_key = None
        self.map_ticks = 0

        left = tk.Frame(self)
        left.grid(row=0, column=0, padx=10, pady=10, sticky="n")
        right = tk.Frame(self)
        right.grid(row=0, column=1, padx=10, pady=10, sticky="n")
        map_frame = tk.Frame(self)
        map_frame.grid(row=0, column=2, padx=10, pady=10, sticky="n")

        self._build_left(left)
        self._build_right(right)
        self._build_map(map_frame)

        self.bind_all("<KeyPress>", self.on_key_press)
        self.bind_all("<KeyRelease>", self.on_key_release)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self._tick()

    # ------------------------------------------------------------------ UI
    def _build_left(self, parent):
        tk.Label(parent, text="Connection string:").pack()
        self.conn_entry = tk.Entry(parent, width=30)
        self.conn_entry.insert(0, "tcp:127.0.0.1:5762")
        self.conn_entry.pack(pady=5)

        tk.Button(parent, text="Connect", width=14, command=self.connect_vehicle).pack(pady=3)

        tk.Label(parent, text="Flight modes:").pack(pady=(10, 0))
        mode_frame = tk.Frame(parent)
        mode_frame.pack(pady=5)
        for i, mode in enumerate(MODES):
            tk.Button(
                mode_frame, text=mode, width=12,
                command=lambda m=mode: self.set_mode(m),
            ).grid(row=i // 2, column=i % 2, padx=2, pady=2)

        tk.Button(parent, text="Arm", width=14, command=self.arm).pack(pady=(10, 3))

        # Takeoff with altitude input
        tk.Label(parent, text="Takeoff altitude (m):").pack(pady=(10, 0))
        takeoff_frame = tk.Frame(parent)
        takeoff_frame.pack(pady=5)
        self.alt_entry = tk.Entry(takeoff_frame, width=6)
        self.alt_entry.insert(0, "10")
        self.alt_entry.pack(side="left", padx=3)
        tk.Button(takeoff_frame, text="Takeoff", width=10, command=self.takeoff).pack(side="left", padx=3)

        self.info_var = tk.StringVar(value="")
        tk.Label(parent, textvariable=self.info_var, fg="gray30", wraplength=220).pack(pady=3)

        tk.Button(parent, text="Close Connection", width=14, command=self.close_connection).pack(pady=(10, 3))

    def _build_right(self, parent):
        # Status panel
        status = tk.LabelFrame(parent, text="Current state", padx=10, pady=5)
        status.pack(fill="x")
        self.conn_label = tk.Label(status, text="Connected: No", fg="red", anchor="w")
        self.conn_label.pack(fill="x")
        self.mode_label = tk.Label(status, text="Mode: -", anchor="w")
        self.mode_label.pack(fill="x")
        self.armed_label = tk.Label(status, text="Armed: -", anchor="w")
        self.armed_label.pack(fill="x")
        self.alt_label = tk.Label(status, text="Altitude: -", anchor="w")
        self.alt_label.pack(fill="x")
        self.heading_label = tk.Label(status, text="Yaw: -", anchor="w")
        self.heading_label.pack(fill="x")

        # RC visualization (like Mission Planner)
        rc = tk.LabelFrame(parent, text="RC channels (sent values)", padx=10, pady=5)
        rc.pack(fill="x", pady=(10, 0))

        sticks = tk.Frame(rc)
        sticks.pack()
        left_box = tk.Frame(sticks)
        left_box.grid(row=0, column=0, padx=5)
        right_box = tk.Frame(sticks)
        right_box.grid(row=0, column=1, padx=5)

        tk.Label(left_box, text="Throttle / Yaw").pack()
        self.left_stick = tk.Canvas(left_box, width=STICK_SIZE, height=STICK_SIZE, bg="white",
                                    highlightthickness=1, highlightbackground="gray")
        self.left_stick.pack()
        tk.Label(right_box, text="Pitch / Roll").pack()
        self.right_stick = tk.Canvas(right_box, width=STICK_SIZE, height=STICK_SIZE, bg="white",
                                     highlightthickness=1, highlightbackground="gray")
        self.right_stick.pack()

        self.rc_text = tk.Label(rc, text="", justify="left", font=("TkFixedFont", 9))
        self.rc_text.pack(pady=(5, 0), anchor="w")

        # Keyboard control
        self.rc_var = tk.BooleanVar(value=False)
        tk.Checkbutton(
            parent, text="Keyboard control (RC override)",
            variable=self.rc_var, command=self.on_rc_toggle,
        ).pack(anchor="w", pady=(10, 0))
        tk.Label(
            parent, justify="left", fg="gray30",
            text="W/S: throttle   A/D: yaw   X: throttle to minimum\n"
                 "Left/Right: roll   Up/Down: pitch\n"
                 "Use the English keyboard layout.\n"
                 "Fly in STABILIZE (arm first).",
        ).pack(anchor="w")

    # ---------------------------------------------------------- connection
    def connect_vehicle(self):
        if self.vehicle is not None or self.connecting:
            return
        conn_str = self.conn_entry.get().strip()
        self.connecting = True
        self.conn_label.config(text="Connected: connecting...", fg="orange")
        # Run in a thread so the window doesn't freeze
        threading.Thread(target=self._connect, args=(conn_str,), daemon=True).start()

    def _connect(self, conn_str):
        try:
            vehicle = connect(conn_str, wait_ready=False, heartbeat_timeout=30)
            self.vehicle = vehicle
        except Exception as e:
            msg = str(e)  # save the message now, before 'e' disappears
            self.after(0, lambda: messagebox.showerror("Connection error", msg))
        finally:
            self.connecting = False

    def _check(self):
        if self.vehicle is None:
            messagebox.showwarning("Not connected", "Connect to the vehicle first.")
            return False
        return True

    def close_connection(self):
        self.rc_var.set(False)
        self.on_rc_toggle()
        if self.vehicle is not None:
            try:
                self.vehicle.close()
            except Exception:
                pass
            self.vehicle = None
        self.info_var.set("")
        self._reset_map()

    def on_close(self):
        self.close_connection()
        self.destroy()

    # ------------------------------------------------------------- actions
    def set_mode(self, mode):
        if self._check():
            self.vehicle.mode = VehicleMode(mode)

    def arm(self):
        if self._check():
            self.vehicle.armed = True

    def takeoff(self):
        if not self._check():
            return
        try:
            alt = float(self.alt_entry.get().replace(",", "."))
        except ValueError:
            messagebox.showwarning("Takeoff", "Enter the altitude in meters, for example 10.")
            return
        if not 1 <= alt <= 100:
            messagebox.showwarning("Takeoff", "Altitude must be between 1 and 100 meters.")
            return
        # Keyboard override must not fight the takeoff command
        self.rc_var.set(False)
        self.on_rc_toggle()
        threading.Thread(target=self._takeoff, args=(alt,), daemon=True).start()

    def _info(self, text):
        self.after(0, lambda: self.info_var.set(text))

    def _takeoff(self, alt):
        v = self.vehicle
        try:
            self._info("Takeoff: switching to GUIDED...")
            v.mode = VehicleMode("GUIDED")
            start = time.time()
            while v.mode.name != "GUIDED":
                if time.time() - start > 10:
                    raise RuntimeError("Could not switch to GUIDED mode.")
                time.sleep(0.5)

            self._info("Takeoff: arming...")
            start = time.time()
            while not v.armed:
                v.armed = True
                if time.time() - start > 20:
                    raise RuntimeError(
                        "Could not arm. Check the PreArm messages in the MAVProxy console and try again.")
                time.sleep(1)

            self._info(f"Takeoff: climbing to {alt:g} m")
            v.simple_takeoff(alt)
        except Exception as e:
            msg = str(e)
            self._info("Takeoff failed")
            self.after(0, lambda: messagebox.showerror("Takeoff", msg))

    # ----------------------------------------------------- keyboard control
    def on_rc_toggle(self):
        if self.rc_var.get():
            if self.vehicle is None:
                self.rc_var.set(False)
                messagebox.showwarning("Not connected", "Connect to the vehicle first.")
                return
            try:
                alt = self.vehicle.location.global_relative_frame.alt
            except Exception:
                alt = None
            # Already flying (for example after Takeoff): start at 1500 = hold altitude
            self.throttle = RC_MID if (alt or 0) > 1 else RC_MIN
            self.focus_set()  # take the focus away from the entry fields
        else:
            self.pressed.clear()
            self.throttle = RC_MIN
            self._clear_override()

    def _clear_override(self):
        if self.vehicle is not None:
            try:
                self.vehicle.channels.overrides = {}  # give control back to the real RC
            except Exception:
                pass

    def on_key_press(self, event):
        if isinstance(event.widget, tk.Entry):
            return
        key = event.keysym if len(event.keysym) > 1 else event.keysym.lower()
        if key not in CONTROL_KEYS:
            return
        # Linux sends fake release/press pairs while a key is held: ignore the release
        job = self.release_jobs.pop(key, None)
        if job is not None:
            self.after_cancel(job)
        self.pressed.add(key)

    def on_key_release(self, event):
        key = event.keysym if len(event.keysym) > 1 else event.keysym.lower()
        if key not in CONTROL_KEYS:
            return
        self.release_jobs[key] = self.after(40, lambda: self._release(key))

    def _release(self, key):
        self.release_jobs.pop(key, None)
        self.pressed.discard(key)

    # ------------------------------------------------------------------ map
    def _build_map(self, parent):
        box = tk.LabelFrame(parent, text="Map", padx=5, pady=5)
        box.pack()
        if not MAP_AVAILABLE:
            tk.Label(box, fg="red", justify="left",
                     text="Map is not available.\nInstall it with:\n  pip install tkintermapview"
                     ).pack(padx=20, pady=20)
            return
        self.map_widget = tkintermapview.TkinterMapView(box, width=480, height=430, corner_radius=0)
        self.map_widget.pack()
        self.map_widget.set_position(0, 0)
        self.map_widget.set_zoom(2)

        controls = tk.Frame(box)
        controls.pack(fill="x", pady=(5, 0))
        self.follow_var = tk.BooleanVar(value=True)
        tk.Checkbutton(controls, text="Follow drone", variable=self.follow_var).pack(side="left")
        tk.Button(controls, text="Clear trail", command=self._clear_trail).pack(side="right")

    def _arrow_icon(self, yaw_deg):
        """Arrow pointing in the yaw direction (0 = North, clockwise). Cached every 5 degrees."""
        key = int(round(yaw_deg / 5.0)) % 72
        icon = self.icon_cache.get(key)
        if icon is None:
            big = 96
            img = Image.new("RGBA", (big, big), (0, 0, 0, 0))
            draw = ImageDraw.Draw(img)
            draw.polygon([(48, 6), (78, 84), (48, 66), (18, 84)],
                         fill=(230, 60, 40, 255), outline=(255, 255, 255, 255))
            img = img.rotate(-key * 5, resample=Image.BICUBIC)  # PIL rotates counterclockwise
            img = img.resize((44, 44), Image.LANCZOS)
            icon = ImageTk.PhotoImage(img)
            self.icon_cache[key] = icon  # keep a reference, otherwise Tk drops the image
        return key, icon

    def _update_map(self):
        if self.map_widget is None or self.vehicle is None:
            return
        try:
            loc = self.vehicle.location.global_relative_frame
            lat, lon = loc.lat, loc.lon
            yaw = self.vehicle.attitude.yaw
        except Exception:
            return
        # No GPS position yet
        if lat is None or lon is None or (abs(lat) < 1e-6 and abs(lon) < 1e-6):
            return

        yaw_deg = math.degrees(yaw) % 360 if yaw is not None else 0.0
        key, icon = self._arrow_icon(yaw_deg)

        if self.drone_marker is None:
            self.drone_marker = self.map_widget.set_marker(lat, lon, icon=icon, icon_anchor="center")
            self.last_icon_key = key
            self.map_widget.set_position(lat, lon)
            self.map_widget.set_zoom(18)
        else:
            self.drone_marker.set_position(lat, lon)
            if key != self.last_icon_key:
                self.drone_marker.change_icon(icon)
                self.last_icon_key = key
            if self.follow_var.get():
                self.map_widget.set_position(lat, lon)

        self._update_trail(lat, lon)

    def _update_trail(self, lat, lon):
        if self.trail:
            last_lat, last_lon = self.trail[-1]
            dy = (lat - last_lat) * 110540.0
            dx = (lon - last_lon) * 111320.0 * math.cos(math.radians(lat))
            if math.hypot(dx, dy) < TRAIL_MIN_STEP_M:
                return
        self.trail.append((lat, lon))
        if len(self.trail) > TRAIL_MAX_POINTS:
            self.trail.pop(0)
        if len(self.trail) >= 2:
            if self.trail_path is None:
                self.trail_path = self.map_widget.set_path(list(self.trail), color="#3E69CB", width=3)
            else:
                self.trail_path.set_position_list(list(self.trail))

    def _clear_trail(self):
        self.trail = []
        if self.trail_path is not None:
            self.trail_path.delete()
            self.trail_path = None

    def _reset_map(self):
        """Called when the connection is closed."""
        if self.map_widget is None:
            return
        if self.drone_marker is not None:
            self.drone_marker.delete()
            self.drone_marker = None
        self.last_icon_key = None
        self._clear_trail()

    # ------------------------------------------------------------ main loop
    def _tick(self):
        self._update_rc()
        self._update_status()
        self.map_ticks += 1
        if self.map_ticks % MAP_EVERY_N_TICKS == 0:
            self._update_map()
        self.after(TICK_MS, self._tick)

    def _update_rc(self):
        active = self.rc_var.get() and self.vehicle is not None
        if active:
            p = self.pressed
            self.roll = RC_MID + (DEFLECT if "Right" in p else 0) - (DEFLECT if "Left" in p else 0)
            # Stick forward = lower PWM, so Up arrow lowers the pitch value
            self.pitch = RC_MID + (DEFLECT if "Down" in p else 0) - (DEFLECT if "Up" in p else 0)
            self.yaw = RC_MID + (DEFLECT if "d" in p else 0) - (DEFLECT if "a" in p else 0)
            try:
                mode = self.vehicle.mode.name
            except Exception:
                mode = None
            if mode in ("STABILIZE", "ACRO"):
                # Throttle is motor power: it stays where you leave it
                if "w" in p:
                    self.throttle = min(RC_MAX, self.throttle + THROTTLE_STEP)
                if "s" in p:
                    self.throttle = max(RC_MIN, self.throttle - THROTTLE_STEP)
            else:
                # LOITER / ALT_HOLD / POSHOLD: throttle is climb rate, 1500 = hold altitude
                self.throttle = RC_MID + (DEFLECT if "w" in p else 0) - (DEFLECT if "s" in p else 0)
            if "x" in p:
                self.throttle = RC_MIN
            try:
                # ArduCopter channels: 1 roll, 2 pitch, 3 throttle, 4 yaw
                self.vehicle.channels.overrides = {
                    "1": self.roll, "2": self.pitch, "3": self.throttle, "4": self.yaw,
                }
            except Exception:
                pass
        else:
            self.roll = self.pitch = self.yaw = RC_MID
            self.throttle = RC_MIN

        self._draw_stick(self.left_stick, self.yaw, self.throttle, up_is_high=True)
        self._draw_stick(self.right_stick, self.roll, self.pitch, up_is_high=False)
        self.rc_text.config(
            text=f"CH1 Roll     {self.roll}\n"
                 f"CH2 Pitch    {self.pitch}\n"
                 f"CH3 Throttle {self.throttle}\n"
                 f"CH4 Yaw      {self.yaw}"
        )

    def _draw_stick(self, canvas, x_value, y_value, up_is_high):
        s = STICK_SIZE
        canvas.delete("all")
        canvas.create_line(s / 2, 0, s / 2, s, fill="lightgray")
        canvas.create_line(0, s / 2, s, s / 2, fill="lightgray")
        x = 4 + (x_value - RC_MIN) / (RC_MAX - RC_MIN) * (s - 8)
        if up_is_high:
            y = 4 + (RC_MAX - y_value) / (RC_MAX - RC_MIN) * (s - 8)
        else:
            y = 4 + (y_value - RC_MIN) / (RC_MAX - RC_MIN) * (s - 8)
        canvas.create_oval(x - 6, y - 6, x + 6, y + 6, fill="tomato", outline="")

    def _update_status(self):
        v = self.vehicle
        if v is None:
            if not self.connecting:
                self.conn_label.config(text="Connected: No", fg="red")
            self.mode_label.config(text="Mode: -")
            self.armed_label.config(text="Armed: -")
            self.alt_label.config(text="Altitude: -")
            self.heading_label.config(text="Yaw: -")
            return
        try:
            hb = v.last_heartbeat
            if hb is not None and hb < 5:
                self.conn_label.config(text="Connected: Yes", fg="green")
            else:
                self.conn_label.config(text="Connected: no heartbeat", fg="orange")

            mode = v.mode.name if v.mode is not None else None
            self.mode_label.config(text=f"Mode: {mode or '-'}")

            armed = v.armed
            self.armed_label.config(text="Armed: " + ("-" if armed is None else ("Yes" if armed else "No")))

            alt = v.location.global_relative_frame.alt
            self.alt_label.config(text="Altitude: " + ("-" if alt is None else f"{alt:.1f} m"))

            yaw = v.attitude.yaw
            self.heading_label.config(text="Yaw: " + ("-" if yaw is None else f"{math.degrees(yaw) % 360:.0f}\u00b0"))
        except Exception:
            pass


if __name__ == "__main__":
    GCS().mainloop()
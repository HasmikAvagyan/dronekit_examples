import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping  # fix for Python 3.10+

import threading
import tkinter as tk
from tkinter import messagebox

from dronekit import connect, VehicleMode


class GCS(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("GCS")
        self.geometry("360x520")
        self.vehicle = None

        # Connection string input
        tk.Label(self, text="Connection string:").pack(pady=(10, 0))
        self.conn_entry = tk.Entry(self, width=35)
        self.conn_entry.insert(0, "udp:127.0.0.1:5760")
        self.conn_entry.pack(pady=5)

        # Connect button
        self.connect_btn = tk.Button(self, text="Connect", command=self.connect_vehicle)
        self.connect_btn.pack(pady=5)

        self.status = tk.Label(self, text="Status: disconnected", fg="red")
        self.status.pack(pady=5)

        # Mode buttons
        tk.Label(self, text="Flight modes:").pack(pady=(10, 0))
        mode_frame = tk.Frame(self)
        mode_frame.pack(pady=5)
        for mode in ["GUIDED", "STABILIZE", "LOITER", "LAND", "RTL"]:
            tk.Button(
                mode_frame, text=mode, width=12,
                command=lambda m=mode: self.set_mode(m),
            ).pack(pady=2)

        # Arm button
        tk.Button(self, text="Arm", width=12, command=self.arm).pack(pady=(10, 2))

        # Close connection button
        tk.Button(self, text="Close Connection", command=self.close_connection).pack(pady=5)

    def connect_vehicle(self):
        conn_str = self.conn_entry.get().strip()
        self.status.config(text="Status: connecting...", fg="orange")
        # Run in a thread so the window doesn't freeze
        threading.Thread(target=self._connect, args=(conn_str,), daemon=True).start()

    def _connect(self, conn_str):
        try:
            self.vehicle = connect(conn_str, wait_ready=True, timeout=60, heartbeat_timeout=30)
            self.after(0, lambda: self.status.config(text="Status: connected", fg="green"))
        except Exception as e:
            msg = str(e)  # save the message now, before 'e' disappears
            self.after(0, lambda: self.status.config(text="Status: disconnected", fg="red"))
            self.after(0, lambda: messagebox.showerror("Connection error", msg))

    def _check(self):
        if self.vehicle is None:
            messagebox.showwarning("Not connected", "Connect to the vehicle first.")
            return False
        return True

    def set_mode(self, mode):
        if self._check():
            self.vehicle.mode = VehicleMode(mode)

    def arm(self):
        if self._check():
            self.vehicle.armed = True

    def close_connection(self):
        if self.vehicle is not None:
            self.vehicle.close()
            self.vehicle = None
        self.status.config(text="Status: disconnected", fg="red")


if __name__ == "__main__":
    GCS().mainloop()
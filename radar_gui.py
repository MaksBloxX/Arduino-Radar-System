import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import math
import time
from collections import deque


# ============================================================
# SETTINGS
# ============================================================

BAUD_RATE = 9600

WINDOW_WIDTH = 1250
WINDOW_HEIGHT = 760

DEFAULT_RANGE = 100
DEFAULT_DETECTION = 50

# Sensor limits (HC-SR04 style). Readings outside are treated as "no echo".
SENSOR_MIN_CM = 2
SENSOR_MAX_CM = 400

# --- Object detection tuning ---
MIN_OBJECT_POINTS = 3      # fewer points than this = noise, not an object
MAX_GAP_DEG = 4            # max angular gap inside one object (tolerates missed readings)
JUMP_ABS_CM = 10           # max distance change between neighbouring points ...
JUMP_REL = 0.20            # ... or this fraction of the distance, whichever is larger
MIN_LIFETIME = 2.0         # minimum seconds a point stays (auto-extends to sweep period)

# Sweep beam trail length
TRAIL_LENGTH = 14


# ============================================================
# RADAR APPLICATION
# ============================================================

class RadarApp:

    def __init__(self, root):

        self.root = root
        self.root.title("Arduino Radar System v3.1 - Object Mapping")
        self.root.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        self.root.minsize(1000, 650)
        self.root.configure(bg="#050505")

        # Serial
        self.arduino = None
        self.connected = False
        self.running = False

        # Current data
        self.angle = 90
        self.distance = SENSOR_MAX_CM

        # Settings
        self.radar_range = DEFAULT_RANGE
        self.detection_limit = DEFAULT_DETECTION

        # Geometry (set properly in draw_radar)
        self.center_x = 400
        self.center_y = 470
        self.radar_radius = 400

        # Scan data
        # bins: rounded angle -> {"angle", "dist", "t"}
        self.bins = {}
        self.beam_trail = deque(maxlen=TRAIL_LENGTH)
        self.objects = []

        # Sweep tracking (used to auto-size point lifetime)
        self.last_angle = None
        self.direction = 0
        self.last_flip = None
        self.half_period = 1.0
        self.point_lifetime = MIN_LIFETIME

        self.mapping_enabled = True

        # UI
        self.create_styles()
        self.create_header()
        self.create_connection_panel()
        self.create_main_area()
        self.create_control_panel()
        self.create_status_bar()

        self.refresh_ports()

        # Static radar grid only redraws on resize / range change
        self.canvas.bind("<Configure>", lambda e: self.draw_radar())

        self.root.protocol("WM_DELETE_WINDOW", self.close_application)
        self.root.after(50, self.update_application)

    # ========================================================
    # UI HELPERS
    # ========================================================

    def create_styles(self):
        self.style = ttk.Style()
        try:
            self.style.theme_use("clam")
        except tk.TclError:
            pass

    def make_button(self, parent, text, command, bg, fg,
                    active="#303030", size=10, **kw):
        return tk.Button(
            parent, text=text, command=command,
            bg=bg, fg=fg,
            activebackground=active, activeforeground="white",
            relief="flat", font=("Consolas", size, "bold"), **kw
        )

    # ========================================================
    # HEADER
    # ========================================================

    def create_header(self):

        header = tk.Frame(self.root, bg="#090909", height=65)
        header.pack(fill="x")
        header.pack_propagate(False)

        tk.Label(
            header,
            text="📡  ARDUINO RADAR SYSTEM  •  OBJECT MAPPING",
            bg="#090909", fg="#00ff66",
            font=("Consolas", 20, "bold")
        ).pack(side="left", padx=20)

        self.header_status = tk.Label(
            header, text="● DISCONNECTED",
            bg="#090909", fg="#ff3333",
            font=("Consolas", 12, "bold")
        )
        self.header_status.pack(side="right", padx=20)

    # ========================================================
    # CONNECTION PANEL
    # ========================================================

    def create_connection_panel(self):

        panel = tk.Frame(self.root, bg="#101010", height=65)
        panel.pack(fill="x", padx=10, pady=(8, 5))
        panel.pack_propagate(False)

        tk.Label(
            panel, text="COM PORT", bg="#101010", fg="#777777",
            font=("Consolas", 9)
        ).pack(side="left", padx=(15, 5))

        self.port_combo = ttk.Combobox(panel, width=12, state="readonly")
        self.port_combo.pack(side="left", padx=5)

        self.make_button(
            panel, "⟳ REFRESH", self.refresh_ports,
            "#202020", "#00ff66", padx=10
        ).pack(side="left", padx=5)

        self.connect_button = self.make_button(
            panel, "CONNECT", self.toggle_connection,
            "#003d20", "#00ff66", active="#005c30", padx=15
        )
        self.connect_button.pack(side="left", padx=10)

        tk.Label(
            panel, text="BAUD", bg="#101010", fg="#777777",
            font=("Consolas", 9)
        ).pack(side="left", padx=(25, 5))

        tk.Label(
            panel, text=str(BAUD_RATE), bg="#151515", fg="#00ff66",
            width=8, font=("Consolas", 10, "bold")
        ).pack(side="left")

    # ========================================================
    # MAIN AREA
    # ========================================================

    def create_main_area(self):

        main = tk.Frame(self.root, bg="#050505")
        main.pack(fill="both", expand=True, padx=10)

        radar_frame = tk.Frame(main, bg="#050505")
        radar_frame.pack(side="left", fill="both", expand=True)

        self.canvas = tk.Canvas(
            radar_frame, bg="#000000",
            highlightthickness=1, highlightbackground="#003d20"
        )
        self.canvas.pack(fill="both", expand=True)

        # Info panel
        self.info_panel = tk.Frame(main, bg="#101010", width=275)
        self.info_panel.pack(side="right", fill="y", padx=(10, 0))
        self.info_panel.pack_propagate(False)

        tk.Label(
            self.info_panel, text="LIVE DATA",
            bg="#101010", fg="#00ff66",
            font=("Consolas", 16, "bold")
        ).pack(pady=(14, 8))

        self.angle_label = self.create_data_label(self.info_panel, "ANGLE")
        self.distance_label = self.create_data_label(self.info_panel, "DISTANCE")
        self.object_label = self.create_data_label(self.info_panel, "OBJECT")
        self.count_label = self.create_data_label(self.info_panel, "OBJECTS FOUND")
        self.nearest_label = self.create_data_label(self.info_panel, "NEAREST")
        self.scan_label = self.create_data_label(self.info_panel, "SCAN STATUS")
        self.port_label = self.create_data_label(self.info_panel, "PORT")

        tk.Frame(self.info_panel, bg="#222222", height=2).pack(
            fill="x", padx=20, pady=8
        )

        self.mapping_var = tk.BooleanVar(value=True)

        tk.Checkbutton(
            self.info_panel, text="Object Mapping",
            variable=self.mapping_var, command=self.toggle_mapping,
            bg="#101010", fg="#00ff66", selectcolor="#202020",
            activebackground="#101010", activeforeground="#00ff66",
            font=("Consolas", 10, "bold")
        ).pack(pady=3)

        tk.Label(
            self.info_panel, text="SERIAL DATA",
            bg="#101010", fg="#00ff66",
            font=("Consolas", 10, "bold")
        ).pack(pady=(6, 3))

        self.serial_text = tk.Text(
            self.info_panel, bg="#050505", fg="#00cc55",
            insertbackground="#00ff66",
            font=("Consolas", 8), height=6, relief="flat"
        )
        self.serial_text.pack(fill="both", expand=True, padx=15, pady=8)

    def create_data_label(self, parent, title):

        frame = tk.Frame(parent, bg="#101010")
        frame.pack(fill="x", padx=20, pady=2)

        tk.Label(
            frame, text=title, bg="#101010", fg="#666666",
            font=("Consolas", 8)
        ).pack(anchor="w")

        value = tk.Label(
            frame, text="--", bg="#101010", fg="#00ff66",
            font=("Consolas", 15, "bold")
        )
        value.pack(anchor="w")

        return value

    # ========================================================
    # CONTROL PANEL
    # ========================================================

    def create_control_panel(self):

        panel = tk.Frame(self.root, bg="#101010", height=100)
        panel.pack(fill="x", padx=10, pady=8)
        panel.pack_propagate(False)

        self.make_button(
            panel, "▶ START RADAR", self.start_radar,
            "#004d26", "#00ff66", active="#007a3d",
            size=11, padx=18, pady=8
        ).pack(side="left", padx=(15, 5), pady=15)

        self.make_button(
            panel, "■ STOP", self.stop_radar,
            "#4d0000", "#ff4444", active="#750000",
            size=11, padx=20, pady=8
        ).pack(side="left", padx=5)

        self.make_button(
            panel, "⌫ CLEAR MAP", self.clear_map,
            "#202020", "#00ff66", padx=15, pady=8
        ).pack(side="left", padx=(15, 5))

        # RANGE
        tk.Label(
            panel, text="RANGE", bg="#101010", fg="#777777",
            font=("Consolas", 8)
        ).pack(side="left", padx=(20, 5))

        self.range_value = tk.Label(
            panel, text=f"{DEFAULT_RANGE} cm", bg="#101010", fg="#00ff66",
            font=("Consolas", 9, "bold"), width=7
        )

        self.range_scale = tk.Scale(
            panel, from_=20, to=400, resolution=10,
            orient="horizontal", length=160,
            bg="#101010", fg="#00ff66", troughcolor="#202020",
            highlightthickness=0, command=self.change_range
        )
        self.range_scale.set(DEFAULT_RANGE)
        self.range_scale.pack(side="left")
        self.range_value.pack(side="left")

        # DETECTION
        tk.Label(
            panel, text="DETECT", bg="#101010", fg="#777777",
            font=("Consolas", 8)
        ).pack(side="left", padx=(15, 5))

        self.detect_value = tk.Label(
            panel, text=f"{DEFAULT_DETECTION} cm", bg="#101010", fg="#ff4444",
            font=("Consolas", 9, "bold"), width=7
        )

        self.detect_scale = tk.Scale(
            panel, from_=5, to=400, resolution=5,
            orient="horizontal", length=130,
            bg="#101010", fg="#ff4444", troughcolor="#202020",
            highlightthickness=0, command=self.change_detection
        )
        self.detect_scale.set(DEFAULT_DETECTION)
        self.detect_scale.pack(side="left")
        self.detect_value.pack(side="left")

        self.make_button(
            panel, "⛶", self.toggle_fullscreen,
            "#202020", "#00ff66", size=15, width=3
        ).pack(side="right", padx=15)

    # ========================================================
    # STATUS BAR
    # ========================================================

    def create_status_bar(self):

        self.status_bar = tk.Label(
            self.root, text="Ready • Select Arduino COM port",
            bg="#003018", fg="#00ff66", anchor="w", padx=15,
            font=("Consolas", 9)
        )
        self.status_bar.pack(fill="x")

    # ========================================================
    # PORTS / CONNECTION
    # ========================================================

    def refresh_ports(self):

        names = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = names

        if "COM6" in names:
            self.port_combo.set("COM6")
        elif names:
            self.port_combo.current(0)
        else:
            self.port_combo.set("")

        self.status_bar.config(text=f"{len(names)} serial port(s) found")

    def toggle_connection(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):

        port = self.port_combo.get()

        if not port:
            messagebox.showwarning("No COM Port", "Select an Arduino COM port.")
            return

        try:
            self.arduino = serial.Serial(port, BAUD_RATE, timeout=0.05)

            # Arduino resets when the port opens; wait for it
            self.status_bar.config(text="Connecting...")
            self.root.update_idletasks()
            time.sleep(2)

            self.arduino.reset_input_buffer()
            self.connected = True

            self.connect_button.config(
                text="DISCONNECT", bg="#4d0000", fg="#ff4444"
            )
            self.header_status.config(text="● CONNECTED", fg="#00ff66")
            self.port_label.config(text=port)
            self.status_bar.config(text=f"Connected • {port} • {BAUD_RATE} baud")
            self.log_serial("CONNECTED")

        except serial.SerialException as error:
            self.arduino = None
            messagebox.showerror("Connection Error", str(error))

    def disconnect(self):

        self.stop_radar()

        try:
            if self.arduino:
                self.arduino.close()
        except Exception:
            pass

        self.arduino = None
        self.connected = False

        self.connect_button.config(text="CONNECT", bg="#003d20", fg="#00ff66")
        self.header_status.config(text="● DISCONNECTED", fg="#ff3333")
        self.port_label.config(text="--")
        self.status_bar.config(text="Disconnected")

    def handle_connection_lost(self):

        self.running = False
        self.connected = False

        try:
            if self.arduino:
                self.arduino.close()
        except Exception:
            pass

        self.arduino = None

        self.header_status.config(text="● DISCONNECTED", fg="#ff3333")
        self.connect_button.config(text="CONNECT", bg="#003d20", fg="#00ff66")
        self.port_label.config(text="--")
        self.status_bar.config(text="Arduino connection lost")

    # ========================================================
    # START / STOP
    # ========================================================

    def start_radar(self):

        if not self.connected:
            messagebox.showwarning("Not Connected", "Connect Arduino first.")
            return

        try:
            self.arduino.write(b"START\n")
        except Exception:
            self.handle_connection_lost()
            return

        self.running = True
        self.reset_scan_state()

        self.status_bar.config(text="Radar scanning • Object mapping active")
        self.log_serial(">>> START")

    def stop_radar(self):

        if self.connected and self.arduino:
            try:
                self.arduino.write(b"STOP\n")
            except Exception:
                pass

        self.running = False
        self.status_bar.config(text="Radar stopped")

    def reset_scan_state(self):
        self.bins.clear()
        self.beam_trail.clear()
        self.objects = []
        self.last_angle = None
        self.direction = 0
        self.last_flip = None

    # ========================================================
    # SETTINGS CALLBACKS
    # ========================================================

    def change_range(self, value):
        self.radar_range = int(float(value))
        self.range_value.config(text=f"{self.radar_range} cm")
        self.draw_radar()

    def change_detection(self, value):
        self.detection_limit = int(float(value))
        self.detect_value.config(text=f"{self.detection_limit} cm")

    def toggle_mapping(self):
        self.mapping_enabled = self.mapping_var.get()
        if not self.mapping_enabled:
            self.bins.clear()
            self.objects = []
            self.canvas.delete("object")

    def clear_map(self):
        self.bins.clear()
        self.beam_trail.clear()
        self.objects = []
        self.canvas.delete("object")
        self.canvas.delete("sweep")
        self.status_bar.config(text="Object map cleared")

    # ========================================================
    # SERIAL LOG
    # ========================================================

    def log_serial(self, text):

        self.serial_text.insert("end", text + "\n")
        self.serial_text.see("end")

        lines = int(self.serial_text.index("end-1c").split(".")[0])
        if lines > 100:
            self.serial_text.delete("1.0", "20.0")

    # ========================================================
    # COORDINATES
    # ========================================================

    def angle_to_xy(self, angle, distance):
        """Screen coordinates. 0° = LEFT, 90° = TOP, 180° = RIGHT."""

        radius = self.radar_radius * distance / self.radar_range
        rad = math.radians(angle)

        x = self.center_x - radius * math.cos(rad)
        y = self.center_y - radius * math.sin(rad)

        return x, y

    @staticmethod
    def angle_to_cm(angle, distance):
        """Real-world coordinates (cm), used for width estimates."""
        rad = math.radians(angle)
        return -distance * math.cos(rad), distance * math.sin(rad)

    # ========================================================
    # STATIC RADAR GRID (only redrawn on resize / range change)
    # ========================================================

    def draw_radar(self):

        self.canvas.delete("radar")

        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()

        if width < 100:
            width = 800
        if height < 100:
            height = 500

        self.center_x = width // 2
        self.center_y = height - 30
        self.radar_radius = max(50, min(width * 0.43, height - 80))

        # Distance rings
        for i in range(1, 6):

            distance = self.radar_range * i / 5
            radius = self.radar_radius * i / 5

            self.canvas.create_arc(
                self.center_x - radius, self.center_y - radius,
                self.center_x + radius, self.center_y + radius,
                start=0, extent=180,
                outline="#006633", width=1, style="arc", tags="radar"
            )

            self.canvas.create_text(
                self.center_x + 8, self.center_y - radius,
                text=f"{int(distance)} cm",
                fill="#008844", anchor="w",
                font=("Consolas", 8), tags="radar"
            )

        # Base line
        self.canvas.create_line(
            self.center_x - self.radar_radius, self.center_y,
            self.center_x + self.radar_radius, self.center_y,
            fill="#00aa55", width=2, tags="radar"
        )

        # Angle lines + labels
        for angle in range(0, 181, 30):

            x, y = self.angle_to_xy(angle, self.radar_range)

            self.canvas.create_line(
                self.center_x, self.center_y, x, y,
                fill="#003d22", width=1, tags="radar"
            )

            lx, ly = self.angle_to_xy(
                angle, self.radar_range * (self.radar_radius + 22) / self.radar_radius
            )

            self.canvas.create_text(
                lx, ly, text=f"{angle}°", fill="#00cc66",
                font=("Consolas", 9, "bold"), tags="radar"
            )

        # Detection limit ring (dashed red) so you can see what is "in zone"
        limit = min(self.detection_limit, self.radar_range)
        r = self.radar_radius * limit / self.radar_range
        self.canvas.create_arc(
            self.center_x - r, self.center_y - r,
            self.center_x + r, self.center_y + r,
            start=0, extent=180, outline="#661111", dash=(4, 4),
            style="arc", tags="radar"
        )

        # Grid always sits under dynamic layers
        self.canvas.tag_lower("radar")

    # ========================================================
    # SWEEP BEAM + TRAIL
    # ========================================================

    def draw_sweep(self):

        self.canvas.delete("sweep")

        n = len(self.beam_trail)

        for i, angle in enumerate(self.beam_trail):

            # oldest = dimmest, newest = brightest
            level = i / max(1, n - 1) if n > 1 else 1.0
            green = int(40 + 215 * level)
            color = f"#00{green:02x}{int(40 + 40 * level):02x}"

            x, y = self.angle_to_xy(angle, self.radar_range)

            self.canvas.create_line(
                self.center_x, self.center_y, x, y,
                fill=color, width=3 if i == n - 1 else 1,
                tags="sweep"
            )

    # ========================================================
    # OBJECT DETECTION
    # ========================================================

    def add_reading(self, angle, distance):
        """Store one reading in its 1° bin. A miss clears that bin."""

        if not self.mapping_enabled:
            return

        key = int(round(angle))

        valid = SENSOR_MIN_CM <= distance < SENSOR_MAX_CM

        if valid:
            self.bins[key] = {
                "angle": angle,
                "dist": distance,
                "t": time.time()
            }
        else:
            # Nothing echoed at this angle on this pass -> whatever was
            # here before is gone (this is what removes stale ghosts)
            self.bins.pop(key, None)

    def expire_points(self):

        now = time.time()
        life = self.point_lifetime

        stale = [k for k, p in self.bins.items() if now - p["t"] > life]
        for k in stale:
            del self.bins[k]

    def find_objects(self):
        """Group neighbouring in-zone points into objects."""

        limit = min(self.detection_limit, self.radar_range)

        pts = sorted(
            (p for p in self.bins.values() if p["dist"] <= limit),
            key=lambda p: p["angle"]
        )

        groups = []
        current = []

        for p in pts:

            if current:
                prev = current[-1]

                gap = p["angle"] - prev["angle"]
                jump = abs(p["dist"] - prev["dist"])
                allowed = max(JUMP_ABS_CM, JUMP_REL * min(p["dist"], prev["dist"]))

                if gap <= MAX_GAP_DEG and jump <= allowed:
                    current.append(p)
                    continue

                groups.append(current)

            current = [p]

        if current:
            groups.append(current)

        objects = []

        for g in groups:

            if len(g) < MIN_OBJECT_POINTS:
                continue

            first, last = g[0], g[-1]

            x1, y1 = self.angle_to_cm(first["angle"], first["dist"])
            x2, y2 = self.angle_to_cm(last["angle"], last["dist"])

            nearest = min(g, key=lambda p: p["dist"])

            objects.append({
                "points": g,
                "avg": sum(p["dist"] for p in g) / len(g),
                "nearest": nearest,
                "center_angle": (first["angle"] + last["angle"]) / 2,
                "width": math.hypot(x2 - x1, y2 - y1),
                "newest": max(p["t"] for p in g)
            })

        objects.sort(key=lambda o: o["nearest"]["dist"])
        return objects

    # ========================================================
    # DRAW OBJECT MAP
    # ========================================================

    def draw_object_map(self):

        self.canvas.delete("object")

        if not self.mapping_enabled:
            return

        now = time.time()

        # Faint raw echoes (everything the sensor saw)
        for p in self.bins.values():
            if p["dist"] > self.radar_range:
                continue
            x, y = self.angle_to_xy(p["angle"], p["dist"])
            self.canvas.create_oval(
                x - 2, y - 2, x + 2, y + 2,
                fill="#335533", outline="", tags="object"
            )

        for index, obj in enumerate(self.objects, 1):

            pts = obj["points"]

            fresh = max(0.3, 1.0 - (now - obj["newest"]) / self.point_lifetime)
            color = f"#{int(255 * fresh):02x}2020"

            coords = []
            for p in pts:
                x, y = self.angle_to_xy(p["angle"], p["dist"])
                coords += [x, y]

            # Edge rays show the object's angular span
            for p in (pts[0], pts[-1]):
                x, y = self.angle_to_xy(p["angle"], p["dist"])
                self.canvas.create_line(
                    self.center_x, self.center_y, x, y,
                    fill="#551111", dash=(2, 5), tags="object"
                )

            # Outline
            self.canvas.create_line(*coords, fill=color, width=3, tags="object")

            # Points
            for i in range(0, len(coords), 2):
                self.canvas.create_oval(
                    coords[i] - 3, coords[i + 1] - 3,
                    coords[i] + 3, coords[i + 1] + 3,
                    fill=color, outline="", tags="object"
                )

            # Label
            lx, ly = self.angle_to_xy(obj["center_angle"], obj["avg"])

            self.canvas.create_text(
                lx, ly - 26,
                text=(
                    f"OBJ {index}\n"
                    f"{obj['nearest']['dist']:.0f} cm • w≈{obj['width']:.0f} cm"
                ),
                fill="#ff5555", font=("Consolas", 9, "bold"),
                tags="object"
            )

    # ========================================================
    # SERIAL READING
    # ========================================================

    def parse_line(self, data):

        data = data.strip().rstrip(".").strip()

        if "," not in data:
            return None

        parts = data.split(",")

        if len(parts) != 2:
            return None

        try:
            angle = float(parts[0])
            distance = float(parts[1])
        except ValueError:
            return None

        if not (0 <= angle <= 180) or distance < 0:
            return None

        return angle, distance

    def track_sweep(self, angle):
        """Detect direction flips to learn the sweep period."""

        if self.last_angle is not None and angle != self.last_angle:

            direction = 1 if angle > self.last_angle else -1

            if self.direction and direction != self.direction:

                now = time.time()

                if self.last_flip is not None:
                    half = now - self.last_flip
                    if half > 0.3:
                        # smooth it
                        self.half_period = 0.7 * self.half_period + 0.3 * half
                        self.point_lifetime = max(
                            MIN_LIFETIME, self.half_period * 2.4
                        )

                self.last_flip = now

            self.direction = direction

        self.last_angle = angle

    def read_serial(self):

        if not self.connected:
            return

        try:
            for _ in range(200):  # cap per tick so the GUI never stalls

                if not self.arduino.in_waiting:
                    break

                raw = self.arduino.readline()
                data = raw.decode("ascii", errors="ignore")

                parsed = self.parse_line(data)

                if parsed is None:
                    continue

                angle, distance = parsed

                self.angle = angle
                self.distance = distance

                self.track_sweep(angle)
                self.beam_trail.append(angle)
                self.add_reading(angle, distance)

                self.log_serial(f"{angle:.0f},{distance:.1f}")

        except (serial.SerialException, OSError):
            self.handle_connection_lost()

    # ========================================================
    # INFO PANEL
    # ========================================================

    def update_information(self):

        self.angle_label.config(text=f"{self.angle:.0f}°")

        if self.distance >= SENSOR_MAX_CM:
            self.distance_label.config(text="OUT OF RANGE")
        else:
            self.distance_label.config(text=f"{self.distance:.1f} cm")

        limit = min(self.detection_limit, self.radar_range)

        if SENSOR_MIN_CM <= self.distance <= limit:
            self.object_label.config(text="● DETECTED", fg="#ff3333")
        else:
            self.object_label.config(text="○ CLEAR", fg="#00ff66")

        self.count_label.config(
            text=str(len(self.objects)),
            fg="#ff3333" if self.objects else "#00ff66"
        )

        if self.objects:
            n = self.objects[0]["nearest"]
            self.nearest_label.config(
                text=f"{n['dist']:.0f}cm @{n['angle']:.0f}°", fg="#ff3333"
            )
        else:
            self.nearest_label.config(text="--", fg="#00ff66")

        if self.running:
            self.scan_label.config(text="SCANNING", fg="#00ff66")
        else:
            self.scan_label.config(text="STOPPED", fg="#ff4444")

    # ========================================================
    # MAIN LOOP
    # ========================================================

    def update_application(self):

        if self.connected:
            self.read_serial()

        self.expire_points()
        self.objects = self.find_objects() if self.mapping_enabled else []

        if self.connected:
            self.draw_sweep()
            self.draw_object_map()
        else:
            self.canvas.delete("sweep")
            self.canvas.delete("object")

        self.update_information()

        self.root.after(50, self.update_application)

    # ========================================================
    # FULLSCREEN / CLOSE
    # ========================================================

    def toggle_fullscreen(self):
        current = self.root.attributes("-fullscreen")
        self.root.attributes("-fullscreen", not current)

    def close_application(self):

        if self.connected and self.arduino:
            try:
                self.arduino.write(b"STOP\n")
                time.sleep(0.1)
                self.arduino.close()
            except Exception:
                pass

        self.root.destroy()


# ============================================================
# RUN
# ============================================================

if __name__ == "__main__":

    root = tk.Tk()
    app = RadarApp(root)
    root.mainloop()

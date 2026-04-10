from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import csv

# --------- CONFIG ---------
CONN_STR_1 = "tcp:127.0.0.1:8100"  # Drone 1 (top)
CONN_STR_2 = "tcp:127.0.0.1:8200"  # Drone 2 (bottom / base)

DEFAULT_ALT_TOP = 7.0      # default meters (top drone)
DEFAULT_ALT_BOTTOM = 5.0   # default meters (base drone)

DOCK_OFFSET = 0.30         # meters above base drone when "docked"
# --------------------------

# State for target altitudes (set by user in menu option 1)
top_target_alt = DEFAULT_ALT_TOP
base_target_alt = DEFAULT_ALT_BOTTOM

# Logging globals
LOG_FILE = "drone_distance_log.csv"
log_writer = None
log_start_time = None
log_file_handle = None


# ---------- Basic helpers ----------

def connect_vehicle(conn_str, name=""):
    print(f"Connecting to {name} at {conn_str} ...")
    vehicle = connect(conn_str, wait_ready=True)
    print(f"{name} connected.")
    return vehicle


def arm_and_takeoff(vehicle, target_alt, name="Vehicle"):
    print(f"{name}: Arming and taking off to {target_alt:.1f} m")

    # Wait until vehicle is armable
    while not vehicle.is_armable:
        print(f"{name}: Waiting for vehicle to become armable...")
        time.sleep(1)

    # Set mode to GUIDED
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        print(f"{name}: Waiting for GUIDED mode...")
        time.sleep(1)

    # Arm
    vehicle.armed = True
    while not vehicle.armed:
        print(f"{name}: Waiting for arming...")
        time.sleep(1)

    # Takeoff
    vehicle.simple_takeoff(target_alt)

    # Wait until near target altitude, but don't get stuck forever
    start_time = time.time()
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"{name}: Altitude = {alt:.2f} m")

        # Accept "close enough": within 0.7 m OR >= 90% of target
        if alt >= target_alt - 0.7 or alt >= target_alt * 0.90:
            print(f"{name}: Reached target altitude (or close enough).")
            break

        if time.time() - start_time > 40:
            print(f"{name}: Takeoff timeout, continuing anyway.")
            break

        time.sleep(1)


def distance_meters(a, b):
    """Approximate horizontal distance in meters between two LocationGlobalRelative points."""
    dlat = a.lat - b.lat
    dlon = a.lon - b.lon
    return math.sqrt((dlat * 1.113195e5) ** 2 + (dlon * 1.113195e5) ** 2)


def distance_3d(top_loc, base_loc):
    """Full 3D distance between drones (horizontal + vertical)."""
    horiz = distance_meters(top_loc, base_loc)
    vert = top_loc.alt - base_loc.alt
    return math.sqrt(horiz**2 + vert**2)


def angle_diff_deg(a, b):
    """Smallest absolute difference between two headings in degrees."""
    d = (a - b + 180.0) % 360.0 - 180.0
    return abs(d)


def log_distance(phase, horiz, vert, dist3d):
    """Log one sample to CSV."""
    global log_writer, log_start_time, log_file_handle
    if log_writer is None or log_start_time is None:
        return
    t = time.time() - log_start_time
    log_writer.writerow(
        [f"{t:.2f}", phase, f"{horiz:.3f}", f"{vert:.3f}", f"{dist3d:.3f}"]
    )
    log_file_handle.flush()


def set_yaw(vehicle, heading_deg):
    """
    Rotate the drone to an absolute yaw heading (0–360 degrees).
    Uses MAV_CMD_CONDITION_YAW.
    """
    print(f"Commanding yaw to {heading_deg:.1f}°")
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        float(heading_deg),  # target yaw
        10.0,                # yaw rate (deg/s)
        1,                   # direction (1=cw)
        0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


# ---------- ALIGN & DOCK ----------

def align_top_over_base(vehicle_top, vehicle_base):
    """
    Move the top drone above the base drone and wait until aligned,
    then match the top drone's yaw to the base drone's yaw.
    """
    global top_target_alt
    print("\n=== Aligning top drone over base drone ===")

    horiz_tol = 0.7   # <- relaxed (meters)
    alt_tol   = 0.7   # meters
    yaw_tol   = 8.0   # degrees
    max_duration = 60  # seconds

    start_time = time.time()

    final_h = None
    final_v = None
    final_d3 = None
    final_yaw_err = None

    while True:
        base_loc = vehicle_base.location.global_relative_frame
        top_loc  = vehicle_top.location.global_relative_frame

        horiz_dist = distance_meters(top_loc, base_loc)
        alt_err    = top_loc.alt - top_target_alt
        d3         = distance_3d(top_loc, base_loc)
        vert_over_base = top_loc.alt - base_loc.alt

        base_heading = getattr(vehicle_base, "heading", None)
        top_heading  = getattr(vehicle_top, "heading", None)
        if base_heading is not None and top_heading is not None:
            yaw_error = angle_diff_deg(top_heading, base_heading)
        else:
            yaw_error = 999.0  # until heading valid

        print(
            f"[ALIGN] horiz={horiz_dist:.2f} m, "
            f"alt_top={top_loc.alt:.2f} m (target {top_target_alt:.2f}), "
            f"3D={d3:.2f} m, yaw_err={yaw_error:.1f}°"
        )

        log_distance("ALIGN", horiz_dist, vert_over_base, d3)

        # Command position
        target = LocationGlobalRelative(base_loc.lat, base_loc.lon, top_target_alt)
        vehicle_top.simple_goto(target, groundspeed=2.0)

        # Command yaw towards base heading if needed
        if base_heading is not None and yaw_error > yaw_tol:
            set_yaw(vehicle_top, base_heading)

        # Record last values for summary
        final_h = horiz_dist
        final_v = vert_over_base
        final_d3 = d3
        final_yaw_err = yaw_error

        # Exit condition
        if horiz_dist < horiz_tol and abs(alt_err) < alt_tol and yaw_error <= yaw_tol:
            print("Alignment complete: position + yaw within tolerance.")
            break

        if time.time() - start_time > max_duration:
            print("ALIGN timeout reached. Stopping alignment loop.")
            break

        time.sleep(1.0)

    print(
        f"ALIGN final: horiz={final_h:.2f} m, vert_over_base={final_v:.2f} m, "
        f"3D={final_d3:.2f} m, yaw_err={final_yaw_err:.1f}°\n"
    )


def dock_top_to_base(vehicle_top, vehicle_base):
    """
    Docking maneuver: keep top drone over base and descend until ~DOCK_OFFSET above it.
    Also ensures yaw is matched to base at the end.
    """
    print("\n=== Docking top drone toward base drone ===")

    horiz_tol = 0.7   # <- relaxed (same as align; autopilot won't do much better with GPS)
    vert_tol  = 0.12
    yaw_tol   = 8.0
    max_duration = 60  # seconds

    start_time = time.time()

    final_h = None
    final_v = None
    final_d3 = None
    final_yaw_err = None

    while True:
        base_loc = vehicle_base.location.global_relative_frame
        top_loc  = vehicle_top.location.global_relative_frame

        desired_alt = base_loc.alt + DOCK_OFFSET
        horiz_dist  = distance_meters(top_loc, base_loc)
        vert_over_base = top_loc.alt - base_loc.alt
        vert_err    = vert_over_base - DOCK_OFFSET
        d3          = distance_3d(top_loc, base_loc)

        base_heading = getattr(vehicle_base, "heading", None)
        top_heading  = getattr(vehicle_top, "heading", None)
        if base_heading is not None and top_heading is not None:
            yaw_error = angle_diff_deg(top_heading, base_heading)
        else:
            yaw_error = 999.0

        print(
            f"[DOCK] horiz={horiz_dist:.2f} m, "
            f"vert_over_base={vert_over_base:.2f} m (target {DOCK_OFFSET:.2f}), "
            f"3D={d3:.2f} m, yaw_err={yaw_error:.1f}°"
        )

        log_distance("DOCK", horiz_dist, vert_over_base, d3)

        # Position command (slow)
        target = LocationGlobalRelative(base_loc.lat, base_loc.lon, desired_alt)
        vehicle_top.simple_goto(target, groundspeed=0.3)

        # Yaw command if needed
        if base_heading is not None and yaw_error > yaw_tol:
            set_yaw(vehicle_top, base_heading)

        final_h = horiz_dist
        final_v = vert_over_base
        final_d3 = d3
        final_yaw_err = yaw_error

        if (
            horiz_dist < horiz_tol
            and abs(vert_err) < vert_tol
            and yaw_error <= yaw_tol
        ):
            print(
                f"Docking complete: top drone is about "
                f"{vert_over_base:.2f} m above base, yaw matched."
            )
            break

        if time.time() - start_time > max_duration:
            print("DOCK timeout reached. Stopping docking loop.")
            break

        time.sleep(0.7)

    print(
        f"DOCK final: horiz={final_h:.2f} m, vert_over_base={final_v:.2f} m, "
        f"3D={final_d3:.2f} m, yaw_err={final_yaw_err:.1f}°\n"
    )


# ---------- RTL & UI ----------

def return_to_takeoff(vehicle_top, vehicle_base):
    """Set both drones to RTL so they go back and land near their takeoff locations."""
    print("\n=== Returning both drones to takeoff location (RTL) ===")

    for vehicle, name in [
        (vehicle_top, "Drone 1 (top)"),
        (vehicle_base, "Drone 2 (bottom/base)")
    ]:
        print(f"{name}: Switching to RTL mode...")
        vehicle.mode = VehicleMode("RTL")

    while True:
        all_landed = True
        for vehicle, name in [
            (vehicle_top, "Drone 1 (top)"),
            (vehicle_base, "Drone 2 (bottom/base)")
        ]:
            loc = vehicle.location.global_relative_frame
            alt = loc.alt
            print(f"{name}: Altitude = {alt:.1f} m, mode = {vehicle.mode.name}")
            if alt > 1.0:
                all_landed = False

        if all_landed:
            print("Both drones appear to be landed near their takeoff points.")
            break

        time.sleep(2)

    print("RTL sequence complete.\n")


def prompt_altitudes():
    """Ask user for top/base altitudes. If user presses Enter, keep defaults."""
    global top_target_alt, base_target_alt
    print("\n--- Set target altitudes for takeoff ---")
    try:
        top_in = input(
            f"Enter TOP drone altitude in meters [{DEFAULT_ALT_TOP}]: "
        ).strip()
        base_in = input(
            f"Enter BASE drone altitude in meters [{DEFAULT_ALT_BOTTOM}]: "
        ).strip()

        if top_in:
            top_target_alt = float(top_in)
        else:
            top_target_alt = DEFAULT_ALT_TOP

        if base_in:
            base_target_alt = float(base_in)
        else:
            base_target_alt = DEFAULT_ALT_BOTTOM

    except ValueError:
        print("Invalid input. Using default altitudes.")
        top_target_alt = DEFAULT_ALT_TOP
        base_target_alt = DEFAULT_ALT_BOTTOM

    print(f"Top drone target altitude: {top_target_alt:.2f} m")
    print(f"Base drone target altitude: {base_target_alt:.2f} m\n")


def show_menu():
    print("============ MENU ============")
    print("1) Arm + takeoff both drones (set heights)")
    print("2) Align top drone over base drone (position + yaw)")
    print("3) Dock top drone toward base drone (position + yaw)")
    print("4) Land both drones (RTL to takeoff location)")
    print("5) Exit program (keep current modes)")
    print("===============================")


def main():
    global log_writer, log_start_time, log_file_handle

    # 1) Connect to both drones (no arming yet)
    vehicle_top = connect_vehicle(CONN_STR_1, "Drone 1 (top)")
    vehicle_base = connect_vehicle(CONN_STR_2, "Drone 2 (bottom / base)")

    # 2) Setup CSV logging
    log_file_handle = open(LOG_FILE, "w", newline="")
    log_writer = csv.writer(log_file_handle)
    log_writer.writerow(["time_s", "phase", "horiz_m", "vert_m", "dist3d_m"])
    log_start_time = time.time()
    print(f"Logging distance data to {LOG_FILE}\n")

    try:
        while True:
            show_menu()
            choice = input("Enter your choice: ").strip()

            if choice == "1":
                prompt_altitudes()
                arm_and_takeoff(vehicle_top, top_target_alt, "Drone 1 (top)")
                arm_and_takeoff(vehicle_base, base_target_alt, "Drone 2 (bottom / base)")

            elif choice == "2":
                align_top_over_base(vehicle_top, vehicle_base)

            elif choice == "3":
                dock_top_to_base(vehicle_top, vehicle_base)

            elif choice == "4":
                return_to_takeoff(vehicle_top, vehicle_base)

            elif choice == "5":
                print("Exiting program without changing modes.")
                break

            else:
                print("Invalid choice. Please select 1, 2, 3, 4, or 5.")
    finally:
        print("\nClosing vehicle connections...")
        try:
            vehicle_top.close()
        except Exception as e:
            print(f"Error closing top vehicle: {e}")
        try:
            vehicle_base.close()
        except Exception as e:
            print(f"Error closing base vehicle: {e}")

        if log_file_handle is not None:
            log_file_handle.close()
            print(f"Log file {LOG_FILE} closed.")

        print("Done.")


if __name__ == "__main__":
    main()

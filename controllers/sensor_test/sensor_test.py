import math
from controller import Supervisor

START_X, START_Y = -4.0, 0.0                 # Start A
GOAL_X, GOAL_Y = 4.0, 0.0                    # Destination B
DROP_SPOTS = [(4.0, -0.35), (4.0, 0.0), (4.0, 0.35)]   # B pe 3 boxes ki jagah

MAX_SPEED = 5.0       # robot ka max 12.3
PICK_DIST = 0.45      # package ke itne paas aake uthao
DROP_DIST = 0.5       # B ke itne paas aake rakho
HOME_DIST = 0.4       # A ke itne paas aake "wapas aa gaya" maano
NEAR_B = 1.2          # B ke itne paas avoidance band
CARRY_Z = 0.42        # robot ke upar box ki height
LABEL_COLOR = 0xFFFF00

SENSE_DIST = 1.0      # itne door se obstacle ko "dekhna" shuru (bada = jaldi aur halka mudega)
CLEAR_TIME = 0.4      # raasta saaf hone ke baad itni der side yaad rakho
PICK_COOLDOWN = 1.0   # pickup ke baad itni der obstacle check band
STUCK_TIME = 3.5      # itni der 15 cm se kam hile to "atak gaya"
TURN_GAIN = 4.0       # mudne ki taakat
FRONT_W = [0.3, 0.6, 0.9, 1.0, 1.0, 0.9, 0.6, 0.3]   # sonar so0..so7 ka wazan (beech wale zyada)

robot = Supervisor()
dt = int(robot.getBasicTimeStep())

# --- 3 packages dhoondho ---
packages = []
for i in (1, 2, 3):
    node = robot.getFromDef("PACKAGE%d" % i)
    if node is None:
        print("ERROR: world me DEF PACKAGE%d nahi mila" % i)
        raise SystemExit
    packages.append(node.getField("translation"))

# --- motors, GPS, compass, sonar ---
left_motor = robot.getDevice("left wheel")
right_motor = robot.getDevice("right wheel")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

gps = robot.getDevice("gps")
gps.enable(dt)
compass = robot.getDevice("compass")
compass.enable(dt)

sonar = []
for i in range(16):
    s = robot.getDevice("so%d" % i)
    s.enable(dt)
    sonar.append(s)

def dist(i):
    return 5.0 * (1.0 - sonar[i].getValue() / 1024.0)

def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

def set_speeds(l, r):
    left_motor.setVelocity(max(-12.3, min(12.3, l)))
    right_motor.setVelocity(max(-12.3, min(12.3, r)))

def stop():
    set_speeds(0.0, 0.0)

avoid_side = 0           # 0 = bypass nahi, -1 = right mudo, +1 = left mudo
clear_since = None
cooldown_until = 0.0

def drive_to(tx, ty, x, y, heading, avoid):
    """Target ki taraf chalo. Obstacle paas aaye to smooth curve me ek taraf se nikalo.
    True return = abhi obstacle bacha raha hai."""
    global avoid_side, clear_since
    now = robot.getTime()
    if now < cooldown_until:
        avoid = False

    err = wrap(math.atan2(ty - y, tx - x) - heading)
    goal_turn = max(-1.0, min(1.0, err))

    m = 0.0
    front_min = 5.0
    if avoid:
        d = [dist(i) for i in range(8)]       # so0..so7 = aage wale sensors
        w = [max(0.0, (SENSE_DIST - d[i]) / SENSE_DIST) * FRONT_W[i] for i in range(8)]
        m = max(w)                            # sabse paas wala obstacle kitna "zyada" hai
        front_min = min(d[2:6])               # seedha saamne ki doori
        left_sum = sum(w[0:4])
        right_sum = sum(w[4:8])
        if m > 0.05:
            clear_since = None
            if avoid_side == 0:               # side ek baar chuno aur us par tike raho
                avoid_side = -1 if left_sum > right_sum else 1
        elif avoid_side != 0:
            if clear_since is None:
                clear_since = now
            elif now - clear_since > CLEAR_TIME:
                avoid_side = 0
                clear_since = None
    else:
        avoid_side = 0
        clear_since = None

    # obstacle nahi aur goal bahut alag direction me: pehle jagah pe ghoomo
    if avoid_side == 0 and abs(err) > 0.6:
        t = 2.0 * max(-1.5, min(1.5, err))
        set_speeds(-t, t)
        return False

    a = min(1.0, 2.0 * m)                     # obstacle jitna paas, avoidance utna zyada
    turn = (1.0 - a) * goal_turn + a * avoid_side
    f = max(0.0, min(1.0, (front_min - 0.25) / 0.65))   # saamne paas hai to dheere chalo
    v = MAX_SPEED * f
    set_speeds(v - TURN_GAIN * turn, v + TURN_GAIN * turn)
    return avoid_side != 0

# --- status variables ---
n = 0                      # abhi kaun sa package (0, 1, 2)
delivered = 0
state = "GO_TO_PACKAGE"
results = []               # har package ka (time, distance, obstacles)
cycle_start = robot.getTime()
cycle_dist = 0.0
cycle_obst = 0
total_obst = 0
was_avoiding = False
prev = None
step = 0

stuck_ref = None           # atakne ke check ke liye pichli position
recover_until = 0.0        # is time tak peeche hat ke ghoomega
recover_dir = 1            # ghoomne ki side (har baar ulti hoti hai)

print("Delivery robot started! Packages: %d" % len(packages))

while robot.step(dt) != -1:
    x, y, _ = gps.getValues()
    c = compass.getValues()
    heading = math.atan2(c[0], c[1])
    now = robot.getTime()

    if prev is not None:
        cycle_dist += math.hypot(x - prev[0], y - prev[1])
    prev = (x, y)

    # --- atakne se wapas nikalne wala hissa ---
    if now < recover_until:
        if now < recover_until - 0.6:
            set_speeds(-MAX_SPEED, -MAX_SPEED)                    # pehle peeche hato
        else:
            set_speeds(-3.0 * recover_dir, 3.0 * recover_dir)     # phir ghoomo
        if state == "CARRYING":
            packages[n].setSFVec3f([x, y, CARRY_Z])               # box saath rahe
        continue

    # --- atak gaya ya nahi check karo (sirf chalte hue states me) ---
    if state in ("GO_TO_PACKAGE", "CARRYING", "RETURNING"):
        if stuck_ref is None or math.hypot(x - stuck_ref[0], y - stuck_ref[1]) > 0.15:
            stuck_ref = (x, y, now)
        elif now - stuck_ref[2] > STUCK_TIME:
            print("Robot atak gaya, peeche hat raha hai...")
            recover_until = now + 1.4
            recover_dir = -recover_dir
            stuck_ref = None
            avoid_side = 0
            clear_since = None
            cooldown_until = now + 1.0
            continue

    avoiding = False
    target_d = 0.0

    if state == "GO_TO_PACKAGE":
        px, py, _ = packages[n].getSFVec3f()
        target_d = math.hypot(px - x, py - y)
        if target_d < PICK_DIST:
            state = "CARRYING"
            avoid_side = 0
            clear_since = None
            packages[n].setSFVec3f([x, y, CARRY_Z])       # turant robot ke upar rakho
            cooldown_until = now + PICK_COOLDOWN
            print("Package %d picked up!" % (n + 1))
        else:
            drive_to(px, py, x, y, heading, False)

    elif state == "CARRYING":
        packages[n].setSFVec3f([x, y, CARRY_Z])      # box robot ke saath chalta hai
        target_d = math.hypot(GOAL_X - x, GOAL_Y - y)
        if target_d < DROP_DIST:
            stop()
            sx, sy = DROP_SPOTS[n]
            packages[n].setSFVec3f([sx, sy, 0.15])   # B pe rakh do
            delivered += 1
            avoid_side = 0
            clear_since = None
            print("Package %d delivered at B!" % (n + 1))
            state = "RETURNING"
        else:
            avoiding = drive_to(GOAL_X, GOAL_Y, x, y, heading, target_d > NEAR_B)

    elif state == "RETURNING":
        target_d = math.hypot(START_X - x, START_Y - y)
        if target_d < HOME_DIST:
            stop()
            elapsed = robot.getTime() - cycle_start
            results.append((elapsed, cycle_dist, cycle_obst))
            print("Package %d done: time %.1f s, distance %.1f m, obstacles avoided %d"
                  % (n + 1, elapsed, cycle_dist, cycle_obst))
            n += 1
            avoid_side = 0
            clear_since = None
            if n >= len(packages):
                state = "DONE"
            else:
                state = "GO_TO_PACKAGE"
                cycle_start = robot.getTime()
                cycle_dist = 0.0
                cycle_obst = 0
        else:
            d_b = math.hypot(GOAL_X - x, GOAL_Y - y)
            avoiding = drive_to(START_X, START_Y, x, y, heading, d_b > NEAR_B)

    elif state == "DONE":
        print("===== ALL DELIVERIES DONE =====")
        for k, (t, d, o) in enumerate(results, 1):
            print("Package %d: %.1f s, %.1f m, %d obstacles" % (k, t, d, o))
        print("TOTAL: %.1f s, %.1f m, %d obstacles avoided" % (
            sum(r[0] for r in results),
            sum(r[1] for r in results),
            total_obst))
        robot.setLabel(0, "ALL DELIVERED!", 0.02, 0.02, 0.07, LABEL_COLOR, 0.0, "Arial")
        break

    # obstacle se bachne ki ginti (ek bypass = ek count)
    if avoiding and not was_avoiding:
        cycle_obst += 1
        total_obst += 1
    was_avoiding = avoiding

    step += 1

    # screen pe live status
    if step % 5 == 0:
        robot.setLabel(0, "State: %s" % state, 0.02, 0.02, 0.06, LABEL_COLOR, 0.0, "Arial")
        robot.setLabel(1, "Delivered: %d / %d" % (delivered, len(packages)), 0.02, 0.07, 0.06, LABEL_COLOR, 0.0, "Arial")
        robot.setLabel(2, "Distance to target: %.2f m" % target_d, 0.02, 0.12, 0.06, LABEL_COLOR, 0.0, "Arial")
        robot.setLabel(3, "Time: %.1f s   Obstacles avoided: %d" % (robot.getTime(), total_obst), 0.02, 0.17, 0.06, LABEL_COLOR, 0.0, "Arial")

    if step % 100 == 0:
        print("state=%s x=%.2f y=%.2f" % (state, x, y))
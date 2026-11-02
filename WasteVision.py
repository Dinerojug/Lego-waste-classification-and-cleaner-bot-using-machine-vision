import sensor, image, time, ml, math, tf
from pupremote import PUPRemoteSensor
from pyb import LED
from ml.utils import NMS
msg = ''
dlabel = 0
xpos = 0
hpos = 0
mode = 0
num = 0
cx = 0
cy = 0
Lastxpos = False
istag = False
# ── Per-class confidence thresholds ──────────────────────────
CLASS_CONFIDENCE = {
'background'       :0.9,
 'Metal can'       :0.79,
 'Metal foil'      :0.68,
 'Paper'           :0.60,
 'Plastic'         :0.73,
 'Plastic bottle'  :0.73
}

# ── Detection tuning ─────────────────────────────────────────
MIN_CONFIDENCE = min(CLASS_CONFIDENCE.values())
def num(n):
    global mode
    mode = n
    return n
# ── PUPRemote Setup (LEGO Spike communication) ──────────────
p = PUPRemoteSensor(power=False)
p.add_command('msg', 'repr', 'repr')
p.add_channel('one_w', to_hub_fmt='bbbb')   # sends int to hub (1=detected, 0=not)
p.add_command('num',from_hub_fmt='b', to_hub_fmt='b')

# ── LED indicator ────────────────────────────────────────────
led = LED(2)
rled = LED(1)
led.on()


# raise to cut false positives (0.0 - 1.0)
MIN_SIZE        = 30   # minimum blob width/height in pixels
MAX_SIZE        = 320  # maximum blob width/height in pixels
MIN_ASPECT      = 0.2 # minimum width/height ratio (filters flat blobs)
MAX_ASPECT      = 30  # maximum width/height ratio
STREAK_NEEDED   = 2 # frames object must appear before confirming detection
ignoreColor = [(68,86,-31,-10,2,22),(47,75,27,56,-21,5)]

# ── Camera Setup ─────────────────────────────────────────────
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.set_windowing((160,160))
sensor.set_auto_whitebal(False)
sensor.set_auto_gain(False)
sensor.set_auto_exposure(False, exposure_us=10000)
sensor.skip_frames(time=2000)
#sensor.set_hmirror(True)    # ← flip horizontal
#sensor.set_vflip(True)



# ── Load Model ───────────────────────────────────────────────
net    = None
labels = None

try:
    net = ml.Model("trained.tflite", load_to_fb= False)

except Exception as e:
    raise Exception(
        'Failed to load "trained.tflite" — '
        'did you copy it to the OpenMV drive? (' + str(e) + ')'
    )

try:
    labels = [line.rstrip('\n') for line in open("labels.txt")]
except Exception as e:
    raise Exception(
        'Failed to load "labels.txt" — '
        'did you copy it to the OpenMV drive? (' + str(e) + ')'
    )

print("Model loaded. Classes:", labels)

# ── Colours for bounding boxes (one per class) ───────────────
COLORS = [
     (255,   0,   0),   # red
     (  0, 255,   0),   # green
     (255, 255,   0),   # yellow
     (  0,   0, 255),   # blue
     (255,   0, 255),   # magenta
     (  0, 255, 255),   # cyan
     (255, 255, 255),   # white
     (255, 128,   0),   # orange
     (128,   0, 255),   # purple
     (  0, 255, 128),   # mint
     (255,   0, 128),   # pink
     (128, 255,   0),   # lime
     (  0, 128, 255),   # sky blue
     (255, 128, 128),   # salmon
     (128, 255, 255),   # light cyan
     (255, 255, 128),   # light yellow
     (128, 128, 255),   # lavender
     (255, 165,   0),   # amber
     (  0, 200, 100),   # emerald
     (200,  50,  50),   # dark red
 ]


threshold_list = [(math.ceil(MIN_CONFIDENCE * 255), 255)]

# ── FOMO Post-Processing ─────────────────────────────────────
def fomo_post_process(model, inputs, outputs):
    """
    Converts raw FOMO model output into a list of detections.
    Returns: list of lists — one inner list per class.
    Each detection is (x, y, w, h, score) in full image pixels.
    """
    ob, oh, ow, oc = model.output_shape[0]

    x_scale = inputs[0].roi[2] / ow
    y_scale = inputs[0].roi[3] / oh
    scale   = min(x_scale, y_scale)

    x_offset = ((inputs[0].roi[2] - (ow * scale)) / 2) + inputs[0].roi[0]
    y_offset = ((inputs[0].roi[3] - (oh * scale)) / 2) + inputs[0].roi[1]

    detections = [[] for _ in range(oc)]

    for i in range(oc):
        # Each channel is a grayscale probability map — GRAYSCALE = 1 byte/pixel
        out_data = outputs[0][0, :, :, i].copy()
        prob_map = image.Image(ow, oh, sensor.GRAYSCALE, buffer=out_data)

        blobs = prob_map.find_blobs(
            threshold_list,
            x_stride=1,
            y_stride=1,
            area_threshold=2,
            pixels_threshold=2,
            merge=True,     # merge nearby detections of the same object
            margin=5
        )

        for b in blobs:
            # Reject blobs that cover most of the AI grid (wall/background noise)
            if b.w() > (ow * 0.95) or b.h() > (oh * 0.95):
                continue

            # Scale blob coordinates back to full image pixels
            rx = int((b.x() * scale) + x_offset)
            ry = int((b.y() * scale) + y_offset)
            # rx = int((b.cx() * scale) + x_offset)
            # ry = int((b.cy() * scale) + y_offset)
            rw = int(b.w() * scale)
            rh = int(b.h() * scale)

            # ── Size filter ──────────────────────────────────

            if not (MIN_SIZE < rw < MAX_SIZE and MIN_SIZE < rh < MAX_SIZE):

                continue

            # ── Aspect ratio filter ──────────────────────────
            aspect = rw / rh if rh > 0 else 999
            if not (MIN_ASPECT < aspect < MAX_ASPECT):
                continue

            score = prob_map.get_statistics(
                thresholds=threshold_list, roi=b.rect()
            ).l_mean() / 255.0

            detections[i].append((rx, ry, rw, rh, score))

    return detections
def is_suppressed(bg_zones, cx, cy,  margin=15):
    if (cx != 0) and (cy!= 0):
        for bzx, bzy, brad in bg_zones:

            # calculate distance from detection center to bg zone center
            dist = math.sqrt((cx - bzx) ** 2 + (cy - bzy) ** 2)

            # if detection is within bg zone radius + margin → suppress
            if dist < (brad + margin):
                return True

    return False  # not suppressed

def Xposcale(num, center):

    scaled = int((num - center) * (100 / 80))
    return scaled
# ── Streak counters (one per non-background class) ───────────
# Streak prevents single-frame false positives being reported
streak = {}

cur_mode = 0
clock = time.clock()
# ── Main Loop
while True:
    clock.tick()
    if mode != cur_mode:
        if mode == 2:
            rled.on()
            led.off()
            hpos = 0
        else:
            #sensor.set_framesize(sensor.QVGA)
            rled.off()
            led.on()
        cur_mode = mode
    if mode == 2:
        istag = False
        test = 0
        Tagid = 5
        cx = 160
        clock.tick()
        img = sensor.snapshot()
        for tag in img.find_apriltags():
            img.draw_rectangle(tag.rect, color=(255, 0, 0))
            img.draw_cross(tag.cx, tag.cy, color=(0, 255, 0))
            Tagid = tag.id
            if Tagid >= 5:
                istag = False
            else:
                istag = True
            cx = tag.cx

            #print_args = (tag.name, tag.id, (180 * tag.rotation) / math.pi)
            #print("Tag Family %s, Tag ID %d, rotation %f (degrees)" % print_args)
            cx = Xposcale(cx,80)

        if istag:
            hpos = min(hpos + 1, 8)
            print(hpos)
        else:
            hpos = max(hpos - 1, 0)
        print(Tagid)
        p.update_channel('one_w', mode, Tagid, cx, hpos)


        p.process()
    else:
        img = sensor.snapshot()
        for tag in img.find_apriltags():
            img.draw_rectangle(tag.rect, color=(255, 0, 0))
            img.draw_cross(tag.cx, tag.cy, color=(0, 255, 0))
            Tagid = tag.id
            if Tagid >= 5:
                istag = False
            else:
                istag = True
        if istag:
            p.process()
            continue
        results = net.predict([img], callback=fomo_post_process)

        # Reset detection state each frame
        detected_label = None
        xpos = 80
        detected = 0
        bg_zones = []
        if results and results[0]:
            for bx, by, bw, bh, bscore in results[0]:

                # ✅ only suppress if background is VERY confident
                if bscore > 0.95:
                    cx   = math.floor(bx + bw / 2)
                    cy   = math.floor(by + bh / 2)
                    rad  = max(bw, bh)
                    bg_zones.append((cx, cy, rad))
                    #print("Suppression zone at (%d,%d) r=%d" % (cx, cy, rad))
                else:
                    cy = 0
                    cx= 0
        for i, detection_list in enumerate(results):
            if i == 0:
               continue   # skip background class

            if not detection_list:
                continue  # no detections this class
            if is_suppressed(bg_zones, cx, cy, margin=20):
                print("suppressed")
                continue


            label = labels[i]
            class_threshold = CLASS_CONFIDENCE.get(label, MIN_CONFIDENCE)

            # Initialise streak counter for new labels
            if label not in streak:
                streak[label] = 0

            #print("── %s ──" % label)

            for x, y, w, h, score in detection_list:
                xpos = (math.floor(x + (w / 2)))
                #Teleportation Check
                if Lastxpos != False:
                    if abs(xpos -Lastxpos) > 30:
                        Lastxpos = False
                        continue
                else:
                    Lastxpos = xpos
                detection_roi = (x, y, w, h)

                blobs = img.find_blobs(ignoreColor, roi=detection_roi , pixels_threshold=100)
                if blobs:
                    img.draw_rectangle((x,y,w,h), color=(128, 128, 128))
                    continue
                if score < class_threshold:
                    continue

                # Draw bounding box

                label = labels[i]
                img.draw_rectangle((x, y, w, h), color=COLORS[i % len(COLORS)], thickness=2)

                # Draw label above box
                img.draw_string(x, max(0, y - 14), "%s %d%%" % (label, int(score * 100)),
                               color=COLORS[i % len(COLORS)], scale=1.4)

                #print("  %s | box=(%d,%d,%d,%d) | conf=%.2f" % (label, x, y, w, h, score))

                detected_label = label
                dlabel= i
                xpos = (math.floor(x + (w / 2)))
                xpos = Xposcale(xpos,80)

        # ── Streak logic ─────────────────────────────────────────
        # Only confirm detection after STREAK_NEEDED consecutive frames
        for label in streak:
            if label == detected_label:
                streak[label] = min(streak[label] + 1, STREAK_NEEDED + 1)
            else:
                streak[label] = max(streak[label] - 1, 0)  # decay, not instant reset

        # Confirm detection only if streak threshold met
            if detected_label and streak.get(detected_label, 0) >= STREAK_NEEDED:
                detected = 1

                led.on()
                print(detected_label, score, 'x', xpos)

                print(dlabel)

            else:
                detected = 0
                led.off()

        # ── Send to LEGO Spike hub ────────────────────────────────


        p.update_channel('one_w', detected, dlabel, xpos, hpos)
        p.process()

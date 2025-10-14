import bpy
import math

# =========================================
# PARAMETERS
# =========================================
RADIUS_M            = 25.0   # cylinder radius (m)
HEIGHT_M            = 10.0   # cylinder height (m), downward along -Z

OPENING_CC_M        = 0.0275   # center-to-center spacing
THREAD_THICKNESS_M  = 0.005   # strand/band width on the surface

# Geometry / quality controls (affect polycount)
ANGULAR_RES_AROUND  = 64     # angle resolution for horizontal bands
Z_SEG_VERT_STRIP    = 1      # segments along Z for vertical strips (1 = top/bottom only)

# (Optional) give physical thickness in normal direction via Solidify
ADD_SOLIDIFY        = False
SOLIDIFY_THICK_M    = 0.01   # thickness (m) if ADD_SOLIDIFY=True

# =========================================
# DERIVED / SAFETY ADJUSTMENTS
# =========================================
circ = 2.0 * math.pi * RADIUS_M

if THREAD_THICKNESS_M <= 0.0:
    raise ValueError("THREAD_THICKNESS_M must be > 0.")
if OPENING_CC_M <= 0.0:
    raise ValueError("OPENING_CC_M must be > 0.")
if THREAD_THICKNESS_M >= OPENING_CC_M:
    print("[WARN] THREAD_THICKNESS_M >= OPENING_CC_M; nudging down to avoid negative clearance.")
    THREAD_THICKNESS_M = 0.95 * OPENING_CC_M

NU = max(1, int(round(circ     / OPENING_CC_M)))  # vertical strips around
NV = max(1, int(round(HEIGHT_M / OPENING_CC_M)))  # horizontal ring bands downwards

pitch_u = circ     / NU
pitch_v = HEIGHT_M / NV

half_ang = (THREAD_THICKNESS_M * 0.5) / RADIUS_M  # radians

print(f"[INFO] NU={NU}, NV={NV}")
print(f"[INFO] Effective pitch U={pitch_u:.6f} m, V={pitch_v:.6f} m")
print(f"[INFO] Clearance (approx) ~ {OPENING_CC_M - THREAD_THICKNESS_M:.6f} m")

# =========================================
# BUILD VERTICES & FACES (single mesh)
# =========================================
verts = []
faces = []

def add_vert(theta, z):
    x = RADIUS_M * math.cos(theta)
    y = RADIUS_M * math.sin(theta)
    verts.append((x, y, z))
    return len(verts) - 1

# 1) VERTICAL STRIPS
for i in range(NU):
    theta_c = (i / NU) * 2.0 * math.pi
    t_left  = theta_c - half_ang
    t_right = theta_c + half_ang

    for j in range(Z_SEG_VERT_STRIP):
        z_top = - (j    ) * (HEIGHT_M / Z_SEG_VERT_STRIP)
        z_bot = - (j + 1) * (HEIGHT_M / Z_SEG_VERT_STRIP)

        # Intended CCW from outside
        v0 = add_vert(t_left,  z_top)
        v1 = add_vert(t_right, z_top)
        v2 = add_vert(t_right, z_bot)
        v3 = add_vert(t_left,  z_bot)
        faces.append((v0, v1, v2, v3))

# 2) HORIZONTAL RING BANDS
for j in range(NV):
    z_center = - j * pitch_v
    z_low  = z_center - THREAD_THICKNESS_M * 0.5
    z_high = z_center + THREAD_THICKNESS_M * 0.5

    base = len(verts)
    for k in range(ANGULAR_RES_AROUND):
        theta = (k / ANGULAR_RES_AROUND) * 2.0 * math.pi
        add_vert(theta, z_low)
        add_vert(theta, z_high)

    for k in range(ANGULAR_RES_AROUND):
        a_low  = base + 2 * k
        b_low  = base + 2 * ((k + 1) % ANGULAR_RES_AROUND)
        b_high = b_low + 1
        a_high = a_low + 1
        faces.append((a_low, b_low, b_high, a_high))

# =========================================
# ENSURE OUTWARD NORMALS BY FACE-BY-FACE WINDING
# (works reliably for non-manifold shells)
# =========================================
def face_normal(idx_triplet):
    a, b, c = idx_triplet
    ax, ay, az = verts[a]
    bx, by, bz = verts[b]
    cx, cy, cz = verts[c]
    ab = (bx-ax, by-ay, bz-az)
    ac = (cx-ax, cy-ay, cz-az)
    # cross = ab x ac
    nx = ab[1]*ac[2] - ab[2]*ac[1]
    ny = ab[2]*ac[0] - ab[0]*ac[2]
    nz = ab[0]*ac[1] - ab[1]*ac[0]
    return (nx, ny, nz)

def dot(u, v):
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2]

def ensure_outward_winding(faces_list):
    fixed = []
    for f in faces_list:
        a, b, c, d = f
        # face centroid
        cx = (verts[a][0] + verts[b][0] + verts[c][0] + verts[d][0]) * 0.25
        cy = (verts[a][1] + verts[b][1] + verts[c][1] + verts[d][1]) * 0.25
        cz = (verts[a][2] + verts[b][2] + verts[c][2] + verts[d][2]) * 0.25

        # outward "expected" direction = radial from Z-axis
        radial = (cx, cy, 0.0)
        # compute current normal from first triangle (a,b,c)
        n = face_normal((a, b, c))

        # if normal points inward (negative dot with radial), flip winding
        if dot(n, radial) < 0:
            fixed.append((a, d, c, b))  # reverse order
        else:
            fixed.append(f)
    return fixed

faces = ensure_outward_winding(faces)

# =========================================
# CREATE MESH + OBJECT
# =========================================
mesh_name = f"CrossMesh_R{RADIUS_M:.2f}_H{HEIGHT_M:.2f}"
mesh = bpy.data.meshes.new(mesh_name)
mesh.from_pydata(verts, [], faces)
mesh.validate(clean_customdata=True)
mesh.update()

obj = bpy.data.objects.new(mesh_name, mesh)
bpy.context.collection.objects.link(obj)
bpy.context.view_layer.objects.active = obj
obj.select_set(True)

# Optional Solidify
if ADD_SOLIDIFY:
    solid = obj.modifiers.new(name="Solidify", type='SOLIDIFY')
    solid.thickness = SOLIDIFY_THICK_M
    solid.offset = 0.0
    solid.use_even_offset = True

# Do NOT rely on auto recalc; we already enforced winding.
# Still, for display, we can enable auto smooth if wanted:
# mesh.use_auto_smooth = True

print("\n[OK] Cross pattern generated with enforced outward normals (face-by-face winding).")
print("Tip: Lower ANGULAR_RES_AROUND for fewer polys on ring bands.")

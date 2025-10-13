import bpy
import math

# =========================================
# PARAMETERE
# =========================================
RADIUS_M   = 10     # Sylinderradius (m)
HEIGHT_M   = 10.0     # Sylinderhøyde (m)

CONE_HEIGHT_M = 4.0   # Konens høyde (m), peker nedover fra sylinderen
CONE_TIP_RADIUS_M = 0.0  # 0.0 = ekte spiss; >0 = liten stump tupp (gir quads helt ned)

# --- Velg én av to måter å spesifisere oppløsning ---
USE_APPROX_SIZE    = True      # True: bruk omtrentlig maskestørrelse
TARGET_CELL_SIZE_M = 0.05      # ønsket ~maskestørrelse (m) hvis USE_APPROX_SIZE=True

# Hvis USE_APPROX_SIZE=False, bruk eksakte tellere:
NU = 50    # masker (quads) rundt omkretsen (samme for sylinder & kon)
NV_CYL = 40  # masker (quads) nedover sylinderen
NV_CONE = 30 # masker (ringer) nedover konen (siste blir triangler hvis CONE_TIP_RADIUS_M=0)

# Gjør cellene mest mulig kvadratiske på sylinderen (justerer NV_CYL basert på NU / omkrets)
FORCE_SQUARE_CELLS_CYL = True
SQUARE_STRATEGY = "match_nv"  # "match_nv" (anbefalt) eller "match_nu"

# (Valgfritt) legg på Wireframe automatisk (for trådnett)
ADD_WIREFRAME  = True
WIRE_THICKNESS = 0.01  # m

# (Valgfritt) rydd scenen først
# for o in list(bpy.context.scene.objects):
#     o.select_set(True)
# bpy.ops.object.delete()


# =========================================
# AVLEDNINGER
# =========================================
circ = 2.0 * math.pi * RADIUS_M

if USE_APPROX_SIZE:
    NU = max(3, int(round(circ / max(1e-6, TARGET_CELL_SIZE_M))))
    NV_CYL = max(1, int(round(HEIGHT_M / max(1e-6, TARGET_CELL_SIZE_M))))
    NV_CONE = max(1, int(round(CONE_HEIGHT_M / max(1e-6, TARGET_CELL_SIZE_M))))
else:
    NU = max(3, int(NU))
    NV_CYL = max(1, int(NV_CYL))
    NV_CONE = max(1, int(NV_CONE))

# Sylinder-steg
u_step_arc  = circ / NU          # buelengde per maske rundt
z_step_cyl  = HEIGHT_M / NV_CYL  # høyde per maske på sylinderen

if FORCE_SQUARE_CELLS_CYL:
    if SQUARE_STRATEGY == "match_nv":
        NV_CYL = max(1, int(round(HEIGHT_M / u_step_arc)))
    elif SQUARE_STRATEGY == "match_nu":
        NU = max(3, int(round(circ / z_step_cyl)))
    # recompute etter eventuell justering
    u_step_arc  = circ / NU
    z_step_cyl  = HEIGHT_M / NV_CYL

# Kon-steg (radialt og vertikalt)
z_step_cone = CONE_HEIGHT_M / NV_CONE
R_top = RADIUS_M
R_tip = max(0.0, float(CONE_TIP_RADIUS_M))  # kan være 0.0 for spiss

print(f"[INFO] NU={NU}, NV_CYL={NV_CYL}, NV_CONE={NV_CONE}")
print(f"[INFO] Sylinder: cell U-arc={u_step_arc:.4f} m, cell Z={z_step_cyl:.4f} m")
print(f"[INFO] Kone: H={CONE_HEIGHT_M:.3f} m, R_top={R_top:.3f} m → R_tip={R_tip:.3f} m")


# =========================================
# GENERER VERTICES
# Struktur:
#  - Sylinder: NV_CYL+1 ringer (z=0 .. -HEIGHT_M)
#  - Kone   : NV_CONE ringer (z=-HEIGHT_M - step .. -HEIGHT_M - H), radius lineært ned
#  - Hvis spiss: ett apex-vert til slutt
# =========================================
verts = []

# Sylinder-ringer
for j in range(NV_CYL + 1):
    z = -j * z_step_cyl
    for i in range(NU):
        theta = (i / NU) * 2.0 * math.pi
        x = RADIUS_M * math.cos(theta)
        y = RADIUS_M * math.sin(theta)
        verts.append((x, y, z))

# Kone-ringer
# Start ved syl.bunn (samme NU per ring), trinn nedover
for k in range(1, NV_CONE + 1):
    t = k / NV_CONE  # 0→1
    r = (1.0 - t) * R_top + t * R_tip
    z = -HEIGHT_M - k * z_step_cone
    for i in range(NU):
        theta = (i / NU) * 2.0 * math.pi
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        verts.append((x, y, z))

apex_index = None
if R_tip == 0.0:
    # legg til én apex-vert i bunn
    verts.append((0.0, 0.0, -HEIGHT_M - CONE_HEIGHT_M))
    apex_index = len(verts) - 1

# indekshjelp
def ring_start_index(ring_idx):
    """Returner startindeks for ring #ring_idx.
       ringer: 0..NV_CYL (sylinder) og NV_CYL+1..NV_CYL+NV_CONE (kone)"""
    return ring_idx * NU

# =========================================
# GENERER FLATER (QUADS + ev. triangel-fan i spiss)
# =========================================
faces = []

# Sylinder-quads
for j in range(NV_CYL):
    base0 = ring_start_index(j)
    base1 = ring_start_index(j + 1)
    for i in range(NU):
        a = base0 + i
        b = base0 + (i + 1) % NU
        c = base1 + (i + 1) % NU
        d = base1 + i
        faces.append((a, b, c, d))

# Kone-quads mellom ringer (så langt vi kan)
# ringer for konen er: idx NV_CYL .. NV_CYL+NV_CONE
for k in range(NV_CYL, NV_CYL + NV_CONE - 1):
    base0 = ring_start_index(k)
    base1 = ring_start_index(k + 1)
    # radius minker gradvis, men vi har samme NU -> rene quads
    for i in range(NU):
        a = base0 + i
        b = base0 + (i + 1) % NU
        c = base1 + (i + 1) % NU
        d = base1 + i
        faces.append((a, b, c, d))

# Nederste overgang:
# - Hvis R_tip > 0: siste ring har fortsatt NU verts -> vi har allerede laget siste quads.
# - Hvis R_tip == 0: siste ring reduseres til spiss (apex) -> triangler fra siste ring til apex
if R_tip == 0.0:
    last_ring_start = ring_start_index(NV_CYL + NV_CONE - 1)
    for i in range(NU):
        a = last_ring_start + i
        b = last_ring_start + (i + 1) % NU
        c = apex_index
        faces.append((a, b, c))  # triangel

# =========================================
# LAG MESH + OBJEKT
# =========================================
mesh = bpy.data.meshes.new(f"CylCone_R{RADIUS_M:.2f}_H{HEIGHT_M:.2f}_NU{NU}_NV{NV_CYL}+{NV_CONE}")
mesh.from_pydata(verts, [], faces)
mesh.validate(clean_customdata=True)
mesh.update()

obj = bpy.data.objects.new(f"CylCone_R{RADIUS_M:.2f}_H{HEIGHT_M:.2f}", mesh)
bpy.context.collection.objects.link(obj)
bpy.context.view_layer.objects.active = obj
obj.select_set(True)

# (valgfritt) wireframe for trådnett
if ADD_WIREFRAME:
    wf = obj.modifiers.new(name="Wireframe", type='WIREFRAME')
    wf.thickness = WIRE_THICKNESS
    wf.use_even_offset = True
    wf.use_replace = True
    wf.offset = 0.0

print("\n[OK] Sylinder + kon er laget og sammenkoblet uten hull.")
print("    • NU styrer masker rundt; NV_CYL og NV_CONE styrer vertikal tetthet.")
print("    • Alternativt: USE_APPROX_SIZE=True med TARGET_CELL_SIZE_M.")
print("    • CONE_TIP_RADIUS_M=0 gir spiss (triangel-fan nederst). >0 gir stump tupp (quads helt ned).")

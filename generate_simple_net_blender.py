import bpy
import math

# =========================================
# PARAMETERE (KUN SYLINDER)
# =========================================
RADIUS_M   = 10.0   # Sylinderradius (m)
HEIGHT_M   = 10.0   # Sylinderhøyde (m), nedover i -Z

# --- Velg én av to måter å spesifisere oppløsning ---
USE_APPROX_SIZE    = True       # True: bruk omtrentlig maskestørrelse
TARGET_CELL_SIZE_M = 0.05       # ønsket ~maskestørrelse (m) hvis USE_APPROX_SIZE=True

# Hvis USE_APPROX_SIZE=False, bruk eksakte tellere:
NU = 50     # masker (quads) rundt omkretsen
NV_CYL = 40 # masker (quads) nedover sylinderen

# Gjør cellene mest mulig kvadratiske på sylinderen (justerer NV_CYL basert på NU / omkrets)
FORCE_SQUARE_CELLS_CYL = True   # anbefalt
SQUARE_STRATEGY = "match_nv"    # "match_nv" (anbefalt) eller "match_nu"

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
else:
    NU = max(3, int(NU))
    NV_CYL = max(1, int(NV_CYL))

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

print(f"[INFO] NU={NU}, NV_CYL={NV_CYL}")
print(f"[INFO] Sylinder: cell U-arc={u_step_arc:.4f} m, cell Z={z_step_cyl:.4f} m")

# =========================================
# GENERER VERTICES
# Struktur:
#  - Sylinder: NV_CYL+1 ringer (z=0 .. -HEIGHT_M)
# =========================================
verts = []
for j in range(NV_CYL + 1):
    z = -j * z_step_cyl
    for i in range(NU):
        theta = (i / NU) * 2.0 * math.pi
        x = RADIUS_M * math.cos(theta)
        y = RADIUS_M * math.sin(theta)
        verts.append((x, y, z))

def ring_start_index(ring_idx):
    """Startindeks for ring #ring_idx."""
    return ring_idx * NU

# =========================================
# GENERER FLATER (QUADS)
# =========================================
faces = []
for j in range(NV_CYL):
    base0 = ring_start_index(j)
    base1 = ring_start_index(j + 1)
    for i in range(NU):
        a = base0 + i
        b = base0 + (i + 1) % NU
        c = base1 + (i + 1) % NU
        d = base1 + i
        faces.append((a, b, c, d))

# =========================================
# LAG MESH + OBJEKT
# =========================================
mesh = bpy.data.meshes.new(f"Cylinder_R{RADIUS_M:.2f}_H{HEIGHT_M:.2f}_NU{NU}_NV{NV_CYL}")
mesh.from_pydata(verts, [], faces)
mesh.validate(clean_customdata=True)
mesh.update()

obj = bpy.data.objects.new(f"Cylinder_R{RADIUS_M:.2f}_H{HEIGHT_M:.2f}", mesh)
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

print("\n[OK] Sylinder er laget (uten kon).")
print("    • NU styrer masker rundt; NV_CYL styrer vertikal tetthet.")
print("    • Alternativt: USE_APPROX_SIZE=True med TARGET_CELL_SIZE_M for enkel kontroll.")

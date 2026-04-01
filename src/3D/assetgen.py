import math

with open("sin_table.hex", "w") as f:
    for i in range(1024):
        val = int(round(math.sin(2 * math.pi * i / 1024) * 256)) & 0xFFFF
        f.write(f"{val:04X}\n")

print("sin_table.hex written — 1024 lines")



PROJ_DIST = 320.0 / math.tan(math.radians(30))  # = 554.256

with open("col_angle.hex", "w") as f:
    for c in range(640):
        val = int(round(math.atan2(c - 320, PROJ_DIST) * 1024 / (2 * math.pi))) & 0x3FF
        f.write(f"{val:03X}\n")

print("col_angle.hex written — 640 lines")


PROJ_DIST = 320.0 / math.tan(math.radians(30))  # = 554.256

with open("rcp_table.hex", "w") as f:
    for i in range(256):
        if i == 0:
            val = 479  # index 0 = zero distance = cap at full screen height
        else:
            val = min(479, int(round(PROJ_DIST * 16 / i)))
        f.write(f"{val:03X}\n")

print("rcp_table.hex written — 256 lines")

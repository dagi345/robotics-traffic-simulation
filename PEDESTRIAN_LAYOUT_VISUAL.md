# Pedestrian Crowd Layout - Visual Guide

## Top-Down View of Intersection

```
                        NORTH
                          ↓
                    
            ped_5 (0, 14.7)
        ped_3  ped_4 (±0.5, 14.1)
        ped_1  ped_2 (±0.5, 13.5)
    ================================
    |   Crosswalk (Y=10 to 13)    |
    ================================
    |                              |
    |                              |
    |         INTERSECTION         |
    |            CENTER            |
    |            (0, 0)            |
    |                              |
    |                              |
    ================================
    |   Crosswalk (Y=-13 to -10)  |
    ================================
        ped_1  ped_2 (±0.5, -13.5)
        ped_3  ped_4 (±0.5, -14.1)
            ped_5 (0, -14.7)
                    
                          ↑
                        SOUTH


WEST ←                                    → EAST

ped_5                                     ped_5
(-14.7, 0)                              (14.7, 0)

ped_3  ped_4                            ped_3  ped_4
(-14.1, ±0.5)                          (14.1, ±0.5)

ped_1  ped_2                            ped_1  ped_2
(-13.5, ±0.5)                          (13.5, ±0.5)

|========|                              |========|
Crosswalk                                Crosswalk
(X=-13 to -10)                          (X=10 to 13)
```

## Detailed Crosswalk Views

### North Crosswalk (Looking South)
```
        Sidewalk Area
    =====================
    
    Back Row (Y=14.7):
           [ped_5]
    
    Middle Row (Y=14.1):
        [ped_3] [ped_4]
    
    Front Row (Y=13.5):
        [ped_1] [ped_2]
    
    =====================
         Curb Edge
    =====================
    
    Crosswalk (zebra stripes)
    =====================
    
           Road
```

### South Crosswalk (Looking North)
```
           Road
    
    =====================
    Crosswalk (zebra stripes)
    =====================
    
         Curb Edge
    =====================
    
    Front Row (Y=-13.5):
        [ped_1] [ped_2]
    
    Middle Row (Y=-14.1):
        [ped_3] [ped_4]
    
    Back Row (Y=-14.7):
           [ped_5]
    
    =====================
        Sidewalk Area
```

### East Crosswalk (Looking West)
```
Sidewalk | Curb | Crosswalk | Road
         |      |           |
  ped_5  |      |           |  Back (X=14.7)
         |      |           |
  ped_3  |      |           |  Middle (X=14.1)
  ped_4  |      |           |
         |      |           |
  ped_1  |      |           |  Front (X=13.5)
  ped_2  |      |           |
         |      |           |
```

### West Crosswalk (Looking East)
```
Road | Crosswalk | Curb | Sidewalk
     |           |      |
     |           |      |  ped_5    Back (X=-14.7)
     |           |      |
     |           |      |  ped_3    Middle (X=-14.1)
     |           |      |  ped_4
     |           |      |
     |           |      |  ped_1    Front (X=-13.5)
     |           |      |  ped_2
     |           |      |
```

## Spacing Measurements

### Lateral Spacing (Side-by-Side)
```
    [ped_1]  <-- 0.5m -->  [ped_2]
    
    Position:
    ped_1: X = -0.5m
    ped_2: X = +0.5m
    
    Total width: 1.0m
```

### Longitudinal Spacing (Front-to-Back)
```
    [ped_1] [ped_2]  Front Row
         ↓
      0.6m spacing
         ↓
    [ped_3] [ped_4]  Middle Row
         ↓
      0.6m spacing
         ↓
       [ped_5]       Back Row
```

## Orientation Arrows

### North Crosswalk
```
    [ped_1] [ped_2]
       ↓       ↓      All facing SOUTH
    [ped_3] [ped_4]   (toward crosswalk)
       ↓       ↓
       [ped_5]
          ↓
```

### South Crosswalk
```
       [ped_5]
          ↑
    [ped_3] [ped_4]   All facing NORTH
       ↑       ↑      (toward crosswalk)
    [ped_1] [ped_2]
       ↑       ↑
```

### East Crosswalk
```
    [ped_5]  ←
    [ped_3]  ←  All facing WEST
    [ped_4]  ←  (toward crosswalk)
    [ped_1]  ←
    [ped_2]  ←
```

### West Crosswalk
```
    →  [ped_5]
    →  [ped_3]  All facing EAST
    →  [ped_4]  (toward crosswalk)
    →  [ped_1]
    →  [ped_2]
```

## 3D Perspective View

```
                    North Pedestrians
                    (facing south ↓)
                    
                    🚶 🚶
                    🚶 🚶
                      🚶
                    
    ═══════════════════════════════════
    ║                                 ║
    ║         CROSSWALK (white)       ║
    ║                                 ║
    ═══════════════════════════════════
    ║                                 ║
    ║                                 ║
    ║                                 ║
West║          INTERSECTION           ║East
Peds║             CENTER              ║Peds
→🚶 ║                                 ║🚶←
→🚶 ║                                 ║🚶←
→🚶 ║                                 ║🚶←
→🚶 ║                                 ║🚶←
→🚶 ║                                 ║🚶←
    ║                                 ║
    ║                                 ║
    ║                                 ║
    ═══════════════════════════════════
    ║                                 ║
    ║         CROSSWALK (white)       ║
    ║                                 ║
    ═══════════════════════════════════
                    
                      🚶
                    🚶 🚶
                    🚶 🚶
                    
                    South Pedestrians
                    (facing north ↑)
```

## Key Measurements

### Intersection Geometry
- **Stop lines:** ±13m from center
- **Crosswalk zones:** ±10m to ±13m
- **Curb positions:** ±13.5m from center
- **Pedestrian waiting area:** ±13.5m to ±14.7m

### Pedestrian Spacing
- **Lateral (side-by-side):** 0.5m
- **Longitudinal (rows):** 0.6m
- **Total crowd depth:** 1.2m (3 rows)
- **Total crowd width:** 1.0m (2 pedestrians)

### Safety Clearances
- **From road edge:** 0.5m minimum
- **From crosswalk:** 0.5m minimum
- **Between pedestrians:** 0.5m minimum
- **From intersection center:** 13.5m minimum

## Crowd Formation Pattern

```
Formation: 2-2-1 (Front to Back)

Row 1 (Front):  👤 👤    (2 pedestrians)
Row 2 (Middle): 👤 👤    (2 pedestrians)
Row 3 (Back):     👤      (1 pedestrian)

Total: 5 pedestrians per crosswalk
Depth: 1.2m (3 rows × 0.6m spacing)
Width: 1.0m (2 pedestrians × 0.5m spacing)
```

## Comparison: Before vs After

### Before (Old Layout)
```
❌ Only 2 pedestrians per crosswalk
❌ Spread far apart (6m+ spacing)
❌ Some facing wrong directions
❌ Positioned too far from curb
❌ Not realistic crowd appearance
```

### After (New Layout)
```
✅ 5 pedestrians per crosswalk
✅ Realistic spacing (0.5-0.6m)
✅ All facing correct direction
✅ Positioned at curb edge
✅ Realistic waiting crowd appearance
```

---

**Visual Guide Created:** December 30, 2025
**Purpose:** Help understand realistic pedestrian crowd layout
**Status:** Complete and accurate

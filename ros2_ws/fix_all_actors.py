#!/usr/bin/env python3
"""
Fix all actor definitions by removing the VelocityControl plugin.
Actors don't support this plugin - they use trajectory-based movement.
"""

import re

# Read the world file
with open('src/smart_traffic_system/worlds/intersection.world', 'r') as f:
    content = f.read()

# Pattern to match plugin blocks with mismatched closing tags
# This handles the case where </actor> appears instead of </plugin></actor>
pattern1 = r'(<plugin filename="gz-sim-velocity-control-system"[^>]*>.*?<initial_angular>0 0 0</initial_angular>\s*)</actor>'
content = re.sub(pattern1, r'</plugin>\n    </actor>', content, flags=re.DOTALL)

# Pattern to match complete plugin blocks that are properly closed
pattern2 = r'\s*<plugin filename="gz-sim-velocity-control-system"[^>]*>.*?</plugin>'
content = re.sub(pattern2, '', content, flags=re.DOTALL)

# Write the updated content
with open('src/smart_traffic_system/worlds/intersection.world', 'w') as f:
    f.write(content)

print("Successfully fixed all actor definitions")
print("Removed VelocityControl plugins from all actors")

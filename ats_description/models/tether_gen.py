import os
import math

def generate_tether(num_links=30, total_length=5.0):
    # Parâmetros Físicos
    radius = 0.01 
    mass = 0.02    # 20g por elo
    
    # Target Relativo
    tx, ty, tz = 0.8, 0.0, 0.35 

    L_end = 0.05
    L_mid = (total_length - 2 * L_end) / (num_links - 2)
    
    def get_curve_point(s_len):
        s = s_len / total_length
        lx, ly, lz = tx * s, ty * s, tz * s
        slack_factor = math.sin(math.pi * s)
        y_osc = 1.0 * math.sin(math.pi * s)
        z_arc = 0.5 * slack_factor
        return (lx, ly + y_osc, lz + z_arc)

    joint_points = []
    current_accumulated_len = 0.0
    joint_points.append(get_curve_point(0.0))

    for i in range(num_links):
        step = L_end if (i == 0 or i == num_links - 1) else L_mid
        current_accumulated_len += step
        joint_points.append(get_curve_point(min(current_accumulated_len, total_length)))

    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
    <model name="tether">
        <self_collide>true</self_collide>
"""

    for i in range(num_links):
        p1, p2 = joint_points[i], joint_points[i+1]
        cx, cy, cz = (p1[0]+p2[0])/2, (p1[1]+p2[1])/2, (p1[2]+p2[2])/2
        dx, dy, dz = p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        dist_final = L_end if (i == 0 or i == num_links - 1) else dist
        
        seg_yaw = math.atan2(dy, dx)
        seg_pitch = math.atan2(math.sqrt(dx**2 + dy**2), dz)

        # INÉRCIA ULTRA-ALTA (0.1)
        ixx_yy = 0.1 
        izz = 0.01 

        sdf += f"""
        <link name="link_{i}">
            <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 {seg_pitch:.4f} {seg_yaw:.4f}</pose>
            <velocity_decay>
                <linear>0.5</linear>
                <angular>0.5</angular>
            </velocity_decay>
            <inertial>
                <mass>{mass}</mass>
                <inertia>
                    <ixx>{ixx_yy:.4f}</ixx><ixy>0</ixy><ixz>0</ixz>
                    <iyy>{ixx_yy:.4f}</iyy><iyz>0</iyz>
                    <izz>{izz:.4f}</izz>
                </inertia>
            </inertial>
            <visual name="visual_cylinder">
                <geometry><cylinder><radius>{radius}</radius><length>{dist_final:.4f}</length></cylinder></geometry>
                <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
            </visual>
            <visual name="visual_joint">
                <pose>0 0 {-dist_final/2:.4f} 0 0 0</pose>
                <geometry><sphere><radius>0.0105</radius></sphere></geometry>
                <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
            </visual>
            <collision name="collision">
                <geometry><cylinder><radius>{radius}</radius><length>{dist_final:.4f}</length></cylinder></geometry>
                <surface>
                    <contact><collide_bitmask>0x02</collide_bitmask></contact>
                </surface>
            </collision>
        </link>"""
        
        if i > 0:
            sdf += f"""
        <joint name="joint_{i-1}_{i}" type="ball">
            <parent>link_{i-1}</parent>
            <child>link_{i}</child>
            <pose>0 0 {-dist_final/2:.4f} 0 0 0</pose>
            <dynamics>
                <damping>2.0</damping>
                <friction>0.1</friction>
            </dynamics>
        </joint>"""

    sdf += """
    </model>
</sdf>
"""
    return sdf

if __name__ == "__main__":
    tether_dir = "src/ats_description/models/tether"
    os.makedirs(tether_dir, exist_ok=True)
    with open(os.path.join(tether_dir, "model.sdf"), "w") as f:
        f.write(generate_tether(num_links=30, total_length=5.0))
    print(f"Modelo Tether (Ultra Estável - Velocity Decay) gerado em: {tether_dir}")

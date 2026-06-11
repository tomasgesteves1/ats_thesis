import os
import math

def generate_tether(num_links=30, total_length=5.0):
    # Parâmetros Físicos
    radius = 0.01 
    mass = 0.02    # 20g por elo
    
    # Target Relativo (Barco -> Drone)
    tx, ty, tz = 0.8, 0.0, 0.35 

    # Geometria do "Retângulo" (U invertido)
    dh = math.sqrt(tx**2 + ty**2)
    l1 = (total_length - dh + tz) / 2.0
    l3 = l1 - tz
    
    # Tamanho base uniforme
    l_link_nominal = total_length / num_links
    
    # Miniatura (20%) para o primeiro e último elos
    l_mini = l_link_nominal * 0.2
    
    # O comprimento restante é distribuído pelos elos do meio
    remaining_length = total_length - (2 * l_mini)
    l_mid = remaining_length / (num_links - 2)

    def get_path_point(s):
        s = max(0.0, min(s, total_length))
        if s <= l1:
            # Segmento 1: Vertical para cima (Z+)
            return (0.0, 0.0, s)
        elif s <= l1 + dh:
            # Segmento 2: Horizontal (X/Y)
            ratio = (s - l1) / dh if dh > 0 else 1.0
            return (tx * ratio, ty * ratio, l1)
        else:
            # Segmento 3: Vertical para baixo (Z-)
            ratio = (s - (l1 + dh)) / l3 if l3 > 0 else 1.0
            return (tx, ty, l1 - ratio * l3)

    # Gerar pontos de junta com comprimentos variáveis
    joint_points = []
    current_s = 0.0
    joint_points.append(get_path_point(current_s))
    
    for i in range(num_links):
        if i == 0 or i == (num_links - 1):
            step = l_mini
        else:
            step = l_mid
        
        current_s += step
        # Garante precisão no final
        if i == (num_links - 1):
            current_s = total_length
            
        joint_points.append(get_path_point(current_s))

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
        
        seg_yaw = math.atan2(dy, dx)
        seg_pitch = math.atan2(math.sqrt(dx**2 + dy**2), dz)

        # Inércia elevada para estabilidade
        ixx_yy = 0.01 
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
                <geometry><cylinder><radius>{radius}</radius><length>{dist:.4f}</length></cylinder></geometry>
                <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
            </visual>
            <visual name="visual_joint">
                <pose>0 0 {-dist/2:.4f} 0 0 0</pose>
                <geometry><sphere><radius>0.0105</radius></sphere></geometry>
                <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
            </visual>
            <collision name="collision">
                <geometry><cylinder><radius>{radius}</radius><length>{dist:.4f}</length></cylinder></geometry>
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
            <pose>0 0 {-dist/2:.4f} 0 0 0</pose>
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
    tether_dir = "src/description/models/tether"
    os.makedirs(tether_dir, exist_ok=True)
    with open(os.path.join(tether_dir, "model.sdf"), "w") as f:
        f.write(generate_tether(num_links=30, total_length=5.0))
    print(f"Modelo Tether (Extremidades Miniatura) gerado em: {tether_dir}")

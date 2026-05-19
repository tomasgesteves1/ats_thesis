import os
import math

def generate_tether(num_links=10):
    # Parâmetros Físicos
    radius = 0.005 
    mass = 0.02
    
    # Target Relativo (Hook -> Drone Body)
    tx, ty, tz = 0.8, 0.0, 0.35
    total_dist = math.sqrt(tx**2 + ty**2 + tz**2)

    # Cálculo dinâmico para que os elos das pontas tenham 30% do comprimento dos intermédios
    # 2 * (0.3 * L_mid) + (num_links - 2) * L_mid = total_dist
    # (0.6 + num_links - 2) * L_mid = total_dist
    # (num_links - 1.4) * L_mid = total_dist
    mid_link_length = total_dist / (num_links - 1.4)
    end_link_length = 0.3 * mid_link_length

    # Raio da esfera da junta proporcional ao elo mais curto
    joint_visual_radius = 0.01
    
    # Ângulos
    yaw = math.atan2(ty, tx)
    pitch = math.atan2(math.sqrt(tx**2 + ty**2), tz)

    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
    <model name="tether">
        <self_collide>true</self_collide>
"""

    current_dist = 0.0
    for i in range(num_links):
        # Determinar comprimento do elo atual
        if i == 0 or i == num_links - 1:
            L = end_link_length
        else:
            L = mid_link_length
            
        # Posição do centro do cilindro (offset de L/2 em relação ao ponto de início do elo)
        # Mas para simplificar a interpolação, usamos a fração da distância total
        fraction = current_dist / total_dist if total_dist > 0 else 0
        # Ajuste para que o cilindro fique centrado no segmento
        # Pose do link é o centro do cilindro
        center_dist = current_dist + L/2
        f_center = center_dist / total_dist
        
        curr_x = tx * f_center
        curr_y = ty * f_center
        curr_z = tz * f_center
        
        sdf += f"""
        <link name="link_{i}">
            <pose>{curr_x} {curr_y} {curr_z} 0 {pitch} {yaw}</pose>
            <inertial>
                <mass>{mass}</mass>
                <inertia><ixx>1e-1</ixx><iyy>1e-1</iyy><izz>1e-2</izz></inertia>
            </inertial>
            <visual name="visual_cylinder">
                <geometry><cylinder><radius>{radius}</radius><length>{L}</length></cylinder></geometry>
                <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
            </visual>"""
        
        # Adicionar visual da JUNTA (esfera vermelha no início do elo)
        # A junta está no início do elo (relative dist = -L/2 no eixo Z local)
        if i > 0:
            sdf += f"""
            <visual name="visual_joint">
                <pose>0 0 {-L/2} 0 0 0</pose>
                <geometry><sphere><radius>{joint_visual_radius}</radius></sphere></geometry>
                <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
            </visual>"""
            
        sdf += f"""
            <collision name="collision">
                <geometry><cylinder><radius>{radius}</radius><length>{L}</length></cylinder></geometry>
                <surface>
                    <contact>
                        <collide_bitmask>0x01</collide_bitmask>
                    </contact>
                </surface>
            </collision>
        </link>"""
        
        if i > 0:
            # Junta liga o fim do elo anterior ao início do elo atual
            sdf += f"""
        <joint name="joint_{i-1}_{i}" type="ball">
            <parent>link_{i-1}</parent>
            <child>link_{i}</child>
            <pose>0 0 {-L/2} 0 0 0</pose>
            <axis><xyz>1 0 0</xyz></axis><axis2><xyz>0 1 0</xyz></axis2>
        </joint>"""
        
        current_dist += L

    sdf += """
    </model>
</sdf>
"""
    return sdf

if __name__ == "__main__":
    tether_dir = "src/ats_description/models/tether"
    os.makedirs(tether_dir, exist_ok=True)
    with open(os.path.join(tether_dir, "model.sdf"), "w") as f:
        f.write(generate_tether())
    print(f"Modelo Tether (Joints Visíveis + Pontas Curtas) gerado em: {tether_dir}")

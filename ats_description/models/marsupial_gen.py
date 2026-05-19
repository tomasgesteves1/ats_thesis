import os
import argparse

def generate_marsupial(num_links=50, total_length=7.5):
    link_length = total_length / num_links # 0.15m
    radius = 0.01
    mass = 0.03    
    
    # Ponto de ancoragem inicial (CENTRALIZADO EM 0,0)
    start_x = 0.0
    start_y = 0.0
    start_z = 1.32
    
    # Drone no topo do cabo esticado (8.82m) centralizado
    drone_x = 0.0
    drone_y = 0.0
    drone_z = start_z + total_length

    # Início do Super-SDF
    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
    <model name="marsupial">
        <pose>0 0 0 0 0 0</pose>
        <self_collide>false</self_collide>

        <!-- [Módulo 1] O Barco (WAM-V) -->
        <include>
            <uri>model://wamv</uri>
            <name>barco</name>
            <pose>0 0 0 0 0 0</pose>
        </include>

        <!-- [Módulo 2] O Drone (x500) - No topo do cabo, no centro -->
        <include merge='true'>
            <uri>model://x500</uri>
            <pose>{drone_x} {drone_y} {drone_z} 0 0 0</pose>
        </include>

        <!-- [Módulo 3] O Tether Anchor - No centro da plataforma -->
        <link name="tether_anchor_base">
            <pose>{start_x} {start_y} {start_z} 0 0 0</pose>
            <inertial>
                <mass>0.01</mass>
                <inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz></inertia>
            </inertial>
        </link>

        <joint name="tether_to_boat" type="universal">
            <parent>barco::wamv/base_link</parent>
            <child>tether_anchor_base</child>
            <axis><xyz>1 0 0</xyz><dynamics><damping>0.1</damping><friction>0.05</friction></dynamics></axis>
            <axis2><xyz>0 1 0</xyz><dynamics><damping>0.1</damping><friction>0.05</friction></dynamics></axis2>
        </joint>
"""

    # Gerar elos do cabo (Verticalmente no centro)
    for i in range(num_links):
        is_last = (i == num_links - 1)
        link_name = "link_tip" if is_last else f"link_{i}"
        
        # Posição central de cada elo na vertical (X=0, Y=0)
        z_pos = start_z + (i * link_length) + (link_length / 2)
        ixx = (1/12) * mass * (3 * radius**2 + link_length**2)

        sdf += f"""
        <link name="{link_name}">
            <pose>{start_x} {start_y} {z_pos} 0 0 0</pose>
            <self_collide>true</self_collide>
            <inertial>
                <mass>{mass}</mass>
                <inertia><ixx>{ixx}</ixx><iyy>{ixx}</iyy><izz>{0.5*mass*radius**2}</izz></inertia>
            </inertial>
            <velocity_decay><linear>0.5</linear><angular>0.5</angular></velocity_decay>
            <visual name="visual">
                <geometry><cylinder><radius>{radius}</radius><length>{link_length*0.98}</length></cylinder></geometry>
                <material>
                    <ambient>{'1 0 0 1' if is_last else '0.2 0.2 0.2 1'}</ambient>
                    <diffuse>{'1 0 0 1' if is_last else '0.1 0.1 0.1 1'}</diffuse>
                </material>
            </visual>
            <collision name="collision">
                <geometry><cylinder><radius>{radius}</radius><length>{link_length*0.90}</length></cylinder></geometry>
                <surface><contact><collide_bitmask>0x01</collide_bitmask></contact></surface>
            </collision>
        </link>"""

    # Juntas
    sdf += f"""
        <joint name="joint_start" type="universal">
            <parent>tether_anchor_base</parent><child>link_0</child>
            <pose>0 0 {-link_length/2} 0 0 0</pose>
            <axis><xyz>1 0 0</xyz><dynamics><damping>0.1</damping><friction>0.05</friction></dynamics></axis>
            <axis2><xyz>0 1 0</xyz><dynamics><damping>0.1</damping><friction>0.05</friction></dynamics></axis2>
        </joint>"""

    for i in range(num_links - 1):
        child_name = "link_tip" if (i + 1 == num_links - 1) else f"link_{i+1}"
        sdf += f"""
        <joint name="joint_{i}" type="universal">
            <parent>link_{i}</parent><child>{child_name}</child>
            <pose>0 0 {-link_length/2} 0 0 0</pose>
            <axis><xyz>1 0 0</xyz><dynamics><damping>0.1</damping><friction>0.05</friction></dynamics></axis>
            <axis2><xyz>0 1 0</xyz><dynamics><damping>0.1</damping><friction>0.05</friction></dynamics></axis2>
        </joint>"""

    # Ligação ao drone
    sdf += f"""
        <joint name="tether_to_drone" type="universal">
            <parent>link_tip</parent>
            <child>base_link</child>
            <pose>0 0 0 0 0 0</pose>
            <axis><xyz>1 0 0</xyz><dynamics><damping>0.1</damping><friction>0.05</friction></dynamics></axis>
            <axis2><xyz>0 1 0</xyz><dynamics><damping>0.1</damping><friction>0.05</friction></dynamics></axis2>
        </joint>
    </model>
</sdf>"""
    return sdf

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.parse_args()
    output_path = os.path.abspath("src/ats_description/models/marsupial/model.sdf")
    with open(output_path, "w") as f:
        f.write(generate_marsupial())
    print(f"SDF Marsupial gerado no CENTRO (X=0, Y=0): {output_path}")

import sys
import os

"""
Script para gerar um tether modular para Gazebo Harmonic.
Cria um cabo flexível composto por múltiplos links cilíndricos ligados por universal joints.
"""

def generate_tether(num_links, link_length=0.15, radius=0.01, mass=0.05):
    base_length = 0.02 # Pequeno elo para fixação na plataforma
    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
    <model name="tether">
        <pose>0 0 0 0 0 0</pose>
        <self_collide>false</self_collide>

        <!-- Link de Base para conexão com o barco via âncora -->
        <link name="link_base">
            <pose>0 0 {base_length/2} 0 0 0</pose>
            <inertial>
                <mass>0.01</mass>
                <inertia>
                    <ixx>0.00001</ixx><iyy>0.00001</iyy><izz>0.00001</izz>
                </inertia>
            </inertial>
            <visual name="visual">
                <geometry><cylinder><radius>{radius}</radius><length>{base_length}</length></cylinder></geometry>
                <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
            </visual>
        </link>
"""
    
    # Gerar a corrente de elos
    for i in range(num_links):
        z_pos = base_length + (i * link_length) + (link_length / 2)
        # Inércia correta do cilindro
        ixx = (1/12) * mass * (3 * radius**2 + link_length**2)
        iyy = ixx
        izz = 0.5 * mass * radius**2

        sdf += f"""
        <link name="link_{i}">
            <pose>0 0 {z_pos} 0 0 0</pose>
            <inertial>
                <mass>{mass}</mass>
                <inertia>
                    <ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>
                </inertia>
            </inertial>
            <velocity_decay>
                <linear>0.8</linear><angular>0.8</angular>
            </velocity_decay>
            <visual name="visual">
                <geometry><cylinder><radius>{radius}</radius><length>{link_length*0.95}</length></cylinder></geometry>
                <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
            </visual>
            <collision name="collision">
                <geometry><cylinder><radius>{radius}</radius><length>{link_length*0.95}</length></cylinder></geometry>
            </collision>
        </link>"""

    # Juntas Universal (mais estáveis que Ball Joints em correntes)
    # Primeira junta (Base -> Link 0)
    sdf += f"""
        <joint name="joint_base" type="universal">
            <parent>link_base</parent><child>link_0</child>
            <pose>0 0 {-link_length/2} 0 0 0</pose>
            <axis><xyz>1 0 0</xyz><dynamics><damping>0.1</damping><friction>0.05</friction></dynamics></axis>
            <axis2><xyz>0 1 0</xyz><dynamics><damping>0.1</damping><friction>0.05</friction></dynamics></axis2>
        </joint>"""

    # Juntas subsequentes
    for i in range(num_links - 1):
        sdf += f"""
        <joint name="joint_{i}" type="universal">
            <parent>link_{i}</parent><child>link_{i+1}</child>
            <pose>0 0 {-link_length/2} 0 0 0</pose>
            <axis><xyz>1 0 0</xyz><dynamics><damping>0.1</damping><friction>0.05</friction></dynamics></axis>
            <axis2><xyz>0 1 0</xyz><dynamics><damping>0.1</damping><friction>0.05</friction></dynamics></axis2>
        </joint>"""

    sdf += """
    </model>
</sdf>"""
    return sdf

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Generate a tether SDF model for Gazebo Harmonic.")
    parser.add_argument("-n", "--num_links", type=int, default=20, help="Number of links in the tether.")
    parser.add_argument("-l", "--length", type=float, default=0.15, help="Length of each link in meters.")
    parser.add_argument("-r", "--radius", type=float, default=0.01, help="Radius of the tether links in meters.")
    parser.add_argument("-m", "--mass", type=float, default=0.05, help="Mass of each link in kg.")

    args = parser.parse_args()

    output_path = os.path.join(os.path.dirname(__file__), "tether.sdf")
    with open(output_path, "w") as f:
        f.write(generate_tether(args.num_links, args.length, args.radius, args.mass))
    
    total_length = args.num_links * args.length
    total_mass = args.num_links * args.mass
    print(f"Tether SDF gerado em: {output_path}")
    print(f"Propriedades: {args.num_links} elos | Comprimento Total: ~{total_length:.2f}m | Massa Total: ~{total_mass:.2f}kg | Raio: {args.radius}m")

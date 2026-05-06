import os
import argparse

def generate_marsupial(num_links, link_length=0.15, radius=0.01, mass=0.05):
    # Início do Super-SDF
    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
    <model name="marsupial">
        <pose>0 0 0 0 0 0</pose>

        <!-- [Módulo 1] O Barco (WAM-V) -->
        <include>
            <uri>model://wamv</uri>
            <name>barco</name>
            <pose>0 0 0 0 0 0</pose>
        </include>

        <!-- [Módulo 2] O Drone (x500) -->
        <include merge='true'>
            <uri>model://x500</uri>
            <pose>0.0 0 1.48 0 0 0</pose>
        </include>

        <!-- [Módulo 3] O Tether -->
        <link name="tether_anchor_base">
            <pose>-0.70 0 1.3 0 0 0</pose>
            <inertial>
                <mass>0.01</mass>
                <inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz></inertia>
            </inertial>
        </link>

        <joint name="tether_to_boat" type="universal">
            <parent>barco::wamv/base_link</parent>
            <child>tether_anchor_base</child>
        </joint>
"""

    # Gerar elos do cabo
    # Ponto de partida (âncora no barco)
    start_x = -0.70
    start_z = 1.32

    for i in range(num_links):
        is_last = (i == num_links - 1)
        link_name = "link_tip" if is_last else f"link_{i}"
        
        # Posicionamento linear inicial (podemos inclinar depois se necessário)
        z_pos = start_z + (i * link_length) + (link_length / 2)
        ixx = (1/12) * mass * (3 * radius**2 + link_length**2)

        sdf += f"""
        <link name="{link_name}">
            <pose>{start_x} 0 {z_pos} 0 0 0</pose>
            <inertial>
                <mass>{mass}</mass>
                <inertia><ixx>{ixx}</ixx><iyy>{ixx}</iyy><izz>{0.5*mass*radius**2}</izz></inertia>
            </inertial>
            <velocity_decay><linear>0.8</linear><angular>0.8</angular></velocity_decay>
            <visual name="visual">
                <geometry><cylinder><radius>{radius}</radius><length>{link_length*0.95}</length></cylinder></geometry>
                <material>
                    <ambient>{'1 0 0 1' if is_last else '0.2 0.2 0.2 1'}</ambient>
                    <diffuse>{'1 0 0 1' if is_last else '0.1 0.1 0.1 1'}</diffuse>
                </material>
            </visual>
            <collision name="collision">
                <geometry><cylinder><radius>{radius}</radius><length>{link_length*0.95}</length></cylinder></collision>
        </link>"""

    # Juntas do Cabo
    sdf += """
        <joint name="joint_start" type="universal">
            <parent>tether_anchor_base</parent><child>link_0</child>
            <axis><xyz>1 0 0</xyz></axis><axis2><xyz>0 1 0</xyz></axis2>
        </joint>"""

    for i in range(num_links - 1):
        child_name = "link_tip" if (i + 1 == num_links - 1) else f"link_{i+1}"
        sdf += f"""
        <joint name="joint_{i}" type="universal">
            <parent>link_{i}</parent><child>{child_name}</child>
            <pose>0 0 {-link_length/2} 0 0 0</pose>
            <axis><xyz>1 0 0</xyz></axis><axis2><xyz>0 1 0</xyz></axis2>
        </joint>"""

    # Ligação determinística ao drone (base_link está na raiz pelo merge)
    sdf += f"""
        <joint name="tether_to_drone" type="universal">
            <parent>link_tip</parent>
            <child>base_link</child>
            <pose>0 0 {link_length/2} 0 0 0</pose>
            <axis><xyz>1 0 0</xyz></axis><axis2><xyz>0 1 0</xyz></axis2>
        </joint>
    </model>
</sdf>"""
    return sdf

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--num_links", type=int, default=15)
    args = parser.parse_args()

    # Caminho absoluto para o modelo marsupial
    output_path = os.path.abspath("src/ats_description/models/marsupial/model.sdf")
    
    with open(output_path, "w") as f:
        f.write(generate_marsupial(args.num_links))
    print(f"Marsupial Master SDF gerado em: {output_path}")

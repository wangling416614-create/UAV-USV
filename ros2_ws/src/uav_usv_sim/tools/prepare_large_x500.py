#!/usr/bin/env python3
"""Prepare the PX4 x500 geometry used by the UAV-USV simulations.

The PX4 dynamics, sensors and motor plugins remain authoritative. The
optional Unity M3-F900 profile replaces only render visuals. Product vehicles
stay at their documented 1:1 metric size in both Gazebo and Unity.
"""

import argparse
import os
import shutil
import xml.etree.ElementTree as ET


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--px4-dir', required=True)
    parser.add_argument('--scale', type=float, default=3.5)
    parser.add_argument('--camera-width', type=int, default=640)
    parser.add_argument('--camera-height', type=int, default=480)
    parser.add_argument('--camera-rate', type=float, default=20.0)
    parser.add_argument(
        '--unity-m3-f900-visuals',
        action='store_true',
        help='replace stock x500 visuals with the Unity M3-F900 profile',
    )
    return parser.parse_args()


def scale_values(text, factors):
    values = [float(value) for value in text.split()]
    for index, factor in enumerate(factors):
        if index < len(values):
            values[index] *= factor
    return ' '.join(f'{value:.10g}' for value in values)


def _material(parent, color, metallic=0.0):
    material = ET.SubElement(parent, 'material')
    ambient = ET.SubElement(material, 'ambient')
    ambient.text = ' '.join(
        f'{max(0.0, min(1.0, component * 0.45)):.6g}'
        for component in color
    ) + ' 1'
    diffuse = ET.SubElement(material, 'diffuse')
    diffuse.text = ' '.join(f'{component:.6g}' for component in color) + ' 1'
    specular = ET.SubElement(material, 'specular')
    highlight = 0.12 + 0.55 * metallic
    specular.text = f'{highlight:.6g} {highlight:.6g} {highlight:.6g} 1'


def _visual(link, name, pose, geometry_tag, geometry_values, color,
            metallic=0.0):
    visual = ET.SubElement(link, 'visual', {'name': name})
    pose_element = ET.SubElement(visual, 'pose')
    pose_element.text = ' '.join(f'{value:.10g}' for value in pose)
    geometry = ET.SubElement(visual, 'geometry')
    shape = ET.SubElement(geometry, geometry_tag)
    for tag, value in geometry_values:
        element = ET.SubElement(shape, tag)
        if isinstance(value, (tuple, list)):
            element.text = ' '.join(f'{item:.10g}' for item in value)
        else:
            element.text = f'{value:.10g}'
    _material(visual, color, metallic)


def _box(link, name, pose, size, color, metallic=0.0):
    _visual(link, name, pose, 'box', (('size', size),), color, metallic)


def _cylinder(link, name, pose, radius, length, color, metallic=0.0):
    _visual(
        link,
        name,
        pose,
        'cylinder',
        (('radius', radius), ('length', length)),
        color,
        metallic,
    )


def _model_pose_z(model):
    pose = model.find('pose')
    if pose is None or not pose.text:
        return 0.0
    values = [float(value) for value in pose.text.split()]
    return values[2] if len(values) >= 3 else 0.0


def replace_with_unity_m3_f900_visuals(root):
    """Mirror Unity's displayed M3-F900 ratio without changing PX4 control."""
    model = root.find('model')
    if model is None:
        raise RuntimeError('PX4 x500 SDF has no model element')

    links = {
        link.get('name'): link
        for link in model.findall('link')
    }
    base = links.get('base_link')
    if base is None:
        raise RuntimeError('PX4 x500 SDF has no base_link')

    for link in links.values():
        for visual in list(link.findall('visual')):
            link.remove(visual)

    # The stock x500 collision geometry is scaled for its original frame and
    # otherwise leaves a much larger invisible airframe around the Unity
    # M3-F900 profile. Keep one exact 1.20 x 1.20 x .55 m product envelope;
    # sensors and PX4 motor dynamics remain untouched.
    for link in links.values():
        for collision in list(link.findall('collision')):
            link.remove(collision)

    # Product geometry is already expressed in metres and remains 1:1.
    visual_scale = 1.0

    # Unity uses a ground-level vehicle root. PX4 keeps base_link above the
    # skids, so base-link values compensate for the nested model pose.
    base_z = _model_pose_z(model)
    carbon = (0.025, 0.030, 0.035)
    accent = (0.880, 0.080, 0.020)
    sensor = (0.240, 0.270, 0.290)
    glass = (0.030, 0.220, 0.320)

    def base_pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        return (x, y, z - base_z, roll, pitch, yaw)

    def visual_base_pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        return (
            x * visual_scale,
            y * visual_scale,
            z * visual_scale - base_z,
            roll,
            pitch,
            yaw,
        )

    def visual_size(values):
        return tuple(value * visual_scale for value in values)

    def visual_box(link, name, pose, size, color, metallic=0.0):
        _box(
            link,
            name,
            pose,
            visual_size(size),
            color,
            metallic,
        )

    def visual_cylinder(
        link,
        name,
        pose,
        radius,
        length,
        color,
        metallic=0.0,
    ):
        _cylinder(
            link,
            name,
            pose,
            radius * visual_scale,
            length * visual_scale,
            color,
            metallic,
        )

    collision = ET.SubElement(base, 'collision', {
        'name': 'm3_f900_product_envelope',
    })
    collision_pose = ET.SubElement(collision, 'pose')
    collision_pose.text = ' '.join(
        f'{value:.10g}' for value in base_pose(0, 0, .275)
    )
    collision_geometry = ET.SubElement(collision, 'geometry')
    collision_box = ET.SubElement(collision_geometry, 'box')
    collision_size = ET.SubElement(collision_box, 'size')
    collision_size.text = '1.2 1.2 .55'

    visual_box(base, 'sealed_carbon_body', visual_base_pose(0, 0, .325),
               (.43, .32, .16), carbon, .35)
    visual_box(base, 'top_cover', visual_base_pose(0, 0, .425),
               (.31, .25, .05), accent, .10)
    visual_box(base, 'jetson_orin_nano', visual_base_pose(-.07, 0, .235),
               (.14, .11, .055), sensor, .20)
    visual_box(base, 'san_60_m2', visual_base_pose(.105, 0, .225),
               (.075, .075, .07), sensor, .20)
    visual_box(base, 'd435', visual_base_pose(.225, 0, .285),
               (.025, .09, .045), glass, .18)
    # Unity primitive cylinders are one diameter wide and two units tall.
    visual_cylinder(
        base,
        'rtk_antenna',
        visual_base_pose(-.08, 0, .505),
        .0225,
        .09,
        sensor,
        .20,
    )

    motors = (
        (.318, -.318, .405, 135.0),
        (-.318, .318, .405, -45.0),
        (.318, .318, .405, 45.0),
        (-.318, -.318, .405, -135.0),
    )
    for index, (x, y, z, unity_yaw_degrees) in enumerate(motors):
        yaw = -unity_yaw_degrees * 3.141592653589793 / 180.0
        visual_box(
            base,
            f'folding_arm_{index}',
            visual_base_pose(
                x * .5,
                y * .5,
                (.37 + z) * .5,
                0,
                0,
                yaw,
            ),
            (.042, .45, .035),
            carbon,
            .35,
        )
        visual_cylinder(
            base,
            f'motor_{index}',
            visual_base_pose(x, y, z),
            .0375,
            .11,
            accent if index % 2 == 0 else carbon,
            .25,
        )

        rotor = links.get(f'rotor_{index}')
        if rotor is None:
            raise RuntimeError(f'PX4 x500 SDF has no rotor_{index}')
        rotor_pose = rotor.find('pose')
        if rotor_pose is None:
            raise RuntimeError(f'rotor_{index} has no pose')
        rotor_values = [float(value) for value in rotor_pose.text.split()]
        while len(rotor_values) < 6:
            rotor_values.append(0.0)
        rotor_values[0] = x
        rotor_values[1] = y
        rotor_values[2] = .447 - base_z
        rotor_pose.text = ' '.join(
            f'{value:.10g}' for value in rotor_values
        )
        # Keep the physical PX4 rotor link at the true actuator centre. The
        # render blades live on base_link so they cannot orbit around a
        # displaced physical joint while it spins.
        visual_box(
            base,
            f'propeller_{index}_blade_a',
            visual_base_pose(x, y, .447),
            (.56, .024, .009),
            carbon,
            .35,
        )
        visual_box(
            base,
            f'propeller_{index}_blade_b',
            visual_base_pose(x, y, .447),
            (.024, .56, .009),
            carbon,
            .35,
        )

    radians = 3.141592653589793 / 180.0
    for side in (-.19, .19):
        sign = -1.0 if side < 0 else 1.0
        visual_box(
            base,
            'landing_leg_front_neg' if side < 0 else 'landing_leg_front_pos',
            visual_base_pose(
                .13,
                side,
                .145,
                0,
                10.0 * sign * radians,
                0,
            ),
            (.025, .025, .27),
            carbon,
            .35,
        )
        visual_box(
            base,
            'landing_leg_rear_neg' if side < 0 else 'landing_leg_rear_pos',
            visual_base_pose(
                -.13,
                side,
                .145,
                0,
                -10.0 * sign * radians,
                0,
            ),
            (.025, .025, .27),
            carbon,
            .35,
        )
        visual_box(
            base,
            'landing_skid_neg' if side < 0 else 'landing_skid_pos',
            visual_base_pose(0, side, .018),
            (.42, .025, .025),
            carbon,
            .35,
        )

    model.insert(
        0,
        ET.Comment(
            ' Unity M3-F900 visual shell=1:1; physics=1:1 '
        ),
    )


def scale_model(source, destination, scale, unity_m3_f900_visuals=False):
    if unity_m3_f900_visuals and abs(scale - 1.0) > 1e-9:
        raise ValueError(
            'Unity M3-F900 geometry is already expressed at 1:1 metric '
            'scale; use --scale 1.0'
        )
    tree = ET.parse(source)
    root = tree.getroot()

    for pose in root.iter('pose'):
        if pose.text:
            pose.text = scale_values(pose.text, (scale, scale, scale))

    for geometry in root.iter('geometry'):
        for size in geometry.iter('size'):
            if size.text:
                size.text = scale_values(size.text, (scale, scale, scale))
        for mesh_scale in geometry.iter('scale'):
            if mesh_scale.text:
                mesh_scale.text = scale_values(
                    mesh_scale.text,
                    (scale, scale, scale),
                )
        for tag in ('radius', 'length'):
            for value in geometry.iter(tag):
                if value.text:
                    value.text = f'{float(value.text) * scale:.10g}'

    # This legacy geometric scaling mode is retained for non-product visual
    # experiments.  The Unity M3-F900 path above only accepts scale=1 so model
    # geometry and sensor poses are not accidentally scaled here.
    for inertial in root.iter('inertial'):
        inertia = inertial.find('inertia')
        if inertia is None:
            continue
        for tag in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'):
            value = inertia.find(tag)
            if value is not None and value.text:
                value.text = f'{float(value.text) * scale:.10g}'

    if unity_m3_f900_visuals:
        # The generated profile moves the physical motors from the stock
        # x500's 0.174 m arm coordinates to 0.318 m.  Match that increase in
        # rotational inertia so the actuator torque / angular-acceleration
        # ratio seen by PX4 remains close to the tuned stock vehicle.  Do not
        # scale model poses, collision geometry or the nested camera model.
        arm_ratio = 0.318 / 0.174
        for inertial in root.iter('inertial'):
            inertia = inertial.find('inertia')
            if inertia is None:
                continue
            for tag in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'):
                value = inertia.find(tag)
                if value is not None and value.text:
                    value.text = f'{float(value.text) * arm_ratio:.10g}'

    root.insert(
        0,
        ET.Comment(
            f' UAV_USV generated large x500, geometric scale={scale:g} '
        ),
    )
    if unity_m3_f900_visuals:
        replace_with_unity_m3_f900_visuals(root)
    ET.indent(tree, space='  ')
    tree.write(destination, encoding='UTF-8', xml_declaration=True)


def prepare_file(path, scale, unity_m3_f900_visuals=False):
    backup = path + '.uav_usv_unscaled'
    if not os.path.exists(backup):
        shutil.copy2(path, backup)
    scale_model(backup, path, scale, unity_m3_f900_visuals)


def prepare_camera(path, width, height, rate):
    backup = path + '.uav_usv_unscaled'
    if not os.path.exists(backup):
        shutil.copy2(path, backup)

    tree = ET.parse(backup)
    root = tree.getroot()
    sensor = root.find(".//sensor[@type='camera']")
    if sensor is None:
        raise RuntimeError(f'No camera sensor found in {path}')
    image = sensor.find('./camera/image')
    if image is None:
        raise RuntimeError(f'No camera image configuration found in {path}')
    image.find('width').text = str(width)
    image.find('height').text = str(height)
    sensor.find('update_rate').text = f'{rate:g}'
    root.insert(
        0,
        ET.Comment(
            f' UAV_USV camera {width}x{height} at {rate:g} FPS '
        ),
    )
    tree.write(path, encoding='UTF-8', xml_declaration=True)


def main():
    args = parse_args()
    model_root = os.path.join(
        os.path.expanduser(args.px4_dir),
        'Tools',
        'simulation',
        'gz',
        'models',
    )
    base_path = os.path.join(model_root, 'x500_base', 'model.sdf')
    camera_model_path = os.path.join(
        model_root, 'x500_mono_cam_down', 'model.sdf'
    )
    for path in (base_path, camera_model_path):
        if not os.path.isfile(path):
            raise FileNotFoundError(path)
    prepare_file(base_path, args.scale, args.unity_m3_f900_visuals)
    prepare_file(camera_model_path, args.scale)
    camera_path = os.path.join(model_root, 'mono_cam', 'model.sdf')
    if not os.path.isfile(camera_path):
        raise FileNotFoundError(camera_path)
    prepare_camera(
        camera_path,
        args.camera_width,
        args.camera_height,
        args.camera_rate,
    )
    print(f'Prepared PX4 x500 visual/collision scale: {args.scale:g}x')
    if args.unity_m3_f900_visuals:
        print('Prepared PX4 visual profile: Unity M3-F900')
    print(
        'Prepared PX4 camera: '
        f'{args.camera_width}x{args.camera_height}@{args.camera_rate:g}'
    )


if __name__ == '__main__':
    main()

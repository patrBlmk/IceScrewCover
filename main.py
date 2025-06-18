import math

import numpy as np
import trimesh
from trimesh import transformations
from shapely.geometry import Polygon

def create_main_body(outer_radius, inner_radius, height, ring_sections):
    part = trimesh.creation.annulus(r_min=inner_radius,
                                    r_max=outer_radius,
                                    height=height,
                                    sections=ring_sections)

    return part


def create_base(part, outer_radius, inner_radius, base_height, ring_sections, base_radius, height, plank_width):
    # Create base ring
    base = trimesh.creation.annulus(r_min=base_radius,
                                    r_max=outer_radius * 0.99,
                                    height=base_height,
                                    sections=ring_sections)
    base = base.apply_translation((0, 0, - height / 2 + base_height / 2))
    part = part.union(base)

    # Create connector for rope
    box = trimesh.creation.box(extents=(inner_radius * 2, plank_width, base_height))
    box = box.apply_translation((0, 0, - height / 2 + base_height / 2))
    part = part.union(box)

    return part

def add_grippers(part, outer_radius, inner_radius, num_grippers, rotate_grippers, height):
    box_sz = (outer_radius - inner_radius) * 0.8

    for i in range(num_grippers):
        angle_curr = 2 * math.pi / num_grippers * i

        # Create gripper box
        box = trimesh.creation.box((box_sz, box_sz, height))

        # Rotate it
        rot_angle = angle_curr
        if rotate_grippers:
            rot_angle = rot_angle + math.pi / 4
        rot_matrix = transformations.rotation_matrix(rot_angle, [0, 0, 1], [0, 0, 0])
        box.apply_transform(rot_matrix)

        # Move it to outer ring
        transl_x = math.cos(angle_curr) * outer_radius
        transl_y = math.sin(angle_curr) * outer_radius
        box.apply_translation((transl_x,
                               transl_y,
                               0))
        part = part.union(box)
    return part

def add_slots(part, height, slot_depth, slot_width, outer_radius):
    if slot_width > 0 and slot_depth > 0:
        offset_box = height / 2 - slot_depth / 2
        box_1 = trimesh.creation.box(extents=(3 * outer_radius, slot_width, slot_depth))
        box_2 = trimesh.creation.box(extents=(slot_width, 3 * outer_radius, slot_depth))
        box_1.apply_translation((0, 0, offset_box))
        box_2.apply_translation((0, 0, offset_box))
        part = part.difference(box_1)
        part = part.difference(box_2)

    return part

def create_thread_positive(thread_pitch, thread_depth, inner_radius, height, ring_sections):
    inner_radius = inner_radius + thread_depth
    num_rotations = height / thread_pitch
    num_path_points = round(ring_sections * (num_rotations + 1)) # + 1 to ensure thread is long enough
    angle_delta = 2 * math.pi / ring_sections
    height_delta = thread_pitch / ring_sections
    offset_z = height_delta * num_path_points / 2

    shape = Polygon([[-thread_depth, thread_depth], [0, 0], [-thread_depth, 0], [-2*thread_depth, 0], [-2*thread_depth, thread_depth]])
    path = np.zeros(shape=(num_path_points, 3))

    for i in range(num_path_points):
        transl_x = math.cos(angle_delta * i) * inner_radius
        transl_y = math.sin(angle_delta * i) * inner_radius
        transl_z = height_delta * i - offset_z
        path[i] = [transl_x, transl_y, transl_z]

    return trimesh.creation.sweep_polygon(shape, path)

def add_threads(part, thread_pitch, thread_depth, inner_radius, height, ring_sections):
    thread = create_thread_positive(thread_pitch, thread_depth, inner_radius, height, ring_sections)
    return part.difference(thread)

def trim_top(part, thread_depth, inner_radius, height, ring_sections):
    radius_use = thread_depth + inner_radius
    cone = trimesh.creation.cone(radius=radius_use,
                                 height=radius_use,
                                 sections=ring_sections)

    # Cut off the tip
    box = trimesh.creation.box(extents=(2*radius_use, 2*radius_use, 2*radius_use))
    box.apply_translation((0, 0, thread_depth + radius_use))
    cone = cone.difference(box)

    # Flip cone
    rot_matrix = transformations.rotation_matrix(math.pi, [1, 0, 0], [0, 0, 0])
    cone.apply_transform(rot_matrix)

    # Move it up
    cone.apply_translation((0, 0, height/2))

    part = part.difference(cone)
    return part

def create_threaded_ring(outer_radius, inner_radius, height, thread_pitch, slot_depth, slot_width, base_height, base_radius):
    ring_sections = 100
    num_grippers = 10
    thread_depth = 1
    rotate_grippers = False
    plank_width = outer_radius / 5

    part = create_main_body(outer_radius, inner_radius, height, ring_sections)
    part = add_grippers(part, outer_radius, inner_radius, num_grippers, rotate_grippers, height)
    part = add_threads(part, thread_pitch, thread_depth, inner_radius, height, ring_sections)
    part = add_slots(part, height, slot_depth, slot_width, outer_radius)
    part = create_base(part, outer_radius, inner_radius, base_height, ring_sections, base_radius, height, plank_width)
    part = trim_top(part, thread_depth, inner_radius, height, ring_sections)

    return part


# Create the threaded ring
threaded_ring = create_threaded_ring(outer_radius=17,
                                     inner_radius=15,
                                     height=30,
                                     thread_pitch=8,
                                     slot_depth=22,
                                     slot_width=4,
                                     base_height=2,
                                     base_radius=10)

# Export to STL
threaded_ring.export('C:/threaded_ring.stl')
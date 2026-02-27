"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
Stacking related utility functions
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def generate_stacking_layout_3d(
    pallet_width,
    pallet_length,
    box_width,
    box_length,
    box_height,
    layers=1,
    offset_x=0,
    offset_y=0,
    offset_z=0,
    spacing_x=0,
    spacing_y=0,
    spacing_z=0,
):
    """
    Generate a 3D stacking layout with box spacing.
    args:
        pallet_width: pallet width (X direction)
        pallet_length: pallet length (Y direction)
        box_width: box width (X direction)
        box_length: box length (Y direction)
        box_height: box height (Z direction)
        layers: number of layers, default 1
        offset_x, offset_y, offset_z: pallet offset in the coordinate system, default 0
        spacing_x, spacing_y, spacing_z: box spacing in each direction, default 0
    returns:
        layout: list, each element (index, x, y, z)
    """
    layout = []

    cols = int((pallet_width + spacing_x) // (box_width + spacing_x))
    rows = int((pallet_length + spacing_y) // (box_length + spacing_y))
    idx = 0
    for layer in range(layers):
        for i in range(cols):
            for j in range(rows):
                x = offset_x + i * (box_width + spacing_x)
                y = offset_y + j * (box_length + spacing_y)
                z = offset_z + layer * (box_height + spacing_z)
                layout.append((idx, x, y, z))
                idx += 1
    return layout


def plot_layout_3d(layout, box_width, box_length, box_height, placed_ids):
    """
    Visualize the 3D stacking layout using a 3D coordinate system:
    - Stored boxes: opaque display
    - Unstored boxes: semi-transparent display

    Parameters:
        layout: layout list returned by generate_stacking_layout_3d
        box_width, box_length, box_height: box 3D dimensions
        placed_ids: list of stored box IDs

    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    def draw_cube(x, y, z, dx, dy, dz, alpha):
        # define 8 vertices of the cube

        verts = [
            [x, y, z],
            [x + dx, y, z],
            [x + dx, y + dy, z],
            [x, y + dy, z],
            [x, y, z + dz],
            [x + dx, y, z + dz],
            [x + dx, y + dy, z + dz],
            [x, y + dy, z + dz],
        ]
        # define 6 faces of the cube
        faces = [
            [verts[i] for i in [0, 1, 2, 3]],
            [verts[i] for i in [4, 5, 6, 7]],
            [verts[i] for i in [0, 1, 5, 4]],
            [verts[i] for i in [2, 3, 7, 6]],
            [verts[i] for i in [1, 2, 6, 5]],
            [verts[i] for i in [4, 7, 3, 0]],
        ]
        poly = Poly3DCollection(faces, linewidths=0.5, edgecolors="black")
        poly.set_alpha(alpha)
        ax.add_collection3d(poly)

    # draw the layout
    for idx, x, y, z in layout:
        # Set different opacity based on whether it’s placed
        alpha = 1.0 if idx in placed_ids else 0.2
        draw_cube(x, y, z, box_width, box_length, box_height, alpha)
        # Add text label
        cx, cy, cz = x + box_width / 2, y + box_length / 2, z + box_height / 2
        ax.text(cx, cy, cz, str(idx), ha="center", va="center", fontsize=8)

    # Set the axis labels
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("3D Stacking Layout")
    # Set the axis limits based on the layout
    xs = [x for _, x, _, _ in layout] + [x + box_width for _, x, _, _ in layout]
    ys = [y for _, _, y, _ in layout] + [y + box_length for _, _, y, _ in layout]
    zs = [z for _, _, _, z in layout] + [z + box_height for _, _, _, z in layout]
    ax.set_xlim(min(xs), max(xs))
    ax.set_ylim(min(ys), max(ys))
    ax.set_zlim(min(zs), max(zs))

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    pallet_w, pallet_l = 1.2, 1.0  # pallet size
    box_w, box_l, box_h = 0.4, 0.5, 0.3  # box size
    layers = 3  # number of layers
    layout3d = generate_stacking_layout_3d(
        pallet_w,
        pallet_l,
        box_w,
        box_l,
        box_h,
        layers=layers,
        spacing_x=0.05,
        spacing_y=0.05,
        spacing_z=0.05,
    )
    print(layout3d)
    placed = [0, 1, 2, 3]  # IDs of placed boxes
    plot_layout_3d(layout3d, box_w, box_l, box_h, placed)

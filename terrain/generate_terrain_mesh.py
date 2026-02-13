import numpy as np
from PIL import Image

def load_heightmap(filename):
    """Load heightmap PNG and normalize to 0-1 range."""
    img = Image.open(filename)
    data = np.array(img, dtype=np.float32)
    
    # Normalize based on bit depth
    if data.max() > 255:
        data = data / 65535.0
    else:
        data = data / 255.0
    
    return data

def generate_grid_colors(height, width, grid_spacing=10):
    """Generate vertex colors with grid lines every grid_spacing meters."""
    colors = []
    sand_color = (0.76, 0.70, 0.50)
    grid_color = (0.3, 0.25, 0.15)
    
    for y in range(height):
        for x in range(width):
            # Calculate world position
            wx = (x / (width - 1)) * 100 - 50  # -50 to 50
            wy = (y / (height - 1)) * 100 - 50
            
            # Check if on grid line (every grid_spacing meters)
            on_grid_x = abs(wx % grid_spacing) < 0.5 or abs(wx % grid_spacing) > grid_spacing - 0.5
            on_grid_y = abs(wy % grid_spacing) < 0.5 or abs(wy % grid_spacing) > grid_spacing - 0.5
            
            if on_grid_x or on_grid_y:
                colors.append(grid_color)
            else:
                colors.append(sand_color)
    
    return colors

def heightmap_to_obj(heightmap_file, output_file, size_x=100, size_y=100, height_scale=0.5):
    """Convert heightmap to OBJ mesh file with grid overlay."""
    
    print(f"Loading heightmap from {heightmap_file}...")
    heightmap = load_heightmap(heightmap_file)
    h, w = heightmap.shape
    
    print(f"Heightmap size: {w}x{h}")
    print(f"Generating mesh with size {size_x}x{size_y}m, height scale {height_scale}m...")
    
    vertices = []
    normals = []
    
    # Generate grid colors (10m spacing)
    colors = generate_grid_colors(h, w, grid_spacing=10)
    
    # Scale factors
    scale_x = size_x / (w - 1)
    scale_y = size_y / (h - 1)
    
    # Center offset
    offset_x = size_x / 2
    offset_y = size_y / 2
    
    # Generate vertices
    for y in range(h):
        for x in range(w):
            vx = x * scale_x - offset_x
            vy = y * scale_y - offset_y
            vz = heightmap[y, x] * height_scale
            vertices.append((vx, vy, vz))
    
    # Generate normals
    for y in range(h):
        for x in range(w):
            if x > 0:
                hL = heightmap[y, x-1]
            else:
                hL = heightmap[y, x]
            if x < w-1:
                hR = heightmap[y, x+1]
            else:
                hR = heightmap[y, x]
            if y > 0:
                hD = heightmap[y-1, x]
            else:
                hD = heightmap[y, x]
            if y < h-1:
                hU = heightmap[y+1, x]
            else:
                hU = heightmap[y, x]
            
            nx = (hL - hR) * height_scale / (2 * scale_x)
            ny = (hD - hU) * height_scale / (2 * scale_y)
            nz = 1.0
            
            length = np.sqrt(nx*nx + ny*ny + nz*nz)
            normals.append((nx/length, ny/length, nz/length))
    
    # Generate faces - correct winding for upward-facing normals
    faces = []
    for y in range(h - 1):
        for x in range(w - 1):
            v0 = y * w + x + 1
            v1 = y * w + x + 2
            v2 = (y + 1) * w + x + 1
            v3 = (y + 1) * w + x + 2
            
            faces.append((v0, v1, v2))
            faces.append((v1, v3, v2))
    
    # Write OBJ file
    print(f"Writing OBJ file to {output_file}...")
    with open(output_file, 'w') as f:
        f.write("# Terrain mesh with grid overlay\n")
        f.write(f"# Vertices: {len(vertices)}, Faces: {len(faces)}\n\n")
        
        for v, c in zip(vertices, colors):
            f.write(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f} {c[0]:.3f} {c[1]:.3f} {c[2]:.3f}\n")
        
        f.write("\n")
        for n in normals:
            f.write(f"vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}\n")
        
        f.write("\n")
        for face in faces:
            f.write(f"f {face[0]}//{face[0]} {face[1]}//{face[1]} {face[2]}//{face[2]}\n")
    
    print(f"Done! Grid lines every 10m.")

def main():
    heightmap_to_obj(
        "sand_heightmap.png", 
        "terrain_mesh.obj",
        size_x=100,
        size_y=100,
        height_scale=0.5
    )

if __name__ == "__main__":
    main()

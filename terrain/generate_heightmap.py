import numpy as np
from PIL import Image
import random

def generate_terrain(width, height, seed=None):
    """Generate terrain with randomized bumps and flat patches."""
    if seed is not None:
        np.random.seed(seed)
        random.seed(seed)
    
    # Create base terrain with random noise
    # Use multiple octaves of noise for natural-looking terrain
    Z = np.zeros((height, width))
    
    # Large-scale rolling terrain
    freq1 = random.uniform(0.02, 0.05)
    phase1 = random.uniform(0, 2*np.pi)
    x = np.linspace(0, width * freq1, width)
    y = np.linspace(0, height * freq1, height)
    X, Y = np.meshgrid(x, y)
    Z += np.sin(X + phase1) * np.cos(Y + phase1) * 0.15
    
    # Medium bumps (rover-sized, ~2-3m wavelength)
    freq2 = random.uniform(0.1, 0.15)
    phase2 = random.uniform(0, 2*np.pi)
    x2 = np.linspace(0, width * freq2, width)
    y2 = np.linspace(0, height * freq2, height)
    X2, Y2 = np.meshgrid(x2, y2)
    Z += np.sin(X2 + phase2) * np.cos(Y2 * 0.8 + phase2) * 0.2
    
    # Small texture noise
    Z += np.random.random((height, width)) * 0.1
    
    # Add random Gaussian bumps
    num_bumps = random.randint(20, 40)
    for _ in range(num_bumps):
        cx = random.randint(0, width-1)
        cy = random.randint(0, height-1)
        radius = random.randint(5, 20)
        amplitude = random.uniform(0.1, 0.3)
        
        for dy in range(-radius, radius+1):
            for dx in range(-radius, radius+1):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < width and 0 <= ny < height:
                    dist = np.sqrt(dx**2 + dy**2)
                    if dist < radius:
                        Z[ny, nx] += amplitude * np.exp(-dist**2 / (radius/2)**2)
    
    # Create flat patches (set regions to smooth base level)
    num_flat = random.randint(5, 15)
    flat_level = Z.mean()
    for _ in range(num_flat):
        cx = random.randint(20, width-20)
        cy = random.randint(20, height-20)
        patch_w = random.randint(15, 40)
        patch_h = random.randint(15, 40)
        
        x1, x2 = max(0, cx - patch_w//2), min(width, cx + patch_w//2)
        y1, y2 = max(0, cy - patch_h//2), min(height, cy + patch_h//2)
        
        # Smooth transition to flat
        for py in range(y1, y2):
            for px in range(x1, x2):
                # Distance from patch center
                dx = abs(px - cx) / (patch_w/2)
                dy = abs(py - cy) / (patch_h/2)
                blend = max(dx, dy)
                if blend < 0.7:
                    Z[py, px] = flat_level * 0.3  # Nearly flat
                elif blend < 1.0:
                    t = (blend - 0.7) / 0.3
                    Z[py, px] = Z[py, px] * t + flat_level * 0.3 * (1-t)
    
    # Normalize to 0-1 range
    Z = (Z - Z.min()) / (Z.max() - Z.min())
    
    # Scale for 16-bit heightmap
    Z_16bit = (Z * 65535).astype(np.uint16)
    
    return Z_16bit

def main():
    import sys
    
    width = 257
    height = 257
    filename = "sand_heightmap.png"
    
    # Optional seed from command line
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else None
    
    print(f"Generating {width}x{height} randomized heightmap...")
    if seed:
        print(f"Using seed: {seed}")
    else:
        print("Using random seed (pass integer argument to reproduce)")
    
    data = generate_terrain(width, height, seed)
    
    img = Image.fromarray(data, mode='I;16')
    img.save(filename)
    print(f"Saved to {filename}")
    print("Features: Random bumps + flat patches")

if __name__ == "__main__":
    main()

import numpy as np
import matplotlib.pyplot as plt
from scipy.constants import speed_of_light
from scipy.signal import convolve
import random
import time
from typing import List, Dict, Tuple

class UWBEnvironment:
    def __init__(self, max_range: float = 500.0, 
                urban_environment: bool = True,
                noise_level: float = 0.1):
        self.max_range = max_range
        self.urban_environment = urban_environment
        self.noise_level = noise_level
        self.obstacles = []
        self.height = 4.5  # Default height (4-5 feet)
        
    def add_obstacle(self, position: Tuple[float, float], size: float):
        """Add an obstacle in the environment"""
        self.obstacles.append((position, size))
    
    def calculate_path_loss(self, distance: float) -> float:
        """Calculate signal attenuation due to environment"""
        if self.urban_environment:
            # Urban environment attenuation model
            path_loss = 20 * np.log10(distance) + 32.4 + 20 * np.log10(6.5e9)  # 6.5 GHz
            for obs in self.obstacles:
                obs_pos, obs_size = obs
                if np.linalg.norm(np.array(obs_pos) - np.array([0, 0])) < distance:
                    path_loss += 10  # Additional loss for each obstacle
        else:
            # Open space attenuation model
            path_loss = 20 * np.log10(distance) + 20 * np.log10(6.5e9)
        return path_loss

class UWBDevice:
    def __init__(self, position: Tuple[float, float] = (0, 0),
                is_anchor: bool = True,
                channel: int = 5):
        self.position = np.array(position)
        self.is_anchor = is_anchor
        self.channel = channel  # 5 (6.5 GHz) or 9 (8 GHz)
        self.antenna_delay = 0.0
        self.ranging_results = []
        
    def set_position(self, x: float, y: float):
        self.position = np.array([x, y])

class UWBSystem:
    def __init__(self, environment: UWBEnvironment):
        self.environment = environment
        self.anchors: List[UWBDevice] = []
        self.tags: List[UWBDevice] = []
        self.channel = 5  # Default channel
        
    def add_anchor(self, position: Tuple[float, float] = (0, 0)):
        if len(self.anchors) < 8:
            anchor = UWBDevice(position, is_anchor=True, channel=self.channel)
            self.anchors.append(anchor)
            return anchor
        raise ValueError("Maximum number of anchors (8) reached")
    
    def add_tag(self, position: Tuple[float, float] = (0, 0)):
        if len(self.tags) < 32:
            tag = UWBDevice(position, is_anchor=False, channel=self.channel)
            self.tags.append(tag)
            return tag
        raise ValueError("Maximum number of tags (32) reached")
    
    def calculate_distance(self, anchor: UWBDevice, tag: UWBDevice) -> Tuple[float, float]:
        """
        Simulate ToF ranging between anchor and tag
        Returns: (distance in meters, RSSI in dBm)
        """
        # Calculate true distance
        distance = np.linalg.norm(anchor.position - tag.position)
        
        # Apply environmental effects
        path_loss = self.environment.calculate_path_loss(distance)
        
        # Add noise
        noise = np.random.normal(0, self.environment.noise_level)
        distance += noise
        
        # Calculate RSSI
        rssi = -path_loss + 20  # Base power level
        
        return distance, rssi
    
    def perform_ranging(self, anchor: UWBDevice, tag: UWBDevice) -> Dict:
        """Simulate a complete ranging process"""
        distance, rssi = self.calculate_distance(anchor, tag)
        
        result = {
            'anchor_position': anchor.position.tolist(),
            'tag_position': tag.position.tolist(),
            'measured_distance': distance,
            'rssi': rssi,
            'timestamp': time.time(),
            'channel': self.channel
        }
        
        return result

class Visualization:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_title('UWB Positioning System')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        
    def update_plot(self, system: UWBSystem):
        """Update the visualization with current positions and ranging results"""
        self.ax.clear()
        
        # Plot anchors
        for anchor in system.anchors:
            self.ax.plot(anchor.position[0], anchor.position[1], 'ro', markersize=8)
            self.ax.text(anchor.position[0], anchor.position[1] + 0.5, f'A{system.anchors.index(anchor)}')
        
        # Plot tags
        for tag in system.tags:
            self.ax.plot(tag.position[0], tag.position[1], 'bo', markersize=8)
            self.ax.text(tag.position[0], tag.position[1] + 0.5, f'T{system.tags.index(tag)}')
        
        # Plot obstacles
        for obs in system.environment.obstacles:
            pos, size = obs
            circle = plt.Circle(pos, size, color='gray', alpha=0.3)
            self.ax.add_patch(circle)
        
        self.ax.grid(True)
        self.ax.axis('equal')
        plt.pause(0.1)

def main():
    # Create environment
    env = UWBEnvironment(urban_environment=True)
    
    # Create UWB system
    system = UWBSystem(env)
    
    # Add anchors in a square formation
    anchor_positions = [(0, 0), (100, 0), (100, 100), (0, 100)]
    for pos in anchor_positions:
        system.add_anchor(pos)
    
    # Add tag
    tag = system.add_tag((50, 50))
    
    # Add obstacle
    env.add_obstacle((50, 75), 10)
    
    # Create visualization
    vis = Visualization()
    
    # Simulate movement
    for t in range(100):
        # Move tag in a circular path
        angle = t * 2 * np.pi / 50
        tag.set_position(50 + 30 * np.cos(angle), 50 + 30 * np.sin(angle))
        
        # Perform ranging with each anchor
        for anchor in system.anchors:
            result = system.perform_ranging(anchor, tag)
            print(f"Anchor {system.anchors.index(anchor)} - Distance: {result['measured_distance']:.2f}m, RSSI: {result['rssi']:.1f}dBm")
        
        # Update visualization
        vis.update_plot(system)
        time.sleep(0.1)

if __name__ == "__main__":
    main()

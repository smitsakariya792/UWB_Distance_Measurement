# UWB Distance Measurement Simulation

This simulation implements a software-based model of the ESP32 DW3000 UWB distance measurement system, replicating the behavior of the Makerfabs MaUWB DW3000 module.

## Features

- Simulates Time-of-Flight (ToF) ranging principle
- Models environmental effects (urban vs open space)
- Supports up to 8 anchors and 32 tags
- Real-time visualization of positioning
- Configurable environmental parameters
- Simulates RSSI measurements
- Adjustable noise and interference

## Requirements

- Python 3.8+
- Required packages (install using `pip install -r requirements.txt`):
  - numpy
  - matplotlib
  - scipy
  - opencv-python
  - pyserial
  - python-dotenv

## Key Components

1. **UWBEnvironment**: Models the physical environment including:
   - Urban vs open space propagation
   - Obstacles and their effects
   - Signal attenuation
   - Noise levels

2. **UWBDevice**: Represents both anchors and tags with:
   - Position tracking
   - Channel configuration
   - Ranging results storage

3. **UWBSystem**: Manages the complete system including:
   - Anchor and tag management
   - Distance calculations
   - Ranging simulations
   - Channel configuration

4. **Visualization**: Provides real-time 2D visualization of:
   - Anchor and tag positions
   - Obstacles
   - Movement paths

## Running the Simulation

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Run the simulation:
```bash
python src/uwb_simulation.py
```

## Configuration

The simulation can be configured by modifying parameters in the main function:
- `urban_environment`: Set to True for urban simulation, False for open space
- `anchor_positions`: Define the positions of anchors
- `obstacles`: Add obstacles to the environment
- `noise_level`: Adjust the level of environmental noise

## Simulation Parameters

- Maximum Range: 500m (urban), 600m (open space)
- Default Height: 4.5m (4-5 feet)
- UWB Channels: 5 (6.5 GHz) and 9 (8 GHz)
- Device Limits: 8 anchors, 32 tags

## Mathematical Basis

The distance calculation is based on ToF principle:
- Distance = (Roundtrip Time × Speed of Light) / 2
- Path Loss Model: 20 * log10(distance) + 32.4 + 20 * log10(frequency)
- RSSI calculation based on path loss and environmental effects

## Notes

This is a software-only simulation and does not require any physical hardware. The simulation models the key aspects of UWB ranging while providing flexibility to adjust environmental conditions and system parameters.

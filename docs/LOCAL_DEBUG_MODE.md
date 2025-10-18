# NavSim Local Debug Mode

## Overview

The `navsim_local_debug` tool allows you to test planning algorithms locally without needing the navsim-online server. This is useful for:

- **Algorithm Development**: Quickly iterate on planning algorithms
- **Debugging**: Test specific scenarios in isolation
- **Performance Testing**: Measure planning performance without network overhead
- **CI/CD**: Automated testing of planning algorithms

---

## Quick Start

### 1. Build the Project

```bash
cd navsim-local
mkdir -p build && cd build
cmake ..
make -j4
```

### 2. Run a Simple Test

```bash
cd navsim-local
./build/navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --planner StraightLinePlanner
```

Expected output:
```
=== NavSim Local Debug Tool ===
...
[5/5] Planning result:
  Success: yes
  Planner: StraightLinePlanner
  Trajectory points: 50
  Computation time: 0.22 ms
```

---

## Command-Line Options

### Required Arguments

- `--scenario <file>`: Path to JSON scenario file
- `--planner <name>`: Planner plugin name or path

### Optional Arguments

- `--perception <plugins>`: Comma-separated list of perception plugins (e.g., `GridMapBuilder,ESDFBuilder`)
- `--visualize`: Enable real-time visualization (requires ImGui)
- `--verbose`: Enable detailed logging
- `--output <file>`: Save planning result to JSON file
- `--help`: Show help message

---

## Usage Examples

### Example 1: Basic Planning

```bash
./build/navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --planner StraightLinePlanner
```

### Example 2: With Verbose Logging

```bash
./build/navsim_local_debug \
  --scenario scenarios/parking_scenario.json \
  --planner StraightLinePlanner \
  --verbose
```

### Example 3: Using Full Plugin Path

```bash
./build/navsim_local_debug \
  --scenario scenarios/dynamic_obstacles.json \
  --planner /path/to/my_custom_planner/build/libmy_planner.so
```

### Example 4: With Perception Plugins

```bash
./build/navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --planner JpsPlanner \
  --perception GridMapBuilder,ESDFBuilder
```

### Example 5: Save Result to File

```bash
./build/navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --planner StraightLinePlanner \
  --output result.json
```

---

## Available Planners

### Built-in Planners

1. **StraightLinePlanner**
   - Simple straight-line planner
   - No obstacle avoidance
   - Good for testing basic functionality
   - **No dependencies** (works without perception plugins)

2. **AStarPlanner**
   - Grid-based A* search
   - Requires ESDF map
   - **Dependencies**: GridMapBuilder, ESDFBuilder

3. **JpsPlanner**
   - Jump Point Search planner
   - Requires ESDF map
   - **Dependencies**: GridMapBuilder, ESDFBuilder

### Custom Planners

You can load custom planners by:
1. Providing the full path to the `.so` file
2. Placing the plugin in one of the search directories (see Plugin Loading below)

---

## Plugin Loading

### Short Name Resolution

When you specify a short name like `JpsPlanner`, the tool searches in these directories (in order):

1. `build/plugins/planning/`
2. `plugins/planning/`
3. `build/plugins/perception/`
4. `plugins/perception/`
5. `~/.navsim/plugins/`
6. `./external_plugins/<name>/build/`
7. `$NAVSIM_PLUGIN_PATH/`

The tool recursively searches subdirectories and looks for `lib<name>_plugin.so`.

### Full Path Loading

You can also provide the full path to a plugin:

```bash
--planner /home/user/my_planner/build/libmy_planner.so
```

---

## Scenario Files

Scenario files are in JSON format. See `scenarios/README.md` for detailed format documentation.

### Available Test Scenarios

1. **simple_corridor.json**: Basic corridor with static obstacles
2. **parking_scenario.json**: Tight parking space
3. **dynamic_obstacles.json**: Moving vehicles with predicted trajectories

### Creating Custom Scenarios

1. Copy an existing scenario:
   ```bash
   cp scenarios/simple_corridor.json scenarios/my_scenario.json
   ```

2. Edit the JSON file to modify:
   - Ego vehicle start pose and velocity
   - Goal pose
   - Static obstacles
   - Dynamic obstacles

3. Test your scenario:
   ```bash
   ./build/navsim_local_debug --scenario scenarios/my_scenario.json --planner StraightLinePlanner
   ```

---

## Troubleshooting

### Problem: Plugin not found

**Error message**:
```
[DynamicPluginLoader] Failed to find plugin: MyPlanner
```

**Solutions**:
1. Check plugin name spelling
2. Verify plugin is built: `ls build/plugins/planning/`
3. Use full path: `--planner /full/path/to/libmy_planner.so`
4. Check `$NAVSIM_PLUGIN_PATH` environment variable

### Problem: Planner initialization failed

**Error message**:
```
[PlannerPluginManager] Failed to initialize primary planner
```

**Solutions**:
1. Check planner configuration in the code
2. Enable verbose mode: `--verbose`
3. Verify planner dependencies are loaded

### Problem: ESDF map not available

**Error message**:
```
Planner 'JpsPlanner' is not available: ESDF map not available in context
```

**Solutions**:
1. Use a planner that doesn't require ESDF (e.g., StraightLinePlanner)
2. Load perception plugins: `--perception GridMapBuilder,ESDFBuilder`
3. Modify scenario to include perception data

### Problem: Scenario fails to load

**Error message**:
```
[ScenarioLoader] Failed to load scenario
```

**Solutions**:
1. Validate JSON syntax: `jq . scenarios/my_scenario.json`
2. Check all required fields are present
3. Verify numeric values are valid (no NaN or Inf)

---

## Performance Tips

1. **Disable visualization** for performance testing (don't use `--visualize`)
2. **Use release build** for accurate performance measurements:
   ```bash
   cmake -DCMAKE_BUILD_TYPE=Release ..
   make -j4
   ```
3. **Reduce planning horizon** in scenario file for faster planning
4. **Profile with verbose mode** to identify bottlenecks

---

## Integration with CI/CD

### Example GitHub Actions Workflow

```yaml
name: Test Planning Algorithms

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Build
        run: |
          cd navsim-local
          mkdir build && cd build
          cmake ..
          make -j4
      - name: Test StraightLinePlanner
        run: |
          cd navsim-local
          ./build/navsim_local_debug \
            --scenario scenarios/simple_corridor.json \
            --planner StraightLinePlanner
      - name: Test with all scenarios
        run: |
          cd navsim-local
          for scenario in scenarios/*.json; do
            echo "Testing $scenario"
            ./build/navsim_local_debug \
              --scenario "$scenario" \
              --planner StraightLinePlanner
          done
```

---

## Next Steps

- **Develop custom planners**: See `templates/` directory for plugin templates
- **Create more scenarios**: Add complex test cases in `scenarios/`
- **Enable visualization**: Use `--visualize` to see planning results in real-time
- **Integrate with navsim-online**: Use the full system for interactive testing

---

## See Also

- [Scenario Format Documentation](../scenarios/README.md)
- [Plugin Development Guide](PLUGIN_DEVELOPMENT.md)
- [Refactoring Proposal](../REFACTORING_PROPOSAL.md)


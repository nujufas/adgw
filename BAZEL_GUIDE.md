# Bazel Build System - Quick Reference

## Project converted to Bazel successfully! ✅

### Build System Files

- **MODULE.bazel** - Bzlmod configuration (Bazel 9.0.0 uses this by default)
- **WORKSPACE** - Legacy workspace file (kept for compatibility)
- **.bazelrc** - Bazel configuration (compiler flags, optimization settings)
- **BUILD files** - In each directory defining build targets

### Available Targets

```
cc_library rule //middleware:middleware                    # All middleware components
cc_library rule //middleware/data:control_command          # ControlCommand only
cc_library rule //middleware/data:data                     # All data structures
cc_library rule //middleware/data:scene_state              # SceneState only
cc_library rule //middleware/data:sensor_data              # SensorData only
cc_library rule //middleware/data:vehicle_state            # VehicleState only
cc_library rule //middleware/utils:serialization           # Serialization utilities
cc_binary rule //tests:size_report                         # Size verification tool
cc_test rule //tests:test_data_structures                  # Unit tests
```

## Common Commands

### Build

```bash
# Build everything
bazel build //...

# Build specific target
bazel build //middleware/data:vehicle_state
bazel build //tests:test_data_structures

# Build with debug info
bazel build --compilation_mode=dbg //...

# Build optimized
bazel build --compilation_mode=opt //...
```

### Test

```bash
# Run all tests
bazel test //...

# Run specific test with output
bazel test //tests:test_data_structures --test_output=all

# Run tests in debug mode
bazel test --compilation_mode=dbg //tests:test_data_structures
```

### Run

```bash
# Run size report
bazel run //tests:size_report

# Run test directly
bazel run //tests:test_data_structures
```

### Query

```bash
# List all targets
bazel query //...

# List all tests
bazel query 'kind(cc_test, //...)'

# Show dependencies
bazel query 'deps(//tests:test_data_structures)'

# Show reverse dependencies
bazel query 'rdeps(//..., //middleware/data:vehicle_state)'
```

### Clean

```bash
# Clean build outputs
bazel clean

# Clean everything including external repositories
bazel clean --expunge
```

### Info

```bash
# Show build info
bazel info

# Show output paths
bazel info bazel-bin
bazel info bazel-genfiles

# Show workspace
bazel info workspace
```

## Configuration

### .bazelrc Settings

- **C++ Standard**: C++17
- **Compiler Warnings**: -Wall -Wextra -Wpedantic
- **Default Mode**: Optimized (opt)
- **Test Output**: Errors only (use --test_output=all for full output)
- **Color Output**: Enabled

### Compilation Modes

1. **fastbuild** (default for development)
   - Fast compilation
   - Minimal optimization
   
2. **opt** (current default via .bazelrc)
   - Optimized (-O3)
   - Release mode

3. **dbg**
   - Debug symbols (-g)
   - No optimization (-O0)

## Dependencies

The project uses:
- **rules_cc@0.2.14** (via Bzlmod) - C++ build rules
- No external dependencies for Phase 1

## Advantages of Bazel

1. **Fast Incremental Builds** - Only rebuilds changed files
2. **Hermetic** - Reproducible builds
3. **Scalable** - Handles large codebases efficiently
4. **Cross-Platform** - Linux, macOS, Windows
5. **Remote Execution** - Can distribute builds (advanced)
6. **Dependency Management** - Bzlmod for modern dependency handling

## IDE Integration

### VS Code
- Install "Bazel" extension
- Use Bazel language server for BUILD file editing

### CLion
- Built-in Bazel support
- Import as Bazel project

## Migration from CMake

The project was migrated from CMake to Bazel:

**Before (CMake)**:
- CMakeLists.txt
- build/ directory
- ninja / make commands

**After (Bazel)**:
- MODULE.bazel + WORKSPACE
- BUILD files in each directory
- bazel-* output directories (gitignored)
- bazel commands

All functionality preserved:
✓ Build targets
✓ Tests
✓ Include paths (using full paths: middleware/data/*.h)
✓ Compiler flags
✓ Optimization settings

## Troubleshooting

### Issue: "Unable to find package"
**Solution**: Check MODULE.bazel has correct dependencies

### Issue: Include path errors
**Solution**: Use full paths from workspace root: `#include "middleware/data/vehicle_state.h"`

### Issue: Slow first build
**Solution**: Normal - Bazel downloads and caches dependencies

### Issue: "Another command is running"
**Solution**: Wait for lock to release or kill bazel server: `bazel shutdown`

## Next Steps

When adding new code:
1. Create/update BUILD file in the directory
2. Add `cc_library` or `cc_binary` target
3. Specify `srcs`, `hdrs`, and `deps`
4. Use full paths in #include statements
5. Run `bazel build //...` to verify

Example:
```python
cc_library(
    name = "my_new_module",
    hdrs = ["my_new_module.h"],
    srcs = ["my_new_module.cpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//middleware/data:vehicle_state",
    ],
)
```

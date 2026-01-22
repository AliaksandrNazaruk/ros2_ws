# Final Critical Fixes Applied

## ✅ Patch 1: Map Staleness Check Fixed

**Problem**: Map staleness check was broken for static/latched maps. Map server publishes once (transient_local), not continuously.

**Solution**: Removed staleness check for map topic. Map is now checked only for "received at least once". Staleness check kept only for `/amcl_pose` (streaming topic).

**Files Changed**:
- `src/nav2_readiness_gate.cpp` - `checkTopics()` method completely rewritten

**Key Changes**:
- Map: Only checks `map_received_` (no timeout)
- AMCL pose: Checks `amcl_received_` AND staleness (timeout)
- Better error aggregation: collects all missing topics before reporting

## ✅ Patch 2: LifecycleChecker Works with LifecycleNode

**Problem**: LifecycleChecker only accepted `rclcpp::Node::SharedPtr`, so lifecycle checks were skipped when gate was created from `LifecycleNode`.

**Solution**: 
- Changed `LifecycleChecker` to use `std::variant<Node::SharedPtr, LifecycleNode::SharedPtr>`
- Used `std::visit` to create clients for both node types
- Updated `Nav2ReadinessGate` to always create checker (for both node types)

**Files Changed**:
- `include/aehub_nav2_readiness/lifecycle_checker.hpp` - Added variant support
- `src/lifecycle_checker.cpp` - Implemented variant-based client creation
- `src/nav2_readiness_gate.cpp` - Always creates checker for both node types

## ✅ Patch 3: Removed Static State from changed()

**Problem**: `changed()` used `static ReadinessLevel` which was shared across all gate instances, causing unexpected behavior.

**Solution**: Replaced static variable with member variable `last_reported_level_`.

**Files Changed**:
- `include/aehub_nav2_readiness/nav2_readiness_gate.hpp` - Added `last_reported_level_` member
- `src/nav2_readiness_gate.cpp` - `changed()` now uses member variable

## ✅ Patch 4: TF Frames Made Configurable

**Problem**: TF frames were hardcoded ("map", "base_link"), causing false NOT_READY on robots with different frame names.

**Solution**: Added constructor parameters for `global_frame` and `robot_base_frame` with defaults.

**Files Changed**:
- `include/aehub_nav2_readiness/nav2_readiness_gate.hpp` - Added parameters to constructors
- `src/nav2_readiness_gate.cpp` - Updated constructors and `checkTF()` to use configurable frames

**Default Values**:
- `global_frame`: "map"
- `robot_base_frame`: "base_link"

## Summary

All critical defects identified have been fixed:

1. ✅ Map staleness check removed (latched topics don't need staleness)
2. ✅ LifecycleChecker works with both Node and LifecycleNode
3. ✅ Static state removed from `changed()` (now instance-based)
4. ✅ TF frames are configurable (no more hardcoded "map"/"base_link")
5. ✅ Better error aggregation in `checkTopics()` (doesn't overwrite reasons)

**Status**: Ready for testing on real Nav2 system.

## Testing Checklist

After these fixes, verify:

- [ ] Map received once → gate doesn't go NOT_READY due to "stale map"
- [ ] Lifecycle check works when gate created from LifecycleNode
- [ ] Multiple gate instances don't interfere with each other's `changed()` state
- [ ] TF check works with custom frame names (e.g., "base_footprint")
- [ ] All missing topics are reported in `missing` vector (not just last one)

# Changelog

## [1.0.7] - 2026-11-11
### Fixed
- libavoid: routing is now deterministic: ties in `CmpVertInf`, crossing removal and `CmpNodesInDim` are broken by creation order instead of heap layout.
- libavoid: `getNewUniqueId()` is safe to call from several threads, so concurrent Routers give the same result as a single thread.
- Corrected the description of `ConnRef::hasValidRoute`.

## [1.0.6] - 2026-02-23
### Added
- `ConnRef::hasValidRoute` and `ConnRef::hasCrossingObstacles`.
### Fixed
- Router destructor assertion when objects (e.g. connections) were added but `processTransaction` was not called.

## [1.0.5] - 2026-02-23
### Fixed
- Reroute a connection if its checkpoints were changed.

## [1.0.4] - 2025-04-23
### Fixed
- Better detection of whether variables in orthogonal nudging are satisfied, avoiding an assertion error.

## [1.0.3] - 2025-04-19
### Added
- `ConnRef::setCallbackFunction` accepts `std::function`.
### Changed
- CI: Ubuntu 24.04 and macOS 13.

## [1.0.2] - 2025-04-11
### Fixed
- Another assertion error in orthogonal nudging, for variables with `id == channelRightID` (similar to 1.0.1).

## [1.0.1] - 2025-04-10
### Fixed
- Assertion error in orthogonal nudging (#24).

## [1.0.0] - 2024-03-12
- First versioned release of the adaptagrams package.

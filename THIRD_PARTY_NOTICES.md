# Third-Party Notices

This repository includes code and assets from multiple upstream projects. The
top-level Apache-2.0 license applies to repository-owned code unless a file,
package, or subdirectory states a different license.

## Vendored Source

| Path | Component | License status | Notes |
| --- | --- | --- | --- |
| `agentic_robot/thirdparty/src/navigation2-humble` | ROS Navigation2 Humble | Apache-2.0, see bundled `LICENSE` | Vendored upstream source. Prefer system ROS packages or a documented fork/submodule before public release. |
| `agentic_robot/thirdparty/src/rpg_vikit-ros2` | rpg_vikit ROS2 port | Needs upstream license confirmation | Bundled source has README but no top-level license file found in this checkout. Confirm upstream origin, commit, and license before release. |
| `agentic_robot/core/src/fast_livo` | FAST-LIVO integration | License mismatch requires review | `README.md` states GPLv2, while `package.xml` states BSD. Resolve before publishing binaries or claiming repository-wide licensing. |
| `agentic_robot/core/src/fast_livo/thirdparty/fast_gicp` | fast_gicp | BSD in `package.xml` | Nested third-party code is retained under its upstream terms. |
| `agentic_robot/core/src/fast_livo/thirdparty/fast_gicp/thirdparty/nvbio` | NVBIO and nested contrib libraries | See bundled `LICENSE` and nested contrib licenses | Includes additional third-party code such as lz4 and libdivsufsort-lite. |

## Models And Data

Model weights, datasets, generated logs, build outputs, and Hydra output
directories are not intended to be tracked in this source repository. Download
locations, checksums, and license terms should be documented separately before
publishing any model or dataset artifacts.

## Open Items Before Public Release

- Confirm the upstream URL, commit, and license for every vendored component.
- Decide whether vendored third-party code should remain in-tree, move to
  submodules, or be replaced with package-manager dependencies.
- Resolve the FAST-LIVO license discrepancy between README and `package.xml`.
- Add exact model provenance and license information for perception and mapping
  weights distributed outside this repository.

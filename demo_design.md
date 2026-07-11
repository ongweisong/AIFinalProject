# Demo Design Notes

This demo implements the technical prototype described in the project proposal: a small 2D platformer level is converted into a directed movement graph, then enemy-specific A* searches run over that graph.

Requirement mapping:

- Platforms: the level has two ground platforms plus several raised ledges at different heights.
- Nodes: stable graph nodes are created at platform edges, ladder endpoints, tunnel endpoints, and the clicked goal position.
- Walk edges: nodes on the same platform are connected both directions.
- Jump edges: directed links are added between platform edges when they fit the broad geometric candidate envelope. Each enemy then validates the link against its own jump distance, jump height, and landing limits.
- Drop edges: downward links are accepted only when horizontal drift and fall height are safe for the active enemy.
- Climb edges: the ladder connects lower and upper nodes, but only enemies with climb capability can use it.
- Dig edges: the tunnel connects the two ground platforms, but only enemies with dig capability can use it.
- Enemy profiles: Mole can dig but cannot jump or climb, Goblin can climb and make moderate jumps, and Wolf can make stronger jumps but cannot climb or dig.
- Runtime pathfinding: the selected enemy runs A* from the fixed start node to the clicked goal node and follows the highlighted path in real time.
- Reachability feedback: the status row reports whether each enemy can reach the current goal. Invalid edge diagnostics label rejected links with reasons such as `needs dig`, `needs climb`, `jump dx`, `jump height`, or `unsafe fall`.
- Naive comparison: the right panel runs a generic grid A* over the same rectangles. Because it treats empty air as traversable space, it often finds unsupported paths that are invalid for a platformer enemy.

The implementation is deliberately dependency-free Win32 C++ so the demo can be built with Visual Studio without requiring SFML, raylib, SDL, or asset downloads.

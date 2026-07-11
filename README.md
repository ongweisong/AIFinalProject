# Platformer Movement Graph Demo

Run `PlatformerPathGraphDemo.exe` from this folder after building the project.

Controls:

- Click inside the level to move the goal to the nearest platform surface.
- Use the Mole, Goblin, and Wolf buttons to compare different ability profiles.
- Toggle invalid edges to see rejected links and the reason they are unavailable.

The left view shows the platformer movement graph. Edge colours encode the movement type: walk, jump, drop, climb, and dig. The yellow route is the selected enemy's A* path. The status row reports which enemy types can reach the clicked goal.

The right view runs a naive grid A* on the same geometry. Its path intentionally ignores gravity, jump ranges, ladders, tunnels, and enemy abilities, so it often finds a path through unsupported air. This makes the movement-graph approach easier to compare against a generic grid search.

Build from the command line with Visual Studio Build Tools:

```bat
cd source
msbuild PlatformerPathGraphDemo.sln /p:Configuration=Release /p:Platform=x64
```

The executable is written to the `demo` folder.

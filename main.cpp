#define NOMINMAX
#include <windows.h>
#include <windowsx.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

/*
 * Platformer movement-graph pathfinding demo
 * ------------------------------------------
 * The level is represented by a small set of platforms plus a ladder and a
 * tunnel.  BuildBaseLevel creates graph nodes at platform edges and at the
 * special traversal entrances, then creates directed walk, jump, drop, climb,
 * and dig edges.  An edge describes a possible movement, while EnemyProfile
 * decides whether a particular enemy can actually use it.
 *
 * Whenever the user moves the goal, the goal is attached to the nearest
 * platform and A* is run once for each enemy type.  The selected enemy follows
 * its valid graph path while the renderer displays usable links, rejected
 * links and their rejection reasons, and the final highlighted route.  A
 * deliberately naive four-neighbour grid search is shown beside the graph to
 * demonstrate why collision-free grid movement alone is not a sufficient
 * movement model for a platform game.
 *
 * Coordinates in the graph are level-local.  ToScreen and FromScreen are the
 * boundary between those coordinates and Win32 client coordinates.
 */
namespace
{
constexpr int kEnemyCount = 3;
constexpr float kLevelW = 840.0f;
constexpr float kLevelH = 580.0f;
constexpr int kLevelLeft = 24;
constexpr int kLevelTop = 112;
constexpr int kPanelLeft = 900;
constexpr int kPanelTop = 112;
constexpr UINT_PTR kTimerId = 1;

struct Vec2
{
    float x = 0.0f;
    float y = 0.0f;
};

struct FRect
{
    float x = 0.0f;
    float y = 0.0f;
    float w = 0.0f;
    float h = 0.0f;
};

enum class NodeRole
{
    Edge,
    Ladder,
    Tunnel,
    Goal
};

enum class EdgeKind
{
    Walk,
    Jump,
    Drop,
    Climb,
    Dig
};

enum class EnemyKind
{
    Mole = 0,
    Goblin = 1,
    Wolf = 2
};

// Static collision surface and the graph nodes placed at its two ends.
struct Platform
{
    std::string name;
    FRect rect;
    int leftNode = -1;
    int rightNode = -1;
};

// A movement-graph waypoint.  platform is -1 only for nodes off a surface.
struct Node
{
    int id = -1;
    Vec2 p;
    int platform = -1;
    NodeRole role = NodeRole::Edge;
    int side = 0;
    std::string label;
};

// Directed traversal candidate plus cached geometry used by ability checks.
struct Edge
{
    int from = -1;
    int to = -1;
    EdgeKind kind = EdgeKind::Walk;
    float cost = 0.0f;
    float dx = 0.0f;
    float up = 0.0f;
    float down = 0.0f;
};

// Movement abilities and limits that turn the shared graph into an enemy-specific graph.
struct EnemyProfile
{
    EnemyKind kind;
    const char* name;
    const char* shortName;
    COLORREF color;
    bool canJump = false;
    bool canClimb = false;
    bool canDig = false;
    float maxJumpX = 0.0f;
    float maxJumpUp = 0.0f;
    float maxJumpDown = 0.0f;
    float maxDropX = 0.0f;
    float safeFall = 0.0f;
    float speed = 110.0f;
};

// A* output.  edges and nodes are stored in start-to-goal traversal order.
struct PathResult
{
    bool found = false;
    float cost = 0.0f;
    int expanded = 0;
    std::vector<int> nodes;
    std::vector<int> edges;
};

// Result of the comparison search; unsupportedRatio measures travel through open air.
struct GridPath
{
    bool found = false;
    float unsupportedRatio = 0.0f;
    int expanded = 0;
    std::vector<Vec2> points;
};

struct Button
{
    RECT rect;
    int id = 0;
    std::string label;
};

// Complete model and UI state for the single-window demo.
struct AppState
{
    std::vector<Platform> platforms;
    std::vector<Node> baseNodes;
    std::vector<Edge> baseEdges;
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    std::array<PathResult, kEnemyCount> reach;
    GridPath gridPath;
    int startNode = -1;
    int goalNode = -1;
    int goalPlatform = 4;
    Vec2 goalPoint { 380.0f, 225.0f };
    EnemyKind selectedEnemy = EnemyKind::Goblin;
    bool showInvalidEdges = true;
    size_t agentSegment = 0;
    float agentT = 0.0f;
    Vec2 agentPos;
    DWORD lastTick = 0;
};

AppState g_app;
HWND g_hwnd = nullptr;

float Distance(Vec2 a, Vec2 b)
{
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

float Clamp(float value, float lo, float hi)
{
    return std::max(lo, std::min(value, hi));
}

int EnemyIndex(EnemyKind kind)
{
    return static_cast<int>(kind);
}

const std::array<EnemyProfile, kEnemyCount>& Profiles()
{
    // The mole uses the tunnel, the goblin uses the ladder and modest jumps,
    // and the wolf trades special traversal for the strongest jump and drop.
    static const std::array<EnemyProfile, kEnemyCount> profiles {
        EnemyProfile {
            EnemyKind::Mole,
            "Mole",
            "M",
            RGB(126, 87, 194),
            false,
            false,
            true,
            0.0f,
            0.0f,
            0.0f,
            70.0f,
            95.0f,
            95.0f,
        },
        EnemyProfile {
            EnemyKind::Goblin,
            "Goblin",
            "G",
            RGB(46, 154, 91),
            true,
            true,
            false,
            145.0f,
            90.0f,
            70.0f,
            110.0f,
            160.0f,
            115.0f,
        },
        EnemyProfile {
            EnemyKind::Wolf,
            "Wolf",
            "W",
            RGB(43, 117, 205),
            true,
            false,
            false,
            220.0f,
            130.0f,
            110.0f,
            155.0f,
            205.0f,
            140.0f,
        },
    };
    return profiles;
}

const EnemyProfile& CurrentProfile()
{
    return Profiles()[EnemyIndex(g_app.selectedEnemy)];
}

const char* EdgeKindName(EdgeKind kind)
{
    switch (kind)
    {
    case EdgeKind::Walk:
        return "walk";
    case EdgeKind::Jump:
        return "jump";
    case EdgeKind::Drop:
        return "drop";
    case EdgeKind::Climb:
        return "climb";
    case EdgeKind::Dig:
        return "dig";
    }
    return "";
}

COLORREF EdgeColor(EdgeKind kind)
{
    switch (kind)
    {
    case EdgeKind::Walk:
        return RGB(70, 125, 170);
    case EdgeKind::Jump:
        return RGB(219, 127, 48);
    case EdgeKind::Drop:
        return RGB(31, 160, 170);
    case EdgeKind::Climb:
        return RGB(71, 151, 84);
    case EdgeKind::Dig:
        return RGB(129, 83, 177);
    }
    return RGB(80, 80, 80);
}

POINT ToScreen(Vec2 p)
{
    return POINT { static_cast<LONG>(std::lround(kLevelLeft + p.x)), static_cast<LONG>(std::lround(kLevelTop + p.y)) };
}

Vec2 FromScreen(int x, int y)
{
    return Vec2 { static_cast<float>(x - kLevelLeft), static_cast<float>(y - kLevelTop) };
}

RECT MakeRect(int left, int top, int right, int bottom)
{
    RECT r { left, top, right, bottom };
    return r;
}

bool PtInRectInt(const RECT& r, int x, int y)
{
    return x >= r.left && x <= r.right && y >= r.top && y <= r.bottom;
}

void AddNode(std::vector<Node>& nodes, int platform, NodeRole role, int side, Vec2 p, const std::string& label, int* outId = nullptr)
{
    const int id = static_cast<int>(nodes.size());
    nodes.push_back(Node { id, p, platform, role, side, label });
    if (outId)
    {
        *outId = id;
    }
}

float EdgeCost(EdgeKind kind, float distance)
{
    switch (kind)
    {
    case EdgeKind::Walk:
        return distance;
    case EdgeKind::Jump:
        return distance * 1.15f + 35.0f;
    case EdgeKind::Drop:
        return distance * 1.05f + 10.0f;
    case EdgeKind::Climb:
        return distance * 1.2f + 8.0f;
    case EdgeKind::Dig:
        return distance * 1.45f + 45.0f;
    }
    return distance;
}

void AddEdge(std::vector<Edge>& edges, const std::vector<Node>& nodes, int from, int to, EdgeKind kind)
{
    const Vec2 a = nodes[from].p;
    const Vec2 b = nodes[to].p;
    const float dx = std::abs(a.x - b.x);
    const float up = std::max(0.0f, a.y - b.y);
    const float down = std::max(0.0f, b.y - a.y);
    const float dist = Distance(a, b);
    edges.push_back(Edge { from, to, kind, EdgeCost(kind, dist), dx, up, down });
}

void AddWalkEdges(std::vector<Edge>& edges, const std::vector<Node>& nodes)
{
    // Every waypoint on one platform is mutually reachable by walking.  This
    // also connects edge nodes to ladder, tunnel, and goal nodes on that surface.
    for (size_t i = 0; i < nodes.size(); ++i)
    {
        for (size_t j = i + 1; j < nodes.size(); ++j)
        {
            if (nodes[i].platform == nodes[j].platform && nodes[i].platform >= 0)
            {
                AddEdge(edges, nodes, static_cast<int>(i), static_cast<int>(j), EdgeKind::Walk);
                AddEdge(edges, nodes, static_cast<int>(j), static_cast<int>(i), EdgeKind::Walk);
            }
        }
    }
}

bool EdgeIsUsable(const Edge& edge, const EnemyProfile& profile, std::string* reason)
{
    // Graph construction records geometrically plausible candidates.  This
    // second-stage filter applies the selected enemy's abilities and limits.
    switch (edge.kind)
    {
    case EdgeKind::Walk:
        return true;
    case EdgeKind::Climb:
        if (!profile.canClimb)
        {
            if (reason)
            {
                *reason = "needs climb";
            }
            return false;
        }
        return true;
    case EdgeKind::Dig:
        if (!profile.canDig)
        {
            if (reason)
            {
                *reason = "needs dig";
            }
            return false;
        }
        return true;
    case EdgeKind::Jump:
        if (!profile.canJump)
        {
            if (reason)
            {
                *reason = "no jump";
            }
            return false;
        }
        if (edge.dx > profile.maxJumpX)
        {
            if (reason)
            {
                *reason = "jump dx";
            }
            return false;
        }
        if (edge.up > profile.maxJumpUp)
        {
            if (reason)
            {
                *reason = "jump height";
            }
            return false;
        }
        if (edge.down > profile.maxJumpDown)
        {
            if (reason)
            {
                *reason = "bad landing";
            }
            return false;
        }
        return true;
    case EdgeKind::Drop:
        if (edge.dx > profile.maxDropX)
        {
            if (reason)
            {
                *reason = "drop dx";
            }
            return false;
        }
        if (edge.down > profile.safeFall)
        {
            if (reason)
            {
                *reason = "unsafe fall";
            }
            return false;
        }
        return true;
    }
    return false;
}

PathResult FindPath(int start, int goal, const EnemyProfile& profile, const std::vector<Node>& nodes, const std::vector<Edge>& edges)
{
    // A* runs over only those directed edges that the supplied profile can use.
    struct Item
    {
        int node = -1;
        float f = 0.0f;
        bool operator<(const Item& other) const
        {
            return f > other.f;
        }
    };

    PathResult result;
    const int n = static_cast<int>(nodes.size());
    const float inf = std::numeric_limits<float>::infinity();
    std::vector<float> g(n, inf);
    std::vector<int> cameNode(n, -1);
    std::vector<int> cameEdge(n, -1);
    std::vector<char> closed(n, 0);
    std::vector<std::vector<int>> adjacency(n);

    // Build an enemy-specific adjacency list without modifying the shared graph.
    for (int i = 0; i < static_cast<int>(edges.size()); ++i)
    {
        std::string unused;
        if (EdgeIsUsable(edges[i], profile, &unused))
        {
            adjacency[edges[i].from].push_back(i);
        }
    }

    // Straight-line distance is an admissible lower bound because every edge
    // cost is at least its geometric length.
    auto heuristic = [&](int a) {
        return Distance(nodes[a].p, nodes[goal].p);
    };

    std::priority_queue<Item> open;
    g[start] = 0.0f;
    open.push(Item { start, heuristic(start) });

    while (!open.empty())
    {
        const Item current = open.top();
        open.pop();
        if (closed[current.node])
        {
            continue;
        }
        closed[current.node] = 1;
        ++result.expanded;

        if (current.node == goal)
        {
            result.found = true;
            break;
        }

        for (int edgeIndex : adjacency[current.node])
        {
            const Edge& edge = edges[edgeIndex];
            const int next = edge.to;
            const float nextG = g[current.node] + edge.cost;
            if (nextG < g[next])
            {
                g[next] = nextG;
                cameNode[next] = current.node;
                cameEdge[next] = edgeIndex;
                open.push(Item { next, nextG + heuristic(next) });
            }
        }
    }

    if (!result.found)
    {
        return result;
    }

    // Follow predecessor links backwards, then reverse both sequences so the
    // animation can consume them from start to goal.
    result.cost = g[goal];
    for (int at = goal; at != -1; at = cameNode[at])
    {
        result.nodes.push_back(at);
        if (cameEdge[at] != -1)
        {
            result.edges.push_back(cameEdge[at]);
        }
    }
    std::reverse(result.nodes.begin(), result.nodes.end());
    std::reverse(result.edges.begin(), result.edges.end());
    return result;
}

bool PointInsidePlatform(Vec2 p, const Platform& platform)
{
    const FRect& r = platform.rect;
    return p.x >= r.x && p.x <= r.x + r.w && p.y >= r.y && p.y <= r.y + r.h;
}

bool PointIsSupported(Vec2 p)
{
    for (const Platform& platform : g_app.platforms)
    {
        const FRect& r = platform.rect;
        if (p.x >= r.x - 10.0f && p.x <= r.x + r.w + 10.0f && std::abs((p.y + 10.0f) - r.y) < 16.0f)
        {
            return true;
        }
    }

    if (std::abs(p.x - 205.0f) < 14.0f && p.y >= 395.0f && p.y <= 525.0f)
    {
        return true;
    }

    if (p.x >= 340.0f && p.x <= 515.0f && std::abs(p.y - 548.0f) < 18.0f)
    {
        return true;
    }

    return false;
}

GridPath BuildNaiveGridPath()
{
    // This comparison intentionally models only blocked/free cells.  It has no
    // gravity, support, jump arc, ladder, tunnel, or enemy-ability semantics.
    struct Item
    {
        int index = -1;
        float f = 0.0f;
        bool operator<(const Item& other) const
        {
            return f > other.f;
        }
    };

    GridPath result;
    constexpr int cell = 20;
    const int cols = static_cast<int>(std::ceil(kLevelW / cell));
    const int rows = static_cast<int>(std::ceil(kLevelH / cell));
    const int total = cols * rows;

    auto toIndex = [&](int x, int y) {
        return y * cols + x;
    };
    auto toCell = [&](Vec2 p) {
        int x = static_cast<int>(Clamp(std::floor(p.x / cell), 0.0f, static_cast<float>(cols - 1)));
        int y = static_cast<int>(Clamp(std::floor((p.y - 10.0f) / cell), 0.0f, static_cast<float>(rows - 1)));
        return POINT { x, y };
    };
    auto center = [&](int index) {
        const int x = index % cols;
        const int y = index / cols;
        return Vec2 { x * static_cast<float>(cell) + cell * 0.5f, y * static_cast<float>(cell) + cell * 0.5f };
    };
    auto blocked = [&](int x, int y) {
        Vec2 p { x * static_cast<float>(cell) + cell * 0.5f, y * static_cast<float>(cell) + cell * 0.5f };
        for (const Platform& platform : g_app.platforms)
        {
            if (PointInsidePlatform(p, platform))
            {
                return true;
            }
        }
        return false;
    };
    auto nearestOpen = [&](POINT startCell) {
        if (!blocked(startCell.x, startCell.y))
        {
            return startCell;
        }
        for (int radius = 1; radius < 8; ++radius)
        {
            for (int dy = -radius; dy <= radius; ++dy)
            {
                for (int dx = -radius; dx <= radius; ++dx)
                {
                    const int nx = startCell.x + dx;
                    const int ny = startCell.y + dy;
                    if (nx >= 0 && nx < cols && ny >= 0 && ny < rows && !blocked(nx, ny))
                    {
                        return POINT { nx, ny };
                    }
                }
            }
        }
        return startCell;
    };

    POINT s = nearestOpen(toCell(g_app.nodes[g_app.startNode].p));
    POINT g = nearestOpen(toCell(g_app.goalPoint));
    const int start = toIndex(s.x, s.y);
    const int goal = toIndex(g.x, g.y);

    const float inf = std::numeric_limits<float>::infinity();
    std::vector<float> dist(total, inf);
    std::vector<int> came(total, -1);
    std::vector<char> closed(total, 0);
    std::priority_queue<Item> open;

    auto heuristic = [&](int index) {
        const Vec2 a = center(index);
        const Vec2 b = center(goal);
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    };

    dist[start] = 0.0f;
    open.push(Item { start, heuristic(start) });

    const int offsets[4][2] {
        { 1, 0 },
        { -1, 0 },
        { 0, 1 },
        { 0, -1 },
    };

    while (!open.empty())
    {
        const Item item = open.top();
        open.pop();
        if (closed[item.index])
        {
            continue;
        }
        closed[item.index] = 1;
        ++result.expanded;
        if (item.index == goal)
        {
            result.found = true;
            break;
        }

        const int cx = item.index % cols;
        const int cy = item.index / cols;
        for (const auto& offset : offsets)
        {
            const int nx = cx + offset[0];
            const int ny = cy + offset[1];
            if (nx < 0 || nx >= cols || ny < 0 || ny >= rows || blocked(nx, ny))
            {
                continue;
            }
            const int next = toIndex(nx, ny);
            const float nd = dist[item.index] + 1.0f;
            if (nd < dist[next])
            {
                dist[next] = nd;
                came[next] = item.index;
                open.push(Item { next, nd + heuristic(next) });
            }
        }
    }

    if (!result.found)
    {
        return result;
    }

    for (int at = goal; at != -1; at = came[at])
    {
        result.points.push_back(center(at));
    }
    std::reverse(result.points.begin(), result.points.end());

    // Quantify the conceptual failure by checking how much of the path is not
    // supported by a platform, ladder, or tunnel.
    int unsupported = 0;
    for (Vec2 p : result.points)
    {
        if (!PointIsSupported(p))
        {
            ++unsupported;
        }
    }
    result.unsupportedRatio = result.points.empty() ? 0.0f : static_cast<float>(unsupported) / static_cast<float>(result.points.size());
    return result;
}

void AddDirectedPlatformLink(int platformA, int platformB)
{
    // Link the pair of facing platform edges.  Screen y increases downwards,
    // so a positive `up` value means that the destination is visually higher.
    const Platform& a = g_app.platforms[platformA];
    const Platform& b = g_app.platforms[platformB];
    const float centerA = a.rect.x + a.rect.w * 0.5f;
    const float centerB = b.rect.x + b.rect.w * 0.5f;
    const bool targetRight = centerB > centerA;
    const int from = targetRight ? a.rightNode : a.leftNode;
    const int to = targetRight ? b.leftNode : b.rightNode;
    const Vec2 p0 = g_app.baseNodes[from].p;
    const Vec2 p1 = g_app.baseNodes[to].p;
    const float dx = std::abs(p0.x - p1.x);
    const float up = std::max(0.0f, p0.y - p1.y);
    const float down = std::max(0.0f, p1.y - p0.y);

    // Discard candidates outside the demo's broad geometric envelope.  Exact
    // per-enemy limits are applied later by EdgeIsUsable.
    if (dx > 285.0f || up > 175.0f || down > 240.0f)
    {
        return;
    }

    const EdgeKind kind = down > 45.0f ? EdgeKind::Drop : EdgeKind::Jump;
    AddEdge(g_app.baseEdges, g_app.baseNodes, from, to, kind);
}

void BuildBaseLevel()
{
    // Construct immutable level geometry first.  The movable goal is added to
    // a copy of this base graph by RebuildScenarioGraph.
    g_app.platforms.clear();
    g_app.baseNodes.clear();
    g_app.baseEdges.clear();

    g_app.platforms.push_back(Platform { "Ground A", FRect { 50.0f, 525.0f, 290.0f, 24.0f } });
    g_app.platforms.push_back(Platform { "Ground B", FRect { 515.0f, 525.0f, 270.0f, 24.0f } });
    g_app.platforms.push_back(Platform { "Left Ledge", FRect { 105.0f, 395.0f, 235.0f, 18.0f } });
    g_app.platforms.push_back(Platform { "Mid Ledge", FRect { 430.0f, 340.0f, 230.0f, 18.0f } });
    g_app.platforms.push_back(Platform { "Top Ledge", FRect { 270.0f, 225.0f, 215.0f, 18.0f } });
    g_app.platforms.push_back(Platform { "Right Shelf", FRect { 610.0f, 450.0f, 160.0f, 18.0f } });

    for (int i = 0; i < static_cast<int>(g_app.platforms.size()); ++i)
    {
        Platform& platform = g_app.platforms[i];
        AddNode(g_app.baseNodes, i, NodeRole::Edge, -1, Vec2 { platform.rect.x, platform.rect.y }, platform.name + " L", &platform.leftNode);
        AddNode(g_app.baseNodes, i, NodeRole::Edge, 1, Vec2 { platform.rect.x + platform.rect.w, platform.rect.y }, platform.name + " R", &platform.rightNode);
    }

    int ladderBottom = -1;
    int ladderTop = -1;
    int tunnelLeft = -1;
    int tunnelRight = -1;
    AddNode(g_app.baseNodes, 0, NodeRole::Ladder, 0, Vec2 { 205.0f, 525.0f }, "Ladder bottom", &ladderBottom);
    AddNode(g_app.baseNodes, 2, NodeRole::Ladder, 0, Vec2 { 205.0f, 395.0f }, "Ladder top", &ladderTop);
    AddNode(g_app.baseNodes, 0, NodeRole::Tunnel, 0, Vec2 { 340.0f, 525.0f }, "Tunnel left", &tunnelLeft);
    AddNode(g_app.baseNodes, 1, NodeRole::Tunnel, 0, Vec2 { 515.0f, 525.0f }, "Tunnel right", &tunnelRight);

    // Walking connects all nodes that share a platform; ladder and tunnel
    // connections are explicitly bidirectional special-movement edges.
    AddWalkEdges(g_app.baseEdges, g_app.baseNodes);
    AddEdge(g_app.baseEdges, g_app.baseNodes, ladderBottom, ladderTop, EdgeKind::Climb);
    AddEdge(g_app.baseEdges, g_app.baseNodes, ladderTop, ladderBottom, EdgeKind::Climb);
    AddEdge(g_app.baseEdges, g_app.baseNodes, tunnelLeft, tunnelRight, EdgeKind::Dig);
    AddEdge(g_app.baseEdges, g_app.baseNodes, tunnelRight, tunnelLeft, EdgeKind::Dig);

    // Test every ordered platform pair because jump/drop reachability and edge
    // type depend on the direction of travel.
    for (int a = 0; a < static_cast<int>(g_app.platforms.size()); ++a)
    {
        for (int b = 0; b < static_cast<int>(g_app.platforms.size()); ++b)
        {
            if (a != b)
            {
                AddDirectedPlatformLink(a, b);
            }
        }
    }

    g_app.startNode = g_app.platforms[0].leftNode;
}

void ResetAgent()
{
    g_app.agentSegment = 0;
    g_app.agentT = 0.0f;
    g_app.agentPos = g_app.nodes[g_app.startNode].p;
    g_app.lastTick = GetTickCount();
}

void RecomputePaths()
{
    // Keep reachability results for all three profiles so the status row can
    // update immediately even when only one enemy is currently animated.
    for (const EnemyProfile& profile : Profiles())
    {
        g_app.reach[EnemyIndex(profile.kind)] = FindPath(g_app.startNode, g_app.goalNode, profile, g_app.nodes, g_app.edges);
    }
    g_app.gridPath = BuildNaiveGridPath();
    ResetAgent();
}

void RebuildScenarioGraph()
{
    // Recreate the transient graph from the fixed level before inserting the
    // current goal.  This prevents old goal nodes and links from accumulating.
    g_app.nodes = g_app.baseNodes;
    g_app.edges = g_app.baseEdges;

    g_app.goalNode = static_cast<int>(g_app.nodes.size());
    g_app.nodes.push_back(Node { g_app.goalNode, g_app.goalPoint, g_app.goalPlatform, NodeRole::Goal, 0, "Goal" });

    // A goal on a platform behaves like another waypoint on that surface and
    // therefore receives walk links in both directions.
    for (int i = 0; i < g_app.goalNode; ++i)
    {
        if (g_app.nodes[i].platform == g_app.goalPlatform)
        {
            AddEdge(g_app.edges, g_app.nodes, i, g_app.goalNode, EdgeKind::Walk);
            AddEdge(g_app.edges, g_app.nodes, g_app.goalNode, i, EdgeKind::Walk);
        }
    }

    RecomputePaths();
}

void SetGoalFromClick(Vec2 click)
{
    // Snap an arbitrary click to the nearest walkable platform top.  The inset
    // keeps the marker away from the extreme platform corners.
    float best = std::numeric_limits<float>::infinity();
    int bestPlatform = 0;
    Vec2 bestPoint;

    for (int i = 0; i < static_cast<int>(g_app.platforms.size()); ++i)
    {
        const FRect& r = g_app.platforms[i].rect;
        const float x = Clamp(click.x, r.x + 12.0f, r.x + r.w - 12.0f);
        const float y = r.y;
        const float dx = click.x - x;
        const float dy = click.y - y;
        const float score = dx * dx + dy * dy;
        if (score < best)
        {
            best = score;
            bestPlatform = i;
            bestPoint = Vec2 { x, y };
        }
    }

    g_app.goalPlatform = bestPlatform;
    g_app.goalPoint = bestPoint;
    RebuildScenarioGraph();
}

std::vector<POINT> EdgePolyline(const Edge& edge, const std::vector<Node>& nodes)
{
    // Produce the same visual movement shapes used by the agent: a quadratic
    // arc for jumps, a bowed underground curve for digging, and lines otherwise.
    const Vec2 a = nodes[edge.from].p;
    const Vec2 b = nodes[edge.to].p;
    std::vector<POINT> points;

    if (edge.kind == EdgeKind::Jump)
    {
        const float apex = std::min(a.y, b.y) - 60.0f - edge.up * 0.25f;
        const Vec2 control { (a.x + b.x) * 0.5f, apex };
        for (int i = 0; i <= 18; ++i)
        {
            const float t = i / 18.0f;
            const float u = 1.0f - t;
            Vec2 p {
                u * u * a.x + 2.0f * u * t * control.x + t * t * b.x,
                u * u * a.y + 2.0f * u * t * control.y + t * t * b.y,
            };
            points.push_back(ToScreen(p));
        }
    }
    else if (edge.kind == EdgeKind::Dig)
    {
        for (int i = 0; i <= 20; ++i)
        {
            const float t = i / 20.0f;
            Vec2 p {
                a.x + (b.x - a.x) * t,
                a.y + (b.y - a.y) * t + std::sin(t * 3.14159265f) * 48.0f,
            };
            points.push_back(ToScreen(p));
        }
    }
    else
    {
        points.push_back(ToScreen(a));
        points.push_back(ToScreen(b));
    }

    return points;
}

Vec2 PointOnEdge(const Edge& edge, float t)
{
    // Evaluate an edge at normalized progress t for animation.  These formulas
    // mirror EdgePolyline so the agent remains centered on the displayed link.
    const Vec2 a = g_app.nodes[edge.from].p;
    const Vec2 b = g_app.nodes[edge.to].p;
    t = Clamp(t, 0.0f, 1.0f);

    if (edge.kind == EdgeKind::Jump)
    {
        const float apex = std::min(a.y, b.y) - 60.0f - edge.up * 0.25f;
        const Vec2 c { (a.x + b.x) * 0.5f, apex };
        const float u = 1.0f - t;
        return Vec2 {
            u * u * a.x + 2.0f * u * t * c.x + t * t * b.x,
            u * u * a.y + 2.0f * u * t * c.y + t * t * b.y,
        };
    }

    if (edge.kind == EdgeKind::Dig)
    {
        return Vec2 {
            a.x + (b.x - a.x) * t,
            a.y + (b.y - a.y) * t + std::sin(t * 3.14159265f) * 48.0f,
        };
    }

    return Vec2 {
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
    };
}

void AdvanceAgent(float dt)
{
    // Spend this frame's travel budget across as many path edges as necessary;
    // this keeps motion speed independent of timer frequency and edge length.
    const PathResult& path = g_app.reach[EnemyIndex(g_app.selectedEnemy)];
    if (!path.found || path.nodes.size() < 2)
    {
        g_app.agentPos = g_app.nodes[g_app.startNode].p;
        return;
    }

    const EnemyProfile& profile = CurrentProfile();
    float travel = dt * profile.speed;
    while (travel > 0.0f && g_app.agentSegment < path.edges.size())
    {
        const Edge& edge = g_app.edges[path.edges[g_app.agentSegment]];
        const float length = std::max(1.0f, Distance(g_app.nodes[edge.from].p, g_app.nodes[edge.to].p));
        const float remaining = (1.0f - g_app.agentT) * length;
        if (travel < remaining)
        {
            g_app.agentT += travel / length;
            travel = 0.0f;
        }
        else
        {
            travel -= remaining;
            g_app.agentT = 0.0f;
            ++g_app.agentSegment;
        }
    }

    if (g_app.agentSegment >= path.edges.size())
    {
        g_app.agentPos = g_app.nodes[g_app.goalNode].p;
    }
    else
    {
        g_app.agentPos = PointOnEdge(g_app.edges[path.edges[g_app.agentSegment]], g_app.agentT);
    }
}

void DrawTextAAt(HDC hdc, int x, int y, const std::string& text, COLORREF color)
{
    SetTextColor(hdc, color);
    SetBkMode(hdc, TRANSPARENT);
    TextOutA(hdc, x, y, text.c_str(), static_cast<int>(text.size()));
}

void DrawCenteredText(HDC hdc, const RECT& rect, const std::string& text, COLORREF color)
{
    SetTextColor(hdc, color);
    SetBkMode(hdc, TRANSPARENT);
    DrawTextA(hdc, text.c_str(), static_cast<int>(text.size()), const_cast<RECT*>(&rect), DT_CENTER | DT_VCENTER | DT_SINGLELINE);
}

void FillRectColor(HDC hdc, const RECT& rect, COLORREF color)
{
    HBRUSH brush = CreateSolidBrush(color);
    FillRect(hdc, &rect, brush);
    DeleteObject(brush);
}

void StrokeRectColor(HDC hdc, const RECT& rect, COLORREF color)
{
    HBRUSH brush = CreateSolidBrush(color);
    FrameRect(hdc, &rect, brush);
    DeleteObject(brush);
}

void DrawLine(HDC hdc, POINT a, POINT b, COLORREF color, int width = 1, int style = PS_SOLID)
{
    HPEN pen = CreatePen(style, width, color);
    HGDIOBJ oldPen = SelectObject(hdc, pen);
    MoveToEx(hdc, a.x, a.y, nullptr);
    LineTo(hdc, b.x, b.y);
    SelectObject(hdc, oldPen);
    DeleteObject(pen);
}

void DrawPolyline(HDC hdc, const std::vector<POINT>& points, COLORREF color, int width, int style = PS_SOLID)
{
    if (points.size() < 2)
    {
        return;
    }

    HPEN pen = CreatePen(style, width, color);
    HGDIOBJ oldPen = SelectObject(hdc, pen);
    Polyline(hdc, points.data(), static_cast<int>(points.size()));
    SelectObject(hdc, oldPen);
    DeleteObject(pen);
}

void DrawArrowHead(HDC hdc, const std::vector<POINT>& points, COLORREF color)
{
    if (points.size() < 2)
    {
        return;
    }

    const POINT end = points.back();
    POINT prev = points[points.size() - 2];
    float dx = static_cast<float>(end.x - prev.x);
    float dy = static_cast<float>(end.y - prev.y);
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 0.1f)
    {
        return;
    }
    dx /= len;
    dy /= len;
    const float nx = -dy;
    const float ny = dx;
    const float size = 7.0f;
    POINT wingA { static_cast<LONG>(end.x - dx * size + nx * size * 0.55f), static_cast<LONG>(end.y - dy * size + ny * size * 0.55f) };
    POINT wingB { static_cast<LONG>(end.x - dx * size - nx * size * 0.55f), static_cast<LONG>(end.y - dy * size - ny * size * 0.55f) };
    DrawLine(hdc, end, wingA, color, 1);
    DrawLine(hdc, end, wingB, color, 1);
}

void DrawCircle(HDC hdc, POINT center, int radius, COLORREF fill, COLORREF outline)
{
    HBRUSH brush = CreateSolidBrush(fill);
    HPEN pen = CreatePen(PS_SOLID, 1, outline);
    HGDIOBJ oldBrush = SelectObject(hdc, brush);
    HGDIOBJ oldPen = SelectObject(hdc, pen);
    Ellipse(hdc, center.x - radius, center.y - radius, center.x + radius, center.y + radius);
    SelectObject(hdc, oldBrush);
    SelectObject(hdc, oldPen);
    DeleteObject(brush);
    DeleteObject(pen);
}

void DrawLevelGeometry(HDC hdc)
{
    // Draw physical level features before graph diagnostics and the agent.
    RECT levelRect = MakeRect(kLevelLeft, kLevelTop, kLevelLeft + static_cast<int>(kLevelW), kLevelTop + static_cast<int>(kLevelH));
    FillRectColor(hdc, levelRect, RGB(242, 244, 247));
    StrokeRectColor(hdc, levelRect, RGB(190, 198, 208));

    HPEN gridPen = CreatePen(PS_SOLID, 1, RGB(228, 232, 238));
    HGDIOBJ oldPen = SelectObject(hdc, gridPen);
    for (int x = 40; x < static_cast<int>(kLevelW); x += 40)
    {
        MoveToEx(hdc, kLevelLeft + x, kLevelTop, nullptr);
        LineTo(hdc, kLevelLeft + x, kLevelTop + static_cast<int>(kLevelH));
    }
    for (int y = 40; y < static_cast<int>(kLevelH); y += 40)
    {
        MoveToEx(hdc, kLevelLeft, kLevelTop + y, nullptr);
        LineTo(hdc, kLevelLeft + static_cast<int>(kLevelW), kLevelTop + y);
    }
    SelectObject(hdc, oldPen);
    DeleteObject(gridPen);

    for (const Platform& platform : g_app.platforms)
    {
        RECT r = MakeRect(
            kLevelLeft + static_cast<int>(platform.rect.x),
            kLevelTop + static_cast<int>(platform.rect.y),
            kLevelLeft + static_cast<int>(platform.rect.x + platform.rect.w),
            kLevelTop + static_cast<int>(platform.rect.y + platform.rect.h));
        FillRectColor(hdc, r, RGB(105, 113, 125));
        RECT top = MakeRect(r.left, r.top, r.right, r.top + 4);
        FillRectColor(hdc, top, RGB(75, 149, 106));
    }

    POINT ladderA = ToScreen(Vec2 { 205.0f, 395.0f });
    POINT ladderB = ToScreen(Vec2 { 205.0f, 525.0f });
    DrawLine(hdc, POINT { ladderA.x - 10, ladderA.y }, POINT { ladderB.x - 10, ladderB.y }, EdgeColor(EdgeKind::Climb), 3);
    DrawLine(hdc, POINT { ladderA.x + 10, ladderA.y }, POINT { ladderB.x + 10, ladderB.y }, EdgeColor(EdgeKind::Climb), 3);
    for (int y = ladderA.y + 12; y < ladderB.y; y += 20)
    {
        DrawLine(hdc, POINT { ladderA.x - 12, y }, POINT { ladderA.x + 12, y }, EdgeColor(EdgeKind::Climb), 2);
    }

    std::vector<POINT> tunnel;
    for (int i = 0; i <= 20; ++i)
    {
        const float t = i / 20.0f;
        Vec2 p { 340.0f + (515.0f - 340.0f) * t, 525.0f + std::sin(t * 3.14159265f) * 48.0f };
        tunnel.push_back(ToScreen(p));
    }
    DrawPolyline(hdc, tunnel, EdgeColor(EdgeKind::Dig), 4, PS_DOT);
}

void DrawGraph(HDC hdc)
{
    // Invalid special links are drawn first so usable links and the selected
    // A* route remain visually dominant.  Walk links need no rejection label.
    const EnemyProfile& profile = CurrentProfile();

    if (g_app.showInvalidEdges)
    {
        int labels = 0;
        for (const Edge& edge : g_app.edges)
        {
            std::string reason;
            if (edge.kind != EdgeKind::Walk && !EdgeIsUsable(edge, profile, &reason))
            {
                std::vector<POINT> points = EdgePolyline(edge, g_app.nodes);
                DrawPolyline(hdc, points, RGB(191, 91, 91), 1, PS_DASH);
                if (labels < 12 && !reason.empty())
                {
                    const POINT mid = points[points.size() / 2];
                    DrawTextAAt(hdc, mid.x + 3, mid.y + 3, reason, RGB(132, 61, 61));
                    ++labels;
                }
            }
        }
    }

    for (const Edge& edge : g_app.edges)
    {
        std::string reason;
        if (EdgeIsUsable(edge, profile, &reason))
        {
            const int width = edge.kind == EdgeKind::Walk ? 1 : 2;
            const COLORREF color = EdgeColor(edge.kind);
            std::vector<POINT> points = EdgePolyline(edge, g_app.nodes);
            DrawPolyline(hdc, points, color, width);
            if (edge.kind != EdgeKind::Walk)
            {
                DrawArrowHead(hdc, points, color);
            }
        }
    }

    const PathResult& path = g_app.reach[EnemyIndex(g_app.selectedEnemy)];
    if (path.found)
    {
        for (int edgeIndex : path.edges)
        {
            std::vector<POINT> points = EdgePolyline(g_app.edges[edgeIndex], g_app.nodes);
            DrawPolyline(hdc, points, RGB(247, 209, 72), 5);
        }
    }

    for (const Node& node : g_app.nodes)
    {
        COLORREF fill = RGB(248, 248, 248);
        COLORREF outline = RGB(80, 88, 98);
        int radius = 5;
        if (node.role == NodeRole::Ladder)
        {
            outline = EdgeColor(EdgeKind::Climb);
        }
        else if (node.role == NodeRole::Tunnel)
        {
            outline = EdgeColor(EdgeKind::Dig);
        }
        else if (node.role == NodeRole::Goal)
        {
            fill = RGB(247, 209, 72);
            outline = RGB(84, 78, 36);
            radius = 8;
        }
        DrawCircle(hdc, ToScreen(node.p), radius, fill, outline);
    }
}

void DrawAgentAndMarkers(HDC hdc)
{
    POINT start = ToScreen(g_app.nodes[g_app.startNode].p);
    DrawCircle(hdc, start, 8, RGB(255, 255, 255), RGB(33, 33, 33));
    DrawTextAAt(hdc, start.x - 12, start.y - 28, "Start", RGB(40, 43, 48));

    POINT goal = ToScreen(g_app.goalPoint);
    DrawTextAAt(hdc, goal.x - 12, goal.y - 28, "Goal", RGB(40, 43, 48));

    const EnemyProfile& profile = CurrentProfile();
    POINT agent = ToScreen(g_app.agentPos);
    DrawCircle(hdc, agent, 11, profile.color, RGB(30, 34, 40));
    DrawTextAAt(hdc, agent.x - 4, agent.y - 7, profile.shortName, RGB(255, 255, 255));
}

void DrawLegend(HDC hdc)
{
    const int x = kLevelLeft;
    const int y = kLevelTop + static_cast<int>(kLevelH) + 14;
    const std::array<EdgeKind, 5> kinds { EdgeKind::Walk, EdgeKind::Jump, EdgeKind::Drop, EdgeKind::Climb, EdgeKind::Dig };
    int cursor = x;
    for (EdgeKind kind : kinds)
    {
        RECT swatch = MakeRect(cursor, y + 4, cursor + 26, y + 10);
        FillRectColor(hdc, swatch, EdgeColor(kind));
        DrawTextAAt(hdc, cursor + 34, y, EdgeKindName(kind), RGB(49, 53, 59));
        cursor += 105;
    }
    DrawTextAAt(hdc, cursor + 10, y, "yellow path = selected enemy A*", RGB(49, 53, 59));
}

void DrawButtons(HDC hdc)
{
    HFONT font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));
    HGDIOBJ oldFont = SelectObject(hdc, font);

    DrawTextAAt(hdc, 24, 16, "Movement Graph Pathfinding Demo", RGB(30, 34, 40));
    DrawTextAAt(hdc, 24, 84, "Click inside the level to move the goal; buttons change the active enemy and edge diagnostics.", RGB(64, 69, 77));

    int x = 24;
    for (const EnemyProfile& profile : Profiles())
    {
        RECT r = MakeRect(x, 46, x + 96, 74);
        const bool selected = profile.kind == g_app.selectedEnemy;
        FillRectColor(hdc, r, selected ? RGB(39, 45, 54) : RGB(236, 239, 243));
        StrokeRectColor(hdc, r, selected ? RGB(39, 45, 54) : RGB(180, 188, 198));
        DrawCenteredText(hdc, r, profile.name, selected ? RGB(255, 255, 255) : RGB(38, 43, 49));
        x += 106;
    }

    RECT toggle = MakeRect(x + 10, 46, x + 156, 74);
    FillRectColor(hdc, toggle, g_app.showInvalidEdges ? RGB(236, 239, 243) : RGB(246, 247, 249));
    StrokeRectColor(hdc, toggle, RGB(180, 188, 198));
    DrawCenteredText(hdc, toggle, g_app.showInvalidEdges ? "Invalid edges: on" : "Invalid edges: off", RGB(38, 43, 49));

    int statusX = x + 180;
    for (const EnemyProfile& profile : Profiles())
    {
        const PathResult& path = g_app.reach[EnemyIndex(profile.kind)];
        RECT dot = MakeRect(statusX, 52, statusX + 12, 64);
        FillRectColor(hdc, dot, path.found ? RGB(62, 150, 82) : RGB(184, 72, 72));
        StrokeRectColor(hdc, dot, RGB(60, 66, 74));
        std::ostringstream label;
        label << profile.name << ": " << (path.found ? "reachable" : "blocked");
        DrawTextAAt(hdc, statusX + 18, 49, label.str(), RGB(38, 43, 49));
        statusX += 150;
    }

    SelectObject(hdc, oldFont);
}

void DrawComparisonPanel(HDC hdc)
{
    // Render a scaled copy of the level with the naive grid path and metrics
    // beside the movement-aware graph view.
    RECT panel = MakeRect(kPanelLeft, kPanelTop, kPanelLeft + 330, kPanelTop + 580);
    FillRectColor(hdc, panel, RGB(242, 244, 247));
    StrokeRectColor(hdc, panel, RGB(190, 198, 208));
    DrawTextAAt(hdc, kPanelLeft, 84, "Naive grid comparison", RGB(42, 46, 52));

    constexpr float scale = 0.38f;
    constexpr int originX = kPanelLeft + 8;
    constexpr int originY = kPanelTop + 24;

    auto map = [&](Vec2 p) {
        return POINT {
            static_cast<LONG>(originX + p.x * scale),
            static_cast<LONG>(originY + p.y * scale),
        };
    };

    HPEN gridPen = CreatePen(PS_SOLID, 1, RGB(226, 230, 236));
    HGDIOBJ oldPen = SelectObject(hdc, gridPen);
    for (int x = 0; x <= static_cast<int>(kLevelW); x += 40)
    {
        MoveToEx(hdc, originX + static_cast<int>(x * scale), originY, nullptr);
        LineTo(hdc, originX + static_cast<int>(x * scale), originY + static_cast<int>(kLevelH * scale));
    }
    for (int y = 0; y <= static_cast<int>(kLevelH); y += 40)
    {
        MoveToEx(hdc, originX, originY + static_cast<int>(y * scale), nullptr);
        LineTo(hdc, originX + static_cast<int>(kLevelW * scale), originY + static_cast<int>(y * scale));
    }
    SelectObject(hdc, oldPen);
    DeleteObject(gridPen);

    for (const Platform& platform : g_app.platforms)
    {
        RECT r = MakeRect(
            originX + static_cast<int>(platform.rect.x * scale),
            originY + static_cast<int>(platform.rect.y * scale),
            originX + static_cast<int>((platform.rect.x + platform.rect.w) * scale),
            originY + static_cast<int>((platform.rect.y + platform.rect.h) * scale));
        FillRectColor(hdc, r, RGB(105, 113, 125));
    }

    if (g_app.gridPath.found && g_app.gridPath.points.size() > 1)
    {
        std::vector<POINT> points;
        for (Vec2 p : g_app.gridPath.points)
        {
            points.push_back(map(p));
        }
        DrawPolyline(hdc, points, RGB(198, 74, 150), 3);
    }

    DrawCircle(hdc, map(g_app.nodes[g_app.startNode].p), 5, RGB(255, 255, 255), RGB(30, 34, 40));
    DrawCircle(hdc, map(g_app.goalPoint), 6, RGB(247, 209, 72), RGB(84, 78, 36));

    const int infoY = originY + static_cast<int>(kLevelH * scale) + 22;
    std::ostringstream selected;
    const PathResult& graphPath = g_app.reach[EnemyIndex(g_app.selectedEnemy)];
    selected << CurrentProfile().name << " graph A*: ";
    if (graphPath.found)
    {
        selected << graphPath.edges.size() << " links, " << graphPath.expanded << " nodes";
    }
    else
    {
        selected << "no valid path";
    }
    DrawTextAAt(hdc, kPanelLeft + 12, infoY, selected.str(), RGB(42, 46, 52));

    std::ostringstream grid;
    grid << "Grid A*: ";
    if (g_app.gridPath.found)
    {
        grid << g_app.gridPath.points.size() << " cells, " << static_cast<int>(g_app.gridPath.unsupportedRatio * 100.0f + 0.5f) << "% unsupported";
    }
    else
    {
        grid << "failed";
    }
    DrawTextAAt(hdc, kPanelLeft + 12, infoY + 24, grid.str(), RGB(42, 46, 52));

    const char* verdict = "Grid moves through empty air because it has no platformer movement model.";
    if (g_app.gridPath.found && g_app.gridPath.unsupportedRatio < 0.1f)
    {
        verdict = "Grid path happens to stay near surfaces here, but it still ignores enemy abilities.";
    }
    DrawTextAAt(hdc, kPanelLeft + 12, infoY + 54, verdict, RGB(93, 67, 108));
}

void Render(HDC hdc, const RECT& client)
{
    FillRectColor(hdc, client, RGB(250, 251, 253));
    DrawButtons(hdc);
    DrawLevelGeometry(hdc);
    DrawGraph(hdc);
    DrawAgentAndMarkers(hdc);
    DrawLegend(hdc);
    DrawComparisonPanel(hdc);
}

void OnPaint(HWND hwnd)
{
    // Double buffering avoids flicker while the timer continuously animates the agent.
    PAINTSTRUCT ps;
    HDC hdc = BeginPaint(hwnd, &ps);
    RECT client;
    GetClientRect(hwnd, &client);

    HDC back = CreateCompatibleDC(hdc);
    HBITMAP bitmap = CreateCompatibleBitmap(hdc, client.right - client.left, client.bottom - client.top);
    HGDIOBJ oldBitmap = SelectObject(back, bitmap);
    Render(back, client);
    BitBlt(hdc, 0, 0, client.right - client.left, client.bottom - client.top, back, 0, 0, SRCCOPY);
    SelectObject(back, oldBitmap);
    DeleteObject(bitmap);
    DeleteDC(back);

    EndPaint(hwnd, &ps);
}

void OnTimer(HWND hwnd)
{
    const DWORD now = GetTickCount();
    const float dt = std::min(0.05f, (now - g_app.lastTick) / 1000.0f);
    g_app.lastTick = now;
    AdvanceAgent(dt);
    InvalidateRect(hwnd, nullptr, FALSE);
}

void SelectEnemy(EnemyKind kind)
{
    g_app.selectedEnemy = kind;
    ResetAgent();
    InvalidateRect(g_hwnd, nullptr, FALSE);
}

void OnLeftButtonDown(HWND hwnd, int x, int y)
{
    // Hit-test controls first; clicks in the level become a new snapped goal.
    int buttonX = 24;
    for (const EnemyProfile& profile : Profiles())
    {
        RECT r = MakeRect(buttonX, 46, buttonX + 96, 74);
        if (PtInRectInt(r, x, y))
        {
            SelectEnemy(profile.kind);
            return;
        }
        buttonX += 106;
    }

    RECT toggle = MakeRect(buttonX + 10, 46, buttonX + 156, 74);
    if (PtInRectInt(toggle, x, y))
    {
        g_app.showInvalidEdges = !g_app.showInvalidEdges;
        InvalidateRect(hwnd, nullptr, FALSE);
        return;
    }

    RECT levelRect = MakeRect(kLevelLeft, kLevelTop, kLevelLeft + static_cast<int>(kLevelW), kLevelTop + static_cast<int>(kLevelH));
    if (PtInRectInt(levelRect, x, y))
    {
        SetGoalFromClick(FromScreen(x, y));
        InvalidateRect(hwnd, nullptr, FALSE);
    }
}

LRESULT CALLBACK WindowProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    // Minimal Win32 message dispatch: timer drives simulation, paint draws the
    // current state, and mouse input changes enemy/diagnostics/goal state.
    switch (message)
    {
    case WM_CREATE:
        SetTimer(hwnd, kTimerId, 16, nullptr);
        return 0;
    case WM_TIMER:
        if (wParam == kTimerId)
        {
            OnTimer(hwnd);
        }
        return 0;
    case WM_LBUTTONDOWN:
        OnLeftButtonDown(hwnd, GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
        return 0;
    case WM_PAINT:
        OnPaint(hwnd);
        return 0;
    case WM_DESTROY:
        KillTimer(hwnd, kTimerId);
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProc(hwnd, message, wParam, lParam);
}

void InitializeApp()
{
    // Build the fixed graph, choose the initial top-ledge goal, and calculate
    // the first set of enemy-specific and naive-grid paths.
    BuildBaseLevel();
    g_app.goalPlatform = 4;
    g_app.goalPoint = Vec2 { 382.0f, 225.0f };
    RebuildScenarioGraph();
}
}

int WINAPI WinMain(HINSTANCE instance, HINSTANCE, LPSTR, int showCommand)
{
    // Standard Win32 setup and message loop; all demo state lives in g_app.
    InitializeApp();

    WNDCLASSEXA wc {};
    wc.cbSize = sizeof(WNDCLASSEXA);
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = instance;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    wc.lpszClassName = "PlatformerPathGraphDemoClass";
    wc.style = CS_HREDRAW | CS_VREDRAW;

    RegisterClassExA(&wc);

    g_hwnd = CreateWindowExA(
        0,
        wc.lpszClassName,
        "Platformer Movement Graph Pathfinding Demo",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        1268,
        780,
        nullptr,
        nullptr,
        instance,
        nullptr);

    if (!g_hwnd)
    {
        return 1;
    }

    ShowWindow(g_hwnd, showCommand);
    UpdateWindow(g_hwnd);

    MSG msg {};
    while (GetMessage(&msg, nullptr, 0, 0))
    {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }

    return static_cast<int>(msg.wParam);
}

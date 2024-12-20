using System;
using System.Linq;
using System.IO;
using System.Text;
using System.Collections;
using System.Numerics;
using System.Collections.Generic;

class Player
{
    static void Main(string[] args)
    {
        string[] inputs;
        inputs = Console.ReadLine().Split(' ');

        int width = int.Parse(inputs[0]);
        int height = int.Parse(inputs[1]);
        Map map = new Map(width, height);
        ProteinReserve playerProteins = new ProteinReserve();
        ProteinReserve opponentProteins = new ProteinReserve();
        HashSet<int> usedOrganismIds = new HashSet<int>();

        // Game Loop
        while (true)
        {
            map.Clear();
            usedOrganismIds.Clear();

            int entityCount = int.Parse(Console.ReadLine());
            for (int i = 0; i < entityCount; i++)
            {
                inputs = Console.ReadLine().Split(' ');
                int x = int.Parse(inputs[0]);
                int y = int.Parse(inputs[1]);
                string type = inputs[2];
                int owner = int.Parse(inputs[3]);
                int organId = int.Parse(inputs[4]);
                string organDir = inputs[5]; // N,E,S,W or X if not an organ
                int organParentId = int.Parse(inputs[6]);
                int organRootId = int.Parse(inputs[7]);

                if (Enum.TryParse(type, true, out Protein.ProteinType pt))
                {
                    map.AddProtein(new Protein
                    {
                        type = pt,
                        position = new Vector2(x, y),
                        harvesters = new List<Organ>(),
                    });
                }

                if (Enum.TryParse(type, true, out Organ.OrganType ot))
                {
                    map.AddOrgan(new Organ
                    {
                        id = organId,
                        organismId = organRootId,
                        type = ot,
                        owner = (Ownership)owner,
                        position = new Vector2(x, y),
                        direction = organDir,
                        usedOrganismIds = usedOrganismIds,
                        playerProteins = playerProteins,
                    });
                }
            }

            map.Refresh();

            // your protein stock
            inputs = Console.ReadLine().Split(' ');
            playerProteins.a = int.Parse(inputs[0]);
            playerProteins.b = int.Parse(inputs[1]);
            playerProteins.c = int.Parse(inputs[2]);
            playerProteins.d = int.Parse(inputs[3]);

            // opponent's protein stock
            inputs = Console.ReadLine().Split(' ');
            opponentProteins.a = int.Parse(inputs[0]);
            opponentProteins.b = int.Parse(inputs[1]);
            opponentProteins.c = int.Parse(inputs[2]);
            opponentProteins.d = int.Parse(inputs[3]);

            int requiredActionsCount = int.Parse(Console.ReadLine()); // your number of organisms, output an action for each one in any order
            for (int i = 0; i < requiredActionsCount; i++)
            {
                // PRINT STATE
                map.PrintState();

                (Organ organ, Protein protein, List<Vector2> path)? closestProteinToAnyOrgan = null;
                (Organ organ, Organ oppOrgan, List<Vector2> path)? closestOpponentOrganToAnyOrgan = null;

                var availablePlayerOrgans = map.PlayerOrgans
                    .Where(x => !usedOrganismIds.Contains(x.organismId))
                    .ToArray();

                foreach (Organ org in availablePlayerOrgans)
                {
                    var pathsToProteins = map.Proteins
                        .Select(x => (protein: x, path: map.CalculatePathHeuristic(org.position, x.position)))
                        .Where(x => x.path != null)
                        .OrderBy(x => x.path.Count)
                        .ToArray();

                    var pathsToOpponent = map.OpponentOrgans
                        .Select(x => (organ: x, path: map.CalculatePath(org.position, x.position)))
                        .Where(x => x.path != null && x.path.Count > 1)
                        .OrderBy(x => x.path.Count)
                        .ToArray();

                    var pathToOpponentRoot = map.OpponentRoot.HasValue
                        ? map.CalculatePath(org.position, map.OpponentRoot.Value.position, true)
                        : null;

                    if (pathsToProteins.Length > 0)
                    {
                        (Protein protein, List<Vector2> path) closestProteinToCurrentOrgan = pathsToProteins.FirstOrDefault();
                        bool isClosest = !closestProteinToAnyOrgan.HasValue || closestProteinToCurrentOrgan.path.Count < closestProteinToAnyOrgan.Value.path.Count;
                        if (isClosest && !closestProteinToCurrentOrgan.protein.isPlayerHarvested)
                        {
                            closestProteinToAnyOrgan = (org, closestProteinToCurrentOrgan.protein, closestProteinToCurrentOrgan.path);
                        }
                    }

                    if (pathsToOpponent.Length > 0)
                    {
                        bool hasValidPathToOpponentRoot = pathToOpponentRoot != null && pathToOpponentRoot.Count >= 2 && map.HasOpponentOrgan(pathToOpponentRoot[1]);
                        (Organ oppOrgan, List<Vector2> path) closestOpponentOrganToCurrentOrgan = hasValidPathToOpponentRoot
                            ? (map.OpponentRoot.Value, pathToOpponentRoot)
                            : pathsToOpponent.FirstOrDefault();
                        
                        bool isClosest = !closestOpponentOrganToAnyOrgan.HasValue || closestOpponentOrganToCurrentOrgan.path.Count < closestOpponentOrganToAnyOrgan.Value.path.Count;
                        if (isClosest)
                        {
                            closestOpponentOrganToAnyOrgan = (org, closestOpponentOrganToCurrentOrgan.oppOrgan, closestOpponentOrganToCurrentOrgan.path);
                        }
                    }
                }

                if (closestProteinToAnyOrgan.HasValue)
                {
                    Organ closestOrgan = closestProteinToAnyOrgan.Value.organ;
                    Protein closestProtein = closestProteinToAnyOrgan.Value.protein;
                    List<Vector2> closestPath = closestProteinToAnyOrgan.Value.path;
                    Vector2 next = closestPath.FirstOrDefault();
                    Console.Error.WriteLine($"Closest protein is {closestProtein.type} at {closestProtein.position} | path: [{string.Join(", ", closestPath.Select(x => x).ToArray())}]");
                    
                    if (closestPath.Count > 2
                        && Map.IsPathStraight(closestPath) 
                        && closestOrgan.type == Organ.OrganType.Sporer 
                        && playerProteins.a > 0 
                        && playerProteins.b > 0 
                        && playerProteins.c > 0 
                        && playerProteins.d > 0)
                    {                    
                        closestOrgan.Spore(closestPath.LastOrDefault());                  
                    }
                    else if (closestPath.Count > 2 
                        && Map.IsPathStraight(closestPath)
                        && playerProteins.a > 0 
                        && playerProteins.b > 1 
                        && playerProteins.c > 0 
                        && playerProteins.d > 1)
                    {
                        Vector2 dir = closestPath[1] - next;
                        closestOrgan.GrowSporer(next, Map.GetDirectionKey(dir));
                    }
                    else if (closestPath.Count == 2 
                        && playerProteins.c > 0 
                        && playerProteins.d > 0)
                    {
                        Vector2 dir = closestPath[1] - next;
                        closestOrgan.GrowHarvester(next, Map.GetDirectionKey(dir));
                    }
                    else if (playerProteins.a > 0)
                    {
                        closestOrgan.GrowBasic(next);
                    }
                    else
                    {
                        Console.Error.WriteLine($"Organism is waiting: insufficient proteins for growth");
                        Console.WriteLine("WAIT");
                    }
                }
                else if (closestOpponentOrganToAnyOrgan.HasValue)
                {
                    Organ closestOrgan = closestOpponentOrganToAnyOrgan.Value.organ;
                    Organ closestOpponentOrgan = closestOpponentOrganToAnyOrgan.Value.oppOrgan;
                    List<Vector2> closestPath = closestOpponentOrganToAnyOrgan.Value.path;
                    Vector2 next = closestPath.FirstOrDefault();

                    Console.Error.WriteLine($"Closest opponent organ is {closestOpponentOrgan.type} at {closestOpponentOrgan.position} | path: [{string.Join(", ", closestPath.Select(x => x).ToArray())}]");
                    
                    bool willHaveAdjacentOpponentOrgan = closestPath.Count >= 2 && map.HasOpponentOrgan(closestPath[1]);
                    if (willHaveAdjacentOpponentOrgan 
                        && playerProteins.b > 0 
                        && playerProteins.c > 0)
                    {
                        Vector2 dir = closestPath[1] - next;        
                        closestOrgan.GrowTentacle(next, Map.GetDirectionKey(dir));
                    }
                    else if (playerProteins.a > 0)
                    {
                        closestOrgan.GrowBasic(next);
                    }
                    else
                    {
                        closestOrgan.GrowAny(next);
                        //Console.Error.WriteLine($"Organism is waiting: insufficient proteins for growth");
                        //Console.WriteLine("WAIT");
                    }
                }
                else
                {
                    var availableSpots = availablePlayerOrgans
                        .Select(x => (organ: x, hasSpot: map.HasAdjacentFreeSpot(x.position, out Vector2[] spots), spot: spots.FirstOrDefault()))
                        .Where(x => x.hasSpot)
                        .ToArray();

                    if (availableSpots.Length > 0)
                    {
                        var first = availableSpots.FirstOrDefault();
                        first.organ.GrowAny(first.spot);
                    }
                    else
                    {
                        Console.Error.WriteLine($"Organism is waiting: no proteins or path to opponent organs available");
                        Console.WriteLine("WAIT");
                    }
                }
            }
        }
    }
}

public enum Ownership
{
    Neutral = -1,
    Opponent = 0,
    Player = 1,
}

public class Map
{
    private int width; // columns
    private int height; // rows
    private List<Protein> proteins;
    private List<Organ> organs;

    public List<Protein> Proteins => proteins;

    public List<Organ> PlayerOrgans => organs
        .Where(x => x.owner == Ownership.Player)
        .ToList();

    public List<Organ> OpponentOrgans => organs
        .Where(x => x.owner == Ownership.Opponent)
        .ToList();

    public Organ? PlayerRoot => PlayerOrgans.FirstOrDefault(x => x.type == Organ.OrganType.Root);
    public Organ? OpponentRoot => OpponentOrgans.FirstOrDefault(x => x.type == Organ.OrganType.Root);

    private static Dictionary<string, Vector2> movementDirections = new Dictionary<string, Vector2>
    {
        { "N", new Vector2(0, -1) },  // Up
        { "S", new Vector2(0, 1) }, // Down
        { "W", new Vector2(-1, 0) }, // Left
        { "E", new Vector2(1, 0) },  // Right
    };

    public Map(int width, int height)
    {
        this.width = width;
        this.height = height;
        this.proteins = new List<Protein>();
        this.organs = new List<Organ>();
    }

    public void AddProtein(Protein protein)
    {
        proteins.Add(protein);
    }

    public void AddOrgan(Organ organ)
    {
        organs.Add(organ);
    }

    public bool HasOpponentOrgan(Vector2 pos)
    {
        return OpponentOrgans.Any(x => x.position == pos);
    }

    public bool HasAdjacentFreeSpot(Vector2 pos, out Vector2[] spots)
    {
        Vector2[] adjacent = movementDirections.Values
            .Select(x => pos + x)
            .ToArray();

        HashSet<Vector2> obstaclePositions = organs
            .Select(x => x.position)
            .Concat(proteins.Select(x => x.position))
            .ToHashSet();

        spots = adjacent
            .Where(x => !obstaclePositions.Contains(x))
            .ToArray();

        return spots.Length > 0;
    }

    public void Refresh()
    {
        var harvesters = organs
            .Where(x => x.type == Organ.OrganType.Harvester)
            .Select(x => (organ: x, facing: x.position + movementDirections[x.direction]))
            .ToArray();

        foreach (var harvester in harvesters)
        {
            Protein[] harvestedProteins = proteins
                .Where(x => x.position == harvester.facing)
                .ToArray();

            if (harvestedProteins.Length > 0)
            {
                Protein prot = harvestedProteins.FirstOrDefault();
                int index = proteins.IndexOf(prot);
                proteins[index].harvesters.Add(harvester.organ);
            }
        }
    }

    public void Clear()
    {
        proteins.Clear();
        organs.Clear();
    }

    public void PrintState()
    {
        Console.Error.WriteLine($"Proteins: [{string.Join(", ", proteins.Select(x => x.ToString()).ToArray())}]");
    }

    public static string GetDirectionKey(Vector2 dir)
    {
        return movementDirections.FirstOrDefault(x => x.Value == dir).Key;
    }

    public static bool IsPathStraight(List<Vector2> path)
    {
        // A path with fewer than 2 points is trivially straight
        if (path.Count < 2) 
            return true;

        // Calculate the direction of the first segment
        var firstDirection = Vector2.Normalize(path[1] - path[0]);

        // Check if all subsequent segments have the same direction
        for (int i = 1; i < path.Count - 1; i++)
        {
            var currentDirection = Vector2.Normalize(path[i + 1] - path[i]);

            // If the direction of any segment differs, the path is not straight
            if (currentDirection != firstDirection)
                return false;
        }

        return true; // All segments have the same direction
    }

    public List<Vector2> CalculatePath(Vector2 origin, Vector2 destination, bool ignoreOpponentOrgans = false)
    {
        var organPositions = organs
            .Where(x => !ignoreOpponentOrgans || x.owner != Ownership.Opponent)
            .Select(x => x.position)
            .Except(new Vector2[] { origin, destination });

        var proteinPositions = proteins
            .Select(x => x.position)
            .Except(new Vector2[] { origin, destination });

        var obstacles = organPositions
            .Concat(proteinPositions)
            .ToHashSet();     

        // BFS queue and visited set
        var queue = new Queue<Vector2>();
        var visited = new HashSet<Vector2>();
        var parent = new Dictionary<Vector2, Vector2>(); // To reconstruct the path

        queue.Enqueue(origin);
        visited.Add(origin);

        while (queue.Count > 0)
        {
            var current = queue.Dequeue();

            // If we've reached the target, reconstruct and return the path
            if (current == destination)
                return ReconstructPath(parent, origin, destination);

            // Explore neighbors
            foreach (var dir in movementDirections.Values)
            {
                var neighbor = current + dir;

                // Check if the neighbor is within bounds and not an obstacle
                if (neighbor.X >= 0 && neighbor.X < width &&
                    neighbor.Y >= 0 && neighbor.Y < height &&
                    !obstacles.Contains(neighbor) &&
                    !visited.Contains(neighbor))
                {
                    queue.Enqueue(neighbor);
                    visited.Add(neighbor);
                    parent[neighbor] = current; // Track the parent of the neighbor
                }
            }
        }

        // If we exhaust the queue without finding the target, it's unreachable
        return null;
    }

    public List<Vector2> CalculatePathHeuristic(Vector2 origin, Vector2 destination, bool ignoreOpponentOrgans = false)
    {
        // Get the list of obstacles
        var organPositions = organs
            .Where(x => !ignoreOpponentOrgans || x.owner == Ownership.Player)
            .Select(x => x.position)
            .Except(new Vector2[] { origin, destination });

        var proteinPositions = proteins
            .Select(x => x.position)
            .Except(new Vector2[] { origin, destination });

        var obstacles = organPositions
            .Concat(proteinPositions)
            .ToHashSet();

        // BFS queue and visited set
        var queue = new PriorityQueue<Vector2, float>();
        var visited = new HashSet<Vector2>();
        var parent = new Dictionary<Vector2, Vector2>(); // To reconstruct the path
        var gScore = new Dictionary<Vector2, float>(); // Distance from start
        var fScore = new Dictionary<Vector2, float>(); // Priority score

        // Directions (cardinal movements)
        var movementDirections = new List<Vector2>
        {
            new Vector2(0, 1),  // Up
            new Vector2(0, -1), // Down
            new Vector2(1, 0),  // Right
            new Vector2(-1, 0)  // Left
        };

        gScore[origin] = 0;
        fScore[origin] = Heuristic(origin, destination);
        queue.Enqueue(origin, fScore[origin]);
        visited.Add(origin);

        while (queue.Count > 0)
        {
            var current = queue.Dequeue();

            // If we've reached the target, reconstruct and return the path
            if (current == destination)
            {
                return ReconstructPath(parent, origin, destination);
            }

            foreach (var dir in movementDirections)
            {
                var neighbor = current + dir;

                // Check if the neighbor is within bounds and not an obstacle
                if (neighbor.X >= 0 && neighbor.X < width &&
                    neighbor.Y >= 0 && neighbor.Y < height &&
                    !obstacles.Contains(neighbor) &&
                    !visited.Contains(neighbor))
                {
                    var tentativeGScore = gScore[current] + 1;

                    if (!gScore.ContainsKey(neighbor) || tentativeGScore < gScore[neighbor])
                    {
                        parent[neighbor] = current;
                        gScore[neighbor] = tentativeGScore;

                        // Penalize paths with straight segments until the remainder is straight
                        var curvinessPenalty = CurvinessHeuristic(parent, current, neighbor, destination);
                        fScore[neighbor] = gScore[neighbor] + Heuristic(neighbor, destination) + curvinessPenalty;

                        if (!visited.Contains(neighbor))
                        {
                            queue.Enqueue(neighbor, fScore[neighbor]);
                            visited.Add(neighbor);
                        }
                    }
                }
            }
        }

        // If we exhaust the queue without finding the target, it's unreachable
        return null;
    }

    private float DirectionAlignmentHeuristic(Vector2 current, Vector2 neighbor, Vector2 target)
    {
        // Compute the straight-line direction to the target
        var targetDirection = Vector2.Normalize(target - current);
        var neighborDirection = Vector2.Normalize(neighbor - current);

        // Compute alignment score (dot product measures alignment)
        float alignment = Vector2.Dot(targetDirection, neighborDirection);

        // Higher alignment is better, so invert it for heuristic purposes
        return 1 - alignment; // Range: 0 (perfect alignment) to 2 (opposite direction)
    }

    private float CurvinessHeuristic(Dictionary<Vector2, Vector2> parent, Vector2 current, Vector2 neighbor, Vector2 destination)
    {
        // If this is the first step, no penalty
        if (!parent.ContainsKey(current))
            return 0;

        var previous = parent[current];
        var currentDirection = Vector2.Normalize(current - previous);
        var nextDirection = Vector2.Normalize(neighbor - current);
        var straightToDestination = Vector2.Normalize(destination - neighbor);

        // Check if the path continues in the same direction
        bool isStraight = currentDirection == nextDirection;

        // Penalize turns unless this step results in the remainder being a straight line
        if (!isStraight)
        {
            // Check if the next segment is straight to the destination
            bool remainderStraight = nextDirection == straightToDestination;

            return remainderStraight ? 0 : 1; // Penalize curvy paths more heavily
        }

        return 0; // No penalty for straight paths
    }

    private float Heuristic(Vector2 a, Vector2 b)
    {
        // Manhattan distance heuristic for grids
        return Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y);
    }

    private List<Vector2> ReconstructPath(Dictionary<Vector2, Vector2> parent, Vector2 start, Vector2 target)
    {
        var path = new List<Vector2>();
        var current = target;

        // Track the previous position to calculate the direction
        Vector2? previous = null;

        // Backtrack from the target to the start
        while (current != start)
        {
            path.Add(current);
            previous = current;
            current = parent[current];
        }

        path.Reverse(); // Reverse the path to go from start to target
        return path;
    }
}

public interface IEntity
{
    Vector2 Position { get; }
}

public struct Protein : IEntity
{
    public ProteinType type;
    public Vector2 position;
    public List<Organ> harvesters;

    Vector2 IEntity.Position => position;
    public bool isPlayerHarvested => harvesters.Any(x => x.owner == Ownership.Player);
    public bool isOpponentHarvested => harvesters.Any(x => x.owner == Ownership.Opponent);

    public enum ProteinType
    {
        A,
        B,
        C,
        D
    }

    public override string ToString()
    {
        return $"{{\"type\": {type}, \"pos\": {position}, \"harvesters\": [{string.Join(", ", harvesters.Select(x => $"{x.position}").ToArray())}]}}";
    }
}

public struct Organ : IEntity
{
    public int id;
    public int organismId;
    public OrganType type;
    public Ownership owner;
    public Vector2 position;
    public string direction;

    public HashSet<int> usedOrganismIds;
    public ProteinReserve playerProteins;

    Vector2 IEntity.Position => position;

    public enum OrganType
    {
        Wall,
        Root,
        Basic,
        Tentacle,
        Harvester,
        Sporer
    }

    public void GrowBasic(Vector2 destination)
    {
        Grow(OrganType.Basic, destination, "N");
        playerProteins.a--;
    }

    public void GrowHarvester(Vector2 destination, string direction)
    {
        Grow(OrganType.Harvester, destination, direction);
        playerProteins.c--;
        playerProteins.d--;
    }

    public void GrowTentacle(Vector2 destination, string direction)
    {
        Grow(OrganType.Tentacle, destination, direction);
        playerProteins.b--;
        playerProteins.c--;
    }

    public void GrowSporer(Vector2 destination, string direction)
    {
        Grow(OrganType.Sporer, destination, direction);
        playerProteins.b--;
        playerProteins.d--;
    }

    public void GrowAny(Vector2 destination)
    {
        if (playerProteins.a > 0)
        {
            GrowBasic(destination);
        }
        else if (playerProteins.c > 0 && playerProteins.d > 0)
        {
            GrowHarvester(destination, "N");
        }
        else if (playerProteins.b > 0 && playerProteins.c > 0)
        {
            GrowTentacle(destination, "N");
        }
        else if (playerProteins.b > 0 && playerProteins.d > 0)
        {
            GrowSporer(destination, "N");
        }
        else
        {
            Wait("failed to grow any: not enough proteins");
        }
    }

    private void Grow(OrganType growthType, Vector2 destination, string direction)
    {
        Console.Error.WriteLine($"Organ {id} (organism {organismId}) is growing a {growthType} from {position} to {destination} facing {direction}");
        Console.WriteLine($"GROW {id} {destination.X} {destination.Y} {growthType.ToString().ToUpper()} {direction}");
        usedOrganismIds.Add(organismId);
    }

    public void Spore(Vector2 destination)
    {
        if (type != OrganType.Sporer)
        {
            Wait("failed to spore: not a sporer");
            return;
        }

        Console.Error.WriteLine($"Organ {id} (organism {organismId}) is sporing a root from {position} to {destination}");
        Console.WriteLine($"SPORE {id} {destination.X} {destination.Y}");
        usedOrganismIds.Add(organismId);
    }

    public void Wait(string reason)
    {
        Console.Error.WriteLine($"Organ {id} (organism {organismId}) is waiting: {reason}");
        Console.WriteLine("WAIT");
    }
}

public class ProteinReserve
{
    public int a;
    public int b;
    public int c;
    public int d;
}

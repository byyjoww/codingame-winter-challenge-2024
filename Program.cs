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

        while (true)
        {
            // New round, reset all data
            map.Clear();            
            playerProteins.Clear();
            opponentProteins.Clear();

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
                    Organ organ = new Organ
                    {
                        id = organId,
                        organism = null, // is set by organism.AddOrgan()
                        type = ot,
                        position = new Vector2(x, y),
                        direction = organDir,
                    };

                    if (!map.Organisms.Any(x => x.id == organRootId))
                    {         
                        Ownership organismOwner = (Ownership)owner;               
                        map.AddOrganism(new Organism
                        {
                            id = organRootId,
                            root = default,
                            organs = new List<Organ>(),
                            owner = organismOwner,
                            isUsed = false,
                            map = map,
                            proteins = organismOwner == Ownership.Player
                                ? playerProteins
                                : opponentProteins,
                        });
                    }

                    map.Organisms
                        .First(x => x.id == organRootId)
                        .AddOrgan(organ);
                }
            }

            // Update player proteins
            inputs = Console.ReadLine().Split(' ');
            playerProteins.a = int.Parse(inputs[0]);
            playerProteins.b = int.Parse(inputs[1]);
            playerProteins.c = int.Parse(inputs[2]);
            playerProteins.d = int.Parse(inputs[3]);

            // Update opponent proteins
            inputs = Console.ReadLine().Split(' ');
            opponentProteins.a = int.Parse(inputs[0]);
            opponentProteins.b = int.Parse(inputs[1]);
            opponentProteins.c = int.Parse(inputs[2]);
            opponentProteins.d = int.Parse(inputs[3]);

            Console.Error.WriteLine("Starting turn");

            // Update map state once all entities have been created
            map.Refresh();
            map.PrintState();            

            // Process player actions
            int requiredActionsCount = int.Parse(Console.ReadLine());
            for (int i = 0; i < requiredActionsCount; i++)
            {
                // The organism that will perform this action
                // TODO: sort priority for organisms that need to go first
                Organism organism = map.PlayerOrganisms[i];
                Console.Error.WriteLine($"Organism {organism.id}'s turn");

                organism.Act();                
            }

            Console.Error.WriteLine("Finished turn");
        }
    }
}

public class Organism
{
    public int id;
    public Organ root;
    public List<Organ> organs;
    public Ownership owner;
    public bool isUsed;

    public Map map;
    public ProteinReserve proteins;    

    public void AddOrgan(Organ organ)
    {
        organ.organism = this;
        organs.Add(organ);

        if (organ.type == Organ.OrganType.Root)
        {
            this.root = organ;
        }
    }

    public void Use()
    {
        isUsed = true;
    }

    public void Act()
    {
        var availableUnharvestedProteins = map.Proteins
            .Where(x => !x.isPlayerHarvested)
            .ToArray();

        if (availableUnharvestedProteins.Length > 0 && TryFindClosestUnharvestedProtein(out var closest))
        {                  
            HarvestProtein( closest.Value.protein, closest.Value.organ, closest.Value.path);
        }
        else if (map.OpponentOrgans.Count > 0 && (TryFindClosestOpponentRoot(out var closestOrgan) || TryFindClosestOpponentOrgan(out closestOrgan)))
        {
            MoveToOrgan(closestOrgan.Value.organ, closestOrgan.Value.oppOrgan, closestOrgan.Value.path);
        }
        else 
        {
            Grow();
        }
    }

    private void HarvestProtein(Protein protein, Organ organ, List<Vector2> path)
    {
        Console.Error.WriteLine($"Harvesting protein {protein.type} at {protein.position} | path: [{string.Join(", ", path.Select(x => x).ToArray())}]");

        Vector2 next = path.FirstOrDefault();
        
        if (path.Count > 2 && Map.IsPathStraight(path) && organ.CanSpore())
        {
            organ.Spore(path[path.Count - 3]);
        }
        else if (path.Count > 4
            && Map.IsPathStraight(path)
            && proteins.a > 0
            && proteins.b > 1
            && proteins.c > 0
            && proteins.d > 1)
        {
            // must be able to grow a sporer and then spore next turn
            Console.Error.WriteLine("Growing sporer");

            Vector2 dir = path[1] - next;
            organ.GrowSporer(next, Map.GetDirectionKey(dir));
        }
        else if (path.Count == 2 && organ.CanGrow(Organ.OrganType.Harvester))
        {
            Console.Error.WriteLine("Growing harvester");

            Vector2 dir = path[1] - next;
            organ.GrowHarvester(next, Map.GetDirectionKey(dir));
        }
        else if (organ.CanGrow(Organ.OrganType.Basic))
        {
            Console.Error.WriteLine("Growing basic");

            organ.GrowBasic(next);
        }
        else
        {
            Grow();
        }
    }

    private void MoveToOrgan(Organ organ, Organ opponent, List<Vector2> path)
    {
        Console.Error.WriteLine($"Moving to opponent organ {opponent.type} at {opponent.position} | path: [{string.Join(", ", path.Select(x => x).ToArray())}]");

        Vector2 next = path.FirstOrDefault();
        
        bool willHaveAdjacentOpponentOrgan = path.Count >= 2 && map.HasOpponentOrgan(path[1]);
        if (willHaveAdjacentOpponentOrgan && organ.CanGrow(Organ.OrganType.Tentacle))
        {
            Console.Error.WriteLine("Growing tentacle");

            Vector2 dir = path[1] - next;        
            organ.GrowTentacle(next, Map.GetDirectionKey(dir));
        }
        else if (organ.CanGrow(Organ.OrganType.Basic))
        {
            Console.Error.WriteLine("Growing basic");

            organ.GrowBasic(next);
        }
        else
        {
            Grow();
        }
    }

    private void Grow()
    {
        // No available proteins to harvest or opponent organs to disrupt
        // Continue growing in random spots until game ends/proteins run out
        Console.Error.WriteLine("[Action] Grow");

        var availableRandomSpots = organs
            .Select(x => (organ: x, hasSpot: map.HasAdjacentFreeSpot(x.position, out Vector2[] spots), spot: spots.FirstOrDefault()))
            .Where(x => x.hasSpot)
            .ToArray();

        if (availableRandomSpots.Length > 0)
        {
            Console.Error.WriteLine("Growing any");

            var first = availableRandomSpots.FirstOrDefault();
            first.organ.GrowAny(first.spot);
        }
        else
        {
            Wait();
        }
    }

    public void Wait()
    {
        Console.Error.WriteLine($"Organism is waiting: no proteins or path to opponent organs available");
        Console.WriteLine("WAIT");
    }

    private bool TryFindClosestUnharvestedProtein(out (Organ organ, Protein protein, List<Vector2> path)? target)
    {
        target = null;
        foreach (Organ org in organs)
        {
            var pathsToProteins = map.Proteins
                .Select(x => (protein: x, path: map.CalculatePathHeuristic(org.position, x.position)))
                .Where(x => x.path != null)
                .Where(x => !x.protein.isPlayerHarvested)
                .OrderBy(x => x.path.Count)
                .ToArray();

            if (pathsToProteins.Length > 0)
            {
                (Protein protein, List<Vector2> path) closestProteinToCurrentOrgan = pathsToProteins.FirstOrDefault();
                if (!target.HasValue || closestProteinToCurrentOrgan.path.Count < target.Value.path.Count)
                {
                    target = (org, closestProteinToCurrentOrgan.protein, closestProteinToCurrentOrgan.path);
                }
            }
        }

        return target.HasValue;
    }

    private bool TryFindClosestOpponentRoot(out (Organ organ, Organ oppOrgan, List<Vector2> path)? targetOrgan)
    {
        targetOrgan = null;
        foreach (Organ org in organs)
        {
            var pathsToOpponentRoots = map.OpponentOrganisms
                .Select(x => x.root)
                .Select(x => (organ: x, path: map.CalculatePath(org.position, x.position)))
                .Where(x => x.path != null && x.path.Count > 1 && !map.HasOpponentOrgan(x.path[0]) && map.HasOpponentOrgan(x.path[1]))
                .OrderBy(x => x.path.Count)
                .ToArray();            

            if (pathsToOpponentRoots.Length > 0)
            {
                var first = pathsToOpponentRoots.FirstOrDefault();      
                if (!targetOrgan.HasValue || first.path.Count < targetOrgan.Value.path.Count)
                {
                    targetOrgan = (org, first.organ, first.path);
                }
            }            
        }

        return targetOrgan.HasValue;
    }

    private bool TryFindClosestOpponentOrgan(out (Organ organ, Organ oppOrgan, List<Vector2> path)? targetOrgan)
    {
        targetOrgan = null;
        foreach (Organ org in organs)
        {
            var pathsToOpponent = map.OpponentOrgans
                .Select(x => (organ: x, path: map.CalculatePath(org.position, x.position)))
                .Where(x => x.path != null && x.path.Count > 1)
                .OrderBy(x => x.path.Count)
                .ToArray();          

            if (pathsToOpponent.Length > 0)
            {
                var first = pathsToOpponent.FirstOrDefault();      
                if (!targetOrgan.HasValue || first.path.Count < targetOrgan.Value.path.Count)
                {
                    targetOrgan = (org, first.organ, first.path);
                }
            }            
        }

        return targetOrgan.HasValue;
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
    private List<Organism> organisms;

    public List<Protein> Proteins => proteins;
    public List<Organism> Organisms => organisms;

    public List<Organ> Organs => organisms
        .SelectMany(x => x.organs)
        .ToList();

    public List<Organism> PlayerOrganisms => organisms
        .Where(x => x.owner == Ownership.Player)
        .ToList();

    public List<Organism> OpponentOrganisms => organisms
        .Where(x => x.owner == Ownership.Opponent)
        .ToList();


    public List<Organ> PlayerOrgans => PlayerOrganisms
        .SelectMany(x => x.organs)
        .ToList();

    public List<Organ> OpponentOrgans => OpponentOrganisms
        .SelectMany(x => x.organs)
        .ToList();

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
        this.organisms = new List<Organism>();
    }

    public void AddProtein(Protein protein)
    {
        proteins.Add(protein);
    }

    public void AddOrganism(Organism organism)
    {
        organisms.Add(organism);
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

        HashSet<Vector2> obstaclePositions = Organs
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
        var harvesters = Organs
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
        organisms.Clear();
    }

    public void PrintState()
    {
        Console.Error.WriteLine($"STATE: {{\"proteins\": [{string.Join(", ", proteins.Select(x => x.ToString()).ToArray())}]}}");
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
        var organPositions = Organs
            .Where(x => !ignoreOpponentOrgans || x.organism.owner != Ownership.Opponent)
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
        var organPositions = Organs
            .Where(x => !ignoreOpponentOrgans || x.organism.owner == Ownership.Player)
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
    public bool isPlayerHarvested => harvesters.Any(x => x.organism.owner == Ownership.Player);
    public bool isOpponentHarvested => harvesters.Any(x => x.organism.owner == Ownership.Opponent);

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
    public Organism organism;
    public OrganType type;
    public Vector2 position;
    public string direction;

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

    public bool CanGrow(OrganType organType)
    {
        switch(organType)
        {
            case OrganType.Basic:
                return organism.proteins.a >= 1;
            case OrganType.Harvester:
                return organism.proteins.c >= 1 && organism.proteins.d >= 1;
            case OrganType.Tentacle:
                return organism.proteins.b >= 1 && organism.proteins.c >= 1;
            case OrganType.Sporer:
                return organism.proteins.b >= 1 && organism.proteins.d >= 1;
        }

        return false;
    }

    public bool CanSpore()
    {
        return type == OrganType.Sporer
            && organism.proteins.a > 0
            && organism.proteins.b > 0
            && organism.proteins.c > 0
            && organism.proteins.d > 0;
    }

    public void GrowBasic(Vector2 destination)
    {
        Grow(OrganType.Basic, destination, "N");
        organism.proteins.a--;
    }

    public void GrowHarvester(Vector2 destination, string direction)
    {
        Grow(OrganType.Harvester, destination, direction);
        organism.proteins.c--;
        organism.proteins.d--;
    }

    public void GrowTentacle(Vector2 destination, string direction)
    {
        Grow(OrganType.Tentacle, destination, direction);
        organism.proteins.b--;
        organism.proteins.c--;
    }

    public void GrowSporer(Vector2 destination, string direction)
    {
        Grow(OrganType.Sporer, destination, direction);
        organism.proteins.b--;
        organism.proteins.d--;
    }

    public void GrowAny(Vector2 destination)
    {
        if (CanGrow(OrganType.Basic))
        {
            GrowBasic(destination);
        }
        else if (CanGrow(OrganType.Harvester))
        {
            GrowHarvester(destination, "N");
        }
        else if (CanGrow(OrganType.Tentacle))
        {
            GrowTentacle(destination, "N");
        }
        else if (CanGrow(OrganType.Sporer))
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
        if (!CanGrow(growthType))
        {
            Wait("failed to grow");
            return;
        }

        Console.Error.WriteLine($"Organ {id} (organism {organism.id}) is growing a {growthType} from {position} to {destination} facing {direction}");
        Console.WriteLine($"GROW {id} {destination.X} {destination.Y} {growthType.ToString().ToUpper()} {direction}");
        organism.Use();
    }

    public void Spore(Vector2 destination)
    {
        if (!CanSpore())
        {
            Wait("failed to spore");
            return;
        }

        Console.Error.WriteLine($"Organ {id} (organism {organism.id}) is sporing a root from {position} to {destination}");
        Console.WriteLine($"SPORE {id} {destination.X} {destination.Y}");
        organism.Use();
    }

    public void Wait(string reason)
    {
        Console.Error.WriteLine($"Organ {id} (organism {organism.id}) is waiting: {reason}");
        Console.WriteLine("WAIT");
    }
}

public class ProteinReserve
{
    public int a;
    public int b;
    public int c;
    public int d;

    public void Clear()
    {
        a = 0;
        b = 0;
        c = 0;
        d = 0;
    }
}

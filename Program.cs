using System;
using System.Linq;
using System.IO;
using System.Text;
using System.Collections;
using System.Numerics;
using System.Collections.Generic;

class Player
{
    private static DateTime ts;    

    static void Main(string[] args)
    {        
        string[] inputs;
        inputs = Console.ReadLine().Split(' ');
        int width = int.Parse(inputs[0]);
        int height = int.Parse(inputs[1]);

        Map map = new Map(width, height);
        ProteinReserve playerProteins = new ProteinReserve();
        ProteinReserve opponentProteins = new ProteinReserve();            
        var sporeQueue = new Queue<Organism.BehaviourType>();

        while (true)
        {
            // New round
            ts = DateTime.UtcNow;
            var proteins = new List<Protein>();
            var organisms = map.Organisms;

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
                    proteins.Add(new Protein
                    {
                        id = $"protein_{pt.ToString().ToLower()}_{x}_{y}",
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

                    // New organism was created last turn
                    var organism = organisms.FirstOrDefault(x => x.id == organRootId);
                    if (organism == null)
                    {                        
                        Ownership organismOwner = (Ownership)owner;
                        var behaviour = organismOwner == Ownership.Player 
                            ? sporeQueue.TryDequeue(out var bh) 
                                ? bh
                                : Organism.BehaviourType.Harvest
                            : default;      
                        
                        organism = new Organism
                        {
                            id = organRootId,
                            root = default,
                            organs = new List<Organ>(),
                            owner = organismOwner,
                            isUsed = false,
                            map = map,
                            sporeQueue = sporeQueue,
                            proteins = organismOwner == Ownership.Player
                                ? playerProteins
                                : opponentProteins,
                            Behaviour = behaviour,
                        };

                        organisms.Add(organism);
                        Console.Error.WriteLine($"Created new organism {organRootId} with behaviour {behaviour}");
                    }

                    organism.AddOrgan(organ);
                }
            }

            // Update map state once all entities have been created
            map.SetProteins(proteins);
            map.SetOrganisms(organisms);
            map.Build();
            // map.PrintState(); 

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

            // Process player actions
            int requiredActionsCount = int.Parse(Console.ReadLine());
            for (int i = 0; i < requiredActionsCount; i++)
            {
                // The organism that will perform this action
                // TODO: sort priority for organisms that need to go first
                Organism organism = map.PlayerOrganisms[i];
                Console.Error.WriteLine($"Organism {organism.id}'s turn ({organism.Behaviour})");

                organism.Plan();
                organism.Act();
            }

            Console.Error.WriteLine("Finished turn");
        }
    }

    public static float GetTimeSinceStartOfTurn()
    {
        return (float)(DateTime.UtcNow - ts).TotalMilliseconds;
    }
}

public interface IBehaviour
{
    void Plan();
    void Act();
}

public abstract class BaseBehaviour : IBehaviour
{
    protected Queue<Action> plan = new Queue<Action>();
    protected Organism organism;
    protected Map map;
    protected ProteinReserve proteins;

    public BaseBehaviour(Organism organism, Map map, ProteinReserve proteins)
    {
        this.organism = organism;
        this.map = map;
        this.proteins = proteins;
    }

    public abstract void Plan();

    public virtual void Act()
    {
        var action = plan.Dequeue();
        action?.Invoke();
    }

    protected void MoveToOrgan(Organ organ, Organ opponent, List<Vector2> path)
    {
        Console.Error.WriteLine($"Moving to opponent organ {opponent.type} at {opponent.position} | path: [{string.Join(", ", path.Select(x => x).ToArray())}]");

        Vector2 next = path.FirstOrDefault();

        bool willHaveAdjacentOpponentOrgan = path.Count >= 2 && map.HasOpponentOrgan(path[1]);
        if (willHaveAdjacentOpponentOrgan && organism.CanGrow(Organ.OrganType.Tentacle))
        {
            Console.Error.WriteLine("Growing tentacle");

            Vector2 dir = path[1] - next;
            organism.GrowTentacle(organ, next, Map.GetDirectionKey(dir));
        }
        else if (organism.CanGrow(Organ.OrganType.Basic))
        {
            Console.Error.WriteLine("Growing basic");

            organism.GrowBasic(organ, next);
        }
        else
        {
            GrowInRandomPosition();
        }
    }

    protected void GrowInRandomPosition()
    {
        // No available proteins to harvest or opponent organs to disrupt
        // Continue growing in random spots until game ends/proteins run out
        Console.Error.WriteLine("[Action] Grow");

        var availableRandomSpots = organism.organs
            .Select(x => (organ: x, hasSpot: map.HasAdjacentFreeSpot(x.position, out Vector2[] spots), spot: spots.FirstOrDefault()))
            .Where(x => x.hasSpot)
            .ToArray();

        if (availableRandomSpots.Length > 0)
        {
            Console.Error.WriteLine("Growing any");

            var first = availableRandomSpots.FirstOrDefault();
            organism.GrowAny(first.organ, first.spot);
        }
        else
        {
            Console.Error.WriteLine("Waiting due to no available spots");

            organism.Wait();
        }
    }

    protected bool TryFindClosestOpponentOrgan(out (Organ organ, Organ oppOrgan, List<Vector2> path)? targetOrgan)
    {
        targetOrgan = null;
        foreach (Organ org in organism.organs)
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

// Responsible for harvesting one of each protein and dividing
public class HarvestBehaviour : BaseBehaviour
{
    private List<Protein> harvestTargets = new List<Protein>();

    public HarvestBehaviour(Organism organism, Map map, ProteinReserve proteins) : base(organism, map, proteins)
    {
        var unharvestedProteins = map.Proteins
            .Where(x => !x.isPlayerHarvested)
            .ToArray();

        var pathsToProteins = unharvestedProteins
            .Select(x => (protein: x, path: map.CalculatePathHeuristic(organism.root.position, x.position)))
            .Where(x => x.path != null)                
            .OrderBy(x => x.path.Count)
            .ToArray();

        var proteinsToCollect = new Protein.ProteinType[] 
        {
            Protein.ProteinType.A, 
            Protein.ProteinType.B, 
            Protein.ProteinType.C, 
            Protein.ProteinType.D,
        };

        var harvestTargetsSet = new HashSet<Protein>();
        var harvestTargetsWithDistance = new Dictionary<Protein, int>();
        foreach (var protType in proteinsToCollect)
        {
            var prots = pathsToProteins
                .Where(x => x.protein.type == protType && !harvestTargetsSet.Contains(x.protein))                
                .ToArray();

            if (prots.Length > 0)
            {
                var p = prots.First();
                harvestTargetsWithDistance.Add(p.protein, p.path.Count);
                harvestTargetsSet.Add(p.protein);
            }
        }

        harvestTargets = harvestTargetsWithDistance
            .OrderBy(x => x.Value)
            .Select(x => x.Key)
            .ToList();
    }

    public override void Plan()
    {
        plan.Clear();

        Console.Error.WriteLine("Evaluating Defense...");

        // first priority is to defend ourselves
        bool foundClosestOrgan = TryFindClosestOpponentOrgan(out var closestOrgan);
        if (map.OpponentOrgans.Count > 0 && foundClosestOrgan && closestOrgan.Value.path.Count == 3)
        {
            Console.Error.WriteLine("[Decision] Defense");
            plan.Enqueue(delegate
            {
                MoveToOrgan(closestOrgan.Value.organ, closestOrgan.Value.oppOrgan, closestOrgan.Value.path);
            });
            return;
        }

        Console.Error.WriteLine("Evaluating Harvest...");

        // second priority is to harvest enough proteins to split
        Protein[] unharvestedProteins = map.Proteins
            .Where(x => !x.isPlayerHarvested)
            .ToArray();

        Protein[] harvestedProteins = map.Proteins
            .Where(x => x.harvesters.Any(x => x.organism.id == organism.id))
            .ToArray();

        if (unharvestedProteins.Length > 0 && TryFindClosestUnharvestedProtein(proteins.GetPriority(0), out var protein))
        {
            Console.Error.WriteLine("[Decision] Harvest");
            plan.Enqueue(delegate
            {
                HarvestProtein(protein.Value.protein, protein.Value.organ, protein.Value.path);                
            });
            return;
        }              

        // last priority is to grow in random positions
        Console.Error.WriteLine("[Decision] Grow");
        plan.Enqueue(delegate
        {
            GrowInRandomPosition();
        });
    }

    protected void HarvestProtein(Protein protein, Organ organ, List<Vector2> path)
    {
        Console.Error.WriteLine($"Harvesting protein {protein.type} at {protein.position} | path: [{string.Join(", ", path.Select(x => x).ToArray())}]");

        Vector2 next = path.FirstOrDefault();

        if (path.Count > 5 && Map.IsPathStraight(path) && organism.CanSpore(organ))
        {
            organism.Spore(organ, path[path.Count - 3], Organism.BehaviourType.Harvest);
        }
        else if (path.Count > 7 && Map.IsPathStraight(path) && organism.CanSplit())
        {
            Console.Error.WriteLine("Growing sporer");

            Vector2 dir = path[1] - next;
            organism.GrowSporer(organ, next, Map.GetDirectionKey(dir));
        }
        else if (path.Count == 2 && organism.CanGrow(Organ.OrganType.Harvester))
        {
            Console.Error.WriteLine("Growing harvester");

            Vector2 dir = path[1] - next;
            organism.GrowHarvester(organ, next, Map.GetDirectionKey(dir));

            Protein targetToRemove = harvestTargets.FirstOrDefault(x => x.type == protein.type);
            harvestTargets.Remove(targetToRemove);
        }
        else if (organism.CanGrow(Organ.OrganType.Basic))
        {
            Console.Error.WriteLine("Growing basic");

            organism.GrowBasic(organ, next);
        }
        else
        {
            GrowInRandomPosition();
        }
    }

    protected bool TryFindClosestUnharvestedProtein(Protein.ProteinType? priority, out (Organ organ, Protein protein, List<Vector2> path)? target)
    {
        target = null;

        Console.Error.WriteLine($"Checking {organism.organs.Count} organs and {harvestTargets.Count} proteins");
        foreach (Organ org in organism.organs)
        {
            var pathsToProteins = harvestTargets
                .Select(x => (protein: x, path: map.CalculatePathHeuristic(org.position, x.position)))
                .Where(x => x.path != null)
                .OrderBy(x =>
                    priority.HasValue && x.protein.type == priority.Value
                        ? 0
                        : 1)
                .ThenBy(x => x.path.Count)
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
}

// Responsible for setting up a defensive perimeter
public class DefensiveBehaviour : BaseBehaviour 
{
    public DefensiveBehaviour(Organism organism, Map map, ProteinReserve proteins) : base(organism, map, proteins)
    {

    }

    public override void Plan()
    {
        Console.Error.WriteLine("Evaluating Defense...");

        // first priority is to defend ourselves
        bool foundClosestOrgan = TryFindClosestOpponentOrgan(out var closestOrgan);
        if (map.OpponentOrgans.Count > 0 && foundClosestOrgan && closestOrgan.Value.path.Count == 3)
        {
            Console.Error.WriteLine("[Decision] Defense");
            plan.Enqueue(delegate
            {
                MoveToOrgan(closestOrgan.Value.organ, closestOrgan.Value.oppOrgan, closestOrgan.Value.path);
            });
            return;
        }

        Console.Error.WriteLine("Evaluating Disruption...");  

        // third priority is to disrupt the opponent
        if (map.OpponentOrgans.Count > 0 && (TryFindClosestOpponentRoot(out var organ) || foundClosestOrgan))
        {
            Console.Error.WriteLine("[Decision] Disrupt");
            organ ??= closestOrgan;
            plan.Enqueue(delegate
            {
                MoveToOrgan(organ.Value.organ, organ.Value.oppOrgan, organ.Value.path);
            });
            return;
        }

        // last priority is to grow in random positions
        Console.Error.WriteLine("[Decision] Grow");
        plan.Enqueue(delegate
        {
            GrowInRandomPosition();
        });
    }

    protected bool TryFindClosestOpponentRoot(out (Organ organ, Organ oppOrgan, List<Vector2> path)? targetOrgan)
    {
        targetOrgan = null;
        foreach (Organ org in organism.organs)
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
}

public class Organism
{
    private BehaviourType behaviourType;
    private IBehaviour behaviour;
    private HashSet<int> organIds = new HashSet<int>();

    public int id;
    public Organ root;
    public List<Organ> organs;
    public List<Organ> harvesters;
    public Ownership owner;
    public bool isUsed;
    public Map map;
    public ProteinReserve proteins;
    public Queue<BehaviourType> sporeQueue;
    public BehaviourType Behaviour
    {
        get => behaviourType;
        set
        {
            behaviour = GetBehaviour(value);
            behaviourType = value;
        }
    }

    public enum BehaviourType
    {
        Harvest,
        Defensive,
    }

    public void AddOrgan(Organ organ)
    {
        if (organIds.Contains(organ.id))
        {
            return;
        }

        organ.organism = this;
        organs.Add(organ);
        organIds.Add(organ.id);

        if (organ.type == Organ.OrganType.Root)
        {
            this.root = organ;
        }
    }

    private IBehaviour GetBehaviour(BehaviourType behaviour)
    {
        switch (behaviour)
        {
            case BehaviourType.Harvest:
            default:
                return new HarvestBehaviour(this, map, proteins);
            case BehaviourType.Defensive:
                return new DefensiveBehaviour(this, map, proteins);
        }
    }

    public void Use()
    {
        isUsed = true;
    }

    public void Plan()
    {
        behaviour.Plan();
    }

    public void Act()
    {
        behaviour.Act();
    }

    public bool CanGrow(Organ.OrganType organType)
    {
        switch(organType)
        {
            case Organ.OrganType.Basic:
                return proteins.a >= 1;
            case Organ.OrganType.Harvester:
                return proteins.c >= 1 && proteins.d >= 1;
            case Organ.OrganType.Tentacle:
                return proteins.b >= 1 && proteins.c >= 1;
            case Organ.OrganType.Sporer:
                return proteins.b >= 1 && proteins.d >= 1;
        }

        return false;
    }

    public bool CanSpore(Organ organ)
    {
        return organ.type == Organ.OrganType.Sporer
            && proteins.a > 0
            && proteins.b > 0
            && proteins.c > 0
            && proteins.d > 0;
    }

    public void GrowBasic(Organ organ, Vector2 destination)
    {
        Grow(organ, Organ.OrganType.Basic, destination, "N");
        proteins.a--;
    }

    public void GrowHarvester(Organ organ, Vector2 destination, string direction)
    {
        Grow(organ, Organ.OrganType.Harvester, destination, direction);
        proteins.c--;
        proteins.d--;
    }

    public void GrowTentacle(Organ organ, Vector2 destination, string direction)
    {
        Grow(organ, Organ.OrganType.Tentacle, destination, direction);
        proteins.b--;
        proteins.c--;
    }

    public void GrowSporer(Organ organ, Vector2 destination, string direction)
    {
        Grow(organ, Organ.OrganType.Sporer, destination, direction);
        proteins.b--;
        proteins.d--;
    }

    public void GrowAny(Organ organ, Vector2 destination)
    {
        if (CanGrow(Organ.OrganType.Basic))
        {
            GrowBasic(organ, destination);
        }
        else if (CanGrow(Organ.OrganType.Harvester))
        {
            GrowHarvester(organ, destination, "N");
        }
        else if (CanGrow(Organ.OrganType.Tentacle))
        {
            GrowTentacle(organ, destination, "N");
        }
        else if (CanGrow(Organ.OrganType.Sporer))
        {
            GrowSporer(organ, destination, "N");
        }
        else
        {
            Wait();
        }
    }

    private void Grow(Organ organ, Organ.OrganType growthType, Vector2 destination, string direction)
    {
        if (!CanGrow(growthType))
        {
            Wait();
            return;
        }

        Console.Error.WriteLine($"Organ {id} (organism {id}) is growing a {growthType} from {organ.position} to {destination} facing {direction}");
        Console.WriteLine($"GROW {id} {destination.X} {destination.Y} {growthType.ToString().ToUpper()} {direction}");
        Use();
    }

    public void Spore(Organ organ, Vector2 destination, BehaviourType behaviour)
    {
        if (!CanSpore(organ))
        {
            Wait();
            return;
        }

        Console.Error.WriteLine($"Organ {id} (organism {id}) is sporing a root from {organ.position} to {destination}");
        Console.WriteLine($"SPORE {id} {destination.X} {destination.Y}");
        sporeQueue.Enqueue(behaviour);
        Use();
    }   

    public void Wait()
    {
        Console.Error.WriteLine($"Organism is waiting: no proteins or path to opponent organs available");
        Console.WriteLine("WAIT");
    }

    public bool CanSplit()
    {
        // must be able to grow a sporer and then spore next turn
        return proteins.a > 0
            && proteins.b > 1
            && proteins.c > 0
            && proteins.d > 1;
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
    private List<Organism> playerOrganisms;
    private List<Organism> opponentOrganisms;
    private List<Organ> organs;
    private List<Organ> playerOrgans;
    private List<Organ> opponentOrgans;

    public List<Protein> Proteins => proteins;
    public List<Organism> Organisms => organisms;
    public List<Organ> Organs => organs;
    public List<Organism> PlayerOrganisms => playerOrganisms;
    public List<Organism> OpponentOrganisms => opponentOrganisms;
    public List<Organ> PlayerOrgans => playerOrgans;
    public List<Organ> OpponentOrgans => opponentOrgans;

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

    public void SetProteins(List<Protein> proteins)
    {
        this.proteins = proteins;
    }

    public void SetOrganisms(List<Organism> organisms)
    {
        this.organisms = organisms;
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
            .Where(x => 
                x.X >= 0 && x.X < width &&
                x.Y >= 0 && x.Y < height &&
                !obstaclePositions.Contains(x))
            .ToArray();

        return spots.Length > 0;
    }

    public void Build()
    {
        organs = organisms
            .SelectMany(x => x.organs)
            .ToList();

        playerOrganisms = organisms
            .Where(x => x.owner == Ownership.Player)
            .ToList();

        opponentOrganisms = organisms
            .Where(x => x.owner == Ownership.Opponent)
            .ToList();

        playerOrgans = PlayerOrganisms
            .SelectMany(x => x.organs)
            .ToList();

        opponentOrgans = OpponentOrganisms
            .SelectMany(x => x.organs)
            .ToList();

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
    public string id;
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
}

public class ProteinReserve
{
    public int a;
    public int b;
    public int c;
    public int d;

    public Protein.ProteinType[] GetPriorities(int threshold)
    {
        var proteinCounts = new Dictionary<Protein.ProteinType, int>
        {
            { Protein.ProteinType.A, a },
            { Protein.ProteinType.B, b },
            { Protein.ProteinType.C, c },
            { Protein.ProteinType.D, d }
        };

        return proteinCounts
            .Where(x => x.Value < threshold)
            .OrderBy(x => x.Value)
            .Select(x => x.Key)
            .ToArray();
    }

    public Protein.ProteinType? GetPriority(int threshold)
    {
        var priorities = GetPriorities(0);
        return priorities.Length > 0 
            ? priorities.First() 
            : null;
    }
}

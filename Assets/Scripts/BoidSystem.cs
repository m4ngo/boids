using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Jobs;

[BurstCompile]
partial struct BoidSystem : ISystem
{
    // World attributes
    public const float CAGE_HALF_SIZE = 20f;
    public const float CAGE_SQUARE_RADIUS = 400f;
    public const int AMOUNT = 200000;

    // Boid attributes
    private const float SPEED = 5f;
    private const float SENSE_DIST = 10f;
    public const float BOID_SCALE = 0.1f;

    // Weights
    private const float SEPARATION = 20f;
    private const float COHESION = 10f;
    private const float ALIGNMENT = 10f;
    private const float OBSTACLE = 30f;

    private EntityQuery boidGroup;

    public void OnCreate(ref SystemState state)
    {
        boidGroup = SystemAPI.QueryBuilder().WithAll<LocalToWorld>().WithAll<BoidComponent>().Build();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        EntityManager entityManager = state.EntityManager;

        NativeArray<float3> positions = new NativeArray<float3>(AMOUNT, Allocator.TempJob);
        NativeArray<float3> headings = new NativeArray<float3>(AMOUNT, Allocator.TempJob);

        JobHandle storePositionsHeadingsHandle = default;
        new StorePositionAndHeadingJob
        {
            positions = positions,
            headings = headings
        }.ScheduleParallel(boidGroup, storePositionsHeadingsHandle).Complete();

        NativeArray<int> boidIndices = new NativeArray<int>(AMOUNT, Allocator.TempJob); // where i = the boid's index, and boidIndices[i] = the boid's merged cell index
        NativeArray<int> cellCount = new NativeArray<int>(AMOUNT, Allocator.TempJob); // where i = the boid's boidIndices[i] index, and cellCount[boidIndices[i]] = the number of boids in its cell
        NativeParallelMultiHashMap<int, int> hashMap = new NativeParallelMultiHashMap<int, int>(AMOUNT, Allocator.TempJob); // where key = the hashed cell, and output = list of indices referring to the boids in that cell

        // Hash all the boid positions into cells
        JobHandle hashPositionsHandle = default;
        float offsetRange = SENSE_DIST / 2f;
        quaternion randomHashRotation = quaternion.Euler(
            UnityEngine.Random.Range(-360f, 360f),
            UnityEngine.Random.Range(-360f, 360f),
            UnityEngine.Random.Range(-360f, 360f)
        );
        float3 randomHashOffset = new float3(
            UnityEngine.Random.Range(-offsetRange, offsetRange),
            UnityEngine.Random.Range(-offsetRange, offsetRange),
            UnityEngine.Random.Range(-offsetRange, offsetRange)
        );

        new HashPositionsToHashMapJob
        {
            hashMap = hashMap.AsParallelWriter(),
            cellRotationVary = randomHashRotation,
            positionOffsetVary = randomHashOffset,
            cellRadius = SENSE_DIST
        }.ScheduleParallel(boidGroup, hashPositionsHandle).Complete();

        // Merge each cell := getting the sum of all the positions and the sum of all the headings of all boids in that cell
        JobHandle mergeJobHandle = default;
        var (keys, length) = hashMap.GetUniqueKeyArray(Allocator.TempJob);
        new MergeCellsJob
        {
            hashMap = hashMap,
            keys = keys,
            length = length,
            cellCount = cellCount,
            positions = positions,
            headings = headings,
            boidIndices = boidIndices
        }.Schedule(keys.Length, mergeJobHandle).Complete();

        // Move the boids
        JobHandle moveBoidsHandle = default;
        new BoidJob
        {
            positions = positions,
            headings = headings,
            cellCount = cellCount,
            boidIndices = boidIndices,
            deltaTime = SystemAPI.Time.DeltaTime
        }.ScheduleParallel(boidGroup, moveBoidsHandle).Complete();

        positions.Dispose();
        headings.Dispose();
        boidIndices.Dispose();
        cellCount.Dispose();
        hashMap.Dispose();
        keys.Dispose();
    }

    private static float MinDistToBorder(float3 pos)
    {
        return CAGE_SQUARE_RADIUS - math.lengthsq(pos);
    }

    [BurstCompile]
    private partial struct StorePositionAndHeadingJob : IJobEntity
    {
        public NativeArray<float3> positions;
        public NativeArray<float3> headings;

        [BurstCompile]
        public void Execute([ReadOnly] ref LocalToWorld transform, [EntityIndexInQuery] int index)
        {
            positions[index] = transform.Position;
            headings[index] = transform.Forward;
        }
    }

    // Assign boids to a cell in the hashmap. Uses random offset and position to remove edge artefacts.
    [BurstCompile]
    private partial struct HashPositionsToHashMapJob : IJobEntity
    {
        public NativeParallelMultiHashMap<int, int>.ParallelWriter hashMap;
        [ReadOnly] public quaternion cellRotationVary;
        [ReadOnly] public float3 positionOffsetVary;
        [ReadOnly] public float cellRadius;

        [BurstCompile]
        public void Execute([ReadOnly] ref LocalToWorld transform, [EntityIndexInQuery] int index)
        {
            var hash = (int)math.hash(new int3(math.floor(math.mul(cellRotationVary, transform.Position + positionOffsetVary) / cellRadius)));
            hashMap.Add(hash, index);
        }
    }

    // Merges the cells to get the sum of the positions and headings. Stores the value in the existing positions and headings arrays.
    [BurstCompile]
    private partial struct MergeCellsJob : IJobFor
    {
        [ReadOnly] public NativeParallelMultiHashMap<int, int> hashMap;
        [ReadOnly] public NativeArray<int> keys;
        [ReadOnly] public int length;
        public NativeArray<int> boidIndices;
        public NativeArray<int> cellCount;
        public NativeArray<float3> positions;
        public NativeArray<float3> headings;

        [BurstCompile]
        public void Execute(int i)
        {
            if (i >= length)
            {
                return;
            }

            int key = keys[i];
            float3 posSum = 0;
            float3 headSum = 0;

            if (hashMap.TryGetFirstValue(key, out int boidIndex, out var iterator))
            {
                int firstIndex = boidIndex;
                boidIndices[firstIndex] = firstIndex;
                cellCount[firstIndex] = 1;
                while (hashMap.TryGetNextValue(out boidIndex, ref iterator))
                {
                    positions[firstIndex] += positions[boidIndex];
                    headings[firstIndex] += headings[boidIndex];
                    boidIndices[boidIndex] = firstIndex;
                    cellCount[firstIndex]++;
                }
            }
        }
    }

    // Calculates the movement of each boid
    [BurstCompile]
    private partial struct BoidJob : IJobEntity
    {
        [ReadOnly] public NativeArray<float3> positions;
        [ReadOnly] public NativeArray<float3> headings;
        [ReadOnly] public NativeArray<int> cellCount;
        [ReadOnly] public NativeArray<int> boidIndices;
        [ReadOnly] public float deltaTime;

        [BurstCompile]
        public void Execute(ref LocalToWorld localToWorld, [EntityIndexInQuery] int index)
        {
            float3 boidPosition = localToWorld.Position;
            int cellIndex = boidIndices[index];

            int nearbyBoidCount = cellCount[cellIndex] - 1;
            float3 positionSum = positions[cellIndex] - localToWorld.Position;
            float3 headingSum = headings[cellIndex] - localToWorld.Forward;

            float3 force = float3.zero;

            if (nearbyBoidCount > 0)
            {
                float3 averagePosition = positionSum / nearbyBoidCount;

                float distToAveragePositionSq = math.lengthsq(averagePosition - boidPosition);
                float maxDistToAveragePositionSq = SENSE_DIST * SENSE_DIST;

                float distanceNormalized = distToAveragePositionSq / maxDistToAveragePositionSq;
                float needToLeave = math.max(1 - distanceNormalized, 0f);

                float3 toAveragePosition = math.normalizesafe(averagePosition - boidPosition);
                float3 averageHeading = headingSum / nearbyBoidCount;

                force += -toAveragePosition * SEPARATION * needToLeave;
                force += toAveragePosition * COHESION;
                force += averageHeading * ALIGNMENT;
            }

            if (MinDistToBorder(boidPosition) < SENSE_DIST)
            {
                force += -math.normalize(boidPosition) * OBSTACLE;
            }

            float3 velocity = localToWorld.Forward * SPEED;
            velocity += force * deltaTime;
            velocity = math.normalize(velocity) * SPEED;

            localToWorld.Value = float4x4.TRS(
                localToWorld.Position + velocity * deltaTime,
                quaternion.LookRotationSafe(velocity, localToWorld.Up),
                new float3(BOID_SCALE)
            );
        }
    }
}

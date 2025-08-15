using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Jobs;
using UnityEngine;

partial struct BoidSystem : ISystem
{
    // World attributes
    public const float CAGE_HALF_SIZE = 15f;
    public const float CAGE_SQUARE_RADIUS = 225f;
    public const int AMOUNT = 5000;

    // Boid attributes
    private const float SPEED = 10f;
    private const float SENSE_DIST = 5f;

    // Weights
    private const float SEPARATION = 1f;
    private const float COHESION = 0f;
    private const float ALIGNMENT = 0f;
    private const float OBSTACLE = 0f;

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
        int iterator = 0;

        NativeArray<Entity> boids = boidGroup.ToEntityArray(Allocator.Temp);
        foreach (Entity entity in boids)
        {
            if (entityManager.HasComponent<BoidComponent>(entity))
            {
                LocalToWorld localTransform = entityManager.GetComponentData<LocalToWorld>(entity);
                positions[iterator] = localTransform.Position;
                headings[iterator] = localTransform.Forward;
                iterator++;
            }
        }

        NativeArray<int> boidIndices = new NativeArray<int>(AMOUNT, Allocator.TempJob); // where i = the boid's index, and boidIndices[i] = the boid's merged cell index
        NativeArray<int> cellCount = new NativeArray<int>(AMOUNT, Allocator.TempJob); // where i = the boid's boidIndices[i] index, and cellCount[boidIndices[i]] = the number of boids in its cell
        NativeParallelMultiHashMap<int, int> hashMap = new NativeParallelMultiHashMap<int, int>(AMOUNT, Allocator.TempJob); // where key = the hashed cell, and output = list of indices referring to the boids in that cell

        // Hash all the boid positions into cells
        JobHandle hashPositionsHandle = default;
        float offsetRange = SENSE_DIST / 2f;
        new HashPositionsToHashMapJob
        {
            hashMap = hashMap.AsParallelWriter(),
            cellRotationVary = quaternion.Euler(
                UnityEngine.Random.Range(-360f, 360f),
                UnityEngine.Random.Range(-360f, 360f),
                UnityEngine.Random.Range(-360f, 360f)
            ),
            positionOffsetVary = new float3(
                UnityEngine.Random.Range(-offsetRange, offsetRange),
                UnityEngine.Random.Range(-offsetRange, offsetRange),
                UnityEngine.Random.Range(-offsetRange, offsetRange)
            ),
            cellRadius = SENSE_DIST
        }.Schedule(boidGroup, hashPositionsHandle).Complete();

        // Merge each cell := getting the sum of all the positions and the sum of all the headings of all boids in that cell
        JobHandle mergeJobHandle = default;
        NativeArray<int> keys = hashMap.GetKeyArray(Allocator.TempJob);
        new MergeCellsJob
        {
            hashMap = hashMap,
            keys = keys,
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
        boids.Dispose();
        boidIndices.Dispose();
        cellCount.Dispose();
        hashMap.Dispose();
        keys.Dispose();
    }

    private static float MinDistToBorder(float3 pos)
    {
        return CAGE_SQUARE_RADIUS - math.lengthsq(pos);
    }

    // Assign boids to a cell in the hashmap. Uses random offset and position to remove edge artefacts.
    [BurstCompile]
    private partial struct HashPositionsToHashMapJob : IJobEntity
    {
        public NativeParallelMultiHashMap<int, int>.ParallelWriter hashMap;
        [ReadOnly] public quaternion cellRotationVary;
        [ReadOnly] public float3 positionOffsetVary;
        [ReadOnly] public float cellRadius;

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
        public NativeArray<int> boidIndices;
        public NativeArray<int> cellCount;
        public NativeArray<float3> positions;
        public NativeArray<float3> headings;

        public void Execute(int i)
        {
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

        public void Execute(ref LocalToWorld transform, [EntityIndexInQuery] int index)
        {
            float3 self = transform.Position;
            int boidIndex = boidIndices[index];

            // Calculate boid forces and apply new position and rotation
            int neighborCount = cellCount[boidIndex] - 1;
            float3 positionSum = positions[boidIndex] - self;  // remember to subtract because we dont include itself
            float3 headingSum = headings[boidIndex] - transform.Forward;

            float3 velocity = float3.zero;

            if (neighborCount > 0)
            {
                float3 averagePosition = positionSum / neighborCount;

                float distToAveragePositionSq = math.lengthsq(averagePosition - self);
                float maxDistToAveragePositionSq = SENSE_DIST * SENSE_DIST;

                float distanceNormalized = distToAveragePositionSq / maxDistToAveragePositionSq;
                float needToLeave = math.max(1 - distanceNormalized, 0f);

                float3 toAveragePosition = math.normalizesafe(averagePosition - self);
                float3 averageHeading = headingSum / neighborCount;
                
                velocity += -toAveragePosition * needToLeave * SEPARATION;
                velocity += toAveragePosition * COHESION;
                velocity += averageHeading *ALIGNMENT;
            }

            if (math.min(math.min(
                (CAGE_HALF_SIZE) - math.abs(self.x),
                (CAGE_HALF_SIZE) - math.abs(self.y)),
                (CAGE_HALF_SIZE) - math.abs(self.z))
                    < SENSE_DIST)
            {
                velocity += -math.normalize(self) * OBSTACLE;
            }

            float3 moveAmount = velocity * SPEED;

            transform.Value = float4x4.TRS(
                self + moveAmount * deltaTime, 
                quaternion.LookRotationSafe(moveAmount, transform.Up), 
                new float3(0.2f)
                );
        }
    }
}

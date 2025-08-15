using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Jobs;
using System;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs.LowLevel.Unsafe;

[JobProducerType(typeof(JobNativeMultiHashMapUniqueHashExtensions.JobNativeMultiHashMapMergedSharedKeyIndicesProducer<>))]
public interface IJobNativeMultiHashMapMergedSharedKeyIndices
{
    // The first time each key (=hash) is encountered, ExecuteFirst() is invoked with corresponding value (=index).
    void ExecuteFirst(int index);

    // For each subsequent instance of the same key in the bucket, ExecuteNext() is invoked with the corresponding
    // value (=index) for that key, as well as the value passed to ExecuteFirst() the first time this key
    // was encountered (=firstIndex).
    void ExecuteNext(int firstIndex, int index);
}

public static class JobNativeMultiHashMapUniqueHashExtensions
{
    internal struct JobNativeMultiHashMapMergedSharedKeyIndicesProducer<TJob>
        where TJob : struct, IJobNativeMultiHashMapMergedSharedKeyIndices
    {
        [ReadOnly] public NativeParallelMultiHashMap<int, int> HashMap;
        internal TJob JobData;

        private static IntPtr s_JobReflectionData;

        internal static IntPtr Initialize()
        {
            if (s_JobReflectionData == IntPtr.Zero)
            {
                s_JobReflectionData = JobsUtility.CreateJobReflectionData(typeof(JobNativeMultiHashMapMergedSharedKeyIndicesProducer<TJob>), typeof(TJob), (ExecuteJobFunction)Execute);
            }

            return s_JobReflectionData;
        }

        delegate void ExecuteJobFunction(ref JobNativeMultiHashMapMergedSharedKeyIndicesProducer<TJob> jobProducer, IntPtr additionalPtr, IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex);

        public static unsafe void Execute(ref JobNativeMultiHashMapMergedSharedKeyIndicesProducer<TJob> jobProducer, IntPtr additionalPtr, IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex)
        {
            while (true)
            {
                int begin;
                int end;

                if (!JobsUtility.GetWorkStealingRange(ref ranges, jobIndex, out begin, out end))
                {
                    return;
                }

                var bucketData = jobProducer.HashMap.GetUnsafeBucketData();
                var buckets = (int*)bucketData.buckets;
                var nextPtrs = (int*)bucketData.next;
                var keys = bucketData.keys;
                var values = bucketData.values;

                for (int i = begin; i < end; i++)
                {
                    int entryIndex = buckets[i];

                    while (entryIndex != -1)
                    {
                        var key = UnsafeUtility.ReadArrayElement<int>(keys, entryIndex);
                        var value = UnsafeUtility.ReadArrayElement<int>(values, entryIndex);
                        int firstValue;

                        NativeParallelMultiHashMapIterator<int> it;
                        jobProducer.HashMap.TryGetFirstValue(key, out firstValue, out it);

                        // [macton] Didn't expect a usecase for this with multiple same values
                        // (since it's intended use was for unique indices.)
                        // https://discussions.unity.com/t/718143/5
                        if (entryIndex == it.GetEntryIndex())
                        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                            JobsUtility.PatchBufferMinMaxRanges(bufferRangePatchData, UnsafeUtility.AddressOf(ref jobProducer), value, 1);
#endif
                            jobProducer.JobData.ExecuteFirst(value);
                        }
                        else
                        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                            var startIndex = math.min(firstValue, value);
                            var lastIndex = math.max(firstValue, value);
                            var rangeLength = (lastIndex - startIndex) + 1;

                            JobsUtility.PatchBufferMinMaxRanges(bufferRangePatchData, UnsafeUtility.AddressOf(ref jobProducer), startIndex, rangeLength);
#endif
                            jobProducer.JobData.ExecuteNext(firstValue, value);
                        }

                        entryIndex = nextPtrs[entryIndex];
                    }
                }
            }
        }
    }

    public static unsafe JobHandle Schedule<TJob>(this TJob jobData, NativeParallelMultiHashMap<int, int> hashMap, int minIndicesPerJobCount, JobHandle dependsOn = new JobHandle())
        where TJob : struct, IJobNativeMultiHashMapMergedSharedKeyIndices
    {
        var jobProducer = new JobNativeMultiHashMapMergedSharedKeyIndicesProducer<TJob>
        {
            HashMap = hashMap,
            JobData = jobData
        };

        var scheduleParams = new JobsUtility.JobScheduleParameters(
            UnsafeUtility.AddressOf(ref jobProducer)
            , JobNativeMultiHashMapMergedSharedKeyIndicesProducer<TJob>.Initialize()
            , dependsOn
            , ScheduleMode.Parallel
        );

        return JobsUtility.ScheduleParallelFor(ref scheduleParams, hashMap.GetUnsafeBucketData().bucketCapacityMask + 1, minIndicesPerJobCount);
    }
}

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
        NativeArray<int> keys = hashMap.GetKeyArray(Allocator.TempJob);
        new MergeCellsJob
        {
            cellCount = cellCount,
            positions = positions,
            headings = headings,
            boidIndices = boidIndices
        }.Schedule(hashMap, 64, mergeJobHandle).Complete();

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

        public void Execute([ReadOnly] ref LocalToWorld transform, [EntityIndexInQuery] int index)
        {
            var hash = (int)math.hash(new int3(math.floor(math.mul(cellRotationVary, transform.Position + positionOffsetVary) / cellRadius)));
            hashMap.Add(hash, index);
        }
    }

    // Merges the cells to get the sum of the positions and headings. Stores the value in the existing positions and headings arrays.
    [BurstCompile]
    private struct MergeCellsJob : IJobNativeMultiHashMapMergedSharedKeyIndices
    {
        public NativeArray<int> boidIndices;
        public NativeArray<float3> positions;
        public NativeArray<float3> headings;
        public NativeArray<int> cellCount;

        public void ExecuteFirst(int firstBoidIndexEncountered)
        {
            boidIndices[firstBoidIndexEncountered] = firstBoidIndexEncountered;
            cellCount[firstBoidIndexEncountered] = 1;
            float3 positionInThisCell = positions[firstBoidIndexEncountered] / cellCount[firstBoidIndexEncountered];
        }

        public void ExecuteNext(int firstBoidIndexAsCellKey, int boidIndexEncountered)
        {
            cellCount[firstBoidIndexAsCellKey] += 1;
            headings[firstBoidIndexAsCellKey] += headings[boidIndexEncountered];
            positions[firstBoidIndexAsCellKey] += positions[boidIndexEncountered];
            boidIndices[boidIndexEncountered] = firstBoidIndexAsCellKey;
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

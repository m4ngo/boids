using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using Unity.Jobs;

partial struct BoidSystem : ISystem
{
    // World attributes
    public const float HALF_CAGE_SIZE = 10f;
    public const int AMOUNT = 1500;

    // Boid attributes
    private const float SPEED = 10f;
    private const float SENSE_DIST = 1f;

    // Weights
    private const float SEPARATION= 1f;
    private const float COHESION = 1f;
    private const float ALIGNMENT = 1f;
    private const float OBSTACLE = 5f;

    // Spatial optimization
    //private NativeParallelMultiHashMap<int3, NativeList<float3x2>> WORLD_MAP;

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        EntityManager entityManager = state.EntityManager;

        NativeArray<Entity> entities = entityManager.GetAllEntities(Allocator.Temp);
        NativeArray<float3> positions = new NativeArray<float3>(AMOUNT, Allocator.TempJob);
        NativeArray<float3> headings = new NativeArray<float3>(AMOUNT, Allocator.TempJob);
        int iterator = 0;

        foreach (Entity entity in entities)
        {
            if (entityManager.HasComponent<BoidComponent>(entity))
            {
                LocalTransform localTransform = entityManager.GetComponentData<LocalTransform>(entity);
                positions[iterator] = localTransform.Position;
                headings[iterator] = localTransform.Forward();
                iterator++;
            }
        }

        var job = new BoidJob
        {
            positions = positions,
            headings = headings,
            deltaTime = SystemAPI.Time.DeltaTime
        };

        EntityQuery query = SystemAPI.QueryBuilder().WithAll<LocalTransform>().WithAll<BoidComponent>().Build();
        job.Schedule(query);
    }

    //private float3 CalculateForces(float3 self)
    //{
    //    float3 separationSum = float3.zero;
    //    float3 positionSum = float3.zero;
    //    float3 headingSum = float3.zero;

    //    // Temp list
    //    NativeList<float3x2> nearby = new NativeList<float3x2>();
    //    foreach (float3x2 near in nearby)
    //    {
    //        float dist = Vector3.Distance(near.c0, self);
    //        separationSum += -(near.c0 - self) * (1f / Mathf.Max(dist, 0.0001f));
    //        positionSum += near.c0;
    //        headingSum += near.c1;
    //    }

    //    float3 separationForce = float3.zero;
    //    float3 cohesionForce = float3.zero;
    //    float3 alignmentForce = float3.zero;
    //    float3 avoidWallsForce = float3.zero;

    //    int total = nearby.Length;

    //    if (total > 0)
    //    {
    //        separationForce = separationSum / total;
    //        cohesionForce = (positionSum / total) - self;
    //        alignmentForce = headingSum / total;
    //    }

    //    if (MinDistToBorder(self) < SENSE_DIST)
    //    {
    //        avoidWallsForce = -Vector3.Normalize(self);
    //    }

    //    return
    //        separationForce * SEPARATION +
    //        cohesionForce * COHESION +
    //        alignmentForce * ALIGNMENT +
    //        avoidWallsForce * OBSTACLE;
    //}

    private static float MinDistToBorder(Vector3 pos)
    {
        return Mathf.Min(Mathf.Min(
            HALF_CAGE_SIZE - Mathf.Abs(pos.x),
            HALF_CAGE_SIZE - Mathf.Abs(pos.y)),
            HALF_CAGE_SIZE - Mathf.Abs(pos.z)
        );
    }

    public partial struct BoidJob : IJobEntity
    {
        [ReadOnly]
        public NativeArray<float3> positions;
        public NativeArray<float3> headings;
        public float deltaTime;

        public void Execute(ref LocalTransform localTransform)
        {
            float3 self = localTransform.Position;

            // Calculate boid forces and apply new position and rotation
            float3 separationSum = float3.zero;
            float3 positionSum = float3.zero;
            float3 headingSum = float3.zero;
            int total = 0;

            for (int i = 0; i < positions.Length; i++)
            {
                float dist = Vector3.Distance(positions[i], self);
                if (dist > SENSE_DIST)
                {
                    continue;
                }
                total++;
                separationSum += -(positions[i]  - self) * (1f / Mathf.Max(dist, 0.0001f));
                positionSum += positions[i];
                headingSum += headings[i];
            }

            float3 separationForce = float3.zero;
            float3 cohesionForce = float3.zero;
            float3 alignmentForce = float3.zero;
            float3 avoidWallsForce = float3.zero;

            if (total > 0)
            {
                separationForce = separationSum / total;
                cohesionForce = (positionSum / total) - self;
                alignmentForce = headingSum / total;
            }

            if (MinDistToBorder(self) < SENSE_DIST)
            {
                avoidWallsForce = -Vector3.Normalize(self);
            }

            float3 moveAmount =
                separationForce * SEPARATION +
                cohesionForce * COHESION +
                alignmentForce * ALIGNMENT +
                avoidWallsForce * OBSTACLE;

            localTransform.Position = localTransform.Position + moveAmount * deltaTime * SPEED;
            localTransform.Rotation = Quaternion.LookRotation(moveAmount);
        }
    }
}

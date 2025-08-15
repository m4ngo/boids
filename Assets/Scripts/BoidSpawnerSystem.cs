using Unity.Burst;
using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Transforms;

partial struct BoidSpawnerSystem : ISystem, ISystemStartStop
{
    [BurstCompile]
    public void OnStartRunning(ref SystemState state)
    {
        if (!SystemAPI.TryGetSingletonEntity<BoidSpawnerComponent>(out Entity spawnerEntity))
        {
            return;
        }

        RefRW<BoidSpawnerComponent> spawner = SystemAPI.GetComponentRW<BoidSpawnerComponent>(spawnerEntity);

        EntityCommandBuffer ecb = new EntityCommandBuffer(Allocator.Temp);

        Random m_Random = Random.CreateFromIndex((uint)SystemAPI.Time.ElapsedTime);

        for (int i = 0; i < BoidSystem.AMOUNT; i++) // Spawn boids
        {
            Entity newEntity = ecb.Instantiate(spawner.ValueRO.prefab);

            ecb.AddComponent(newEntity, new BoidComponent { });
            ecb.SetComponent(newEntity, new LocalTransform { 
                Position = (m_Random.NextFloat3() - new float3(0.5f)) * 2f * BoidSystem.HALF_CAGE_SIZE,
                Rotation = quaternion.Euler((m_Random.NextFloat3() - new float3(0.5f)) * 720f),
                Scale = 0.2f
            });
        }

        ecb.Playback(state.EntityManager);
    }

    [BurstCompile]
    public void OnStopRunning(ref SystemState state)
    {

    }
}

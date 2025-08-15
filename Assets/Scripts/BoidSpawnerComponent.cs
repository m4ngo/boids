using Unity.Entities;
using Unity.Mathematics;

public struct BoidSpawnerComponent : IComponentData
{
    public Entity prefab;
    public int amount;
}

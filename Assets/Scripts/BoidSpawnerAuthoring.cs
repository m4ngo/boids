using UnityEngine;
using Unity.Entities;

public class BoidSpawnerAuthoring : MonoBehaviour
{
    public GameObject prefab;

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawWireCube(
            Vector3.zero,
            new Vector3(
                BoidSystem.CAGE_HALF_SIZE * 2,
                BoidSystem.CAGE_HALF_SIZE * 2,
                BoidSystem.CAGE_HALF_SIZE * 2
            )
        );
    }
}

class BoidSpawnerBaker : Baker<BoidSpawnerAuthoring>
{
    public override void Bake(BoidSpawnerAuthoring authoring)
    {
        Entity entity = GetEntity(TransformUsageFlags.WorldSpace);

        AddComponent(entity, new BoidSpawnerComponent
        {
            prefab = GetEntity(authoring.prefab, TransformUsageFlags.WorldSpace)
        });
    }
}

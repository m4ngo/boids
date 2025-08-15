using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using System.Collections.Generic;

public class BoidSpawnerAuthoring : MonoBehaviour
{
    public GameObject prefab;
    public int amount;

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawWireCube(
            Vector3.zero,
            new Vector3(
                BoidSystem.HALF_CAGE_SIZE * 2,
                BoidSystem.HALF_CAGE_SIZE * 2,
                BoidSystem.HALF_CAGE_SIZE * 2
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
            prefab = GetEntity(authoring.prefab, TransformUsageFlags.WorldSpace),
            amount = authoring.amount
        });
    }
}

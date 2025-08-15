using UnityEngine;
using Unity.Entities;

public class BoidSpawnerAuthoring : MonoBehaviour
{
    public GameObject prefab;

    private void Start()
    {
        Debug.LogError("GAME STARTED");
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(Vector3.zero, BoidSystem.CAGE_HALF_SIZE);
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

using System.Runtime.CompilerServices;

namespace TGC.MonoGame.TP.Fisicas;

public struct SubgroupCollisionFilter
{
    /// <summary>
    /// A mask of 16 bits, each set bit representing a collision group that an object belongs to.
    /// </summary>
    private readonly ushort _subgroupMembership;
    /// <summary>
    /// A mask of 16 bits, each set bit representing a collision group that an object can interact with.
    /// </summary>
    private ushort _collidableSubgroups;
    /// <summary>
    /// Id of the owner of the object. Objects belonging to different groups always collide.
    /// </summary>
    private readonly int _groupId;

    /// <summary>
    /// Initializes a collision filter that belongs to one specific subgroup and can collide with any other subgroup.
    /// </summary>
    /// <param name="groupId">Id of the group that this filter operates within.</param>
    /// <param name="subgroupId">Id of the subgroup to put this collidable into.</param>
    public SubgroupCollisionFilter(int groupId, int subgroupId)
    {
        _groupId = groupId;
        //Debug.Assert(subgroupId >= 0 && subgroupId < 16, "The subgroup field is a ushort; it can only hold 16 distinct subgroups.");
        _subgroupMembership = (ushort)(1 << subgroupId);
        _collidableSubgroups = ushort.MaxValue;
    }

    /// <summary>
    /// Disables a collision between this filter and the specified subgroup.
    /// </summary>
    /// <param name="subgroupId">Subgroup id to disable collision with.</param>
    public void DisableCollision(int subgroupId)
    {
        //Debug.Assert(subgroupId >= 0 && subgroupId < 16, "The subgroup field is a ushort; it can only hold 16 distinct subgroups.");
        _collidableSubgroups ^= (ushort)(1 << subgroupId);
    }

    /// <summary>
    /// Modifies the interactable subgroups such that filterB does not interact with the subgroups defined by filter a and vice versa.
    /// </summary>
    /// <param name="filterA">Filter from which to remove collisions with filter b's subgroups.</param>
    /// <param name="filterB">Filter from which to remove collisions with filter a's subgroups.</param>
    public static void DisableCollision(ref SubgroupCollisionFilter filterA, ref SubgroupCollisionFilter filterB)
    {
        filterA._collidableSubgroups &= (ushort)~filterB._subgroupMembership;
        filterB._collidableSubgroups &= (ushort)~filterA._subgroupMembership;
    }

    /// <summary>
    /// Checks if the filters can collide by checking if b's membership can be collided by a's collidable groups.
    /// </summary>
    /// <param name="a">First filter to test.</param>c
    /// <param name="b">Second filter to test.</param>
    /// <returns>True if the filters can collide, false otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool AllowCollision(in SubgroupCollisionFilter a, in SubgroupCollisionFilter b)
    {
        return a._groupId != b._groupId || (a._collidableSubgroups & b._subgroupMembership) > 0;
    }

}
using System;
using System.Numerics;

namespace NBodySimulation
{
    public abstract class OctreeNode
    {
        public Single Width;
        public Vector3 Center;
    }

    public class OctreeLeafNode : OctreeNode
    {
        public Int32 Index;
    }

    public class OctreeBranchNode : OctreeNode
    {
        public Single Mass;
        public Vector3 MassCenter;
        public OctreeNode?[] Octants = new OctreeNode[8];
    }

    public class Octree
    {
        private OctreeBranchNode _Root;

        public Octree(List<Particle> particles, List<Int32> indices, Single width, Vector3 center)
        {
            _Root = new OctreeBranchNode()
            {
                Width = width,
                Center = center
            };

            foreach (var index in indices)
            {
                var particle = particles[index];
                while (IsOutside(particle, _Root))
                {
                    var octantIndex = OctantIndex(particle, _Root);
                    var newRoot = new OctreeBranchNode()
                    {
                        Width = _Root.Width * 2.0f,
                        Center = _Root.Center
                    };
                    newRoot.Center.X += OctantSign(octantIndex, 0) * _Root.Width / 2.0f;
                    newRoot.Center.Y += OctantSign(octantIndex, 1) * _Root.Width / 2.0f;
                    newRoot.Center.Z += OctantSign(octantIndex, 2) * _Root.Width / 2.0f;
                    newRoot.Octants[7 - octantIndex] = _Root;
                    _Root = newRoot;
                }
                Insert(index, particles, _Root);
            }
            UpdateNodeData(_Root, particles);
        }

        public List<Particle> ApproximateInteractiveParticles(Int32 index, List<Particle> particles,
            Single openingAngle)
        {
            var approximateInteractiveParticles = new List<Particle>();
            ApproximateInteractiveParticles(index, particles, openingAngle, _Root, approximateInteractiveParticles);
            return approximateInteractiveParticles;
        }

        private static void Insert(Int32 index, List<Particle> particles, OctreeBranchNode branchNode)
        {
            var particle = particles[index];
            var octantIndex = OctantIndex(particle, branchNode);
            if (branchNode.Octants[octantIndex] == null)
            {
                var newLeafNode = new OctreeLeafNode();
                newLeafNode.Index = index;
                UpdateOctant(newLeafNode, octantIndex, branchNode);
                branchNode.Octants[octantIndex] = newLeafNode;
            }
            else if (branchNode.Octants[octantIndex] is OctreeLeafNode leafNode)
            {
                var newBranchNode = new OctreeBranchNode()
                {
                    Width = leafNode.Width,
                    Center = leafNode.Center
                };
                Insert(leafNode.Index, particles, newBranchNode);
                Insert(index, particles, newBranchNode);
                branchNode.Octants[octantIndex] = newBranchNode;
            }
            else if (branchNode.Octants[octantIndex] is OctreeBranchNode octantBranchNode)
            {
                Insert(index, particles, octantBranchNode);
            }
        }

        private static bool IsOutside(Particle particle, OctreeNode node)
        {
            var diff = particle.Position - node.Center;
            var nodeHalfWidth = node.Width / 2.0f;
            return MathF.Abs(diff.X) > nodeHalfWidth ||
                MathF.Abs(diff.Y) > nodeHalfWidth ||
                MathF.Abs(diff.Z) > nodeHalfWidth;
        }

        private static UInt32 OctantIndex(Particle particle, OctreeNode node)
        {
            UInt32 index = 0;
            if (particle.Position.X < node.Center.X) index += 1;
            if (particle.Position.Y < node.Center.Y) index += 2;
            if (particle.Position.Z < node.Center.Z) index += 4;
            return index;
        }

        private static Single OctantSign(UInt32 octantIndex, Int32 dimension)
        {
            return (octantIndex >> dimension) % 2 == 0 ? 1.0f : -1.0f;
        }

        private static void UpdateOctant(OctreeNode octant, UInt32 octantIndex, OctreeNode parent)
        {
            octant.Width = parent.Width / 2.0f;
            octant.Center = parent.Center;
            octant.Center.X += OctantSign(octantIndex, 0) * octant.Width / 2.0f;
            octant.Center.Y += OctantSign(octantIndex, 1) * octant.Width / 2.0f;
            octant.Center.Z += OctantSign(octantIndex, 2) * octant.Width / 2.0f;
        }

        private static void UpdateNodeData(OctreeBranchNode branchNode, List<Particle> particles)
        {
            branchNode.Mass = 0.0f;
            branchNode.MassCenter = Vector3.Zero;
            foreach (var octant in branchNode.Octants)
            {
                if (octant is OctreeLeafNode leafNode)
                {
                    var particle = particles[leafNode.Index];
                    branchNode.Mass += particle.Mass;
                    branchNode.MassCenter += particle.Position * particle.Mass;
                }
                else if (octant is OctreeBranchNode octantBranchNode)
                {
                    UpdateNodeData(octantBranchNode, particles);
                }
            }
            if (branchNode.Mass > 0) branchNode.MassCenter /= branchNode.Mass;
        }

        private static void ApproximateInteractiveParticles(Int32 index, List<Particle> particles,
            Single openingAngle, OctreeNode? node, List<Particle> approximateInteractiveParticles)
        {
            if (node is OctreeBranchNode branchNode)
            {
                var particle = particles[index];
                var diff = particle.Position - branchNode.MassCenter;
                if (branchNode.Width / diff.Length() < openingAngle)
                {
                    approximateInteractiveParticles.Add(new Particle()
                    {
                        Mass = branchNode.Mass,
                        Position = branchNode.MassCenter
                    });
                }
                else
                {
                    foreach (var octant in branchNode.Octants)
                    {
                        ApproximateInteractiveParticles(index, particles, openingAngle, octant,
                            approximateInteractiveParticles);
                    }
                }
            }
            else if (node is OctreeLeafNode leafNode)
            {
                if (leafNode.Index != index)
                {
                    approximateInteractiveParticles.Add(new Particle()
                    {
                        Mass = particles[leafNode.Index].Mass,
                        Position = particles[leafNode.Index].Position
                    });
                }
            }
        }
    }
}


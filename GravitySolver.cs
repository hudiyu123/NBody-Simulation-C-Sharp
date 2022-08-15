using System;
using System.Threading.Tasks;
using System.Numerics;

namespace NBodySimulation
{
    public interface IGravitySolver
    {
        public void ComputeAcceleration(List<Particle> particles, List<Int32> indices);
    }

    public class DirectGravitySolver : IGravitySolver
    {
        private Single _G;
        private Single _Softening;

        public DirectGravitySolver(Single G, Single softening)
        {
            _G = G;
            _Softening = softening;
        }

        public void ComputeAcceleration(List<Particle> particles, List<Int32> indices)
        {
            // Clears acceleration for each particle.
            Parallel.ForEach(indices, index =>
            {
                particles[index].Acceloration = Vector3.Zero;
            });

            Parallel.For(0, indices.Count, i =>
            {
                var particle = particles[indices[i]];
                for (int j = 0; j < indices.Count; j++)
                {
                    if (i == j) continue;
                    var otherParticle = particles[indices[j]];
                    var diff = otherParticle.Position - particle.Position;
                    var distance = MathF.Sqrt(diff.LengthSquared() + MathF.Pow(_Softening, 2.0f));
                    var factor = _G / MathF.Pow(distance, 3.0f) * diff;
                    particle.Acceloration += otherParticle.Mass * factor;
                }
            });
        }
    }

    public class BarnesHutGravitySolver : IGravitySolver
    {
        private Single _G;
        private Single _Softening;
        private Single _OpeningAngle;
        private Single _Width;
        private Vector3 _Center;

        public BarnesHutGravitySolver(Single G, Single softening, Single openingAngle, Single width, Vector3 center)
        {
            _G = G;
            _Softening = softening;
            _OpeningAngle = openingAngle;
            _Width = width;
            _Center = center;
        }

        public void ComputeAcceleration(List<Particle> particles, List<Int32> indices)
        {
            var octree = new Octree(particles, indices, _Width, _Center);
            Parallel.ForEach(indices, index =>
            {
                var particle = particles[index];
                particle.Acceloration = Vector3.Zero;
                var approximateInteractiveParticles = octree.ApproximateInteractiveParticles(index, particles,
                    _OpeningAngle);
                foreach (var approximateInteractiveParticle in approximateInteractiveParticles)
                {
                    var diff = approximateInteractiveParticle.Position - particle.Position;
                    var distance = MathF.Sqrt(diff.LengthSquared() + MathF.Pow(_Softening, 2.0f));
                    var factor = _G / MathF.Pow(distance, 3.0f) * diff;
                    particle.Acceloration += approximateInteractiveParticle.Mass * factor;
                }
            });
        }
    }
}

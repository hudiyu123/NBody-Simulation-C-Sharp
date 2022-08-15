using System;

namespace NBodySimulation
{
    public interface IIntegrator
    {
        public void Forward(List<Particle> particles, List<Int32> indices);
    }

    public class LeapfrogIntegrator : IIntegrator
    {
        private Single _TimeStep;

        public LeapfrogIntegrator(Single timeStep)
        {
            _TimeStep = timeStep;
        }

        public void Forward(List<Particle> particles, List<Int32> indices)
        {
            Parallel.ForEach(indices, index =>
            {
                var particle = particles[index];
                particle.Position += 0.5f * _TimeStep * particle.Velocity;
                particle.Velocity += particle.Acceloration * _TimeStep;
                particle.Position += 0.5f * _TimeStep * particle.Velocity;
            });
        }
    }
}

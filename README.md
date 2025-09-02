### Boids: practicing using Unity DOTS
- This repository is a simple project I created to learn Unity DOTS.
- The goal of the project is to simulate hundreds of thousands of boids moving in a 3D environment. For those unaware of what boids are, here is a wikipedia link that explains boids: [Boids](https://en.wikipedia.org/wiki/Boids)
- I combined ECS, Burst, and Jobs together to create a fairly optimized boid simulation.
- The result was 300,000 boids running simultaneously!

### Process
- I started with a very naive approach. Each boid would search through every other existing boid to find the ones within its sensing range. This approach was extremely expensive and only allowed for a few thousand boids.
- I tried using jobs to parallelize this computation, making it marginally faster. (IIRC) I squeaked out about 10,000 boids on my low-end gaming laptop.
- Finally, I made the shift and used a position hashing function to store each boid's position into a 'cell.' This allows boids to lookup the nearby boids by cell, instead of searching through every single other boid. This improved the efficiency enormously. With the addition of Unity's burst compilation, I was able to simulate 300,000 boids.

### Result
https://github.com/user-attachments/assets/bb8b4609-4852-4101-801d-f248e52d3676

*The final result, with 300,000 boids swarming about. There are so many that it almost looks like a fluid mass.*

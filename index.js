// Dot Product Function
function dotProduct(vectorA, vectorB) {
  if (
    !vectorA ||
    !vectorB ||
    typeof vectorA.length !== "number" ||
    typeof vectorB.length !== "number"
  ) {
    throw new TypeError("Both arguments must be array-like structures.");
  }

  if (vectorA.length !== vectorB.length) {
    throw new RangeError("Vectors must be of the same length.");
  }

  let result = 0;
  const len = vectorA.length;
  for (let i = 0; i < len; i++) {
    result += vectorA[i] * vectorB[i];
  }
  return result;
}

// Rigid Body 2D Class
class RigidBody2D {
  constructor(mass, momentOfInertia, pos = { x: 0, y: 0 }, angle = 0) {
    this.mass = mass;
    this.invMass = mass > 0 ? 1.0 / mass : 0.0;
    this.momentOfInertia = momentOfInertia;
    this.invInertia = momentOfInertia > 0 ? 1.0 / momentOfInertia : 0.0;

    this.position = { x: pos.x, y: pos.y };
    this.velocity = { x: 0, y: 0 };
    this.force = { x: 0, y: 0 };

    this.angle = angle;
    this.angularVelocity = 0;
    this.torque = 0;
  }

  applyForce(f) {
    this.force.x += f.x;
    this.force.y += f.y;
  }

  clearForces() {
    this.force = { x: 0, y: 0 };
    this.torque = 0;
  }
}

// Spring 2D Class
class Spring2D {
  constructor(bodyA, bodyB, restLength, stiffness, damping) {
    this.bodyA = bodyA;
    this.bodyB = bodyB;
    this.restLength = restLength;
    this.stiffness = stiffness;
    this.damping = damping;
  }

  update() {
    const dx = this.bodyB.position.x - this.bodyA.position.x;
    const dy = this.bodyB.position.y - this.bodyA.position.y;
    const currentLength = Math.sqrt(dx * dx + dy * dy);

    if (currentLength === 0) return;

    const dirX = dx / currentLength;
    const dirY = dy / currentLength;

    const stretch = currentLength - this.restLength;
    const springForceMagnitude = stretch * this.stiffness;

    const relVelX = this.bodyB.velocity.x - this.bodyA.velocity.x;
    const relVelY = this.bodyB.velocity.y - this.bodyA.velocity.y;

    const relVelAlongNormal = relVelX * dirX + relVelY * dirY;
    const dampingForceMagnitude = relVelAlongNormal * this.damping;

    const totalForce = springForceMagnitude + dampingForceMagnitude;

    this.bodyA.applyForce({ x: dirX * totalForce, y: dirY * totalForce });
    this.bodyB.applyForce({ x: -dirX * totalForce, y: -dirY * totalForce });
  }
}

// Soft Body 2D Class
class SoftBody2D {
  constructor() {
    this.particles = [];
    this.springs = [];
  }

  addParticle(mass, x, y) {
    const particle = new RigidBody2D(mass, 0, { x, y });
    this.particles.push(particle);
    return particle;
  }

  addSpring(indexA, indexB, stiffness, damping) {
    const pA = this.particles[indexA];
    const pB = this.particles[indexB];

    const dx = pB.position.x - pA.position.x;
    const dy = pB.position.y - pA.position.y;
    const restLength = Math.sqrt(dx * dx + dy * dy);

    const spring = new Spring2D(pA, pB, restLength, stiffness, damping);
    this.springs.push(spring);
  }

  applyInternalForces() {
    for (let spring of this.springs) {
      spring.update();
    }
  }

  integrate(dt) {
    for (let p of this.particles) {
      if (p.invMass === 0) continue;

      const accX = p.force.x * p.invMass;
      const accY = p.force.y * p.invMass;

      p.velocity.x += accX * dt;
      p.velocity.y += accY * dt;

      p.position.x += p.velocity.x * dt;
      p.position.y += p.velocity.y * dt;

      p.clearForces();
    }
  }
}

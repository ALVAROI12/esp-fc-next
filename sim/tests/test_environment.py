"""Smoke test: the simulation toolchain imports and pybullet can step a headless world."""

import pybullet as pb


def test_pybullet_headless_step():
    client = pb.connect(pb.DIRECT)
    try:
        pb.setGravity(0, 0, -9.81, physicsClientId=client)
        body = pb.createMultiBody(
            baseMass=0.05,
            baseCollisionShapeIndex=pb.createCollisionShape(pb.GEOM_SPHERE, radius=0.05, physicsClientId=client),
            basePosition=[0, 0, 1],
            physicsClientId=client,
        )
        for _ in range(240):
            pb.stepSimulation(physicsClientId=client)
        z = pb.getBasePositionAndOrientation(body, physicsClientId=client)[0][2]
        assert z < 1.0
    finally:
        pb.disconnect(client)

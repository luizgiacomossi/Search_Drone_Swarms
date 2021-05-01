import direct.directbase.DirectStart
from panda3d.core import Vec3
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletPlaneShape
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletBoxShape

base.cam.setPos(0, -10, 0)
base.cam.lookAt(0, 0, 0)

# World
world = BulletWorld()
world.setGravity(Vec3(0, 0, -9.81))

# Plane
shape = BulletPlaneShape(Vec3(0, 0, 1), 1)
node = BulletRigidBodyNode('Ground')
node.addShape(shape)
np = render.attachNewNode(node)
np.setPos(0, 0, -20)
world.attachRigidBody(node)

# Box
shape = BulletBoxShape(Vec3(0.5, 0.5, 0.5))
node = BulletRigidBodyNode('Box')
node.setMass(1.0)
node.addShape(shape)
np = render.attachNewNode(node)
np.setPos(0, 0, 2)
world.attachRigidBody(node)
model = loader.loadModel('models/box.egg')
model.flattenLight()
model.reparentTo(np)
model.addForce(LinearVectorForce(1,0,) )

# Update
def update(task):
    dt = globalClock.getDt()
    world.doPhysics(dt)
    return task.cont

taskMgr.add(update, 'update')
base.run()
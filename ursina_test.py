from ursina import *
from ursina.shaders import lit_with_shadows_shader # you have to apply this shader to enties for them to recieve shadows.

# create a window
app = Ursina()

EditorCamera()
Entity(model='plane', scale=10, color=color.gray, shader=lit_with_shadows_shader)
Entity(model='cube', y=1, shader=lit_with_shadows_shader)
pivot = Entity()
DirectionalLight(parent=pivot, y=2, z=3, shadows=True)

# start running the game
app.run()

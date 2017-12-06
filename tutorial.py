import pyglet


window = pyglet.window.Window()

quad = pyglet.graphics.vertex_list(4,
    ('v2i', (90, 100,  100, 90, 110, 100, 100, 110)),
    ('c3B', (0, 0, 255, 0, 0, 255, 0, 255, 0,  0, 255, 0)))


@window.event
def on_draw():
    window.clear()
    quad.draw(pyglet.gl.GL_POLYGON)

if __name__ == "__main__":
    pyglet.app.run()